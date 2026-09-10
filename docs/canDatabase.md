# CAN database and C source generation

The CAN frame definitions for Perła live in their own repository,
[Eko-Energia/CAN-DATABASE](https://github.com/Eko-Energia/CAN-DATABASE), pinned here as a git
submodule. The C pack/unpack code for those frames is **generated**, never hand-written, using
the team's cantools fork, [Eko-Energia/cantools](https://github.com/Eko-Energia/cantools).

This page is written so an agent or a new contributor can go from a fresh `git clone` to
generated sources without asking anyone.

> The `.dbc` files are read-only from this repository. Per the CAN-DATABASE README, editing them
> requires a coordinator's authorization and must go through a pull request on that repo. Change
> the database there, then bump the submodule pointer here.

## Layout

| Path | Tracked as | Purpose |
| --- | --- | --- |
| `docs/CAN-DATABASE/` | git submodule, pinned | `CAN_DB.dbc` and `CAN2_DB.dbc`, upstream and unmodified |
| `.claude/tmp/cantools/` | gitignored | Clone of the cantools fork, fetched on demand |
| `.claude/tmp/.venv/` | gitignored | uv environment holding the `cantools` CLI |

The fork is deliberately **not** a submodule. It is a build-time code generator, not something
the firmware depends on, so it does not belong in the dependency graph of this repository. It is
disposable: delete `.claude/tmp/` and re-run the bootstrap below at any time.

## Setup after a fresh clone

```bash
# 1. Fetch the pinned CAN database
git submodule update --init docs/CAN-DATABASE

# 2. Fetch the cantools fork into the ignored scratch directory
git clone https://github.com/Eko-Energia/cantools.git .claude/tmp/cantools

# 3. Build the generator environment
uv venv .claude/tmp/.venv
uv pip install --python .claude/tmp/.venv ./.claude/tmp/cantools
```

Verify:

```bash
.claude/tmp/.venv/bin/cantools --version
```

A version like `0.1.dev1948+g46ec68183` is correct — see [Gotchas](#gotchas). uv provides the
Python interpreter; no system Python needs preparing.

## Generating the C sources

Run from the repository root. `<output-dir>` is wherever the generated pair should land.

```bash
# CAN1 — the bus BMS Master transmits on. Filter to this node.
.claude/tmp/.venv/bin/cantools generate_c_source \
    docs/CAN-DATABASE/CAN_DB.dbc --node BMSMaster -o <output-dir>

# CAN2 — the pack thermistor bus. BMS Master only listens, so do NOT pass --node.
.claude/tmp/.venv/bin/cantools generate_c_source \
    docs/CAN-DATABASE/CAN2_DB.dbc -o <output-dir>
```

The first command writes `CAN_DB.h` / `CAN_DB.c`; the second writes `CAN2_DB.h` / `CAN2_DB.c`.
Output file names come from the input file stem, overridable with `--database-name`.

Generated sources are not committed today. `EKO_Drivers/CAN/` holds the hand-written transport
layer ([can.md](can.md)); the generated files are the frame layer that sits on top of it. Decide
a home for them when the application layer lands, and regenerate rather than edit them.

Useful flags: `--bit-fields` to minimise struct sizes, `--use-float` for single-precision
scaling, `--no-floating-point-numbers` to keep the generated code integer-only. Run
`cantools generate_c_source --help` for the full list.

## Node names

The identifiers the generator expects, read from the pinned `.dbc` files:

| Database | Nodes relevant here | BMS Master's role |
| --- | --- | --- |
| `CAN_DB.dbc` | `BMSMaster` (plus 25 other vehicle nodes) | transmits `BMSMaster_MasterVoltCurrTemp`, `BMSMaster_PCBsTherm1Temp` … `BMSMaster_PCBsTherm9Temp`, `BMSMaster_JK_*` |
| `CAN2_DB.dbc` | `PCBCells1` … `PCBCells7` | receives `PCBCells<x>_Therm<y>`; `BMSMaster` is not a node in this database |

[notionSpec.md](notionSpec.md) writes these as `PBCCELLSx_Thermy` and
`BMSMaster_PCBsTherm<y>Temp1`. The database spells them `PCBCells<x>_Therm<y>` and
`BMSMaster_PCBsTherm<y>Temp`. The database wins.

## Why the fork and not PyPI cantools

The fork preserves DBC identifiers verbatim. Upstream cantools snake_cases them, which mangles
the team's naming and breaks any code written against the database's own names. Generating
`CAN_DB.dbc --node BMSMaster` with each:

| | Fork | Upstream cantools |
| --- | --- | --- |
| Output files | `CAN_DB.h` / `CAN_DB.c` | `can_db.h` / `can_db.c` |
| Struct | `struct BMSMaster_PCBsTherm1Temp_t` | `struct can_db_bms_master_pc_bs_therm1_temp_t` |
| Function | `BMSMaster_PCBsTherm1Temp_pack()` | `can_db_bms_master_pc_bs_therm1_temp_pack()` |

Note `PCBs` becoming `pc_bs`. Do not substitute `pip install cantools` or `uvx cantools`.

## Gotchas

- **`--node` on `CAN2_DB.dbc` fails silently.** `BMSMaster` is not a node in that database, so
  `--node BMSMaster` filters everything out and still exits 0 with a cheerful success message.
  The tell is size: roughly 2 KB of output instead of roughly 94 KB. Omit `--node` for CAN2.
- **The odd version string is expected.** The fork carries no git tags, so `setuptools_scm`
  falls back to `0.1.dev<N>+g<hash>`. It is not a broken install.
- **Installing needs the fork's `.git` directory.** `setuptools_scm` derives the version from git
  history, so install from the clone. Copying the source tree without `.git` breaks the build.
- **`docs/CAN-DATABASE/` is empty after a plain `git clone`.** Submodule contents are not fetched
  by default; run step 1 of the bootstrap.

## Updating the pinned database

```bash
git -C docs/CAN-DATABASE fetch origin
git -C docs/CAN-DATABASE checkout <commit-or-branch>
git add docs/CAN-DATABASE
```

Regenerate the C sources afterwards and check the diff: a renamed signal in the `.dbc` becomes a
renamed struct field, which breaks compilation at the call site rather than at generation time.
