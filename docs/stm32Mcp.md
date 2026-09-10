# stm32-mcp server

[stm32-mcp](https://github.com/shieldyguy/stm32-mcp) gives an AI assistant working in this
repository direct tools to build, flash, and talk to the BMS-Master hardware: CubeIDE headless
builds, ST-Link SWD flashing, serial/VCP exchanges, and live memory monitoring.

> The server is not a sandbox. Its tools drive a real compiler, a real debug probe, and real
> serial ports. Know what is plugged in before letting an agent use them.

## Layout

| Path | Tracked as | Purpose |
| --- | --- | --- |
| `.claude/mcp/stm32-mcp/` | git submodule, pinned | Upstream server source, left unmodified |
| `.claude/mcp/pyproject.toml` | committed | Environment definition; pins `mcp<2` |
| `.claude/mcp/uv.lock` | committed | Exact resolved dependency set |
| `.claude/mcp/.venv/` | gitignored | The synced environment, rebuilt per machine |
| `.mcp.json` | committed | Registers the server with Claude Code |

Claude Code reads project MCP servers **only** from `.mcp.json` at the repository root. The
`.claude/mcp/` directory holds the implementation, not the registration.

## Setup on a new machine

```bash
# 1. Host tools (macOS). CubeIDE must be at /Applications/STM32CubeIDE.app
brew install open-ocd stlink

# 2. Fetch the pinned server source
git submodule update --init .claude/mcp/stm32-mcp

# 3. Build the environment
uv sync --project .claude/mcp
```

Then restart Claude Code. `.claude/settings.json` already lists `stm32` under
`enabledMcpjsonServers`, so no trust prompt appears.

### Prerequisites

| Tool | Needed for | Install |
| --- | --- | --- |
| STM32CubeIDE | `stm32_build`, `stm32_build_and_flash` | ST installer, at `/Applications/STM32CubeIDE.app` |
| OpenOCD | flashing, memory read/write, live monitoring | `brew install open-ocd` |
| stlink | probe enumeration (`stm32_list_probes`) | `brew install stlink` |
| `arm-none-eabi-nm` | resolving variable names from the ELF | ships inside CubeIDE; add its `plugins/…/tools/bin` to `PATH`, or `brew install --cask gcc-arm-embedded` |

Python is handled by uv; no system interpreter needs preparing.

## Why the wrapper pyproject exists

Upstream declares `mcp>=1.0.0`. Resolving that today installs the 2.x SDK, which renamed
`FastMCP` to `MCPServer`, and the server dies on import:

```
ModuleNotFoundError: No module named 'mcp.server.fastmcp'
```

`.claude/mcp/pyproject.toml` therefore depends on the submodule as an editable path source and
adds `mcp<2` alongside it. `uv sync`/`uv lock` accept no `--constraints` flag, so the pin has to
live in a project file — and keeping it in *our* file rather than the submodule's leaves the
upstream checkout clean to pull. Drop the pin once upstream migrates to the 2.x API.

## Permission policy

Set in `.claude/settings.json`, chosen for this board specifically.

| Rule | Tools | Reason |
| --- | --- | --- |
| allow | `stm32_list_probes`, `stm32_board_info`, `serial_list_ports`, `stm32_build`, `stm32_read_memory`, `live_memory_read` | Enumeration, compilation, and reads: no hardware state change |
| ask | `stm32_flash`, `stm32_build_and_flash`, `stm32_write_memory`, `serial_send`, `serial_sequence` | Alters the board or drives the bus; confirm in every permission mode |
| deny | `stm32_fus_upgrade`, `stm32_ble_stack_install`, `stm32_fus_bootstrap` | STM32WB FUS/BLE operations. BMS-Master is an `STM32F105R8Tx`, so these can only be a misfire, and a misfired FUS write bricks the part |

Tools not listed fall through to the normal per-use prompt.

## Verifying it works

Speak MCP to the server directly, without going through Claude Code:

```bash
{ printf '%s\n' \
  '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"smoke","version":"0"}}}' \
  '{"jsonrpc":"2.0","method":"notifications/initialized"}' \
  '{"jsonrpc":"2.0","id":2,"method":"tools/list","params":{}}'; sleep 8; } \
| uv run --project "$PWD/.claude/mcp" stm32-mcp
```

A healthy server answers the `initialize` request and lists 20 tools. Inside Claude Code,
`/mcp` shows `stm32` as connected.

## Updating the pinned server

```bash
git -C .claude/mcp/stm32-mcp fetch origin
git -C .claude/mcp/stm32-mcp checkout <commit-or-tag>
uv sync --project .claude/mcp        # refreshes .claude/mcp/uv.lock
git add .claude/mcp/stm32-mcp .claude/mcp/uv.lock
```

Re-run the verification above before committing: an upstream bump can pull in a dependency set
that no longer imports.

## Notes

- `.mcp.json` uses `${CLAUDE_PROJECT_DIR:-.}`. Despite what the Claude Code documentation
  implies, `CLAUDE_PROJECT_DIR` is **not** exported into the environment of an `.mcp.json`
  server spawn — it is a hooks variable — so the `:-.` fallback is what actually resolves the
  path, against the current working directory. Start `claude` from the repository root.
  Launching it from a subdirectory is a separate project context anyway, with its own approval.
- Confirm registration with `claude mcp get stm32`; it should report
  `Scope: Project config (shared via .mcp.json)` and `Status: ✔ Connected`.
- Builds use the CubeIDE headless builder against a project path — pass this repository root,
  which holds `.project`/`.cproject` for `BMS-Master`.
- The server opens a serial bridge on `127.0.0.1:8765` at startup. Only one instance can hold
  that port, so a second Claude Code session against this repo will log a bind failure.
- Board and probe nicknames persist in the server's own state, keyed by MCU UID and ST-Link
  serial number.
