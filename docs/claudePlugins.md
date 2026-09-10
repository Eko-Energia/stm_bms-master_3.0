# Claude Code plugins

`.claude/settings.json` declares the plugin set this project expects. Declaring a plugin is not
the same as installing it: `enabledPlugins` only marks a plugin enabled, and since Claude Code
v2.1.195 a plugin enabled solely by a project's settings does not load until it is installed on
the machine. On a fresh clone every plugin below must be installed once.

## Install everything

Run from the repository root:

```bash
for p in \
  superpowers \
  context7 \
  code-review \
  code-simplifier \
  github \
  coderabbit \
  skill-creator \
  mattpocock-skills \
  claude-md-management
do
  claude plugin install "$p@claude-plugins-official" --scope project
done

claude plugin install compound-engineering@compound-engineering-plugin --scope project
```

`--scope project` installs the plugin and records it in `.claude/settings.json`, so the file stays
the single source of truth. Restart Claude Code, or run `/reload-plugins`, to activate.

## Plugin set

| Plugin | Marketplace | Provides |
| --- | --- | --- |
| `superpowers` | `claude-plugins-official` | Brainstorming, planning, TDD, systematic debugging workflows |
| `context7` | `claude-plugins-official` | MCP server for live library and API documentation |
| `code-review` | `claude-plugins-official` | Diff review skill |
| `code-simplifier` | `claude-plugins-official` | Simplification agent for recently changed code |
| `github` | `claude-plugins-official` | GitHub MCP server (see troubleshooting) |
| `coderabbit` | `claude-plugins-official` | CodeRabbit review skill and agent |
| `skill-creator` | `claude-plugins-official` | Authoring and evaluating skills |
| `mattpocock-skills` | `claude-plugins-official` | Spec/ticket flows, TDD, domain modelling |
| `claude-md-management` | `claude-plugins-official` | Audits and revises `CLAUDE.md` |
| `compound-engineering` | `compound-engineering-plugin` | Review personas, planning and commit skills |

The `compound-engineering-plugin` marketplace is declared under `extraKnownMarketplaces` in
`.claude/settings.json`. Claude Code registers it without a prompt once the repository folder is
trusted, so only the install step above is needed.

## Scope

Project scope is additive. Plugins enabled in `~/.claude/settings.json` load here as well, and
project settings cannot switch them off short of an explicit `false` entry. The table above is the
set this repository requires, not the complete set active in any given session.

## Verify

`claude plugin list` reports `scope: project` for every repository, so filter by `projectPath`
to see what is installed for this one:

```bash
claude plugin list --json | jq -r --arg p "$PWD" '.[] | select(.projectPath == $p) | .id'
```

This should print all ten entries from the table above. Load failures appear in the `/plugin`
**Errors** tab.

## Troubleshooting

**`github` MCP server fails with `Authorization header is badly formatted`.** The plugin sends
`Bearer ${GITHUB_PERSONAL_ACCESS_TOKEN}`. When that variable is unset the placeholder is sent
literally and the endpoint returns 400. Export a token before starting Claude Code:

```bash
export GITHUB_PERSONAL_ACCESS_TOKEN="$(gh auth token)"
```

The Copilot MCP endpoint may reject a `gho_` OAuth token from `gh`; if it does, use a personal
access token instead.

**Skills missing after an install.** Run `/reload-plugins`, adding `--force` if it warns about
re-reading the conversation. If they are still missing, `rm -rf ~/.claude/plugins/cache`, restart,
and reinstall.
