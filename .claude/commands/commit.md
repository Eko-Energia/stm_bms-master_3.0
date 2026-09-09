### Commit (Conventional Commits)

Purpose: Create one or more git commits for the current changes using
Conventional Commits. First analyze and group related changes, then
create focused commits for each group. Keep messages as short as
possible and as long as necessary to be useful.

Do exactly the following:

1) Check repo state
- Run `git status --porcelain=v1 -z` to detect changes.
- If there are no changes, stop and report "No changes to commit".

2) Collect changes (do not stage yet)
- Detect changed files using `git status --porcelain=v1 -z`.
- Show working tree diffs for context using `git diff`.
- Include untracked files (respect `.gitignore`).

3) Analyze and group changes
- Group changes into logical commits. Use these heuristics:
  - Group by top-level directory/package or clear feature folder.
  - Separate docs (`README`, `*.md`, `docs/`) from code.
  - Separate tests (`tests/`, `__tests__/`, `*.test.*`, `*.spec.*`).
  - Separate config/build/CI (`package.json`, lockfiles, `*.yml` in CI,
    `Dockerfile`, build scripts) from application code.
  - Separate style-only changes (`*.css`, `*.scss`) from logic changes.
  - Use a dedicated group for refactors (no behavior change) when clear.
  - Use a "chore" group for tooling/formatting/minor maintenance.
- Do not split a single file across different groups. If a file mixes
  concerns, assign it to the predominant group and call out the nuance
  in the body text.
- Infer Conventional Commit type for each group:
  - `docs`: documentation files and docs-only edits.
  - `test`: tests only.
  - `style`: formatting or style-only (no behavior change).
  - `build`: build system or external dependencies.
  - `ci`: CI configuration and scripts.
  - `perf`: obvious performance improvements.
  - `fix`: bug fixes.
  - `feat`: new functionality.
  - `refactor`: code changes without behavior change.
  - `chore`: other maintenance tasks.
- Derive a concise scope from the primary area touched (e.g.,
  top-level directory, package name, or feature folder). Use one scope.
- Detect breaking changes when API/schema/contract changes are evident;
  mark with `!` after the type(scope) and add a footer later.

4) Draft messages per group (compatible with commitlint)
- Subject format: `<type>(<scope>): <summary>` in imperative mood,
  ~50 chars max, no period.
- Optional body wrapped at ~72 chars. Include only relevant sections;
  omit empty ones:
  - Why:
    - Brief business/context if helpful.
  - What changed:
    - Key edits (files/areas) and user-visible effects.
  - References:
    - Tickets/links (no secrets).
- If breaking, add a footer line:
  - `BREAKING CHANGE: <brief explanation>`

1) Preview commit plan
- Show the proposed commits in execution order (prefer: schema/migrations
  → backend → frontend → tests → docs → chore/build/ci). For each group
  display:
  - Subject: `<subject>`
  - Body (if any):
    ```
    <body>
    ```
  - Files to include in this commit (planned list)
- Ask: "Reply 'ok' to commit, or suggest edits to subjects/bodies/order
  or regroup. Reply 'single' to squash into one commit, or 'cancel' to
  abort."

1) Confirm and commit
- If the user replies exactly "ok" (case-insensitive):
  - For each group in order:
    - Ensure a clean index: `git reset`
    - Stage only that group's files: `git add -- <files...>`
    - Verify staged files: `git diff --cached --name-only`
    - Prepare commit message (avoid literal "\\n" in history):
      - Compose the message as a file using real newlines:
        - Create tmp file: `msgfile=$(mktemp -t commitmsg.XXXXXX)`
        - Write the message using printf to expand escapes (so "\\n"
          becomes a newline):
          - Subject-only:
            - `printf '%s\n' "$subject" > "$msgfile"`
          - Subject+body (blank line between):
            - `printf '%s\n\n%b\n' "$subject" "$body" > "$msgfile"`
            - Note: `%b` interprets backslash escapes in `$body` so
              sequences like `\\n` turn into real line breaks.
      - Commit using the file: `git commit -F "$msgfile"`
      - Clean up: `rm -f "$msgfile"`
- If the user requests changes to messages/order/groups, revise and
  re-show the preview, then wait for explicit "ok".
- If the user replies "single":
  - Stage all intended files (excluding dependencies):
    - `git reset`
    - `git add -A`
  - Prepare a message file exactly as above using `printf`:
    - Subject-only: `printf '%s\n' "$subject" > "$msgfile"`
    - Subject+body: `printf '%s\n\n%b\n' "$subject" "$body" > "$msgfile"`
  - Commit once with `git commit -F "$msgfile"` and then remove the
    tmp file.
- If the user declines or replies "cancel", stop without committing.

1) Output
- After each successful commit, print the subject and the commit SHA:
  - `git show -s --format=%h\ %s`
- After all commits, print the list in order of creation.

Notes:
- Do not push; only create local commits.
- Exclude generated and dependency folders (e.g., `node_modules`,
  `vendor`) if not already ignored.
- Avoid leaking secrets in messages. Do not paste raw data.
- Prefer several small focused commits. If changes are inseparable,
  use 'single' to squash.
- Partial-file splitting (by hunks) is not attempted; entire files
  belong to a single group.