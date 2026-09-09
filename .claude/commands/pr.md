### Create PR (Good PR, Uses Template If Present)

Purpose: Create a high-quality Pull Request using the repo’s PR template if it exists; otherwise, generate a best-practice PR body with clear sections (Overview, Context, Changes, Tests, Screenshots, Risks, Rollback, Related Issues, Checklist). Assign reviewers, add labels, and link issues. Uses `gh` CLI.

Do exactly the following:

ALWAYS DO A DRAFT PR!

1. Preconditions & tooling
   * Verify Git is available: `git --version`.
   * Verify GitHub CLI is available: `gh --version`.
   * Verify you’re authenticated with GitHub: `gh auth status`.
     * If not authenticated, stop and output: **"GitHub CLI not authenticated. Run: gh auth login"**.

2. Detect repo & branch state
   * Ensure we are inside a Git repo; if not, stop with **"Not a git repository."**.
   * Determine the current branch: `git rev-parse --abbrev-ref HEAD` (call this `$BRANCH`).
   * If `$BRANCH` equals `main` or `master`, warn: **"You are on $BRANCH. PRs should come from a feature branch."** and continue only if the user replies exactly `ok`.
   * Check for uncommitted changes: `git status --porcelain=v1 -z`.
     * If there are unstaged/unstashed changes, ask: **"There are local changes. Reply 'stash' to stash, 'commit' to continue after committing, or 'cancel' to abort."**
       * If `stash`, run `git stash -u -m "temp-pr-stash"` and continue.
       * If `commit`, stop with **"Please commit and re-run the command."**
       * If `cancel`, stop.
   * Check if the branch is ahead/behind remote:
     * Fetch: `git fetch -q`.
     * If branch has no upstream, set upstream when pushing in step 3.

3. Push branch if needed
   * If not pushed: `git push -u origin "$BRANCH"`; on error, stop and report the error.
   * If behind remote, prompt: **"Your branch is behind remote. Reply 'merge' to merge main, 'rebase' to rebase on main, or 'skip' to continue."**
     * For `merge`: `git fetch origin main && git merge --no-edit origin/main`.
     * For `rebase`: `git fetch origin main && git rebase origin/main`.
     * For `skip`: continue.

4. Determine target base branch
   * Default base is `main` if it exists, else `master`.
     * Detect with: `git show-ref --verify --quiet refs/remotes/origin/main` then `refs/remotes/origin/master`.
   * Call this `$BASE`.

5. Collect metadata for the PR
   * Propose a PR title:
     * Prefer the latest conventional commit subject if it exists: `git log -1 --pretty=%s`.
     * Else, format from branch name by replacing `-` and `/` with spaces and title-casing (e.g., `feat/user-login` → `feat: user login`).
     * Ask the user: **"Proposed title: '<title>'. Reply with a new title or 'ok' to accept."**
   * Using Conventional Commit recommandation, draft a short, one-line **Overview** derived from recent commits (last 10): `git log --pretty=format:"- %s" -n 10`.
   * Detect linked issues in branch or commits (e.g., `ABC-123`, `#123`):
     * Grep branch name and last 20 commit messages for patterns: `#[0-9]+` and `[A-Z]{2,}-[0-9]+`.
     * Build a `Relates/Closes` footer suggestion: e.g., `Closes #123` if an issue reference is present.
   * Suggest default reviewers and labels:
     * Reviewers (from CODEOWNERS if present): look up file `.github/CODEOWNERS`; map to GitHub usernames (strip `@`); de-duplicate.
     * Labels: if repo has labels matching `feature`, `bug`, `chore`, `docs`, `refactor`, `ci`, `build`, `perf`, choose based on dominant change type inferred from modified paths (src, tests, docs, ci, etc.). Otherwise, none.

6. Detect and load a PR template (preferred)
   * Search for templates in this order (first match wins):
     1. `.github/PULL_REQUEST_TEMPLATE.md`
     2. `PULL_REQUEST_TEMPLATE.md` at repo root
     3. `.github/PULL_REQUEST_TEMPLATE/*.md` (if multiple, prefer one named `default.md`; if multiple without a `default`, choose the first alphabetically)
   * If a template file is found:
     * Use it “as is” as the base PR body (do not modify headings).
     * Prepare a temporary working copy (call it `$BODYFILE`) and append a **Context** block with auto-generated bullet points (Overview, linked issues, etc.) beneath the first heading.
   * If no template exists, generate a **best-practice PR body** into `$BODYFILE` with the following markdown sections (keep headings exactly; leave empty subsections out):
     ```
     ## Overview
     <1-3 sentences problem statement / motivation>

     ## Context
     <why now / background / any relevant tickets or docs>

     ## Changes
     - <key change 1>
     - <key change 2>
     - …

     ## Screenshots / Demos
     <drag & drop images, or paste recording links>

     ## Tests
     - [ ] Unit tests added/updated
     - [ ] Integration/E2E considerations
     - [ ] Manual QA steps: <steps>

     ## Risks & Impact
     <perf, security, migrations, user-visible changes, rollout plan>

     ## Rollback Plan
     <how to revert safely; data/backfill considerations>

     ## Related
     <Closes #123 / Relates ABC-123 / Docs link>

     ## Checklist
     - [ ] Title is concise and imperative
     - [ ] Scope is focused (small PR if possible)
     - [ ] Breaking changes documented
     - [ ] Docs updated (README/Changelog if needed)
     - [ ] Feature flags / migrations noted
     ```

7. Preview before creating
   * Show a structured preview:
     ```
     PR Title: <title>
     Base: <base>   Head: <branch>
     Labels: <labels or none>
     Reviewers: <reviewers or none>
     Body (first 40 lines):
     --------------------------------
     <body preview>
     --------------------------------
     ```
   * Ask the user: **"Reply 'create' to open the PR, 'edit body' to revise the body, 'edit title' to change the title, 'labels' to edit labels, 'reviewers' to edit reviewers, or 'cancel' to abort."**
     * If `edit body`, open `$BODYFILE` in `$EDITOR` if set, else ask for inline replacement and write to `$BODYFILE`.
     * If `edit title`, capture new title and update.
     * If `labels` or `reviewers`, capture comma-separated lists and update internal sets.

8. Create the PR with `gh`
   * Run exactly (compose args conditionally):
     * `ARGS=( pr create --base "$BASE" --head "$BRANCH" --title "$TITLE" --body-file "$BODYFILE" )`
     * If labels exist: `ARGS+=( --label "<comma-separated-labels>" )`
     * If reviewers exist: `ARGS+=( --reviewer "<comma-separated-reviewers>" )`
     * Execute: `gh "${ARGS[@]}"`
   * On success, capture PR URL: `gh pr view --json url --jq .url` and print:
     * **"PR created: <url>"**
   * If creation fails due to missing permissions (e.g., forks), retry without reviewers/labels; if still failing, output the error and stop.

9. Post-creation niceties (optional, best effort)
   * If a milestone is detected from branch/issue pattern (e.g., `release/*`), attempt: `gh pr edit --milestone "<milestone>"`.
   * If the repo uses auto-merge, enable it if allowed: `gh pr merge --auto --squash` (only if team policy suggests it and the user replies `enable auto-merge`).

10. Cleanup
    * Remove any temp files created for `$BODYFILE`.

Notes:
* Respect the repository’s PR template if present (do not overwrite its headings).
* Keep titles imperative and ≤ 72 chars when possible.
* Prefer small, focused PRs; if large, clearly list risk areas and test plan.
* Never leak secrets in the PR body or screenshots.