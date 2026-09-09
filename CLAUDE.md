# Claude Code Instructions

For work in this project, follow the shared AI guidance in [AGENTS.md](AGENTS.md).

Before producing code or documentation, read the task-specific document listed in `AGENTS.md`. For hardware behavior, use `BMS-Master.ioc` as the source of truth and cross-check `docs/pcb.md`.

Keep the copied project clean: preserve the generated CubeMX structure, keep application logic out of generated files, and do not add the original `BMS_Driver` tree unless the user explicitly requests it. When a document describes reference behavior that is not implemented in this project, label that distinction clearly.

Use the existing naming convention: the documentation directory is `docs`, documentation files use lowercase-first camelCase names, and Markdown files use the `.md` extension.
