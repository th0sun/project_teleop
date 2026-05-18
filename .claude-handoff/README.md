# `.claude-handoff/` — Repo-local Claude Code skill

This directory holds the **progress-report skill** for the project_teleop
repository. It lives in-repo (not under `~/.claude/skills/`) so a new
engineer or AI agent gets the skill for free as soon as they clone the
repo.

## What is in here

```
.claude-handoff/
├── SKILL.md            # skill definition (frontmatter + instructions)
├── progress_report.md  # markdown template the skill fills in
├── run_tests.sh        # helper called by SKILL.md to run unittests
└── README.md           # this file
```

## How an engineer uses it

If you have Claude Code installed and this repo is loaded, just say:

> "give me a progress report"
>
> or
>
> "ความคืบหน้าโปรเจกต์ตอนนี้ถึงไหน"

The skill aggregates branch state, recent commits, AGENTS.md open
items, test pass/fail, and adaptive telemetry into a single screen.

The skill never modifies the repo. Read-only by design.

## How an engineer extends it

Edit `SKILL.md`. The frontmatter `description` field is what triggers
the skill — phrasing changes go there. The body explains what data
the skill should aggregate and in what order.

`run_tests.sh` is independently runnable for humans who want a quick
pass/fail without going through Claude Code. Update it if the test
layout moves.

## How an engineer with their own `~/.claude/` registers it globally

The author keeps a stub at `~/.claude/skills/teleop-progress/SKILL.md`
that points at the repo-local copy. To replicate on a new machine:

```bash
mkdir -p ~/.claude/skills/teleop-progress
ln -sfn $(pwd)/.claude-handoff/SKILL.md \
        ~/.claude/skills/teleop-progress/SKILL.md
```

This is optional. The skill works from the repo path directly.
