---
name: teleop-progress
description: |
  Generate a project-status / progress report for the MG400 VR teleop
  repository. Use this when the operator says things like "progress
  report", "project status", "where are we", "what's next",
  "ความคืบหน้า", "สถานะโปรเจกต์", or asks for a daily standup style
  summary of this repo.

  This skill is REPO-LOCAL: it lives under `.claude-handoff/` of the
  project_teleop repository so a new engineer with no `~/.claude/`
  setup still has it.
---

# Teleop Progress-Report Skill

## When to invoke

Trigger phrases include (English + Thai):
- "progress report"
- "project status"
- "where are we"
- "what's next"
- "ความคืบหน้า"
- "สถานะโปรเจกต์"
- "ถึงไหนแล้ว"
- "standup"

## What the report covers

Compose the report by aggregating the following — in this exact order:

1. **Header**: today's date, current branch (`git branch --show-current`),
   current commit (`git rev-parse --short HEAD`), and whether the working
   tree is clean (`git status -s`).

2. **Today's summary (1-3 lines)**: read `progress_report.md` template,
   summarise commits since yesterday using
   `git log --since="yesterday" --oneline`.

3. **Branch state**: list commits ahead of the merge base with
   `feat/multi-robot-teaching-architecture`:
   ```bash
   git log --oneline $(git merge-base HEAD feat/multi-robot-teaching-architecture)..HEAD
   ```
   For each, give one line "what / why".

4. **Open items (Next-steps queue)**: read `AGENTS.md` section 10 and
   reproduce its checkbox list, marking what has shipped since the
   last report.

5. **Test status**: run `.claude-handoff/run_tests.sh`. Capture pass/fail
   per file. Surface any failures verbatim — never hide them.

6. **Adaptive telemetry**: list the most recent 3 CSV files under
   `~/.ros/adaptive_telemetry/` if present (size + mtime). These are
   the per-cmd analytics from the last few hardware runs.

7. **Known blockers**: cross-reference `AGENTS.md` section 4 (deferred
   / known smells) — flag any that are now actionable because the
   "proof of death needed" condition has been met.

8. **Suggested next action**: pick the topmost unchecked item from the
   AGENTS Next-steps queue and quote its description. Do not invent
   new work.

## How to render

Output as a markdown block, in the operator's language. Default to Thai
when triggered by a Thai phrase, English otherwise. Keep it under
~80 lines so the operator can read it in one screen.

Section headers in the output match the order above (Header → Today
→ Branch → Open items → Tests → Telemetry → Blockers → Next).

## Safety

- Never modify the repo while generating the report. Read-only ops only.
- Do not push commits as a side effect of running this skill.
- Do not delete telemetry CSVs even if "old".
- If `run_tests.sh` exits non-zero, report it. Do not pretend tests pass.
- If `git status -s` shows uncommitted changes, say so in the header —
  do not silently elide.

## Related files

- `AGENTS.md` at repo root — the source of truth this skill reads.
- `.claude-handoff/progress_report.md` — fill-in template.
- `.claude-handoff/run_tests.sh` — test runner the skill calls.
- `.claude-handoff/README.md` — one-page onboarding for this skill.
