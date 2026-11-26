# Workbench redesign — making a run readable

> **Status:** design proposal, nothing implemented. Companion wireframe:
> [`workbench-redesign.png`](workbench-redesign.png) (source: `workbench-redesign.svg`) — open it
> beside this document.
> This plan comes before M4 because the delivery stage will make the panel the place where
> you approve a change; approving on the current layout would be approving blind.

---

## 1. What the screenshot reveals

The panel is not ugly — it is *unreadable in the way that matters*. Six problems, in the
order they cost the reader:

1. **Jargon without a key.** `verified`, `autonomy branch`, `DONE`, and the tab name `Runs`
   each mean something specific and none of them says what. A first-time reader cannot tell
   whether "verified" is good, who verified it, or what "autonomy" governs.
2. **The graph is decorative.** Five boxes, each a label and one word. The single most
   important question — *what did each step actually do?* — is not answered anywhere.
3. **Everything is flattened into a log.** Twelve lines of the same visual weight: a stage
   start, a gate pass, the decision, the diff summary. Timestamps lead every line, so the
   eye reads time instead of meaning.
4. **The data that answers the questions exists and is thrown away.** Fourteen findings,
   nine risks, eight test commands, four gate logs, eleven changed files, two open
   questions — the panel shows a count of ten of those things combined.
5. **Paths and hashes as primary content.** `C:\Users\Sebastian\…\144.verify.json` in
   10 px monospace is the last thing on screen and the least useful.
6. **No way to act.** You can look at a run but not re-run it, open its diff, or copy its
   summary — you go back to the composer and type a command.

---

## 2. What the data already holds

| Available today | Where | Shown? |
|---|---|---|
| 9 risks with severity + mitigation | `144.plan.json` | **no** |
| 11 planned files with the intended change | `144.plan.json` | **no** |
| 8 test commands, 2 open questions | `144.plan.json` | **no** |
| 14 findings with rule, claim, `file:line` evidence, fix | `144.verify.json` | **no** |
| 4 gate results with command, exit code, duration | `144.verify.json` | status word only |
| 4 gate output tails (up to 4 kB each) | `144.verify.json` | **no** |
| implementation commands the agent ran (7) | `144.json` | **no** |
| per-step timestamps | `144.json` events | as monotone text |

**Two gaps that are bugs, not omissions:**

- **The implementer's per-file report is lost.** `record.implementation.changedFiles` stores a
  *number*, not the array. The paths survive in `diffStat`, but **why each file changed** was
  only in the agent's reply and was never persisted. The record must keep the full array.
- **`/api/run` does not serve what the view needs.** It returns the record plus derived stages.
  Plans, findings, gate outputs and per-file reasons are all on disk and never cross the route.

---

## 3. Principles

1. **One question per element.** Header: *what is this and where does it stand?* Steps: *what
   ran?* Detail: *why?* Findings: *what must change?*
2. **Progressive disclosure.** A summary line always; a detail block on demand; the raw log
   behind a disclosure. Nothing is shown before it is needed, and everything is one click away.
3. **Name the machine.** No status word appears without a plain-language phrase next to it.
4. **Evidence or it didn't happen.** A failed gate shows its command and the tail of its
   output; a finding shows `file:line` and a fix; a changed file shows its path and reason.
5. **Act where you decide.** Actions sit in the header of the thing they act on.

---

## 4. The new shape

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ #144  Add model-wide physics display controls              ⚠ Needs attention  │
│ sstarosz/MayaMMD-Dev · branch issue-144-add-model-wide-…   started 2 h ago    │
│ [Re-verify] [Implement] [Open diff] [Copy summary] [Cancel]                   │
├──────────────────────────────────────────────────────────────────────────────┤
│ Planned 11 files · Implemented 11 files (+1680 −24) · Verified 0 blockers    │
│ 4 warnings · 14 findings · Not delivered (M4)                                │
├──────────────────────────────────────────────────────────────────────────────┤
│ STEPS                                                                        │
│ ✔ 1  Intake      2 s      development task, all three sections present       │
│ ✔ 2  Plan        8 min    planner agent · 11 files · 9 risks · 2 questions  ▸ │
│ ✔ 3  Implement  15 min    implementer agent · 11 files · 7 commands         ▸ │
│ ⚠ 4  Verify      3 min    5 reviewers · 4 gates · 0 blockers · 4 warnings   ▾ │
│ ○ 5  Deliver       —      not implemented yet (M4)                           │
└──────────────────────────────────────────────────────────────────────────────┘
```

Expanding **Verify** in place:

```
   WHAT RAN                                          RESULT
   Reviewer · Architecture                           pass    0 findings   high
   Reviewer · Constraints                            warn    2 findings  ▸
   Reviewer · Physics                                warn    6 findings  ▸
   Reviewer · Maya matrix                            pass    0 findings   high
   Reviewer · Tests                                  pass    4 findings  ▸
   Gate · ruff format --check   python -m ruff …     pass    1.0 s
   Gate · ruff check            python -m ruff …     fail    0.4 s  advisory ▸ output
   Gate · mypy                  python -m mypy …     fail    7.4 s  advisory ▸ output
   Gate · pytest                python -m pytest …   pass    8.3 s
```

Expanding **Implement**:

```
   FILES CHANGED (11)
   mmd/maya/physics_draw_modes.py    +354  new batch writer, one undo chunk
   mmd/ui/physics_panel.py           +309  model-row Display section
   …

   COMMANDS RUN (7)  ▸ ruff format --check ✔ · mypy … ✔ · pytest … ✔
   DEVIATIONS        none
   DIFF              11 files, +1680 −24   [open 144.diff]
```

Expanding **Plan**:

```
   RISKS (9)
   high    undo chunk can leak if closeChunk fails → close in finally
   high    refresh inside the combo handler deletes the emitting widget → defer
   …
   OPEN QUESTIONS (2)
   Is #143 expected to land first?
   DIFF BASE COHERENCE   every planned file existed in origin/main except the new one
```

Below the steps, the findings — the reason anyone opens this panel:

```
FINDINGS (14)                       [all] [blocker 0] [warn 4] [note 10]
⚠ Physics · batched scene queries                          mmd/ui/tool_main_widget.py:560
  One model-wide click re-reads every model in the scene…
  Fix: classify from batched reads…
▸ ⚠ Constraints · README describes old behaviour            README.md:61
▸ · 10 more notes
```

---

## 5. Per-step content

| Step | Header line | Expanded detail |
|---|---|---|
| **Intake** | issue state, labels, and which form sections were present | the body's form check, link to the issue |
| **Plan** | scope, file count, risk count, question count | risks with severity and mitigation, planned files with intent, test plan, the diff-base coherence check |
| **Implement** | file count, insertions/deletions, command count, deviations | files with per-file reason and the stat delta, commands actually run, deviation list, diff link |
| **Verify** | reviewers passed/warned, gates passed/failed, blockers/warnings | each reviewer with verdict and findings, each gate with its command, exit code, duration, advisory flag and output tail |
| **Deliver** | "not implemented yet (M4)" | — |

The **"what ran"** column is the answer to the question the current panel cannot answer: for a
gate it is the literal command line; for an agent it is its role plus the tools it had; for a
step with no subprocess it is the one-line description of the read it performed.

---

## 6. Findings treatment

- Sorted **blocker → warn → note**, then by verifier.
- Each finding: severity dot, `verifier · rule` as the title, the `file:line` evidence as a
  small monospace chip on the right, the claim as prose, the fix as a line starting `Fix:`.
- A finding whose blocker was **downgraded for missing evidence says so** — the rule must be
  visible, not silent.
- Filters are counts, not verbs: `[blocker 0] [warn 4] [note 10]`.
- Clicking an evidence chip opens that file in the worktree if the editor integration exists,
  otherwise copies the `path:line`.

---

## 7. Vocabulary

| Now | Replace with |
|---|---|
| tab `Runs` | `Runs` with subtitle *"every run of the task workflow, newest first"* — and it is a **ledger**, not a launcher; the header actions turn it into one |
| `verified` badge | `Decision: needs attention (warn)` / `Decision: ready to deliver` / `Decision: blocked` |
| `autonomy branch` | `Autonomy: branch — commits locally, nothing is pushed without you` |
| `DONE` under a node | `done · 8 min` |
| `warn · 0 blocker(s) · 4 warning(s)` | `0 blockers · 4 warnings` beside a severity chip |
| raw absolute paths | a `Details` disclosure with copy buttons |

---

## 8. Data contract changes

`GET /api/run?repo=&issue=` gains a bounded `detail` object:

```json
{ "steps": [ { "id": "verify", "status": "warn", "startedAt": "…", "durationMs": 191000,
               "what": "5 reviewers · 4 gates",
               "children": [
                 { "kind": "reviewer", "id": "physics", "label": "Physics", "verdict": "warn",
                   "findings": [ … ] },
                 { "kind": "gate", "id": "mypy", "label": "mypy",
                   "command": "python -m mypy mmd tests", "status": "fail", "exitCode": 1,
                   "durationMs": 7440, "advisory": true, "outputTail": "…" } ] } ],
  "findings": [ { "verifier": "physics", "severity": "warn", "rule": "…", "claim": "…",
                  "evidence": "mmd/ui/tool_main_widget.py:560", "suggestion": "…",
                  "downgraded": false } ],
  "risks": [ { "risk": "…", "severity": "high", "mitigation": "…" } ],
  "questions": [ "…" ],
  "files": [ { "path": "mmd/ui/physics_panel.py", "reason": "…", "stat": "+309 −4" } ] }
```

**Caps, so the route stays a route:** 50 findings, 20 risks, 40 files, 2 kB of output tail per
gate, 120 kB total. Everything over a cap is marked `truncated: true` on its section and the
full data stays on disk at the artifact path shown in `Details`.

**Two Host changes first:**

1. `runImplementStage` must persist `implementation.changedFiles` as the **array** (path +
   change), not `.length`. Existing runs cannot recover the reasons; the plan shows paths from
   `diffStat` and marks reasons unavailable.
2. `deriveRun` grows a `steps` derivation with `kind`, `what`, durations and children, plus a
   `detail` assembler that reads `plan.json` and `verify.json` with the caps above.

---

## 9. Interaction rules

- **Accordion, not modal.** One step open at a time by default; shift-click to open several.
  The chosen step survives a refresh.
- **Deep link.** `#workbench/run/144/verify` reopens exactly this state, so a warning can be
  pasted into an issue.
- **Keyboard.** Steps are buttons; `Enter` expands; `↑/↓` moves; `Esc` collapses.
- **Refresh is explicit.** A "run is stale" hint appears when the record mtime changed since
  load, instead of polling.

---

## 10. Phases

| Phase | Work | Acceptance |
|---|---|---|
| **R1 — data** | persist the implementer file array; add `steps` + `detail` to `/api/run` with caps; harness tests for caps and for old records | `curl /api/run` on #144 returns risks, files, findings, gate tails and durations; a run recorded before R1 still renders |
| **R2 — layout** | header + actions, summary strip, vertical step accordion replacing the graph, findings list with severity filters, `Details` disclosure for paths | a reader who has never seen it can say what the task is, how far it got, and whether it needs attention without expanding anything |
| **R3 — drill-down** | per-reviewer findings, per-gate output, per-file diff section (`/api/run/file`) | every on-screen claim is traceable to evidence in one click |
| **R4 — act** | Re-verify, Implement, Cancel, Copy PR summary — calling the same Host functions the tools call | the panel can advance a run without typing a command |

R1 and R2 together are the visible jump; R3 is what makes it trustworthy; R4 belongs with M4.

---

## 11. Risks of this redesign

- **Payload creep.** The caps exist because a run can hold 14 findings and 40 kB of gate
  output today; without them the route becomes a file server. Keep `truncated` honest.
- **Over-building a debugger.** The panel serves "should I deliver this?", not forensic
  archaeology. The raw event log and artifact paths belong behind one disclosure, not on the
  main surface.
- **Stale runs.** A run recorded before R1 lacks file reasons and stage durations; the UI must
  say "not recorded in this run" rather than render an empty box.

---

## 12. Decisions

Settled before R1:

1. **Layout — vertical accordion.** The node strip is replaced, not extended. The pipeline is
   still *named* in the summary strip (`planned → implemented → verified → delivered`), but each
   step gets a row that expands downward. No minimap graph, no side drawer.
2. **Runs advances the selected run; it does not launch new ones.** Header actions act on the
   run you are looking at (Re-verify, Implement, Cancel, Copy summary). Starting a run for a
   new issue stays in the composer, where the issue list and its `/issue-start` command live.
3. **Diffs open externally.** The panel lists changed files with their reason and stat delta and
   offers `Open diff`; per-file hunks are not built. R3 therefore drops the `/api/run/file`
   endpoint from its scope.

Still open, to settle at R2:

- **Density with parallel runs.** One run at a time is the current assumption. If three issues
  run concurrently the list needs grouping by state (needs attention / in progress / delivered)
  rather than a flat newest-first list.
- **Advisory gates vs. findings.** Both render as `warn` today. R2 should separate them
  visibly — an advisory `ruff check` failure is *pre-existing noise in the tree*, while a
  warning finding is *something this diff should fix* — but whether that is a second section or
  a filter is a visual call best made against the built layout.

---

## 13. R1 + R2 delivered

**R1 — data.** `GET /api/run` now returns a bounded `detail`:

- `steps[]` — one per pipeline stage with `what` (a sentence: *"implementer agent · 11 files · 0
  deviations · 7 commands"*), `status`, `startedAt`, `durationMs` computed from **real** events
  only, and `children[]`
- children by kind: `reviewer` (verdict, confidence, its findings), `gate` (**command, status,
  exit code, duration, advisory, output tail**), `file` (path + reason), `command`, `risk`
  (severity + mitigation), `question`, `planfile`
- `findings[]` with verifier, severity, rule, claim, `file:line` evidence, fix and the
  `downgraded` marker; `risks[]`, `questions[]`, `files[]`
- `truncated` per section and `missing` flags, so a run recorded before this work says *"the file
  reasons for this run were not recorded"* instead of rendering an empty box
- caps: 50 findings, 20 risks, 40 files, 2 kB output tail per gate

`runImplementStage` now persists `implementation.changedFiles` as the **array** of
`{path, change}`; older records keep a count and are detected via `missing.fileReasons`.

Measured on the live route for #144: **69.2 kB**, 5 steps, 14 findings, 9 risks, verify duration
**421 s**, four gate children each with a real output tail (2 000 chars for `ruff check` and
`mypy`).

**R2 — layout.** The node strip and the flat event dump are gone. The run detail is:

1. **header** — issue title, repo · branch · started, and a decision pill that says
   `! Needs attention` / `✓ Ready to deliver` / `✕ Blocked by findings`
2. **two copy actions** — summary (a markdown digest of the non-note findings) and the diff path.
   Real mutating actions arrive in R4; the panel does not pretend to have them
3. **summary strip** — counts per step, no log lines
4. **steps accordion** — status glyph, name, what ran, duration; one open at a time
5. **substeps inside a step** — reviewers toggle their findings, gates toggle their output tail,
   files and risks are plain rows with their reason and severity
6. **findings** — severity filters as counts (`all/blocker/warn/note`), each card collapsible with
   claim, evidence and fix
7. **one scroller** for the whole detail, with `Details` (paths) and `Raw event log` behind
   disclosures

Verified: harness **148 checks** including the detail payload, the caps, a legacy record's
`missing.fileReasons`, a rich record's per-file reasons, plan risks/questions as children, and
durations from events. The client slot still registers `github-tasks` in `conversation.view`, and
the live payload round-trips at 69 kB.

Not verified: the rendered layout (no browser control in this session). R3 (per-file diff
sections) and R4 (actions) remain.

---

## 14. R3 delivered

The rule for this phase was "every claim on screen is one click from its evidence". Four things
were missing; all four are in.

**1. Files that were only a count now have names and real numbers.** `deriveRunDetail` takes the
stored patch and counts `+`/`-` lines per `diff --git` section, falling back to the
`diff --stat` table when the patch is absent. So #144 — whose record stored `changedFiles: 11` —
now lists all eleven files with exact counts parsed from its own diff:

| file | |
|---|---|
| `mmd/maya/physics_draw_modes.py` | +354 −0 |
| `mmd/ui/physics_panel.py` | +290 −19 |
| `tests/unit_tests/maya/test_physics_draw_modes.py` | +332 −0 |
| `mmd/ui/physics_controls.py` | +92 −1 |
| `CHANGELOG.md` | +7 −0 |

Reconstructed rows carry `reasonMissing: true` — the per-file *reason* was never recorded for this
run — and the panel says so instead of implying the files changed for no reason.

**2. Gate output shows its command.** Opening a gate now renders the command, a **Copy command**
action, the `last 2 kB of output` marker when the raw output was longer (`ruff check` and `mypy`
in this run both are), and the tail itself.

**3. Reviewer focus.** Every reviewer row carries *"Show only this reviewer"*, and the findings
section gained a **reviewer chip row** beside the severity row, with counts. Filtering is now
`severity × reviewer` instead of severity alone.

**4. Copy where the evidence is.** The `file:line` chip on a finding copies itself; a reconstructed
file row copies its path (the caret flashes `✓`); the row tooltip says *Click to copy*. Diff
hunk viewing stays external, per the decision in §12.

Verified: harness **160 checks**, including patch-stat counting (additions and removals, not a
proportional bar), the diff-stat fallback, reconstruction of a count-only record, a reported file
keeping its reason while gaining its stat, and the live run returning eleven files with non-empty
stats. Payload for #144 is 72.4 kB.

Known limitation, unchanged: the panel cannot see a run that a *different* process is writing; the
60-second client cache and the explicit Refresh are the whole story.

---

## 15. R4 + M4 delivered

**R4 — the panel acts.** The header now has **Re-verify**, **Implement** (enabled only for a
`planned` run), **Cancel run** and **Copy summary**. They post to `POST /issue-flow/api/action`.

Two things had to be discovered the hard way:

1. **A resource route answers GET/HEAD only.** `webServer.readResource` documents the 405 *before
   matching*, and over HTTP a resource route never sees a POST — the first attempt answered 405 to
   its own panel. The action route is therefore a **`handler` route at an exact path**, which reads
   the Node body, wraps it in a `fetch` Request and reuses `handleAction` unchanged.
2. **A stage cannot own its own status string.** After `deliver`, `implement` regressed to
   `pending` because the status was matched by stage name. Status is now derived from a
   `STAGE_ORDER` list with `atLeast(...)`, so a later stage can never un-do an earlier one.

The route is guarded three ways — same origin (falling back to the request's own origin when
`getOrigin()` is unavailable), `content-type: application/json`, and an `x-issue-flow: action`
header (the last two force a CORS preflight a hostile page cannot satisfy). It also needs a
**live session**: the view already holds `sessionId`, and `ctx.agents.get(sessionId)` supplies the
`Agent` that a stage's `startActivation` requires — without it the route answers 409 and changes
nothing. Stages run for minutes, so the route returns **202 started** and the ledger is the
progress bar; failures append an `error` event to the record.

**M4 — delivery.** `issue_flow deliver` requires a `verified` run whose decision is not `fail`, then:

- builds the PR body from evidence — `## Summary` (plan + implementation), `## Changes` (each file
  with its real `+N −M` and reason), `## Validation` (a gate table with **commands, status,
  duration and the advisory marker**, a reviewer table, the decision line, and the findings carried
  forward), plus `Closes #N` only when publishing;
- writes `<issue>.pr.md` and `<issue>.commit.txt` into the ledger;
- **commits** in the worktree with the repository's subject convention (`Area: summary`);
- pushes and opens a **draft** PR only when `publish: true` **and** the autonomy ceiling is `pr` —
  otherwise it emits a `warn` event saying why not, and prints the two exact commands.

It never merges, never closes an issue, never force-pushes.

**First real delivery** — #144, ceiling `branch`:

- commit `cbef43e UI: Add model-wide physics display controls` on
  `issue-144-add-model-wide-physics-display-controls`, worktree clean afterwards
- `144.pr.md` 5 047 bytes: three sections, the four gate rows with real commands and durations,
  five reviewer rows, `Decision: **warn** · 0 blocker(s) · 4 warning(s)`
- ledger stage `ready-to-deliver`, events `committed …`, `not published: the autonomy ceiling is
  "branch", below "pr"`, `ready to deliver; nothing left this machine`
- the run graph now reads `intake done · plan done · implement done · verify done · deliver active`

Live guard check: POST with the header and a dead session → **409**; without the header → **403**;
both GET routes → **200**.

Harness: **173 checks**, covering the PR body builder (title convention, three sections, gate
commands, `Closes` only when publishing), and every action guard (415, 403 twice, 400 twice, 409).

Not verified: the panel buttons in a browser and the push path — publishing needs a `pr` ceiling
and a live session, which this session deliberately does not have. `comment`, `label`, `close` and
`reopen` remain refused; issue writes are the one part of M4 still unimplemented.
