# GitHub Issue Workflow — DSH plugin plan

> **Status:** M0 implemented and installed (read-only intake), now at M0.2: the UI is a
> **Tasks** tab beside Chat and Trajectory in every session (`conversation.view`), and it
> reads `sstarosz/MayaMMD-Dev` (development tasks) by default with `sstarosz/MayaMMD` (user
> reports) beside it. It went Settings page → composer-dock strip → view; see Appendix D for
> why each move happened. The plugin lives at
> `C:\Users\Sebastian\Desktop\AiProjects\DeepseekHarness\plugins\github-issue-flow` and is
> active in the `desktop` profile. M1–M4 are still design.
> This is a workspace planning document, not product documentation.

Everything below was grounded against the running installation (`dsh` `0.2.2-rc.3`,
Desktop profile `desktop`) and against this checkout at HEAD `6201d8a` on
`feature/rigid-body-performance`. Facts marked **UNVERIFIED** were not proven and are
called out where they matter.

---

## 1. Problem and outcome

The work loop for this repository today is manual: read an issue, decide what it means for
`mmd/core`, `mmd/maya`, `mmd/ui` and the native nodes, write the code, remember which of
the ten pre-PR commands apply, remember that `CHANGELOG.md` needs an `[Unreleased]` entry,
and then get a second opinion. The constraints that decide whether a change is acceptable
are documented but scattered across `docs/CPPDevelopment.md`,
`docs/JointVisualization.md`, `docs/NamingConventions.md`, `docs/CustomAttributes.md` and
`docs/MorphImplementation.md` — and two of the sharpest rules (no per-frame `MPlug` reads,
no re-enabled VP2 `requires*` flags) are prose in a design document with no automated check.

**Outcome we want:** pick an issue in the Harness UI, press *Run*, and get back

1. a plan that names the affected layers, files and risks;
2. a branch in an isolated worktree containing the change;
3. findings from several specialist reviewers, each with `file:line` evidence;
4. the real gate commands executed, with pass/fail/skip and log files;
5. a draft pull request whose *Validation* section is generated from evidence, not claims.

A human stays the approver for every action that leaves the machine.

---

## 2. What DSH already gives us

Do not rebuild these. All were confirmed live or in the installed sources.

| Capability       | Where it is                                                                                                            | What we reuse                                                 |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------- |
| Tool registry    | `ctx.tools.register` (`inject: ['tools']`)                                                                             | our two model-facing tools                                    |
| Human commands   | `ctx.commands.register`                                                                                                | `/issue-start`, `/issue-status`                               |
| Host HTTP route  | `ctx.webServer.register({ kind: 'prefix', … })`                                                                        | the panel's read API                                          |
| Client slots     | `ctx.slots.inject('conversation.view', …)`                                                                             | the Tasks tab                                                 |
| Child agents     | `ctx.subagents.startActivation({ provider: 'spawn', delivery: 'caller', outputSchema, … })` → `{ childId, result }`    | every verifier; `result.structured` carries the verdict       |
| Workflow scripts | `ctx.workflowEngine.start({ script, meta, args, parent, signal })`                                                     | optional ad-hoc fan-out (needs the PTC engine mounted)        |
| Git worktrees    | `ctx.worktrees.create(agent, …)` (live service), else `git worktree add`                                               | isolation per issue                                           |
| Skills           | `ctx.skills.register` / `registerProvider`, and filesystem roots `.dsh/skills` (rank 100), `.agents/skills` (rank 200) | the repository knowledge pack                                 |
| MCP              | patch row `@deepseek-ai/dsh-mcp-client`, tools named `mcp__<server>__<tool>`; `ctx.mcpManager.list/probe/restart`      | optional GitHub MCP server                                    |
| Approval         | `ctx.approval.request`                                                                                                 | push / PR / comment gates                                     |
| Background jobs  | `ctx.jobs.start`                                                                                                       | long builds (`cmake`, `ctest`) continue while the panel polls |

**GitHub state today:** there is no core GitHub package. The only first-party GitHub code is
experimental and **change-request only** — `@deepseek-ai/dsh-experimental-forge`,
`-forge-github` (reads PRs through the signed-in `gh` CLI), `-forge-session`, and
`@deepseek-ai/dsh-experimental-client-ui-forge`. None of those rows is mounted in this
profile, and none of them lists, triages, or implements **issues**. That is the gap this
plugin fills. It should deliberately *not* duplicate forge: when the plugin reaches the PR
stage it may defer to `ctx.forge` for read-back (checks, review state) if that provider is
ever enabled.

**Local toolchain (verified on this machine):** `gh` 2.x signed in as `sstarosz` with scopes
`repo gist read:org workflow`; `git`, `cmake`, `ctest`, `node`, `pnpm`; `mayapy` for Maya
2024, 2025, 2026 and 2027; `VCPKG_ROOT=C:\Users\Sebastian\Documents\vcpkg`; repo `.venv`
carrying `ruff` 0.16.1, `pytest`, `mypy`; all four Maya SDKs cached under `out/.sdk/`.

---

## 3. Architecture

Three separable pieces. The knowledge pack is useful even if the plugin never ships.

```
┌─────────────────────────────────────────────────────────────────────────┐
│ 3  Knowledge pack — versioned with the repository                       │
│    .agents/skills/mayammd-{architecture,physics-invariants,maya-matrix, │
│    conventions,testing,changelog-release}/SKILL.md                      │
│    (optionally AGENTS.md at the root, read by dsh-agent-instructions)   │
└───────────────────────────────┬─────────────────────────────────────────┘
                                │ loaded by child agents via the `skill` tool
┌───────────────────────────────▼─────────────────────────────────────────┐
│ 1  Host half — @local/github-issue-flow/index.js                        │
│    config ──► gh adapter ──► run ledger (.dsh/issue-flow/runs/*.json)   │
│           ──► pipeline  ──► subagents (plan, implement, verify)         │
│           ──► gates     ──► pwsh/cmake/pytest/ctest via child agents    │
│    tools: gh_issues, issue_flow    commands: /issue-start, /issue-status│
│    route: /issue-flow/api/*                                             │
└───────────────────────────────┬─────────────────────────────────────────┘
                                │ fetch('/issue-flow/api/…')
┌───────────────────────────────▼─────────────────────────────────────────┐
│ 2  Client half — @local/github-issue-flow/client.js                     │
│    conversation.view "Tasks" tab beside Chat and Trajectory: repo,      │
│    state and text filters, label grouping, /issue-start command per row  │
└─────────────────────────────────────────────────────────────────────────┘
```

**Why a Conversation View, not Settings, a dock row or a sidebar panel:** the issue list is
browsed while working on a session, so it belongs in that column — and it needs the whole
column, not a strip under the composer. Settings was the M0 choice only because it was the
one slot verified live at the time; a composer-dock row (M0.1) fixed the location but not
the space. A `sidebar.panellist` entry looks like the obvious home, but that slot registers
**only an icon** whose id must match a `main`-slot page, and building that page needs
`createNavigationStore` from an internal client package — which a dependency-free plugin
must not `require`. `conversation.view` is a plain list slot: one registrar, one component,
no internal packages, its `label` is literally the tab text, and the shell renders one view
at a time — so no `gh` call happens until the tab is opened.

**Why a Host plugin and not just a skill + prompts:** the verification fan-out has to be
deterministic. The model must not be the thing that decides whether the architecture
reviewer ran, or merges three verdicts into a gate. `ctx.subagents.startActivation` with an
`outputSchema` gives typed verdicts, and the merge stays in plain JavaScript.

**Why the house pattern (webServer route + plain `fetch`) instead of `@Remote`:** `@Remote`
methods need generated Typert schemas. The existing `@local/autonomous-ships-kb` plugin
proves the lighter route works end to end, and it keeps `client.js` a hand-written file with
no build step. Follow that.

---

## 4. Plugin anatomy

```
DeepseekHarness/plugins/github-issue-flow/          (outside this workspace — see §14)
├── package.json           type: module, dsh.bundle.patch, dsh.client{platform:web,immediately:true}
├── cordis.patch.yml       one insert row + optional mcp-client row
├── index.js               Host half: config, tools, commands, route, pipeline
├── client.js              Client half: the Tasks view (conversation.view)
├── locale/en.json         meta.title/description/detail + panel strings
├── locale/pl.json         (the existing plugins are localized pl+en — match that)
├── docs/DETAIL.md         plugin-manager detail page
├── docs/DETAIL.pl.md
├── docs/preview.svg
├── icon.svg
└── skills/issue-flow.md   runtime skill: how to drive the pipeline
```

`package.json` mirrors `@local/autonomous-ships-kb` exactly (same `dsh.bundle`,
`dsh.client`, `exports`, `files` shape) — the only difference is the entry id and package
name.

### 4.1 Tools

Two tools, both single-action enums with bounded output, per the tool-design rules
(purpose first, constraints on the parameter, no repeated facts, no log dumps).

**`gh_issues`** — read and lightly mutate issues.

```js
{
  name: 'gh_issues',
  description: 'Read and update GitHub issues for the repositories configured for this '
    + 'plugin. Writes (comment, label, close) are refused unless autonomy allows them.',
  parameters: {
    type: 'object', additionalProperties: false, required: ['action'],
    properties: {
      action: { type: 'string', enum: ['list', 'show', 'comment', 'label', 'close', 'reopen'] },
      repo:   { type: 'string', description: 'owner/name; defaults to the only configured repository.' },
      number: { type: 'integer', description: 'Required for show and every write.' },
      state:  { type: 'string', enum: ['open', 'closed', 'all'], description: 'list only; default open.' },
      labels: { type: 'array', items: { type: 'string' }, description: 'list only; all must match.' },
      limit:  { type: 'integer', description: 'list only; 1-50, default 20.' },
      body:   { type: 'string', description: 'comment only.' },
      apply:  { type: 'array', items: { type: 'string' }, description: 'label only.' },
    },
  },
  output: {
    schema: { type: 'object', additionalProperties: false,
              properties: { ok: { type: 'boolean' }, text: { type: 'string' } } },
    render: (_args, value) => [{ type: 'text', text: String(value?.text ?? '') }],
  },
  async execute(args) { … },
}
```

`show` returns the normalized `IssueBrief` plus the template-completeness check from
Appendix B, truncates the body at ~8 KB, and states the truncation. `list` returns only
`number`, `title`, labels, assignees, `updatedAt`, `url` — never bodies.

**`issue_flow`** — run the pipeline.

```
action: start | plan | status | verify | deliver | cancel | inventory
issue:  integer          (start, plan, verify, deliver, cancel)
autonomy: 'plan' | 'branch' | 'pr'   (start only; cannot exceed the configured ceiling)
full:   boolean          (verify; enables the expensive gates)
```

`start` returns `{ runId, issue, stage, artifacts: { plan, diff, report }, next }` and
nothing else. Every stage's detail lives in the ledger and is surfaced by `status` and the
panel; the tool must not stream logs into the conversation.

### 4.2 Commands

| Command                                              | Behaviour                                                                                        |
| ---------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| `/issue-start <n> [--plan-only] [--autonomy=branch]` | same code path as `issue_flow start`; logs a `command/run` row, so the human's intent is durable |
| `/issue-status [runId]`                              | current stage, elapsed, gate verdicts, artifact paths                                            |
| `/issue-inventory`                                   | the capability report from §8, printed                                                           |

Commands and tools must call the *same* Host functions — that is the `user-actions`
guidance: one service method, two front doors.

### 4.3 Host route

Read-only, `GET` only, under `/issue-flow/api/`:

| Path                           | Returns                                                                              |
| ------------------------------ | ------------------------------------------------------------------------------------ |
| `/status`                      | configured repositories, `gh` auth state, `gh` version, worktree root, ledger health |
| `/issues?repo=&state=&labels=` | the issue list for the panel                                                         |
| `/runs?repo=`                  | run ledger summaries                                                                 |
| `/run/<runId>`                 | one run: stage timeline, per-verifier verdicts, gate decision, artifact paths        |

Mutations stay in the tools so they carry model-visible arguments and approval; the panel
posts nothing but *intent* — a "Run" button opens the composer pre-filled with
`/issue-start 42`, which keeps the human in the loop for free. (If we later want
fire-and-forget from the panel, `ctx.remote.commands.execute` from the client is the
supported path — **UNVERIFIED** for a plugin-owned command, no shipped example.)

### 4.4 Config

`export const Config` was the plan; a linked local bundle **cannot use it**. The profile's
`node_modules` holds only profile-installed bundles (no `@deepseek-ai/*`), and the plugin
directory has no `node_modules` of its own, so `import z from '@deepseek-ai/schemastery'`
does not resolve. Config is therefore validated by hand in `resolveSettings`, exactly as
`@local/autonomous-ships-kb` does, with the rule that a bad config must leave the plugin
loaded and every operation reporting the error — never fail activation. Read the live values
through the patch's `config:` block. Full reference in §11.

### 4.5 Extra tools worth keeping (cheap, each removes a manual step)

The maintainer is open to a wider tool surface. These are small, non-duplicative, and each
one replaces something a human would otherwise do by hand — order is priority:

| Tool             | Actions                                                                  | Why it earns its schema                                                                                                                  |
| ---------------- | ------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `repo_gates`     | `list` (which gates apply to a path set), `run` (one tier, one worktree) | lets a human or the model run tier A/B on the *current* worktree without starting a whole run — the common case after a hand edit        |
| `issue_worktree` | `create`, `enter`, `list`, `prune`                                       | the same lifecycle the pipeline uses, usable standalone; `prune` is required anyway for the `git worktree add` fallback                  |
| `repo_knowledge` | `skills`, `rules`, `show`                                                | prints the loaded skill catalog plus which review rules apply to a given glob — makes the knowledge pack inspectable instead of implicit |
| `pr_status`      | `checks`, `diff`, `comments`                                             | read-back for the PR a run opened, through `gh` (or `ctx.forge` if that provider is ever enabled)                                        |

Rule for adding any of these: it must not duplicate a shipped tool (`read`, `edit`, `pwsh`,
`web_*`) and must not become a second way to mutate GitHub. Writes stay in `gh_issues`.

---

## 5. The pipeline

One run = one issue = one worktree = one ledger file. Stages, actors and gates:

| #   | Stage         | Actor                                                        | Writes                | Artifact                   | Gate                                                                                                |
| --- | ------------- | ------------------------------------------------------------ | --------------------- | -------------------------- | --------------------------------------------------------------------------------------------------- |
| 0   | **Intake**    | Host code                                                    | ledger only           | `issue.json`               | refuse if closed, already running, or `Wontfix`/`Duplicate`/`Invalid`                               |
| 1   | **Inventory** | Host code                                                    | ledger                | `inventory.json`           | warn-only: missing skills, missing `gh`, missing `mayapy`, missing MCP                              |
| 2   | **Plan**      | child agent `spawn`, read-only, `outputSchema`               | ledger                | `plan.json` + `plan.md`    | stop if `scope: XL` or `needsHuman`; post the plan as a *draft* comment (approval)                  |
| 3   | **Implement** | child agent, workspace-write, `cwd` = worktree               | worktree              | `impl.json` + `diff.patch` | stop if the implementer reports `blocked` or touched paths outside the plan's layer budget          |
| 4   | **Verify**    | 3–5 child agents, read-only, parallel, `outputSchema`        | ledger + `docs/` none | `verify.json`              | deterministic synthesis → `pass` / `warn` / `fail`                                                  |
| 5   | **Repair**    | implementer agent (continued, or re-activated with findings) | worktree              | `impl.json` v2             | max `maxFixRounds` (default 2); re-run only the verifiers whose area changed                        |
| 6   | **Deliver**   | Host code + `gh`                                             | git, GitHub           | `deliver.json`             | approval required; autonomy ceiling; never `--force`, never to the default branch, never auto-merge |
| 7   | **Report**    | Host code                                                    | ledger                | `summary.md`               | —                                                                                                   |

Stage 3 detail: the implementer works from `plan.json`, and its output must repeat the
plan's `files` list with any deviation called out. A deviation is not a failure, but it
invalidates the plan's risk list, so stage 4 re-runs the constraint verifier on the real
diff and the synthesizer adds a `warn` when a deviation touches a high-risk path from §6.

Stage 4 detail: verifiers read the *diff*, not the conversation. The worktree `git diff
<defaultBranch>...HEAD` plus the issue brief and plan is the whole input. This keeps each
verifier's context small and makes the run reproducible.

Stage 6 detail: the PR body is assembled from artifacts — Summary from `plan.json`,
Changes from the diff's file list grouped by layer, Validation from `verify.json` (each gate
with its real command, exit code and log path). Nothing in the body is a claim the run did
not produce. `Closes #N` only when `autonomy: 'pr'` and the gate passed.

### 5.1 State and idempotency

Ledger: `<repoRoot>/.dsh/issue-flow/runs/<owner>__<name>/<issue>.json`, gitignored, holding

```json
{ "runId": "…", "issue": 42, "stage": "verify", "autonomy": "branch",
  "branch": "issue-42-collision-mask-inversion", "worktree": "…/issue-flow/42",
  "startedAt": "…", "updatedAt": "…",
  "agents": [{ "role": "plan", "childId": "…", "label": "plan", "stage": "done" }],
  "gate": { "decision": "fail", "blockers": 1, "warnings": 3 },
  "artifacts": { "plan": "…/plan.json", "report": "…/verify.json" } }
```

`start` refuses a second concurrent run for the same issue (`force` overrides). Runs survive
a restart because the ledger is a file, not memory; `status` is a pure read. Resuming after a
crash is a `start --resume` that re-derives the stage from the ledger and refuses if the
worktree is gone.

### 5.2 Budgets

`maxAgents` (default 12), `maxMinutes` (default 45), `maxFixRounds` (2), `maxBuildMinutes`
(15). The pipeline checks budgets before every activation and disposes live activations on
cancel. Tier-B/C gates are skipped with an explicit `skip: budget` rather than silently.

---

## 6. Verification agents

Each verifier is one `startActivation` with `delivery: 'caller'` and a JSON output schema,
so the parent gets `result.structured` and never parses prose. Read-only: they may run
commands, but the plugin runs them with `cwd` = worktree and the session's file policy.

Shared output schema (subset supported by the structured-output contract):

```json
{ "type": "object", "additionalProperties": false,
  "required": ["verdict", "findings"],
  "properties": {
    "verdict": { "enum": ["pass", "warn", "fail"] },
    "confidence": { "enum": ["low", "medium", "high"] },
    "findings": { "type": "array", "items": {
      "type": "object", "additionalProperties": false,
      "required": ["severity", "rule", "claim"],
      "properties": {
        "severity": { "enum": ["blocker", "warn", "note"] },
        "rule": { "type": "string" },
        "claim": { "type": "string" },
        "evidence": { "type": "string" },
        "suggestion": { "type": "string" } } } } } }
```

`evidence` is required for every `blocker` and must quote a `file:line` from the diff;
the synthesizer downgrades a blocker without evidence to `warn` and records why. That one
rule is what keeps the review honest.

| id            | Reads                                                                               | Blocks on                                                                                                                                                                                                                                                                                                                              | Runs when                                                                                         |
| ------------- | ----------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `arch`        | diff, `plan.json`, `mayammd-architecture` skill                                     | layering violation (`mmd/core` importing Maya or Bullet types, `mmd/maya` importing `mmd/ui`, UI imported at module scope), a new native class not registered in `MayaMMD.cpp` *and* `CMakeLists.txt`, a DG attribute renamed/retyped/renumbered without a re-import note, a Python↔C++ channel other than plugs                       | always                                                                                            |
| `constraints` | diff, `CHANGELOG.md`, `README.md`, `pyproject.toml`, `.clang-format`, `.clang-tidy` | missing `[Unreleased]` entry for a user-visible change, a committed binary/asset, `mmd/_version.py` edited, naming-rule breach (`_Constraint` suffix, DG nodes carrying the model prefix, DAG nodes not), docs left describing old behaviour                                                                                           | always                                                                                            |
| `physics`     | diff, `mayammd-physics-invariants` skill                                            | a new `MPlug`/`MDataBlock` read inside `compute()` or a display path, `requiresGeometryUpdate()`/`requiresUpdateRenderItems()` re-enabled, `MRenderItem::setMatrix()`, a body/joint inserted out of PMX order, rest-pose vs guide-transform conflation, double precision, a unit/reflection conversion moved across the layer boundary | diff touches `mmd/maya/nodes/`, `mmd/core/*simulation*`, `mmd/core/bullet_bridge*`, `rigid_body*` |
| `maya-matrix` | diff, `mayammd-maya-matrix` skill                                                   | a `MAYA_API_VERSION`-dependent call without a guard where 2024–2027 differ, a stock shader name hardcoded outside the existing 2026/Blinn switch, an isolate-select API used unguarded                                                                                                                                                 | diff touches `#include <maya/…>` files or `mmd/maya/`                                             |
| `tests`       | diff, `mayammd-testing` skill, `tests/`                                             | a changed behaviour with no test in the right suite (Catch2 for core math, pytest-with-stub for Python logic, mayapy integration for node behaviour, benchmark for perf), an existing test asserting the old behaviour left unchanged                                                                                                  | always                                                                                            |

And one **executor**, not a reviewer — `gates` — which actually runs commands and returns
exit codes and log paths. It is a child agent rather than Host code because the commands
need the session's shell, sandbox and approval policy:

| Tier          | Commands (from the repo profile)                                                                                                                                              | When                                                                       |
| ------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| A — always    | `ruff format --check mmd tests`; `ruff check mmd tests`; `mypy mmd tests`; `pytest tests/unit_tests tests/integration/core/ --ignore=scripts --ignore=tests/integration/maya` | every run                                                                  |
| B — C++       | `clang-format --dry-run --Werror <changed files>`; configure + build in the worktree; `ctest --preset default --output-on-failure -E "maya-integration"`                      | diff touches `*.cpp`/`*.h`/`*.hpp`, `CMakeLists.txt`, `cmake/`             |
| C — expensive | `ctest --preset default -L maya`; `python tests/benchmarks/run_benchmarks.py rigid-body`                                                                                      | `full: true`, or episode label `Physics`/`Performance`, and assets present |

Two repo-specific traps the executor must avoid, both discovered in the profile:

> **Note:** the existing `out/build/maya2026-release` is a Visual Studio generator build
> with no `compile_commands.json`; `clang-tidy -p` against it does nothing useful. Use a
> dedicated Ninja configure in the worktree (`out/build/issue-flow`), which also gives the
> lockstep `compile_commands.json` CI uses.
>
> **Note:** configuring with default options writes `mmd/_version.py` into the source tree
> and, on Windows, copies `MayaMMD.mod` into `%USERPROFILE%\Documents\maya\<version>\modules`
> (`BUILD_DEV_INSTALL=ON`). The run must configure with `-DBUILD_DEV_INSTALL=OFF` so a review
> never installs itself into the user's Maya.

**Synthesis (plain JS, no model):** dedupe findings by `(rule, evidence)`; any `blocker`
from `arch`, `physics` or `maya-matrix` fails the gate; a failing tier-A command fails the
gate; a failing tier-B/C command fails only if CI would fail on it (clang-tidy is a hard
gate, `ruff check` and `mypy` are advisory in CI — so they downgrade to `warn`, matching
reality rather than aspiration); a `tests` blocker fails only when the changed behaviour is
user-visible. Output `{decision, blockers, warnings, notes, byVerifier}`.

---

## 7. Knowledge layer — repository skills

Six skills in this repository under `.agents/skills/<name>/SKILL.md` (rank 200, discovered by
`@deepseek-ai/dsh-skill-filesystem`, no plugin needed). Kebab-case names, `name` and
`description` in frontmatter, body = the instructions.

| Skill                        | Contents, and why it cannot be inferred from code                                                                                                                                                                                                                                                                                                                                        |
| ---------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mayammd-architecture`       | The layering law, the DG-only Python↔C++ channel, the node/command registration checklist, and where new code belongs. Distilled from `docs/CPPDevelopment.md` and the build files.                                                                                                                                                                                                      |
| `mayammd-physics-invariants` | Per-node cache invalidation (own-input dirt from `setDependentsDirty` + `preEvaluation`, anchors as the `bodyAnchors[i]` input), the ban on per-frame cross-node `MPlug` reads, VP2 dirtying rules, PMX order, the rest/current-pose split, the `K` offset, scrub-back reset, float precision. The highest-value skill: these are prose rules with no test that would catch a violation. |
| `mayammd-maya-matrix`        | Which differences between Maya 2024–2027 are guarded, where the guards live, which stock shader name applies on which version, and what must be exercised per version.                                                                                                                                                                                                                   |
| `mayammd-conventions`        | ruff/mypy settings, clang-format/clang-tidy profile and the pinned 23.1.4, C++ naming (`mCamelCase`, `kCamelCase`, `aCamelCase`), docstring styles, file placement.                                                                                                                                                                                                                      |
| `mayammd-testing`            | Which suite to extend for which change, the exact commands, why `tests/integration/core` is empty in CI but not locally, the benchmark protocol (±40 % variance, medians, interleaved runs), and which behaviour has no automated guard (viewport display, undo of import).                                                                                                              |
| `mayammd-changelog-release`  | Keep a Changelog format, the prose style of `[Unreleased]`, the squash subject `Area: summary (#n)`, PR `## Summary / ## Changes / ## Validation`, and how `extract_release_notes.py` consumes the file.                                                                                                                                                                                 |

Two more skills belong to the plugin, registered at runtime with `ctx.skills.register` so
they travel with it: `issue-flow` (how a run works, what the gates mean, how to read the
ledger) and `issue-triage` (Appendix B: template completeness, label mapping).

> **Note:** the repository has **no** `AGENTS.md`, `CONTRIBUTING.md` or equivalent today.
> Adding a short root `AGENTS.md` (read by `dsh-agent-instructions`) that points at `docs/`
> and the six skills, and restates the reviewer rules from §6, is **approved for M1** — it
> helps every agent working here, not just this plugin.

Do **not** duplicate `docs/` into skills. Skills carry the *actionable rules* and link to the
design document for the reasoning; two copies of a rule drift.

---

## 8. Capability inventory

`issue_flow action=inventory` — and stage 1 of every run — assembles, and records in the
ledger:

| Question                                              | Source                                                                                                | Consequence when missing                                                                       |
| ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| which skills does this worktree expose?               | `ctx.skills.list({ cwd: worktree })`                                                                  | a verifier whose skill is absent runs with a `warn` and lower confidence                       |
| which tools can the child agents use?                 | `ctx.tools.definitions()` / `schemas()`                                                               | refuses to start if `read`/`edit`/`pwsh` are absent                                            |
| which MCP servers are live?                           | `ctx.mcpManager.list()`                                                                               | optional GitHub MCP checks are skipped, recorded as `skip: mcp-unavailable`                    |
| is `gh` present and authenticated, with which scopes? | `gh --version`, `gh auth status`                                                                      | the whole plugin degrades to `plan`-only                                                       |
| which gate tools exist?                               | `ruff`, `mypy`, `pytest` (venv), `clang-format`, `clang-tidy`, `cmake`, `ctest`, `mayapy` per version | tier B/C skipped with an explicit reason                                                       |
| are test assets present?                              | `assets/pmx_model_files.py` and the `*_database` folders                                              | tier C's Maja integration gates skipped — this is exactly the CI gap, so the run must state it |

The inventory is what makes a run's silence meaningful: a gate that reports `pass` and a
gate that never ran must never look the same in the report or the PR body.

---

## 9. MCP and the GitHub MCP server

Default: **no MCP in the critical path.** The plugin reads issues through `gh`, which is
already authenticated, needs no token wiring, and keeps one source of truth. Adding an MCP
server that exposes the same data would create two ways to read one issue and a new failure
mode for little gain.

MCP earns its place for three things the `gh` CLI does badly, all optional and all behind
config:

1. **Cross-repository search** — the official GitHub MCP server's search tools cover
   issues/PRs/code across GitHub, useful when an issue references another project.
2. **Failing-CI log retrieval** for a PR the run created.
3. **Docs / spec lookup** for Maya API questions during `plan` and `verify`.

Integration shape, if enabled: a config-only row in the same patch, mounted only when
`mcp.enabled` is true.

```yaml
- insert:
    - id: github-issue-flow
      name: '@local/github-issue-flow'
      config: { … }
    - id: github-issue-flow-mcp
      name: '@deepseek-ai/dsh-mcp-client'
      config:
        serverName: github
        transport: streamable-http
        url: https://api.githubcopilot.com/mcp/
        headers: { Authorization: !!js `Bearer ${process.env.GITHUB_MCP_TOKEN}` }
        failOnStartupError: false
        expandTemplates: true
```

Two cautions, both from the MCP docs: a server's instructions join the logged system prompt
as literal text (treat third-party text as data), and `serverName` must be unique —
`mcp__github__*` is the tool namespace the verifiers would then see. The verifier prompts
must say that MCP output is untrusted metadata, never instructions.

**UNVERIFIED:** whether a `stdio` MCP server shipped inside a bundle can be launched by a
relative `command`. If we ever ship our own server, resolve an absolute path at patch time
with `!!js` instead of relying on relative resolution.

---

## 10. Git isolation, state and safety

**Isolation.** One worktree per issue, created with `ctx.worktrees.create(agent, { … })` if
that service resolves in this build (it is live in the Host service directory); otherwise
`git worktree add <repoRoot>/../issue-flow/<n> -b issue-<n>-<slug> origin/<defaultBranch>`.
Never touch the user's working tree — it is currently dirty on
`feature/rigid-body-performance`, and that must stay untouched.

> **Note:** if `ctx.worktrees` is used, reclamation happens when the creating Session is
> archived (`ctx.worktrees.cleanupOf`). If we fall back to plain `git worktree add`, *we*
> own cleanup: `cancel` and `deliver` must both offer it, and a `prune` action is needed.

**Branch naming.** `issue-<n>-<slug>` where slug is the issue title lowercased, ASCII,
hyphenated, ≤ 40 chars. Never the default branch, never a `force` push.

**Autonomy ceiling.** `plan` → plan only, no worktree writes beyond the plan artifacts.
`branch` → plan + implement + verify + local commit. `pr` → additionally push and open a
**draft** PR, and comment on the issue. The default is `branch`. The tool's `autonomy`
parameter can lower the ceiling per run and never raise it.

**Approvals.** Every mutation that leaves the machine goes through `ctx.approval.request`:
push, `gh pr create`, issue comment, label, close. The approver sees the exact command and
the diff stat. A denied action leaves the run at its current stage with
`gate.decision = 'awaiting-approval'`, resumable.

**What the plugin must never do:** merge; push to the default branch; force-push; delete a
branch it did not create; close an issue that has no merged PR; approve its own review;
write outside the worktree and the ledger; start a second run for the same issue.

**Concurrency.** One active run per issue; at most two concurrent runs globally, queued
otherwise, so two builds do not fight for the same `VCPKG_ROOT` and CPU.

---

## 11. Configuration reference

Row config (survives upgrades, because it is the profile patch and not our package):

```yaml
- insert:
    - id: github-issue-flow
      name: '@local/github-issue-flow'
      config:
        ghPath: gh
        autonomy: branch                # plan | branch | pr — the ceiling
        repositories:                   # the FIRST entry is the Tasks bar default
          - slug: sstarosz/MayaMMD-Dev  # private development tasks
            root: C:/Users/Sebastian/Desktop/MayaMMD
            defaultBranch: main
            forms: development
          - slug: sstarosz/MayaMMD      # user-facing issue reports
            root: C:/Users/Sebastian/Desktop/MayaMMD
            defaultBranch: main
            forms: mayammd
        budget:
          maxAgents: 12
          maxMinutes: 45
          maxFixRounds: 2
          maxBuildMinutes: 15
        gates:
          tierA: true                   # ruff/format/pytest — cheap
          tierB: false                  # C++ build + clang-tidy — minutes
          tierC: false                  # mayapy integration + benchmarks
        verifiers:
          arch: true
          constraints: true
          physics: auto                 # auto = only when the diff touches physics
          mayaMatrix: auto
          tests: true
        build:
          generator: Ninja
          configureArgs: ['-DBUILD_TESTS=ON', '-DBUILD_DEV_INSTALL=OFF']
          buildDir: out/build/issue-flow
        knowledge:
          skills: [mayammd-architecture, mayammd-physics-invariants,
                   mayammd-maya-matrix, mayammd-conventions,
                   mayammd-testing, mayammd-changelog-release]
        mcp:
          enabled: false
          serverName: github
        write:
          ledger: .dsh/issue-flow
          worktreeRoot: <sibling of repo>
```

Config is validated at activation; a missing `repositories` entry must leave the plugin
loaded but every operation reporting the configuration error — never fail activation, or the
panel cannot show what is wrong.

---

## 12. Milestones

Effort is for one focused agent-day; each milestone ends with something demonstrable.

| M      | Deliverable                                                                                                                                            | Acceptance                                                                                                                                                                                                                           | Effort  |
| ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------- |
| **M0** | Bundle skeleton, `gh_issues` (list/show), `/issue-start` intake, a **Tasks** tab in the conversation listing the configured repositories + `gh` status | The tab lists live tasks from `MayaMMD-Dev`; `gh_issues show 144` returns the development sections it carries; nothing writes anywhere; plugin-manager row applies cleanly                                                           | ½ d     |
| **M1** | Knowledge pack (6 skills + `AGENTS.md`), worktree creation, inventory, plan stage, `plan.json` + `plan.md`, draft comment (approval-gated)             | On a real issue the plan names correct files and layers; the run stops cleanly on `scope: XL`; killing the session mid-run leaves a `status`-readable ledger; the skills load in a `plan` child agent                                | 1 d     |
| **M2** | Implement stage, diff capture, local commit with the repo's subject convention, `/issue-status`                                                        | A one-file issue produces a committed branch in a worktree; the user's working tree is byte-identical afterwards                                                                                                                     | 1–1½ d  |
| **M3** | Five verifiers + gate executor + deterministic synthesis, tier A executed                                                                              | A seeded defect (a `MPlug` read added to `compute()`) is caught by `physics` with `file:line`; a clean docs-only change passes with `pass` and no blockers; a verifier that returns prose instead of JSON fails loudly, not silently | 2 d     |
| **M4** | Deliver stage (push + draft PR + issue comment, approval-gated), `tool.call.toolview` cards, run history in the panel, locale pl/en                    | A draft PR exists whose Validation section reproduces the run's real command/exit-code table; the panel shows the verdict per verifier                                                                                               | 1–1½ d  |
| **M5** | Hardening: multi-repo, tier C with `mayapy`, benchmark gate, MCP search, resume-after-crash, agent-team mode                                           | —                                                                                                                                                                                                                                    | ongoing |

Order is deliberate: M0 proves the plumbing, M1 proves the knowledge pack is worth loading,
M3 is the whole point, and M4 is the only stage that needs network writes — so the risky
part arrives after everything else is trusted.

---

## 13. How we will verify the plugin itself

1. `node --check` on `index.js` and `client.js`; validate the manifest and patch by reading
   them back.
2. Install with `plugin_manager install_bundle` (needs Full access or approval) and read the
   `application` result — not the logs.
3. `cordis_inspect_query` `Config.listConfigs` for the new entry, then the entry itself, to
   confirm the projected config schema.
4. New session: call `gh_issues list` and confirm the tool result, then `issue_flow
   inventory` and compare against the machine facts in §2.
5. Open a session and check the **Tasks** bar in the composer dock against the route JSON —
   the bar must not disagree with `/issue-flow/api/status`.
6. Seeded-defect run for M3, on a throwaway branch of a scratch clone, not on this repo.
7. Restore every piece of user state the test touched: no leftover worktrees, no branch in
   the user's repo, no `.mod` installed into `Documents/maya`, no scheduled task.

---

## 14. Risks and open questions

**Risks**

| Risk                                                           | Mitigation                                                                                                                                                            |
| -------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Verification theatre — verifiers that agree with everything    | the evidence rule (§6); a seeded-defect acceptance test in M3; a `confidence` field the synthesizer can downweight                                                    |
| Build side effects in the user's environment                   | worktree-local build dir, `BUILD_DEV_INSTALL=OFF`, no SDK download (SDKs are cached), cleanup in `cancel`                                                             |
| A verifier burns the budget on a huge diff                     | cap diff bytes handed to each verifier; summarise by file when over the cap, and say so                                                                               |
| Client bundle staleness in Desktop                             | `client.js` is hand-written (no build); verify in a fresh session; the panel is a projection of the Host route, so a stale panel is visibly wrong, not silently wrong |
| Label mismatch (`bug` in templates vs `Bug` in the repository) | Appendix B resolves both spellings; never key policy on a label without a fallback                                                                                    |
| Cost per run                                                   | budgets + tiered gates; `plan`-only is cheap and already useful                                                                                                       |

**Decisions taken** (settled with the maintainer before implementation)

1. **Orchestration locus — plugin code drives `ctx.subagents.startActivation`.** Each stage's
   agent is started with an `outputSchema`, its result arrives as `result.structured`, and
   the gate merge stays deterministic JavaScript. `ctx.workflowEngine` is *not* the pipeline
   engine; a workflow script may later be offered as an ad-hoc alternative for exploratory
   fan-out, never as the gate.
2. **Isolation — one git worktree per issue.** Child agents run with `cwd` = the worktree, so
   the maintainer's dirty `feature/rigid-body-performance` tree is never read-modified and a
   discarded run costs one `git worktree remove`.
3. **Autonomy default — `branch`.** Plan, implement, verify and commit locally without asking;
   push, PR creation, issue comment, label and close each go through `ctx.approval.request`.
   The ceiling is config; the per-run parameter may only lower it.
4. **Repository changes approved.** The six `.agents/skills/` bundles and a root `AGENTS.md`
   land in this repository as part of M1 — they are the knowledge layer, and they are useful
   to any agent working here, not only to this plugin. Extra tool surface is welcome where it
   removes a manual step (see §4.5).

**Remaining open question**

- **Where the plugin lives** — `C:\Users\Sebastian\Desktop\AiProjects\DeepseekHarness\plugins\github-issue-flow`
  alongside the two existing local plugins (recommended; writing there needs an approval, as
  the session sandbox covers this checkout only), or inside this checkout. Deferred to M0.

---

## Appendix A — gate commands, exactly as CI runs them

```bash
# tier A
ruff format --check mmd tests
ruff check mmd tests
mypy mmd tests
pytest tests/unit_tests tests/integration/core/ --ignore=scripts --ignore=tests/integration/maya

# tier B (worktree-local, Ninja, no dev install)
cmake --preset maya2026-release -DBUILD_TESTS=ON -DBUILD_DEV_INSTALL=OFF -G Ninja -B out/build/issue-flow
cmake --build out/build/issue-flow --config Release
ctest --preset default --output-on-failure -E "maya-integration"
clang-format --dry-run --Werror $(git ls-files '*.cpp' '*.h' '*.hpp')
clang-tidy -p out/build/issue-flow --quiet -warnings-as-errors='*,-clang-diagnostic-*' $(git ls-files '*.cpp')

# tier C (only with assets present and a built .mll)
ctest --preset default -L maya
python tests/benchmarks/run_benchmarks.py rigid-body --frames 120
```

CI reality the synthesizer must model: `ruff check` and `mypy` are `continue-on-error`
(advisory), `ruff format --check`, `pytest`, `clang-format`, `clang-tidy` and the C++ build
are hard gates, PR CI builds **Ubuntu + Maya 2026 only**, and the release matrix
(2024–2027 × 3 OSes) runs **no** C++ tests.

---

## Appendix B — issue intake and label mapping

Template completeness (from `.github/ISSUE_TEMPLATE/*.yml`):

- bug → a reproducible description, steps, expected vs actual, repro rate, **Maya version**,
  **plugin version**, **OS**; optional model file and logs. Missing Maya/plugin/OS = `warn`
  (the reviewer cannot reason about version-specific code without them).
- feature → motivation required; suggested solution optional.

Label mapping: the forms request lowercase `bug`/`enhancement`, but the repository's live
labels are capitalised (`Bug`, `Enhancement`, `Physics`, `UI`, `Morphs`, `Motion`, `Bones`,
`Import`, `Materials`, `Pose`, `Infrastructure`, `Documentation`, `Question`, `Duplicate`,
`Invalid`, `Wontfix`, `Help wanted`, `dependencies`, `github_actions`). Resolve
case-insensitively and record which spelling matched.

Skip rules: `Duplicate`, `Invalid`, `Wontfix` → refuse. `Question` → `plan`-only by default.
`Physics`/`Performance` label → suggest `full: true`.

---

## Appendix C — output conventions

PR body, assembled from artifacts:

```markdown
## Summary
<plan.json summary>

## Changes
<files grouped by layer: core / maya / ui / native / tests / docs>

## Validation
| gate    | command                   | result     | log |
| ------- | ------------------------- | ---------- | --- |
| format  | ruff format --check       | pass       | …   |
| unit    | pytest tests/unit_tests … | pass (412) | …   |
| physics | verifier:physics          | warn (2)   | …   |
```

Commit subject: `<Area>: <imperative summary>` where Area is `Physics`, `UI`, `Animation`,
`Import`, `Docs`, `Release` or `Refactor`, matching the squash-merge history. PR title
matches the commit subject; squash appends `(#N)` on merge.

CHANGELOG entry: one bullet under `## [Unreleased]`, bold lead-in, backticked
node/attribute names, and — for anything touching persisted scene data — the explicit
"re-import required" note the existing entries use.

---

## Appendix D — M0 delivery record

Installed at `C:\Users\Sebastian\Desktop\AiProjects\DeepseekHarness\plugins\github-issue-flow`
(`@local/github-issue-flow` 0.1.0, linked into the `desktop` profile, `application: applied`).
Eleven files: `index.js`, `client.js`, `package.json`, `cordis.patch.yml`, `icon.svg`,
`locale/{en,pl}.json`, `docs/{DETAIL.md,DETAIL.pl.md,preview.svg}`, `README.md`.

### Verified

| Claim                                     | How it was checked                                                                                                                                                                                                                                              |
| ----------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| the bundle installs and activates         | `plugin_manager install_bundle` → `application: applied`, `warnings: []`                                                                                                                                                                                        |
| tools are callable by the agent           | `gh_issues list` and `issue_flow inventory` invoked as real tools in a live session                                                                                                                                                                             |
| GitHub reads work                         | `gh_issues list repo=cli/cli` returned real issues; `show 15113` returned title, labels, body and two comments with the tail-truncation notice                                                                                                                  |
| the pipeline refuses writes               | `comment`, `label`, `close`, `reopen`, `start`, `plan`, `verify`, `deliver`, `cancel` each return a message naming the milestone; nothing reached GitHub                                                                                                        |
| intake rules behave                       | 10 synthetic cases: full bug report passes, missing Maya version is required, missing OS is only recommended, `##` headings count, feature form passes, unlabelled issue is *unknown* not *incomplete*, `forms: none` is *unchecked*                            |
| the Tasks tab is registered in the UI     | client `Slots.listSubTree` on `conversation.view` lists occupants `chat` (0), `trajectory` (10), `github-tasks` (20), all active; `conversation.composer.dock` holds only the shipped `activity` and `usage`, and `settings.section` no longer lists the plugin |
| the panel's data source answers           | `GET /issue-flow/api/status` over HTTP → 200 with the configured repository and `writeActions: false`                                                                                                                                                           |
| display assets are well formed            | both locale JSON files parse and carry title/description/detail; both detail documents are exactly one image line; both SVGs decode; every `exports` path resolves                                                                                              |
| the deployed artifact, not a staging copy | a 56-check harness imports the installed `index.js` by file URL and passes 56/56                                                                                                                                                                                |

Harness: `C:\Users\Sebastian\Desktop\MayaMMD\scratch\issue-flow-harness.mjs`
(`node issue-flow-harness.mjs`). It must run outside the session sandbox — captured child
stdio is blocked there with `EPERM`, which is a harness problem, not a plugin one.

### Not verified

- The Tasks tab rendered on screen, in light and dark themes. The slot is live, its data
  route answers, and the view is plain flex inside the shell's own view container; the
  pixels were not observed (no browser control in this session).
- The Plugin Manager row, icon tile and detail page, in English and Polish. The files are
  present, parse, and are declared; the rendering was not observed.
- Anything against a user-facing MayaMMD issue: that repository still has **zero issues**
  (`gh issue list --state all` → `[]`), which is the intended state — it only receives
  reports. Intake is exercised against real tasks in `MayaMMD-Dev` and, for the unlisted
  path, against `cli/cli`.

### Corrections this implementation forced

1. **No `export const Config`** — `@deepseek-ai/schemastery` does not resolve from a linked
   local bundle (see §4.4). Hand-validated config, house style.
2. **Registry scope must be passed, not reached through `agent.ctx`** — reading
   `agent.ctx.tools` from a plugin throws `cannot get property "tools" without inject`. The
   injected registry takes the agent as its `scope` argument instead: `tools.get(name, agent)`
   and `skills.list({ cwd, scope: agent })`. The first live inventory reported *every* preset
   tool missing because it read the global layer; the inventory now names the scope it used
   so the same false negative cannot hide again.
3. **Intake forms are per repository** — applying MayaMMD's bug/feature forms to an unlisted
   repository produced a bogus "missing required" list. A repository now declares
   `forms: mayammd | none` (default `mayammd`); an unlisted repository is reported as
   *not checked* rather than measured against someone else's template.
4. **Module generations are cached** — editing files in the linked plugin directory does not
   hot-reload, and `install_bundle` answers `already-installed`. The working loop is
   `remove_bundle` → `install_bundle`; verify the new behaviour afterwards, because
   `application: applied` alone does not prove the new module is the one running.

### M0.1 — the Tasks bar and the right repository

The first M0 shipped the list into a **Settings** page and pointed at `sstarosz/MayaMMD`.
Both were wrong, and fixing them changed the shape of the client half:

1. **Settings → the conversation dock.** The list is consulted *while* working on a
   session, so it now lives in `conversation.composer.dock`: one row with the repository,
   the loaded count and the three most common labels, expanding in place to the filters and
   the list. The bar caches for a minute in a module-level map that survives remounts, so
   switching sessions does not re-shell out to `gh`. The registration is verified live —
   `conversation.composer.dock` lists `github-issue-flow` at order 10 beside the shipped
   `activity` and `usage` entries — and `settings.section` no longer lists it.
   `sidebar.panellist` was evaluated and rejected: it registers only an icon bound to a
   `main`-slot page, and that page needs an internal client package.
2. **Two repositories, split by purpose.** `sstarosz/MayaMMD-Dev` is the private
   development-task repository and is now the default view; `sstarosz/MayaMMD` keeps its
   user-facing bug and feature forms and is expected to stay near-empty. One checkout backs
   both, and the ledger is keyed per slug.
3. **Intake is per-repository, again.** A development task is not a form submission: it is
   a hand-written `**Problem:**` / `**Scope:**` / `**Acceptance criteria**` body, so
   `forms: development` checks those three sections instead of form fields, and
   `forms: mayammd` keeps the bug/feature checks. Verified against the real repository:
   `gh_issues show 144` returns *"development task · Problem, Scope and Acceptance criteria
   all present"*, and `GET /issue-flow/api/issues?repo=sstarosz/MayaMMD-Dev` returns
   `#145 [UI]`, `#144 [UI]`, `#143 [UI]` over HTTP 200.
4. **The command now names its repository.** With two repositories configured,
   `/issue-start <n>` must not guess: it accepts `--repo owner/name`, refuses an unnamed
   target, and the Tasks bar copies the full form (`/issue-start 144 --repo
   sstarosz/MayaMMD-Dev`).

Harness coverage grew to **66 checks**, including the multi-repository configuration, the
development-task sections in both `**bold**` and `###` heading styles, and a live read of
the private repository.

### M0.2 — from the dock to a view

The composer-dock row moved the list out of Settings but still put it under the chat, in a
strip. The list is browsed, not glanced at, so it is now a **Conversation View**: a
**Tasks** tab beside Chat and Trajectory in every session. Registration is one
`conversation.view` entry — `{ id: 'github-tasks', order: 20, label: () => 'Tasks' }` — and
the label is the tab text. Verified live: `conversation.view` occupants are `chat` (0),
`trajectory` (10), `github-tasks` (20), and `conversation.composer.dock` is back to the
shipped `activity` and `usage` only.

The view gets the full column, so the list gained what a strip could not afford: **grouping
by first label** with sticky section headers and per-group counts, sticky-ish group rows,
the exact `/issue-start <n> --repo owner/name` command rendered on every row with a copy
button, a `n of m shown` badge, and its own scroller (`flex: 1; min-height: 0; overflow-y:
auto`) so the tab scrolls independently of the chat. Because the shell renders one view at
a time, `gh` is only invoked when the tab is opened — the 60-second module cache still
covers back-and-forth switching.

### M0.3 — collapsible groups, GitHub colours, the detail pane

Three UI requests, and one new Host route to support the third:

1. **Collapsible label groups with indented tasks.** Each group header is now a button
   (`aria-expanded`) with a caret, the label's colour swatch, the name and the count; the
   tasks sit in a `.gif-groupbody` with `margin-left` + a left rail, so the hierarchy is
   visible. Collapse state is a `Set` of labels, reset when the repository changes.
2. **Real GitHub label colours.** `normalizeIssue` now carries `labelColors` — a
   `name → 'rrggbb'` map taken from the label objects `gh` already returns — so the view can
   paint each tag with the repository's own colour and pick black or white text by YIQ, the
   way GitHub does. Verified live: `UI` → `efaf07` in both the list and the detail payloads.
   Text formatting ignores the map, so the tool output and intake rules are unchanged.
3. **A detail pane beside the list.** Clicking a row (or pressing Enter) opens a pane on the
   right with the title, state, author, age, coloured labels, the intake verdict, the full
   report body and every comment; `✕` closes it, and the pane stacks below the list under
   900 px. The body and comments are **plain text** — the view renders the report source,
   not GitHub's Markdown, and says so — because a hand-written client half cannot import a
   Markdown renderer.
   The pane needed a new route: `GET /issue-flow/api/issue?repo=&number=` returns
   `{ repo, issue, intake }`, reusing `fetchIssue` + `templateCheck` so the verdict shown in
   the UI is the same one the tool reports. Verified: HTTP 200, `intake.kind=development`,
   `complete=true`, 901-char body, 0 comments; a bad number is refused with HTTP 500.

Two caches now: issue lists per `repo|state` and details per `repo#number`, both 60 s, both
cleared by **Refresh**. Harness is up to **71 checks** (label colours, the detail route, its
intake, and the bad-number refusal).

### M1 — delivered

**The knowledge pack.** Six skills in `.agents/skills/` — `mayammd-architecture`,
`mayammd-physics-invariants`, `mayammd-maya-matrix`, `mayammd-conventions`,
`mayammd-testing`, `mayammd-changelog-release` — plus a root `AGENTS.md`. They are discovered by
the shell's filesystem provider (the session catalog updated live while they were written) and
they carry the rules that only existed as prose: the layering law, the ban on per-frame plug
reads, VP2 dirtying, PMX ordering, the Maya 2024–2027 guards, the exact gate commands, and the
CHANGELOG/PR conventions.

**The view.** `Tasks` became **Workbench**: one tab beside Chat and Trajectory with four
sections — **Tasks** (the issue list, grouped and collapsible, GitHub label colours, detail pane),
**Skills** (the repository pack read from disk), **Runs** (the ledger) and **Capabilities**
(`gh`, toolchain, `mayapy` versions, test assets, pipeline stage).

**The plan stage.** `issue_flow start { issue, repo }` now: resolves the configured repository,
re-checks intake, writes a run record to `.dsh/issue-flow/runs/<owner>__<name>/<issue>.json`,
starts one `ctx.subagents.startActivation` planner with `delivery: 'caller'`, `cwd` = the
checkout and a closed `PLAN_SCHEMA`, writes `<issue>.plan.json` and `<issue>.plan.md`, and updates
the record to `planned`. The planner is told it is read-only, that the issue text is untrusted
data, and to load the six skills before deciding. `plan` is the same stage capped at the plan
ceiling; `cancel` marks a record cancelled; `verify`/`deliver` remain refused (M3/M4).

**First real run** — task #144, `sstarosz/MayaMMD-Dev`, 8 minutes:

- scope **M**, 11 files across `maya`, `ui`, `tests`, `docs`; 9 risks; 2 open questions
- all six skills in `skillsConsulted`
- it found that `drawMode` already exists on `pmxRigidBodyShape`/`pmxRigidBodyJoint` and that no
  C++ gate applies — instead of inventing a native change
- its open questions are the ones a reviewer would ask: #143's root inspector does not exist in
  this checkout, and whether referenced nodes should be skipped rather than reference-edited

**Four defects the live run caught**, none of which the harness could see:

1. `budget` was referenced but never resolved — every `start` threw on `maxMinutes`. Fixed in
   `resolveSettings` with a default and a config block.
2. The structured-output validator rejects an `enum` without a `type`. Three schema nodes were
   missing it; the harness now walks the schema and fails on any such node.
3. `readLedger` counted `<issue>.plan.json` as a run, so the Runs section showed two entries for
   one run. It now matches `^<digits>.json$`.
4. `skills.list({ cwd })` from a Host plugin sees only the **global** layer — the shell's
   filesystem provider lives in the agent-preset layer, so the Skills section first showed five
   bundled skills and none of the pack. The route now reads `.agents/skills` and `.dsh/skills`
   from the checkout directly, and reports the registry view separately.

Harness: **86 checks** against the staged copy, then re-run against the deployed one.

### M2 — delivered

`issue_flow implement { issue, repo }` is the second stage, and it validates the request before it
touches anything: no recorded run → "run issue_flow start first"; a run not in `planned` → the
actual stage; a missing `plan.json` → re-run start. Only then does it need the subagent service.

**Worktree.** `.dsh/issue-flow/worktrees/<issue>` on branch `issue-<n>-<slug>` (slug is ASCII,
hyphenated, capped at 40), based on `origin/<defaultBranch>` and falling back to `HEAD` when no
remote ref exists. A branch that already exists is refused rather than reused. The path lives
inside the ledger directory, which `.gitignore` already covers, so the main checkout's status
stays clean.

**Implementer.** One `spawn` activation, `cwd` = the worktree, `name: 'implementer'`, seeded with
the plan JSON and the rules that matter here: change the planned files or explain the deviation,
never run git, no full build, load the matching skills, add the tests the plan calls for, and set
`blocked` instead of guessing. Output is a closed `IMPL_SCHEMA`.

**Diff capture.** `git add -A` **inside the worktree only**, then `--name-status`, `--stat` and the
full patch into `<issue>.diff`; the record gains `branch`, `worktree`, `base`, `diffPath`,
`diffStat` and an `implementation` summary. Nothing is committed anywhere.

**Cancel** now also removes the worktree and deletes the branch it created.

**First real run** — #144, base `2881742`, 15 minutes:

- stage `implemented`, **11 files, 1680 insertions, 0 deviations**, 87,769-byte diff
- new `mmd/maya/physics_draw_modes.py` (354 lines) plus the panel, controls, tool widget, plugin
  registration, three test files, the integration runner, the docs H3 and a CHANGELOG bullet
- it reported the commands it ran and their results, not intentions

**Independently re-run by me inside the worktree** (the implementer's word is not evidence):

| gate | result |
|---|---|
| `ruff format --check mmd tests` | 99 files already formatted |
| `mypy` on the three changed modules | Success: no issues in 3 source files |
| `pytest tests/unit_tests -q` | **579 passed**, 9 subtests |
| the main checkout | untouched — same four modified files, no new ones from the run |

Two `pytest` errors in my run are the `tmp_path` tests
(`test_parse_pmx_wraps_parse_errors`, `test_parse_vpd_file_invalid_content`) whose temp base is not
writable in this sandbox — exactly the two the implementer deselected, and not code failures.

**Plan staleness that mattered.** The plan was written against a different branch two days earlier,
and the repo had moved to `feature/ai-tools` with uncommitted work. Before implementing I checked
every file the plan names against `origin/main`: four existed, `physics_draw_modes.py` did not (it
is the new one), and the uncommitted work was an unrelated joint-display change. That is the check
M3's first verifier should automate.

### M2.5 — the run graph

The Runs section was a list of one-line summaries; a run now opens a detail pane with a **stage
graph** (Intake → Plan → Implement → Verify → Deliver), each node coloured by status
`done | active | failed | blocked | pending | skipped`, joined by edges that turn green once the
stage behind them is done. Clicking a node filters the log to that stage; clicking it again returns
to the whole-run log.

**The record is the log.** `addEvent` appends `{ at, stage, level, message }` at every transition:
intake result, planner start, plan recorded (scope/files/risks), open questions as `warn`, skills
consulted, worktree created on which branch from which base, implementation result, **every command
the implementer ran**, diff captured with size, blocked reason, errors, and cancellation. Failures
write an `error` event before the run is recorded as failed, so a failed run has a visible reason
rather than a silent stop.

**Old records still render.** `deriveRun` synthesises the missing log from the record's own fields
(intake, plan summary, implementation counts, diff stat) when no events exist, so #144 — written
before events existed — opens with a complete four-line log instead of an empty box.

Host surface: `GET /issue-flow/api/run?repo=&issue=` returns `{ record, run }` where `run` carries
the derived stages, their events and the artifacts; an unknown issue is a 404. The client caches it
for 60 s and **Refresh** clears every cache. Harness: **120 checks**, including failed, blocked and
cancelled derivations and the no-events synthesis path.

Live check on #144: `intake done · plan done · implement done · verify pending · deliver pending`,
four log lines, `branch issue-144-add-model-wide-physics-display-controls`, `144.diff`.

### M3 — delivered

`issue_flow verify { issue, repo }` reviews the worktree and runs the gates, and records one
decision. It requires an `implemented` run and a live worktree, and it never edits anything.

**Five reviewers, read-only, in parallel.** Architecture, Constraints, Physics, Maya matrix and
Tests. Each gets its focus, the plan summary, the changed-file list, the diff (truncated at 60 kB
with a warning event when it is), and the instruction to load its two matching skills first. Output
is a closed `VERIFIER_SCHEMA`: `verdict`, `confidence`, and findings of
`severity | rule | claim | evidence | suggestion`. A nonexistent verdict is a `warn` finding, not a
silent pass.

**The gates are real.** Tier A exactly as `pr-checks.yml` runs them, through the checkout's own
`.venv` so the worktree needs no install: `ruff format --check`, `ruff check`, `mypy`, `pytest
tests/unit_tests tests/integration/core/`. Both run **at the same time** — the gates are shell
work, the reviewers are model work.

**Synthesis is deterministic JavaScript, and it has two hard rules:**

1. A `blocker` without `file:line` evidence is **downgraded to a warning** and the downgrade is
   recorded in the finding.
2. `ruff check` and `mypy` are `continue-on-error` in CI, so their failures are warnings here too;
   `ruff format` and `pytest` are hard gates. A gate that fails for an **environment** reason
   (permission, missing temp dir) is reported as `environment` and never as a blocker.

**First real run** — #144, decision **WARN**, 0 blockers, 4 warnings, 14 findings:

| gates | | | |
|---|---|---|---|
| `ruff format --check` | pass | 1045 ms | |
| `ruff check` | fail | 437 ms | advisory |
| `mypy` | fail | 7440 ms | advisory |
| `pytest` | pass | 8284 ms | |

| verifier | verdict | findings |
|---|---|---|
| Architecture | pass (high confidence) | 0 |
| Constraints | warn | 2 |
| Physics | warn | 6 |
| Maya matrix | pass (high confidence) | 0 |
| Tests | pass | 4 |

What the reviewers found, none of which the gates could see:

- the heading guard in `_sync_physics_selection` also runs for genuine Maya selection changes, so
  clicking a model heading silently stops the inspector following the viewport;
- a model-wide batch writes `drawMode` on N nodes and every write trips `setGeometryDrawDirty`,
  turning one click into a whole-model geometry rebuild — pre-existing per node, new at model scale;
- `DrawModeResult.undo_grouped` is computed and documented but never surfaced, so when Maya refuses
  the chunk the user sees "one undo step" that isn't;
- `README.md` still describes the old panel mirroring, which this change deliberately suspends;
- the panel→widget signal has no end-to-end test, so deleting the `connect` would stay green;
- the new Maya integration suite is the only guard for undo/visibility claims and CI never runs it.

Note the last one: the Tests reviewer reported `pass` **and** four findings about coverage gaps —
the design lets a reviewer say "this passes, but here is what is unguarded", which is the behaviour
that makes the review useful rather than theatrical.

**Three defects the live pass caught:**

1. `deriveRun` showed `implement: pending` next to `verify: done`, because `verified` was not
   treated as having passed implementation.
2. Per-stage log synthesis only ran when a record had *no* events at all, so once verification added
   events the older stages rendered empty logs. Synthesis is now per stage.
3. `formatIntakeSentence` crashed on a record whose `intake` was `{}` — a malformed or hand-edited
   record could take down the whole read path. It is defensive now.

Harness: **136 checks**, including evidence downgrade, advisory-vs-hard gates, environment
failures, and every verify-node status.

**Known noise, for M4:** the advisory gates run on the whole tree, so pre-existing `ruff check` and
`mypy` findings produce two warnings on every run. M4 should scope advisory gates to the changed
files (the way CI uses `clang-tidy-diff`) and record untouched-file findings as a note.

### M4 starts here

Delivery: approval-gated push, draft PR whose Validation section is built from
`144.verify.json`, the issue comment, and the label transitions — plus the repair loop that feeds
findings back to the implementer before anything leaves the machine.
