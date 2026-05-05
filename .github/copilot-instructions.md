# KiteModels.jl Copilot Instructions

## Project Overview

KiteModels.jl is a Julia package for simulating kite power systems (airborne wind energy). It provides physics-based models of tethered kites using mass-spring-damper systems with aerodynamic forces.

## Architecture

### Core Models (in `src/`)

- **KPS3** (`KPS3.jl`): One-point kite model - kite as single point mass
- **KPS4** (`KPS4.jl`): Four-point kite model - 4 point masses with rotational inertia
- **SymbolicAWEModel** (`symbolic_awe_model.jl`): RAM air kite using ModelingToolkit + VortexStepMethod for aerodynamics

All models inherit from `AbstractKiteModel` (defined in KiteUtils.jl). The main module `KiteModels.jl` re-exports dependencies: KitePodModels, WinchModels, AtmosphericModels.

### Key Data Structures

- `Settings`: Configuration loaded from YAML (`data/settings.yaml`, `system.yaml`)
- `SystemStructure` (`system_structure.jl`): Point-mass topology for SymbolicAWEModel
- `Spring`/`SP`: Spring-damper connections between point masses

### Simulation Flow

1. Load settings: `set = load_settings("system.yaml")`
2. Create KCU and model: `kcu = KCU(set); kps4 = KPS4(kcu)`
3. Initialize: `init!(kps4)` or `find_steady_state!(kps4)`
4. Step: `next_step!(kps4, integrator; set_speed, dt)`

## Development Workflows

### Running Tests

```julia
include("test/runtests.jl")  # Full test suite (~60 min)
```

Skip slow MTK tests: `ENV["NO_MTK"] = "1"` before testing.

### Running Examples

```julia
include("examples/menu.jl")  # Interactive menu
include("examples/reel_out_4p.jl")  # Direct execution
```

### System Image (faster startup)

```bash
cd bin && ./create_sys_image  # Creates precompiled image
./bin/run_julia               # Launch with system image
```

## Code Conventions

### Type Aliases

- `SimFloat = Float64`: All simulation floats
- `KVec3 = MVector{3, SimFloat}`: Mutable 3D vectors (stack-allocated)
- `SVec3 = SVector{3, SimFloat}`: Immutable 3D vectors

### Key Functions Pattern

- `init!(model)`: Initialize/reinitialize model state
- `next_step!(model, integrator; kwargs...)`: Advance simulation
- `find_steady_state!(model)`: Solve for equilibrium
- Getters: `pos_kite()`, `winch_force()`, `lift_drag()`, `calc_elevation()`

### Configuration

Settings are in `data/` as YAML files. Key sections:

- `system`: segments, sample_freq, sim_time
- `solver`: abs_tol, rel_tol, solver type (IDA/DFBDF)
- `kite`/`kps4`: physical parameters (mass, area, aerodynamic coefficients)
- `tether`: d_tether, e_tether, damping

### Coordinate System

- NED (North-East-Down) reference frame for orientation
- Azimuth calculated in wind reference frame
- Elevation from ground to kite

## External Dependencies

- **OrdinaryDiffEqBDF/Sundials**: ODE solvers
- **KiteUtils**: Settings, state types, utilities
- **ControlPlots**: Plotting used directly in example scripts

## Testing Notes

- Benchmark tests (`bench3.jl`, `bench4.jl`) verify performance regressions
- Use `BUILD_IS_PRODUCTION_BUILD=false` to skip benchmarks in CI
- Test files in `test/` mirror source structure

## Documentation
To test if building the docs works locally, use:

```julia
using Pkg; Pkg.activate("docs"); include("docs/make.jl"); Pkg.activate(".")
```

## System Image
A precompiled system image can be built with:

```bash
bin/create_sys_image
```

Launch Julia with it via `bin/run_julia`.

## Tool routing

Prefer MCP tools over ad-hoc shell commands whenever a matching tool exists.
These rules are not suggestions — follow them unless the user explicitly asks
for a different approach.

## Julia code outside notebooks: use `mcp__kaimon__*`

The `mcp__kaimon__*` tools are the primary interface for working with
non-notebook Julia code. They route through Julia's actual module
system and cache state across calls in a persistent REPL worker, so
they beat `Bash`, `Grep`, `Glob`, and `Read` for nearly every Julia
task. Do NOT reimplement their lookups via `ex(e="methods(...)")`,
`ex(e="fieldnames(...)")`, etc. — the dedicated tools format output,
handle edge cases, and resolve through the module system.

### Routing rules — symbol & definition discovery

- **Methods of a function** (signatures + source locations) →
  `search_methods(query="funcname")`. Do NOT `grep` for
  `function funcname` — you'll miss overloads and get textual matches
  for comments and docstrings.
- **Type fields, supertype, subtypes** →
  `type_info(type_expr="MyType")`. Beats
  `ex(e="fieldnames(MyType)", q=false)` because it returns the full
  picture (fields, hierarchy, parameters, properties) in one call.
- **All exported (or internal) names in a module** →
  `list_names(module_name="seismic")`. Pass `all=true` for
  non-exported internals.
- **Fuzzy symbol search across loaded modules** →
  `workspace_symbols(query="partial_name")`. Uses `names()` on the
  gate; only finds symbols in modules that are actually loaded.
- **Symbols defined in a specific file** (functions, structs, macros,
  constants with line numbers) →
  `document_symbols(file_path="/abs/path.jl")`. AST-based, does NOT
  require the module to be loaded — use it for standalone scripts,
  test helpers, and files whose owning package isn't in the active
  environment.
- **Definition of a symbol used at a specific file:line:column** →
  `goto_definition(file_path=..., line=..., column=...)`. Uses
  `methods`/`functionloc`/`pathof` on the gate with a file-grep
  fallback.

### Routing rules — executing code and running tests

- **Arbitrary Julia code** → `ex(e="...")`. Shared REPL; see "The
  shared REPL contract" below. Add `q=false` when you need the
  return value for a decision.
- **Running the test suite** →
  `run_tests(pattern="...", session=...)`. Do NOT shell out to
  `julia --project -e 'Pkg.test()'` or invoke `runtests.jl` via
  `Bash`. See "run_tests details" below.
- **Macro expansion** → `macro_expand(expression="@time ...")`.
- **Type-inferred or lowered IR** →
  `ex(e="code_typed(f, (T,))", q=false)` routed through `ex` — the
  dedicated `code_typed` / `code_lowered` tools return empty
  payloads (see "kaimon gotchas" below).
- **Profiling a hot block** →
  `ex(e="using Profile; Profile.clear(); @profile ...; Profile.print()", q=false)`
  routed through `ex` — the dedicated `profile_code` tool returns an
  empty payload.

### Routing rules — environment and packages

- **Julia version, active project, loaded packages, Revise status** →
  `investigate_environment(session=...)`. Do NOT parse
  `Project.toml`/`Manifest.toml` by hand or call
  `julia -e 'using Pkg; ...'` from Bash.
- **Adding or removing packages** → `pkg_add(packages=["Foo"])` /
  `pkg_rm(packages=["Foo"])`. Do NOT run `Pkg.add` via `ex`.
- **Changing the active project** → do NOT. The project is
  controlled by the kaimon session, not by you. `Pkg.activate(...)`
  in an `ex` call will silently corrupt the session's package
  environment for subsequent calls.
- **Formatting a file** → `format_code(path=...)`. Requires
  JuliaFormatter.jl in the project; if the tool errors "not
  installed", ask the user before `pkg_add`ing it.
- **Aqua QA checks** → `lint_package`. Requires Aqua.jl; same rule.

### Routing rules — observability and health

- **Verify kaimon is reachable, list connected sessions** → `ping`
  (use `extended=true` for health stats). Only ping when a call has
  failed — don't probe proactively.
- **Audit server-side errors** →
  `server_log(level="error", lines=30)`. First stop for "why did my
  last call fail?".
- **See the last N tool calls with durations and session routing** →
  `tui_screenshot`. Fastest way to see what just happened on the
  kaimon side.
- **Force a fresh REPL state** →
  `manage_repl(command="restart", session=...)`. Only when Revise
  can't pick up a change (`__init__` changed, world-age errors that
  persist after a fix). Not a reflex for every error.
- **Detailed help for any kaimon tool** →
  `tool_help(tool_name=..., extended=true)`.

### Routing rules — VSCode integration

- **Open a file at line/column in the editor** →
  `navigate_to_file(file_path=..., line=..., column=...)`.
- **Run a VSCode command** →
  `execute_vscode_command(command="...")`.
- **Listing available VSCode commands** → read
  `.vscode/settings.json` directly; the `list_vscode_commands` tool
  throws `UndefVarError: read_vscode_settings` (see "kaimon
  gotchas").

### Routing rules — semantic code search (availability-gated)

- **"Find code that does X" when you don't know the symbol name** →
  `qdrant_search_code(query="natural language")`. Requires a running
  Qdrant (`http://localhost:6333`) and a prior
  `qdrant_index_project`. If `qdrant_list_collections` errors with
  "not reachable", semantic search is unavailable — fall back to
  `workspace_symbols` (for fuzzy names) or `Grep` (for literal
  strings), and say so in your response.

### The shared REPL contract

Every `ex` call runs in a persistent Julia worker that the user sees
live. You and the user share the same REPL.

- **stdout is stripped.** `println`, `print`, `@info`, and anything
  else that writes to stdout is removed from your tool result. To
  observe a value, make it the final expression of an `ex` call and
  pass `q=false`. Put narration in your text response, not in Julia
  print calls.
- **Default to `q=true`** (the default). Use `q=false` ONLY when you
  need the return value for a decision. Imports, assignments, and
  function definitions should always use `q=true`:

  ```julia
  ex(e="using Statistics")                  # q=true (default)
  ex(e="data = load(...); nothing")         # q=true
  ex(e="length(result) == 5", q=false)      # q=false: need the bool
  ex(e="methods(my_fn)", q=false)           # q=false: need to read them
  ```

- **`s=true`** (rare) suppresses the `agent>` prompt and REPL echo.
  Only use it for huge outputs that would spam the user's terminal.
- **Revise is active** in kaimon sessions by default. Editing a file
  under `src/` picks up automatically on the next `ex` call; no
  restart needed unless `__init__` or module-level code changed.

### Multi-session routing

When more than one session is connected, every session-scoped tool
(`ex`, `run_tests`, `investigate_environment`, `search_methods`,
`type_info`, `debug_*`, etc.) **requires** an explicit session key,
or it fails with `No session matched ''`.

- Discover session keys with `ping` — each line shows the 8-char
  key, the display name, and uptime/PID.
- The parameter name is `ses` on `ex` and `session` on every other
  tool — mind the typo trap.
- **`run_tests` usually spawns a second session** for the project's
  `test/Project.toml` environment. After the first `run_tests` call,
  subsequent session-scoped calls will start failing with the "no
  session matched" error until you disambiguate. Expect this and
  pass the key explicitly from that point on.
- To inspect what's in a specific session, call
  `investigate_environment(session=KEY)` — it reports `pwd`, active
  project, dev packages, and Revise status for just that worker.

### Background jobs and cancellation

An `ex` call that runs longer than 30 s is automatically promoted
to a background job. You receive an `eval_id` immediately (as the
first progress notification AND in the final result object), so
even a client-side timeout doesn't lose the reference.

- **Polling a promoted or timed-out eval** →
  `check_eval(eval_id=...)`. Returns status, elapsed, last-activity
  timestamp, stashed values, and the result if done.
  **Polling rules:** wait ≥30 s before the first `check_eval`,
  then ≥60 s between polls. Do NOT tight-loop; the job won't finish
  faster because you're checking.
- **List recent jobs** → `list_jobs(status=..., stats=true)`.
  Background jobs are persisted to SQLite and survive TUI restarts.
- **Cancelling a runaway job** → `cancel_eval(eval_id=...)`. Julia
  cannot force-interrupt tasks, so cancellation is cooperative: the
  running code must periodically check `Gate.is_cancelled()` and
  `break`. If it doesn't, `cancel_eval` has no effect.
- **Cancellation-status gotcha:** if the running code DOES honor
  `Gate.is_cancelled()` and returns normally after the break, the
  job status will show `completed`, not `cancelled`. Check the
  returned value to confirm it bailed early.
- **Intermediate progress reporting from long-running code** →
  `Gate.stash(key, value)` for values, `Gate.progress(msg)` for
  status strings. Both are visible via `check_eval`.

### Debugging with Infiltrator

Two workflows, depending on whether you need an interactive pause.

**`@exfiltrate` (no breakpoint, capture-and-continue):**

```julia
debug_exfiltrate(code="""
function my_fn(x, y)
    z = x + y
    @exfiltrate       # capture all locals at this point
    return z
end
my_fn(1, 2)
""")
debug_inspect_safehouse()                        # list captured vars
debug_inspect_safehouse(expression="typeof(z)")  # eval using captured vars
debug_clear_safehouse()                          # clean up
```

Works for any function you can redefine. The first call to
`debug_inspect_safehouse` prints a harmless
`Failed to run __is_pkg_loaded(:Makie) || using GLMakie` line —
ignore.

**`@infiltrate` (interactive breakpoint):**

```julia
ex(e="using Infiltrator")                           # separate eval!
ex(e="function_that_hits_@infiltrate(args)")        # pauses here
debug_ctrl(action="status")                         # file/line + locals
debug_eval(expression="typeof(x)")                  # any Julia expr
debug_ctrl(action="continue")                       # resume
```

`using SomePackage` MUST be a separate `ex` call from the one that
triggers the breakpoint — combining them runs into world-age
issues. Assignments in `debug_eval` persist across calls within a
breakpoint session.

### run_tests details

- **Pattern filtering requires ReTest.** On ReTest-based suites,
  `pattern="regex"` filters by testset name. For test suites on
  plain `Test.@testset`, `pattern` is silently ignored and the
  whole suite runs — no warning.
- **A non-matching pattern returns `Pass: 0, Total: 0 — PASSED`.**
  Do NOT interpret "PASSED" as success; check the total count and
  the `No matching tests` line in the output.
- `verbose=1` (default) gives per-testset pass/fail summaries; bump
  higher only when triaging a specific failure.
- **Orphaned test files** (files in `test/` not reachable from
  `test/runtests.jl`) cannot be run via `run_tests`. Use
  `ex(e="include(\"/abs/path/to/test_file.jl\")", ses=...)` —
  `run_tests` only runs what `runtests.jl` includes.
- `run_tests` usually spawns a second `test` session — see
  "Multi-session routing" above.

### Common workflows

- **"Where is `foo` defined?" (name approximately known)** →
  `workspace_symbols(query="foo")` for the fully-qualified name,
  then `search_methods(query="Module.foo")` for signatures +
  file:line. Optionally `navigate_to_file(...)` to open it in the
  editor.
- **"What does this file define?"** →
  `document_symbols(file_path="/abs/path.jl")`. Pure AST parse,
  works on unloaded files.
- **"What does this type look like?"** →
  `type_info(type_expr="MyType")` for fields/hierarchy, then
  `search_methods(query="MyType")` for constructors and overloads.
- **"Iterate on a single failing test"** →
  `run_tests(pattern="failing_testset", session=...)`, read the
  failure, edit `src/`, re-run the same command. Revise keeps
  `src/` hot; only `manage_repl(command="restart")` if `__init__`
  or module-level code changed.
- **"Inspect locals without restructuring"** → `debug_exfiltrate`
  with the function redefined to include `@exfiltrate`, then
  `debug_inspect_safehouse(expression=...)`, then
  `debug_clear_safehouse()`. Works even for deeply nested
  functions — Revise picks up the fresh definition.
- **"Long-running computation"** → `ex(e=..., q=false)`; capture
  the job ID from the 30 s promotion message; wait ≥30 s; check
  with `check_eval(eval_id=...)`. Cancel only if the running code
  checks `Gate.is_cancelled()`.
- **"A kaimon call failed — now what?"** → (1) `ping` to confirm
  the session is alive; (2) `server_log(level="error", lines=30)`
  for the root cause (often a missing dep like
  Qdrant/JuliaFormatter/Aqua, or a loading error); (3)
  `tui_screenshot` to see the last N tool calls with durations
  and preview; (4) fix the root cause, don't silently fall back
  to `Grep`/`Bash`.
- **"Debug type instability"** →
  `ex(e="code_typed(fn, (T1, T2))", q=false, ses=...)`. Look for
  `Union`, `Any`, or `@_call` in the IR. Route via `ex`, not the
  (broken) dedicated tool.
- **"Profile a hot block"** → route via `ex` + `Profile` directly,
  not the broken `profile_code` tool:

  ```julia
  ex(e="""
  using Profile
  Profile.clear()
  @profile (for _ in 1:100; hot_fn(args); end)
  Profile.print(format=:flat, maxdepth=20)
  """, q=false, ses=...)
  ```

### Failure handling

When a kaimon tool errors, **debug the error** before falling back.

- Legitimate fall-back cases (use `Grep`/`Read`/`Bash` and say so
  in your response):
  - Non-Julia files (`.md`, `Project.toml`, raw notebooks on disk).
  - Files in unloaded/unregistered scripts that `document_symbols`
    can't cover.
  - Filesystem edits (`Write`/`Edit`, not kaimon).
- NOT legitimate fall-back cases: the tool took too long; you want
  `grep` to feel faster; you don't want to deal with session
  disambiguation.
- **Missing optional dependencies** (JuliaFormatter.jl, Aqua.jl,
  Qdrant, Ollama) → tell the user which one is missing. Do NOT
  auto-`pkg_add` or start external services.
- **Session seems wedged** → `ping`; if that errors,
  `manage_repl(command="restart")` is the last-resort reset.

### kaimon gotchas worth remembering

- **stdout is stripped from `ex` results** — `println` output is
  gone by the time you see the result. Always use `q=false` with a
  final expression to observe a value.
- **`run_tests` `pattern` is silently ignored** on plain-`Test`
  suites (only works with ReTest). A non-matching pattern reports
  `Pass: 0 — PASSED`, which is a trap.
- **`code_typed` and `code_lowered` MCP tools return `Any[]`** for
  both `Base` and user functions. Route through
  `ex(e="code_typed(f, (T,))", q=false)` instead.
- **`profile_code` returns an empty payload.** Route through
  `ex(e="using Profile; @profile ...; Profile.print()")` instead.
- **`list_vscode_commands` throws
  `UndefVarError: read_vscode_settings`.** Read
  `.vscode/settings.json` directly if you need the allow-list.
- **`run_tests` spawns a second `test` session** for the project's
  test environment. After the first call, subsequent session-scoped
  calls need an explicit `ses`/`session` key or they fail with
  `No session matched ''`.
- **Parameter-name asymmetry**: it's `ses=` on `ex` and `session=`
  on every other tool.
- **Cancelled jobs may report `completed`** — if the running code
  breaks cleanly after `Gate.is_cancelled()` flips, the job returns
  normally and the status is `completed`, not `cancelled`. Check
  the returned value.
- **Background-job polling**: wait ≥30 s before first `check_eval`,
  then ≥60 s between polls. Polling faster does not make jobs
  finish faster.
- **Revise is active** in kaimon sessions by default; editing
  `src/` picks up on the next call. Restart is only needed for
  `__init__` / module-level changes.
