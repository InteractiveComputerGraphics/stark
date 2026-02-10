# symx port TODO

Here's a summary of what's done and what's left.

Completed (compiles & runs)
OutputSink — OutputSink.h: single class with verbosity gating, auto-indent, file+console routing, lazy file open
Verbosity enum — solver_utils.h:11: 4-level {Silent, Summary, Step, Full} with backward compat aliases
Context wiring — Context.h: OutputSink sink is a public field on symx::Context
Newton prints — NewtonsMethod.cpp:597: _print() now gates against sink.get_verbosity() and emits through sink.print()
Stark prints — Stark.cpp: all console.print() calls replaced with context->sink.print(). Context+sink created in constructor (before _initialize).
Per-verbosity demo — main.cpp:703: runs 4 verbosity levels, writes .log files to output/verbosity_demo/
Log files location
Output lives in verbosity_demo relative to the build dir (i.e. verbosity_demo):

sink_verbosity_silent__*.txt — 0 bytes
sink_verbosity_summary__*.txt — 41 lines (SymX + frames)
sink_verbosity_step__*.txt — 125 lines (+ settings + Newton + dt/runtime)
sink_verbosity_detail__*.txt — 125 lines (+ line search detail, same here since no LS triggered)
Remaining (not yet started)
7: Refactor Newton fragment-prints into proper line-based calls with auto-indent
8: Upgrade Log → Logger (typed series, stats, YAML output)
10: Delete Console and old Logger classes from stark


- We need to rethink the split between console output and frame output.
    These are independently needed.

- Why `symx::Verbosity` when this is for now exclusively for Newton?
    Maybe other solvers will follow?

- `Silent` is really bad for API.
    It means "print time steps"

- Do we need the scheme for `NewtonsMethod::_print_return` ?

- consistent get_name, get_label, to_string

- Remove alisases and backward compat stuff littered around by copilot

- I am unsure about the compilation print. I feel that should always be tab 0.

* Line search: Decide for apply_increments or global

* TONS of debug commented out stuff everywhere
    Specially collision

* stark should write vtk to .tmp and then rename when finished to avoid crashing 3rd party viz


* Add labels to DoFs for nicer printing

* Quasistatic should let gravity go

* Unify mesh generation. Prefer mid point insertion for quads? What about quadratic quads?

* I think the time stepping logic with adaptivity and the script is broken
    The quasistatic extrussion restarts in wrong ways in respect to the script

* Newton-Stark settings and callbacks are a bit wrangled.
    Conceptually is correct but it is brittle or undocumented that we are setting into stark and then copy into newton

* `std::string to_string` for symx options should be in symx

* symx has a cmake option to set compilation path.
    How do we handle it being in stark.settings?

    There are other legacy symx options inside stark.settings that I have to port over.

* Revamp and cleanup Stark.run_one_step()
    Take inspiration from the new Newton's Method

* Massive duplication of mesh generation and utils
    I think we ship symx with minimal tooling for ease of ramping up
    And preserve the heavy stuff in Stark. This is what stark is.

* Make sure optimization from the ppn branch are ported
    We are building from ~github version

* Check the whole symx use and logic for consistency and latent bugs
    We are comparing many pointers, and fetching and storing... Extensive check!

* There will be a lot of dead code and old/AI comments. Massive cleanup.



# STARK Docs

* Clarify that stark uses a velocity based solver, therefore tolerances, etc are in m/s and time step dependent

