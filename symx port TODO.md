# symx port TODO

- runtime summary
    Add dots
    Move to symx

- log counters might be wrong. Summary is wrong
    hit -> inv(valid)

- Option to disable logging

- enable_output vs set_enabled vs Verbosity::Silent — document the distinction
    Three ways to suppress output:

    Individually control:  
        - VTK
        - console print and file
        - log

- misc takes huge runtime and idk what it is
    I need to know

- Verify copilot all logic is correct. I had may latent bugs such as not passoing optional variables.

- Enable friction!

- Remove bottom print from Newton

- `enable_prints`, `enable_frame_writes`

- Consistent `stark`, `Stark` or `STARK`

- Comments and docs are _very_ inconsistent in style

- debug line search logic inside line search
    Remove _solve_impl

- ToFile print should contain all information

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

* Run all the simulations from PPN to certify it runs as expected



# STARK Docs

* Clarify that stark uses a velocity based solver, therefore tolerances, etc are in m/s and time step dependent

