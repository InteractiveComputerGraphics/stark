# symx port TODO

## Comparison to PPN ref
    - Dont compare CG iterations with Progressive because we are counting failures differently.
    - forcing sequence was a difference. But it is a tradeoff.
    - New CG seems slower on same iterations


## High Priority
- `SolverReturn` is too specific of Newton. It should be general.
    Even if it contains things unused by other solvers (e.g. linear system failure)

- Generally refer to line search more specifically
    armijo
    backtracking
    invalid state

    _everywhere_, also in `SolverReturn`


* Callbacks
    + on_armijo_backtrack_fail
    - after_evaluation

* Newton max iterations as successful

* logger->get_stats and logger->get_statisticcs   ??

* Is there a way to uniformize the Callback structs?
    For expansion between symx and stark
    but also for utils and timings

* Revamp and cleanup Stark.run_one_step()
    Take inspiration from the new Newton's Method

- enable_output individually control:
        - VTK
        - console print and file
        - log

        Verbosity::Disabled

- ToFile print should contain all information

* `std::string to_string` for symx options should be in symx

* Verify logic for adaptive stiffness and time step size upon failures
    - Invalid needs its max iterations, independent from armijo

* Add `step` to the armijo condition.

* I think the time stepping logic with adaptivity and the script is broken
    The quasistatic extrussion restarts in wrong ways in respect to the script


## Low Priority

- Consistent `stark`, `Stark` or `STARK`

- Comments and docs are _very_ inconsistent in style

- consistent get_name, get_label, to_string

* TONS of debug commented out stuff everywhere

* Logger and Console use internally in symx

* Try failing all possible ways
    TooManyIterations, LinearSystemFail...

* Try direct solve

* Revise numerics
    - line search implementation
    - forcing sequence
    - all tolerances

* Decide for forcing sequence or not
    If so, remove cg_abs_tolerance from settings

* Unify mesh generation. Prefer mid point insertion for quads? What about quadratic quads?

* Massive duplication of mesh generation and utils
    I think we ship symx with minimal tooling for ease of ramping up
    And preserve the heavy stuff in Stark. This is what stark is.

* symx has a cmake option to set compilation path.
    How do we handle it being in stark.settings?

    There are other legacy symx options inside stark.settings that I have to port over.

* Make sure all details from the ppn branch are ported
    We are building from ~github version

* Check the whole symx use and logic for consistency and latent bugs
    We are comparing many pointers, and fetching and storing... Extensive check!

* There will be a lot of dead code and old/AI comments. Massive cleanup.

* Run all the simulations from PPN to certify it runs as expected

* Revise that rigid body inertia is correct. It is formulated as velocity.



# STARK Docs

* Clarify that stark uses a velocity based solver, therefore tolerances, etc are in m/s and time step dependent

