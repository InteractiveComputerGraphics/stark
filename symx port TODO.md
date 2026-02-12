# symx port TODO

- Comparison to PPN ref
    - Dont compare CG iterations with Progressive because we are counting failures differently.
    - forcing sequence was a difference. But it is a tradeoff.
    - New CG seems slower on same iterations

- zero CG iterations print. Cant be

- Exits without notice upon some fails

- Summary: [total | avg | [min, max]] for key metrics (newton, ls, cg its)

- runtime summary move to symx with arg `double total_time`

- Settings print with scientific notation

- Refactor all related to line search in Newton's Method

- Refactor retrying for line plot

* Revamp and cleanup Stark.run_one_step()
    Take inspiration from the new Newton's Method

- debug line search logic inside line search
    Remove _solve_impl

- enable_output individually control:  
        - VTK
        - console print and file
        - log

- Consistent `stark`, `Stark` or `STARK`

- ToFile print should contain all information

- Comments and docs are _very_ inconsistent in style

- consistent get_name, get_label, to_string

* `std::string to_string` for symx options should be in symx

* Line search: Decide for apply_increments or global

* TONS of debug commented out stuff everywhere
    Specially collision

* stark should write vtk to .tmp and then rename when finished to avoid crashing 3rd party viz

* Add labels to DoFs for nicer printing

* Quasistatic should let gravity go

* Verify logic for adaptive stiffness and time step size upon failures

* Try failing all possible ways
    TooManyIterations, LinearSystemFail...

* Try direct solve



* Add `step` to the armijo condition.

* Unify mesh generation. Prefer mid point insertion for quads? What about quadratic quads?

* Massive duplication of mesh generation and utils
    I think we ship symx with minimal tooling for ease of ramping up
    And preserve the heavy stuff in Stark. This is what stark is.


* I think the time stepping logic with adaptivity and the script is broken
    The quasistatic extrussion restarts in wrong ways in respect to the script

* symx has a cmake option to set compilation path.
    How do we handle it being in stark.settings?

    There are other legacy symx options inside stark.settings that I have to port over.


* Make sure all details from the ppn branch are ported
    We are building from ~github version

* Check the whole symx use and logic for consistency and latent bugs
    We are comparing many pointers, and fetching and storing... Extensive check!

* There will be a lot of dead code and old/AI comments. Massive cleanup.

* Run all the simulations from PPN to certify it runs as expected



# STARK Docs

* Clarify that stark uses a velocity based solver, therefore tolerances, etc are in m/s and time step dependent

