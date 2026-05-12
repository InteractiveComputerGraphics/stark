# symx port TODO

## Post Eurographics

Cleanup files and comments
Docs
    - Explain all parameters
    - Explain all callbacks
    - Color with VSCode style
    - Panic sheet
Readme
LinkedIn Post
pip install


### Claude
- don't exit but throw




## Comparison to PPN ref
    - Dont compare CG iterations with Progressive because we are counting failures differently.
    - forcing sequence was a difference. But it is a tradeoff.
    - New CG seems slower on same iterations


## High Priority
* Logger and Console use internally in symx
    But most internal prints are just error messages...


## Low Priority

- Consistent `stark`, `Stark` or `STARK`

- Comments and docs are _very_ inconsistent in style

- consistent get_name, get_label, to_string

* TONS of debug commented out stuff everywhere

* Try failing all possible ways
    TooManyIterations, LinearSystemFail...

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

* I think the time stepping logic with adaptivity and the script is broken
    The quasistatic extrussion restarts in wrong ways in respect to the script

* Revise that rigid body inertia is correct. It is formulated as velocity.



# STARK Docs

* Clarify that stark uses a velocity based solver, therefore tolerances, etc are in m/s and time step dependent

