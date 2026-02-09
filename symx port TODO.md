# symx port TODO

* PPN Frictional contact
    !! Some vertices seems to not be included in the line search / energy eval / contact detection ...
        They keep moving freely
    - I have never checked evalaute_E
        conditionals, resetting thread buffers...
        But this does not explain negative curvatures and _some_ jumps
    - I need to manually inspect what the proximity detection is producing
    - color bins

    Generally the problem is that I have not tested a few things in symx:
        - Dynamic connectivities
        - Empty declared


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

