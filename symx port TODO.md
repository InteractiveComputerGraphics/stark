# symx port TODO


HI. I am doing a very large port between codebases after a large revamping of my symx library.
Before we continue, some notes:
    - Feel free to compile the `examples` executable and run it for both the new and reference implementation
    - Compile with -j8 as to not overflow my RAM
    - You can find the reference implementation that 100% has worked for _years_ in `REFERENCE IMPLEMENTATION`
    - Make sure to build the reference implementation without MKL `cmake .. -DCMAKE_BUILD_TYPE=Release -DSYMX_USE_MKL:STRING=OFF -DSTARK_USE_MKL:STRING=OFF`
        The current implementation does not have that dependency.
    - `examples` in both codebases are already running the a similar experiment.
        Results will not necessarily be the same because the port is still WIP. But it should be similar.

The goal of this refactor is to bring a new version of `symx` into `stark`. Tons of things have changed in symx, but ultimately it does exactly the same thing.
Notably, `NewtonsMethod` is now part of symx, where before it was in `stark`.

Your task is to consolidate logging and timing infrastructure.
First, give a very thorough look at the current and reference implementation.
Then, run both `examples` executable and look at the output. The new one is a mess (to fix), the ref one is nice and we can take it as a goal.
For now do not implement anything. First, propose a new infra solution for all of this with a high level specific proposal.
Who owns what? How access happens? How can the user override, redirect? How can it all be nullified for benchmark-type experiments? etc
SymX will provide this infra and stark will tag along.
Pay attention also to the final print as it is relevant.

Many prints in the new implementation will need to be reformatted to stay clean and compact.
Maybe there is also a better way to structure prints to avoid the current "stateful" chain where it prints depends on the previous and next to keep things clean.
In this sense, maybe it would be good to have a class to handle the NewtonsMethod print structure.
The rest really is before and after summaries anyway.
But keep in mind that prints should happen frequently, not at the end of a whole Newton step that can take minutes in large simulations.

Generally, before we actually implement the improved system, we need to consider options.

Some notes:
Previously, this was contained in `stark`, but now it should be moved into `symx`.
SymX will provide facilities to log, time and console output with the standard verbosity options.
Keep in mind that SymX will be an independent library to stark. Other people can use it to build other stuff. So it should be independent, and facilitate parent applications to work with it.
Because symx does not technically know about "simulation" (it's just NewtonsMethod on any potential), we cannot flag verbosity as `Frames`, `TimeSteps`, ... Maybe we need other way/naming to indicate verbosity.
Performance is also a concern, frequent hits to virtual functions and flushes can be very slow.


So yeah, give me a proposal on what options you think suit the current codebase.






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

