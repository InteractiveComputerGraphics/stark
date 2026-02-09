# symx port TODO

I generally agree with your proposal. Here are my points/corrections:

- print tabs should be customizable so the upper lib (stark) can accommodate.
    E.g. root_tab = 1 (so stark has the 0th); and tab_size = 2 (two spaces)
- We don't need to explicitly indicate tabs in out messages, symx will do that based on the verbosity the message comes with
- I think we can pay the price of fmt always even if the message will not be printed. Otherwise the code is going to be hell. I was more worried about hitting flushes and newlines like crazy.
- `print_raw()` -> `print()` (without verbosity; just print)
- OutputSink requires that the file exists only if needed. Not always.
- I agree we should accummulate the msg per line and print it at once.
    maybe we use two modes: `print` and `append`+`flush_line`.
    Append can do the iconic "AAAA | BBBB | CCCC" and flush indicates actual print

    This might be a bad idea. Leave the proposal flexible in this regard because we will have to see once we are actually implementing this.
    Your example output is definitely the goal, so that's correct. I am just wondering about the ergonomics to get there.






- Use the current `examples` to test your implementation. When you are happy with a draft version for me to review, please generate a different file per verbosity level inside `output/` and point me to it.
- Similarly, point me to where the `.log` lives for the final simulation for me to review.




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

