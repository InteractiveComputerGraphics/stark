# symx port TODO

I generally agree with your proposal. Here are my points/corrections:

- If I remember correctly, most prints inside symx are internal and mostly errors and warnings.
    Let's leave those alone for the time being. Focus exclusively in what concern simulation output.
    We will deal with error, warnings and info inside symx later on.
- I do not want to have two layers of console verbosity. stark should use symx's sink and console verbosity levels.
    If symx is `Silent` the user sees just the time step header and the frames being written (hopefully with a decent format). This is intended. If the user does not even want to see that, they just `stark::settings.output.enable_console_output = false`, which should disable the sink.
- I am unsure about the `NewtonOutputFormatter`. It seems a bit over-engineered. Although I understand that it's necessary. Output _can_ be messy, but if we use the Verbosity as a tab depth, maybe it's all just simple prints? Worth considering before committing to even more abstractions and boilerplate.
- I do not have the need of backwards compatibility. We are building this brand new. 
- stark will have to add stuff to the logger, such as time step information that symx does not have.
- I am unsure about the final stark print. It should be a few stark global numbers, followed by the key summary numbers from NewtonsMethod.
- No more mutex on prints.
- We probably want to move the Logger and Console to symx's Context. Then everybody has easy access.
- Because stark console (and logger) does not exist anymore, sending output to a file is now responsibility of symx's outputsink. Modes: PrintOnly, FileOnly, PrintAndFile.

- I forgot to mention that the `.log` files will need to be easily readable in python for further processing.
    We don't have to do any python right now, but it's a golden opportunity to make it machine-readable.
    It should be easy for the user to take a look with an editor of course, that's why the summary should be included at the top.
    I would be happy using existing formats (yaml?) if that would facilitate anything.
- Consider that timing can add a lot of boiler plate to Newton's Method. Therefore, I would really appreciate ergonomics here.

Revise the existing proposal and give me a summary (in the chat) with the changes/counterpoints.





- Use the current `examples` to test your implementation. When you are happy with a draft version for me to review, please generate a different file per verbosity level inside `output/` and point me to it.
- Similarly, point me to where the `.log` lives for the final simulation for me to review.






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

