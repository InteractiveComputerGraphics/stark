Make sure you compile with `-j8` to avoid crashing my PC due to lack of RAM

I have carried out a massive refactor in the symx codebase as part of stark. you can find the original reference implementation in `REFERENCE IMPLEMENTATION/`. Tons of things have changed, but the core functionality (simulation using IPC) remains. It's just a refactor/revamp.

I have set up a quick experiment both in the current and the reference implementations. It's the same one: a twisting piece of cloth. Contact is disabled so it's just a wrapping shell. There is inertia, strain, strain limiting and not much more. Simple. We are using ProjectedNewton in both examples.

I checked the visual output and while not identical due to the complex motion, they both look plausible for the setting.

The problem is that both implementations are consistently giving different numerical behavior: reference takes more newton iterations and fewer CG iterations. New takes fewer newton iterations and more CG iterations. this is consistent at larger scales and also when I activate frictional contact.

Your task is to run both experiments as they are in their respective `examples` executable and inspect the output to verify that the above is correct. Then, go into both libraries and look for differences that can be the cause of this. Important: Leave contact aside, it is a huge can of worms; the problem is already evident without contact.

Focus on the key stuff that changed the most: symx. You should also check that the corresponding energy definitions are the same, as well as the mesh declaration and so on. Also, NewtonsMethod was totally revamped along with other numerical infrastructure. But the thing that can easily bite us is to assume that symx is correct or equivalent to the old one. I recently had a latent (stateful) bug in `CompiledInLoop_run` that was incredibly hard to find. Do not leave a stone unturned.

Now, I know tons of stuff has changed. You do not need to enumerate all the changes. Only the behavior altering ones. Maybe it's just a parameter that got lost in the port. Maybe it's simply double counting. Or maybe the energy evaluation magically identically for other problems (it does) but for this one there is a case difference and it's totally wrong.

Compile and run as many times as you need. Each run takes 8 seconds. Be thorough. Do not carry out large changes in the codebase. Feel free to try small things, but don't refactor anything. Non-small changes should be first discussed with me.


