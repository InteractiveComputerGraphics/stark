Make sure you compile with `-j8` to avoid crashing my PC due to lack of RAM

- It's important to be able to fully disable both the Output and the Logger if they are disabled.
    Guard everything so that every call is immediately exit.
    Of course, the final print as well as writing the yaml should be guarded and skipped in that case.
    Similar for any other risk of reading the empty classes.
    Do this improvement now.
- Forget about Silent not be really silent. We do disable and that's the way to turn it off.
- After making sure both the Logger and the OutputSink can be fully disabled, run the current experiment in that mode and see what happens to "misc".
    I am aware that we are paying the price of fmt all the strings anyway. I can live with that.