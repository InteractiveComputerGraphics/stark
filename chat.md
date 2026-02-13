Make sure you compile with `-j8` to avoid crashing my PC due to lack of RAM

- CG tolerances
    This is a bug! Or at least something uninteded.

    Please, bring CG tolerance criteria to standard practice.
    My _intention_ is to use mainly the relative tolerance as this seems common in my field (computer graphics).
    "Run CG until the residual is reduced 3 o 4 orders of magnitude"
    I added an extra absolute threshold so the user can avoid a run to the bottom in weird cases.
    Then, eventually I felt compelled to try forcing sequence as the absolute tolerance, which significantly improved performance overall.
    But I am conflicted: 3 thresholds! 2 relative, one absolute. This cannot be standard practice I assume.

    I imagined using relative tolerance as main. And then using max(abs_tol, forcing_tol).
    But I don't know how standard that is.
    Please, address and standardize this once and for all.

- correctly account for the change in `du_dot_grad` for all the steps reductions before the armijo test
    Be elegant about it.



