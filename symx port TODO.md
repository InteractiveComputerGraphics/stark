# symx port TODO

* symx has a cmake option to set compilation path.
    How do we handle it being in stark.settings?

    There are other legacy symx options inside stark.settings that I have to port over.

* Different Newton convergence criteria
    `struct Residual { ResidualType type; double tolerance; };`

* Revamp and cleanup Stark.run_one_step()
    Take inspiration from the new Newton's Method
