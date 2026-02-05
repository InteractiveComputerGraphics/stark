# symx port TODO

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

