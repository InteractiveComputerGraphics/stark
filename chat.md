I have implemented some of your suggestions.

- The current problem does not seem related to max_step as it runs. This is normal and expected. The energy evaluation fails before the simulation has a chance to get into actual penetration.
- Part of the intention for the refactor is to make Newton work with step increments rather than abs values. The logic for `shrink=0.5` should be identical. Please, verify this by looking into the relevant calls.
- I noted that the new version codegen for the derivative evaluation is _a bit_ different. I attributed this to symx having a big revamp. But it could be that something is being set incorrectly. Please, follow the symbols path to craft the expressions because maybe there are being built incorrectly. SymX internals are thoroughly tested, but it also be wrong of course.
- Edge-edge is intentionally disabled for this test.

To clarify: The simulation seems to work perfectly fine without contact enabled. This indicates to me that the error is somewhere that contact touches. Contact adds a huge complexity: complex symbolic expression generators, contact detection, many callbacks... I presume something around here is incorrect. But it could be also a latent bug that just shows up now.

Give me a new list of potential issues to analyze. Please, bring back previous suggestions that you think I should address if needed.





New List of Potential Issues to Analyze
🔴 HIGH PRIORITY - Most likely causes
1. Armijo Threshold Step Scaling (Verify this is intentional)

Location: NewtonsMethod.cpp:436-438
Issue: The Armijo threshold is computed once as E0 + beta * du_dot_grad (full step), but the classical Armijo condition is E(u + s*du) ≤ E0 + beta * s * du_dot_grad (scaled by step s).
Both versions do this: Reference has the same formula.
Effect: As step shrinks, E approaches E0, but E_threshold is fixed. If E0 > E_threshold (which should be true), the condition E < E_threshold may never be satisfied if the energy floor is higher than E_threshold.
Suggested check: Print E0, E_threshold, and du_dot_grad at line search entry to verify the threshold is reasonable.
2. Contact Set Changing During Line Search

Location: _before_energy_evaluation__update_contacts callback
Issue: Proximity detection runs for every energy evaluation during line search. If different contacts are detected at different step sizes, the energy landscape is discontinuous.
Symptom: This would explain why (E - E_bt)/E_bt stays positive even as step→0 — at tiny steps, the contact set might include fewer/different pairs than at larger steps, leading to an energy offset.
Suggested check: Add debugging to print the number of contacts detected at each energy evaluation.
3. Verify the Incremental DOF Update Logic

Location: NewtonsMethod.cpp:401-403 and NewtonsMethod.cpp:426
Your clarification says the shrink=0.5 logic should be identical. Let me verify:
Start: DOFs at u
apply_scaled_du(1.0): DOFs = u + 1.0*du
Invalid, shrink to 0.5: apply_scaled_du(-0.5): DOFs = u + 1.0*du - 0.5*du = u + 0.5*du ✓
Invalid, shrink to 0.25: apply_scaled_du(-0.25): DOFs = u + 0.5*du - 0.25*du = u + 0.25*du ✓
The math is correct. However, this assumes apply_dof_increment is thread-safe and the DOFs are being modified in-place correctly.
🟡 MEDIUM PRIORITY - Worth checking
4. Symbolic Expression Construction for Contact

Location: EnergyFrictionalContact.cpp:1382-1388
Difference from reference:
New: mws.make_vectors(this->dyn->v1.data, conn)
Reference: energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, conn)
Question: Does the new API correctly identify these vectors as DOFs for differentiation? The get_symbols mechanism matches by container address, so this should work if this->dyn->v1.data is the same container registered via add_dof.
Suggested check: Verify that the gradient has the expected non-zero structure by comparing gradient output vs. reference.
5. Barrier Potential Sign Convention

Location: EnergyFrictionalContact.cpp:1208
Formula: k * (dhat - d)^3 / 3
Check: When d < dhat, barrier should be positive and repulsive. The gradient dE/dd = -k*(dhat-d)^2 should be negative (pointing away from contact). Verify this sign convention matches expectations.
6. Distance Functions

Location: distances.cpp
The distance_point_plane uses sqrt(sq_distance) which should always be positive. But sq_distance_point_plane computes d = vpa.dot(n); return d*d where d can be negative (signed distance). The square makes it positive, but this means the gradient might have sign issues.
Suggested check: Verify numerical gradient matches analytic gradient for contact energy.
🟢 LOWER PRIORITY - Less likely but worth mentioning
7. Projection to PD Mode

The output shows ph = 100.0% indicating full Hessian projection. This should not cause issues but verify projection isn't over-regularizing.
8. CG Solver Convergence

CG iterations are reasonable (6-28 per Newton step). Not suspicious.
9. Newton Convergence Before Contact Kicks In

The simulation converges fine (1-7 Newton iterations) until contact becomes significant around Frame 30. This confirms the issue is contact-specific.
Recommended Debugging Steps
Add contact count logging: In _before_energy_evaluation__update_contacts, print how many contacts are found. See if this number changes during line search.

Add E0/E_threshold/du_dot_grad logging: At line search entry, print:


printf("LS: E0=%.6e, E_threshold=%.6e, du_dot_grad=%.6e\n", E0, E_threshold, du_dot_grad);
Numerical gradient check: Add a debug mode that computes dE/dv1 numerically using finite differences and compares to the symbolic gradient.

Freeze contact set during line search: As an experiment, modify _before_energy_evaluation__update_contacts to skip re-running collision detection during line search (use a flag). This would test if contact set changes are causing the issue.





🔴 CRITICAL ISSUE #2: Line Search Applies Increments Instead of Setting Absolute Position
Location: NewtonsMethod.cpp:401-403 and NewtonsMethod.cpp:426

New implementation (WRONG):

Reference implementation (CORRECT):

The Problem:
Your apply_scaled_du(-step) is trying to undo and reapply by adding -step * du, but this logic is incorrect:

If you applied +step and now want 0.5*step, you need to add -(0.5*step) not -(0.5*step) again
You're accumulating errors in the DOF state
The reference always uses set_dofs from a saved base state u0, which is clean and correct.

Impact: DOF state becomes corrupted, leading to wrong energy evaluations.

🟠 HIGH ISSUE #3: Missing Max Step Callback (CCD) Before Line Search
Location: NewtonsMethod.cpp:175-180

New implementation:

Reference implementation:

Where _inplace_max_step_in_search_direction:

Calls run_max_allowed_step() (CCD)
Then loops shrinking step until run_is_intermidiate_state_valid() returns true
The Problem:
In the new code, you compute max allowed step, then scale du, then enter line search where you start at step=1.0. But the reference has a clear separation:

Find max valid step (step_valid_configuration)
Start line search at that step, not at 1.0
Your line search starts at 1.0 and relies on the first loop to find validity, but this is entangled with Armijo checking in a broken way.

🟠 HIGH ISSUE #4: Energy Not Evaluated at Valid Step Before Armijo Check
Location: NewtonsMethod.cpp:428-435

Reference workflow:

Find valid configuration at step_valid_configuration
Evaluate E at that step
Check Armijo condition E > suitable_backtracking_energy
If fails, shrink and re-evaluate
New workflow:

First loop shrinks until valid
Second loop starts at current step but only evaluates E inside the loop
Armijo threshold computed with potentially wrong step
Problem: The first Armijo iteration evaluates E after computing the threshold, meaning the first check happens, but the threshold was computed with a stale step value.

🟡 MEDIUM ISSUE #5: Potentials Return Scalar Instead of Calling energy.set()
Location: Throughout EnergyFrictionalContact.cpp

New implementation:

Reference implementation:

Where _set_barrier_potential calls energy.set(E).

Impact: This is likely fine if your new symx API expects return values, but if there's any mismatch in how the potential is accumulated, it could cause wrong gradients/Hessians.

🟡 MEDIUM ISSUE #6: Callback Structure Difference
New: stark.callbacks.newton.add_before_energy_evaluation(...)

Reference: stark.callbacks.add_before_energy_evaluation(...)

You've correctly moved Newton-specific callbacks into a nested newton object, and it appears to be wired correctly at Stark.cpp:247:

Verify: Make sure this copy happens after all energies are registered but before Newton solve. If energies register callbacks at construction time, the copy timing matters.

🟡 MEDIUM ISSUE #7: set_max_allowed_step vs add_max_allowed_step
New solver_utils.h:

Reference Callbacks.h:

Problem: The name says set but implementation does push_back. This is misleading but not necessarily broken. However, check if any code actually registers a max_allowed_step callback for CCD. If not, run_max_allowed_step() always returns 1.0 and CCD is bypassed.

🟢 LOW ISSUE #8: Missing run_on_intermidiate_state_invalid in Line Search
Reference (in _inplace_max_step_in_search_direction):
When step becomes too small, it calls:

New code: Calls run_on_intermediate_state_invalid() in the first loop, but the logic is different.

🟢 LOW ISSUE #9: Edge-Edge Contacts Commented Out in New Code
Location: EnergyFrictionalContact.cpp:29-36

This is intentional for debugging, but make sure to restore when ready.

Summary of Fixes Needed (Priority Order)
Fix the line search to use absolute DOF setting (save u0, use set_dofs not incremental apply_dof_increment)
Fix Armijo threshold to use full step (or recompute per iteration with current step)
Align the "find valid step" and "Armijo line search" logic with reference implementation
Verify max_allowed_step callback is registered for CCD if needed
Verify callback copy timing in Stark::_initialize()
The first two issues are almost certainly causing your infinite backtracking.