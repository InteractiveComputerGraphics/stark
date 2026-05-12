Starting: *Apply small fixes* (3/4)

Renamed `global_point_velocity_in_rigib_body` to `global_point_velocity_in_rigid_body` - 3 edits across 3 files.

Now fix the remaining typos and dead comments all at once: 

Replacing 15 lines with 10 lines in [](file:///home/fernandez/libs/stark/stark/src/models/rigidbodies/EnergyRigidBodyInertia.cpp), [](file:///home/fernandez/libs/stark/stark/src/utils/mesh_utils.cpp), [](file:///home/fernandez/libs/stark/examples/main.cpp), [](file:///home/fernandez/libs/stark/stark/src/models/interactions/EnergyFrictionalContact.cpp), [](file:///home/fernandez/libs/stark/stark/src/core/Settings.h), [](file:///home/fernandez/libs/stark/stark/extern/symx/src/compile/Compilation.h)


```

```

```

```

```

```

```

```

```

```

```

```
Let me verify the Compilation.h change looks clean: 

Read [](file:///home/fernandez/libs/stark/stark/extern/symx/src/compile/Compilation.h#68-68), lines 68 to 82

Replacing 4 lines with 3 lines in [](file:///home/fernandez/libs/stark/stark/extern/symx/src/compile/Compilation.h)


```

```
Now let me also verify the Settings.h change: 

Read [](file:///home/fernandez/libs/stark/stark/src/core/Settings.h#30-30), lines 30 to 45

Good. Now let me check whether the `add_force_at` inconsistency and other report-worthy issues need a deeper look before writing the report. Let me also scan the pystark and README for additional issues:  

Ran terminal command:  grep -rn "rigib_body\|TOOGLE\|learnSPH\|consequtive\|// DEBUG\|// struct symx::NewtonSettings\|Static methods removed" \
  /home/fernandez/libs/stark/stark/src/ \
  /home/fernandez/libs/stark/stark/extern/symx/src/ \
  /home/fernandez/libs/stark/examples/ 2>/dev/null

All clean. Now let me do a more thorough scan for report-worthy issues before writing the final report: 

Ran terminal command:  grep -rn "add_force_at\b" /home/fernandez/libs/stark/stark/src/ 2>/dev/null | grep -v "add_force_at_centroid"

Read [](file:///home/fernandez/libs/stark/stark/src/models/rigidbodies/RigidBodyHandler.cpp#88-88), lines 88 to 120

Ran terminal command:  grep -rn "add_force_at\b" /home/fernandez/libs/stark/ 2>/dev/null | grep -v "add_force_at_centroid\|tests/extern\|symx/extern\|build/"

Read [](file:///home/fernandez/libs/stark/stark/extern/symx/src/solver/solver_utils.h#118-118), lines 118 to 220

Read [](file:///home/fernandez/libs/stark/stark/extern/symx/src/solver/solver_utils.h#220-220), lines 220 to 260

Ran terminal command:  head -100 /home/fernandez/libs/stark/README.md

Ran terminal command:  wc -l /home/fernandez/libs/stark/stark/src/models/deformables/line/*.{cpp,h} \
  /home/fernandez/libs/stark/stark/src/models/deformables/surface/*.{cpp,h} \
  /home/fernandez/libs/stark/stark/src/models/deformables/volume/*.{cpp,h} \
  /home/fernandez/libs/stark/stark/src/models/deformables/point/*.{cpp,h} 2>/dev/null | sort -rn | head -20

Ran terminal command:  grep -rn "^\s*//.*[;{}=+\-*/]" \
  /home/fernandez/libs/stark/stark/src/models/deformables/ \
  /home/fernandez/libs/stark/stark/src/models/rigidbodies/ \
  2>/dev/null | grep -v "//.*//\|// *\!" | head -30

Ran terminal command:  grep -rn "^\s*//.*[;{}=]" \
  /home/fernandez/libs/stark/stark/src/models/deformables/ \
  /home/fernandez/libs/stark/stark/src/models/rigidbodies/ \
  2>/dev/null | head -30

Read [](file:///home/fernandez/libs/stark/stark/src/models/deformables/volume/EnergyTetStrain.cpp#1-1), lines 1 to 90

Read [](file:///home/fernandez/libs/stark/stark/src/models/deformables/volume/EnergyTetStrain.cpp#75-75), lines 75 to 181

Read [](file:///home/fernandez/libs/stark/stark/src/models/deformables/volume/EnergyTetStrain.h#1-1), lines 1 to 44

Ran terminal command:  grep -n "^\s*//" /home/fernandez/libs/stark/stark/src/models/deformables/surface/EnergyTriangleStrain.cpp | head -20
echo "---"
grep -n "^\s*//" /home/fernandez/libs/stark/stark/src/models/deformables/surface/EnergyDiscreteShells.cpp | head -20
echo "---"
grep -n "^\s*//" /home/fernandez/libs/stark/stark/src/models/deformables/line/EnergySegmentStrain.cpp | head -20

Read [](file:///home/fernandez/libs/stark/stark/src/models/deformables/surface/EnergyTriangleStrain.cpp#1-1), lines 1 to 90

Read [](file:///home/fernandez/libs/stark/stark/src/models/deformables/line/EnergySegmentStrain.cpp#1-1), lines 1 to 90

Searched for text `tet_jacobian`, 4 results

Searched for text `triangle_jacobian`, 4 results

Read [](file:///home/fernandez/libs/stark/stark/src/models/deformables/deformable_tools.cpp#1-1), lines 1 to 35

Ran terminal command:  ls /home/fernandez/libs/stark/stark/extern/symx/extern/ 2>/dev/null

Read [](file:///home/fernandez/libs/stark/examples/_old)

Ran terminal command:  grep -rn "add_potential\|conn_complete\|conn_elasticity" /home/fernandez/libs/stark/stark/src/models/deformables/volume/ 2>/dev/null

Read [](file:///home/fernandez/libs/stark/stark/src/models/deformables/surface/EnergyDiscreteShells.cpp#1-1), lines 1 to 35

Good, `EnergyDiscreteShells` has an active implementation. Now let me gather a bit more info about the pystark layer and then move to the report: 

Ran terminal command:  ls /home/fernandez/libs/stark/pystark/cpp/ && \
grep -rn "TODO\|FIXME\|deprecated\|old\|Old" /home/fernandez/libs/stark/pystark/cpp/ 2>/dev/null | grep -v "symx/extern\|tests/extern\|.o:" | head -20

Ran terminal command:  grep -rn "add_force_at\b" /home/fernandez/libs/stark/pystark/ 2>/dev/null | grep -v "add_force_at_centroid"

Read [](file:///home/fernandez/libs/stark/pystark/cpp/models/rigidbodies/pystark_RigidBodyHandler.cpp#20-20), lines 20 to 50

Read [](file:///home/fernandez/libs/stark/stark/src/utils/mesh_utils.cpp#68-68), lines 68 to 100

Read [](file:///home/fernandez/libs/stark/stark/src/models/deformables/PointSetHandler.h#45-45), lines 45 to 58

Ran terminal command:  grep -n "^\s*//" /home/fernandez/libs/stark/stark/src/models/rigidbodies/EnergyRigidBodyConstraints.cpp | head -20

Ran terminal command:  grep -n "^\s*//.*[;{}=]" /home/fernandez/libs/stark/stark/src/models/rigidbodies/EnergyRigidBodyConstraints.cpp | head -10
echo "---"
grep -rn "^\s*//.*[;{}=]" /home/fernandez/libs/stark/stark/src/models/interactions/EnergyFrictionalContact.cpp | head -10

Read [](file:///home/fernandez/libs/stark/stark/src/models/rigidbodies/EnergyRigidBodyInertia.cpp#1-1), lines 1 to 104

Ran terminal command:  grep -n "force\|torque" /home/fernandez/libs/stark/stark/src/models/rigidbodies/RigidBodyDynamics.cpp | head -30

Read [](file:///home/fernandez/libs/stark/stark/src/models/rigidbodies/RigidBodyDynamics.cpp#1-1), lines 1 to 50

Read [](file:///home/fernandez/libs/stark/stark/src/models/rigidbodies/RigidBodyDynamics.cpp#120-120), lines 120 to 166

Starting: *Write report of larger issues* (4/4)

All small fixes are in. Here's the full report:

---

## Fixes Applied

| File | Fix |
|---|---|
| rigidbody_transformations.h, .cpp, RigidBodyDynamics.cpp | `rigib_body` → `rigid_body` (semantic rename across 3 files) |
| EnergyRigidBodyInertia.cpp | `non-consequtive` → `non-consecutive` |
| main.cpp | `TOOGLE` → `TOGGLE` |
| mesh_utils.cpp | `learnSPH error` → `Stark error` (copy-paste from old project) |
| main.cpp | Removed `// DEBUG` block and orphaned commented-out `n_threads` line |
| EnergyFrictionalContact.cpp | Removed `// DEBUG` suffix from `// Contact` comment |
| Settings.h | Removed dead `// struct symx::NewtonSettings {};` comment |
| Compilation.h | Removed stale `// (Static methods removed, use global functions in symx namespace)` comment |

---

## Report: Larger Issues to Address

### 🔴 Critical — `EnergyTetStrain` with `elasticity_only = false` is silently inactive

**File:** EnergyTetStrain.cpp

The `add_potential` for `conn_complete` is commented out (the entire block from line 12 to line 77). However, the `add()` method still routes elements to `conn_complete` when `params.elasticity_only == false`. Since no potential is registered for `conn_complete`, any tet mesh set up with `elasticity_only = false` **contributes zero energy** — damping and strain limiting silently don't work.

The `tet_jacobian()` helper still exists in `deformable_tools.h/cpp`, so the commented-out code is structurally compatible with the current codebase. The fix is to restore that block, optionally rewriting it to use the explicit inline form (as done in the elasticity-only path) to avoid the helper if desired. Compare with EnergyTriangleStrain.cpp and EnergySegmentStrain.cpp which correctly register both potentials.

---

### 🟠 Bug — `add_force_at` overwrites instead of accumulating

**File:** RigidBodyHandler.cpp

`add_force_at` uses `=` for both `force` and `torque`, behaving as a setter. This contradicts the naming convention established by `add_force_at_centroid` (which correctly uses `+=`) and the rest of the API (`add_velocity`, `add_torque`, etc.).

The impact: calling `add_force_at` multiple times on the same body in one timestep silently discards all previous forces. The fix depends on intent:
- If it should accumulate: change `=` to `+=` for both lines
- If it should set: rename to `set_force_at` and add a proper `add_force_at` with `+=`

Currently there is no `set_force_at` (only `set_force_at_centroid`), so there's also a missing API gap.

---

### 🟠 Bug — mesh_utils.cpp continues silently after non-triangle face error

**File:** mesh_utils.cpp

When a non-triangular face (e.g. a quad) is encountered in an OBJ file, the error is printed but execution continues. The code then reads only 3 of the 4 (or more) indices, producing a corrupted triangle and misaligning all subsequent index reads. Add `exit(-1)` (or `continue` to skip the face) after the error message to match the rest of the codebase's error handling style.

---

### 🟡 Design — Forces and torques are not reset between timesteps

**File:** RigidBodyDynamics.cpp

`_before_time_step` resets `v1` and `w1` to zero, but **does not reset `force` and `torque`**. This means forces set via `set_force_at_centroid` or `add_force_at_centroid` persist indefinitely across timesteps unless the user manually clears them. This is a subtle footgun for users who expect per-timestep forces to be ephemeral. Consider whether a `_before_time_step` reset of force/torque to zero is the right convention, and document the current behavior explicitly.

---

### 🟡 Design — Pervasive `exit(-1)` error handling

Throughout src and src, fatal errors call `exit(-1)` directly (e.g. Stark.cpp constructor, EnergyRigidBodyInertia.cpp, mesh_utils.cpp). This makes Stark difficult to use as an embedded library and impossible to catch errors in calling code. As the codebase matures and gets used more broadly, moving toward C++ exceptions or at minimum a configurable error callback would be worthwhile.

---

### 🟡 API — PointSetHandler.h compiler workaround comment may be outdated

**File:** PointSetHandler.h

The inline template workaround cites "Apple clang version 15.0.0" as the affected compiler. This was current ~2023. If you're building on more recent toolchains, verify whether the workaround is still necessary and update or remove the comment accordingly.

---

### 🟡 Incomplete feature — No `set_force_at` method

**File:** RigidBodyHandler.h

The API has `set_force_at_centroid` + `add_force_at_centroid` (the symmetrical pair), but only `add_force_at` (no `set_force_at`). A `set_force_at(force, point)` that also computes and sets the resulting torque from the application point would complete the API and disambiguate intent.

---

### 🟡 `Stark::run_one_step` returns `true` on timestep failure

**File:** Stark.cpp

When a timestep fails but the adaptive scheme reduces `dt` to retry, the function sets `success = true` and returns. The variable name `success` implies the step completed successfully, but here it means "simulation should continue". Consider renaming the local variable to `should_continue` to reduce confusion for future maintainers.

--- 

Completed: *Write report of larger issues* (4/4)

The most urgent item to tackle is the **`EnergyTetStrain` silent no-op** — any simulation using volumetric tet meshes with damping or strain limiting is getting zero contribution from those terms right now. Everything else is design-level. What would you like to add to the TODO list?

Made changes.