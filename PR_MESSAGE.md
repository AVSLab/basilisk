## Description
<!-- What approach was taken to satisfy the ticket being addressed? What should reviewers be aware of? How are your
commits organized? -->

Fixes two pre-existing `MJBody` mass-property bugs in the MuJoCo dynamics engine, surfaced by orbital articulated-spacecraft scenarios.

**1. `<body>_com` site reported the body frame origin, not the true center of mass.**
MuJoCo latches each site's `sameframe` class at compile time (`user_model.cc`). The `_com` site is created at the body origin, so it latches to `mjSAMEFRAME_BODY`, and `mj_local2Global` then discards `site_pos` and pins `site_xpos` to the body origin — so the later copy of `body_ipos` into `site_pos` has no effect. Consumers of the `_com` state message (`NBodyGravity` target, `ExponentialAtmosphere` altitude, navigation) got the wrong point, and gravity applied at the site acted off-CoM, torquing hinge-rooted bodies (deploying panels spun up spuriously).
Fix: in `MJBody::configure`, set the site spec pos = `body_ipos` (quat left identity) and force one recompile so it re-latches to `mjSAMEFRAME_BODYROT`. Position then tracks `body_origin + R·site_pos` (= the CoM) every `mj_kinematics`, while orientation stays the body-origin frame. `BODYROT` was chosen over `INERTIA` deliberately: `INERTIA` would also rotate the site into the principal-inertia frame, breaking the documented "`_com` shares the origin frame orientation" contract. A guard (`ipos.norm() > 1e-9`) skips the recompile when the CoM is already at the origin, avoiding an infinite recompile loop (`configure()` runs inside `recompileIfNeeded()`).

**2. Inertia tensor never rescaled when mass changed.**
`MJBody::updateMujocoModelFromMassProps` scaled inertia by `newMass / body_mass`, but overwrote `body_mass` with `newMass` first, making the ratio identically `1.0`. Fix: capture `oldMass` before the overwrite so the ratio is correct.

Both changes are confined to `MJBody.cpp` (+~25 lines).

## Verification
<!-- How were the changes validated? Were any automated tests added, updated, removed, or re-baselined? If you didn't 
add or update any tests justify this choice. -->

Added `test_mass_props_and_com.py` with two regression tests, written test-first and confirmed to fail on the unfixed code before the fix:

- `test_com_site_reports_true_center_of_mass`: a free body with a geom offset from its frame origin; asserts the `_com` site is offset from the body origin by the known geom offset, and that its orientation matches the origin frame (not the principal-inertia frame). Failed before (offset `0.0`), passes after.
- `test_inertia_rescales_with_mass`: a torqued free body whose mass doubles over the run; asserts angular acceleration halves (`α = τ/I`, `I ∝ m`). Failed before (ratio `1.0`), passes after.

Full MuJoCo unit suite: 15 passed. The one remaining failure (`test_objects.py::test_adaptive_free_joint_translation_tolerances_are_stage_independent`) is **pre-existing and unrelated** — verified by reproducing it identically on a clean build with these changes stashed. Its root cause is a separate `.at()`-on-missing-key throw in the adaptive integrator's string-keyed tolerance getter (see Future work).

## Documentation
<!-- What documentation was invalidated by these changes? Which artifacts should reviewers check for accuracy and 
completeness? -->

No documentation invalidated. The `_com` site's documented contract (sits at the CoM, shares the origin frame orientation; `MJBody.h`) is now actually honored rather than changed. No release-note-visible API change.

## Future work
<!-- What next steps can we anticipate from here, if any? -->

- Fix the pre-existing adaptive-integrator tolerance-getter bug: `getRelativeTolerance(std::string)` / the dynObject-keyed getters use `.at()` and throw `std::out_of_range` on a missing key, which breaks `test_adaptive_free_joint_translation_tolerances_are_stage_independent`. Same `.at()`-vs-`operator[]` family already noted in the `feature/mujoco-vizard` review.
- If a runtime-shifting-CoM feature (fuel depletion, slosh) is ever added, revisit whether `body_ipos` needs to update during the sim — currently `updateMujocoModelFromMassProps` changes mass/inertia only, never `body_ipos`, so the CoM offset is frozen between recompiles.
