# Classical Nav2 variants for the office (applied ON TOP of nav2_office_overlay.yaml)

`M20_SITE=office M20_NAV_VARIANT=<name> bash tools/nav/bringup_plan_a.sh`
(no M20_AI_CONTROLLER = classical Nav2). Measured comparison:
docs/ai_nav/office_variants/RESULTS.md. Short version: keep `dwb`. The RPP variants are
blocked by RPP's footprint collision check at 0.85 m doors (fix: collision_monitor);
MPPI needs a CPU-budget test.

| name | planner | controller | idea |
|---|---|---|---|
| `dwb` | NavFn (A*) | DWB | the office default (nav2_office_overlay.yaml alone) |
| `shim_rpp` | NavFn | RotationShim -> RPP | turn in place to the path first, then pure pursuit with footprint collision checking and speed regulated near obstacles |
| `lattice_rpp` | Smac Lattice, diff-drive primitives | RotationShim -> RPP | footprint-aware SE(2) plans: the path itself respects the 0.91 x 0.53 m body through doors |
| `mppi` | NavFn | MPPI (DiffDrive) | sampling MPC over 1000 rollouts, footprint cost critic |

History: RPP 1.1.20 sign-flapped at 0.2 rad/s on this stack in July (nav2_params.yaml
note), so the `*_rpp` variants front it with the RotationShim, which owns the
in-place rotation that was flapping. Measure, don't assume.
