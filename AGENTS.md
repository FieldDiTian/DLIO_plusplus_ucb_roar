# Current workspace setup

The current vehicle localizer is `gicp_interface`, imported from the ART
standalone package. Preserve its VKS inputs, `map -> cg` output contract,
BEST_EFFORT QoS, external small_gicp/race_msgs dependencies, and Laguna profile.
Use its README and source for current behavior. `gicp_localization`, `adapter`,
and the adapter-dependent `scripts/prep_bag.py` have been removed. Replay
existing bags directly; GLIM also reads them directly with `glim_rosbag`.

The notes below predate this migration. Entries naming the removed localizer,
Atlas adapter, or old source line numbers are historical, not instructions for
the current GICP package. GLIM-specific mathematical notes still apply to the
retained GLIM implementation and should be verified against its current source.

---

# AGENTS.md — Notes for AI reviewers

This file documents patterns in DLIO++ that **look** like bugs but aren't,
along with the conditions under which they would become bugs. The goal is
to keep automated reviewers from re-flagging the same false positives every
session.

If you are reviewing this codebase: read this file first.

---

## Confirmed non-issues

### 1. `Isotropic::Information(prior_inf_scale.asDiagonal())` is correct (was — superseded)

**File**: `GLIM/glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp`
(now uses `Diagonal::Precisions(prior_inf_scale)` after commit `d8b2809`)

**What it looks like**: `Isotropic::Information(...)` doesn't exist in GTSAM.
You'll find no method by that name in `gtsam/linear/NoiseModel.h`. A first
read suggests an API mismatch.

**Why it was actually fine**: in C++, static-method name lookup via inheritance
resolves `Isotropic::Information(M)` to `Gaussian::Information(const Matrix&)`
(the inheritance chain is `Isotropic → Diagonal → Gaussian`). It builds a
Gaussian noise model with the diagonal information matrix — functionally
equivalent to `Diagonal::Precisions(v)` for diagonal arguments. The new form
is preferred for clarity but the old form was not buggy.

---

### 2. `getGtPoseAt` is safe against out-of-order GT odom

**File**: `gicp_localization/src/localization.cc:2365-2401` (function),
`callbackGtOdom` at `:2348-2356` (insertion).

**What it looks like**: the function uses `std::lower_bound` and computes
`u = (stamp - a->stamp) / dt_total` without much obvious defense.

**Why it's actually fine**:
- The buffer is kept monotone non-decreasing by an explicit drop at insertion:
  ```cpp
  if (!gt_odom_buffer_.empty() && s.stamp <= gt_odom_buffer_.back().stamp) return;
  ```
  Out-of-order or duplicate GT messages are discarded. `lower_bound` is
  therefore valid.
- `dt_total <= 0.0` is checked before division (line 2392).
- By construction, the chosen `u ∈ [0, 1]` because `lower_bound` puts the
  query strictly between `a->stamp` and `b->stamp`, so no extrapolation.
- Theoretical "tiny dt_total = 1 picosecond" concerns require GT samples a
  picosecond apart — non-physical for any real GNSS/INS publisher.

---

### 3. `callbackGtOdom` setting `first_opt_done = true` is not skipping initialization

**File**: `gicp_localization/src/localization.cc:2307-2318`

**What it looks like**: `applyInitialPose()` is called and then
`geo.first_opt_done = true` is set unconditionally, which on a casual read
appears to skip the first-time observer initialization in
`performLocalization()` at line 2106.

**Why it's actually fine**: `applyInitialPose()` itself zero-initializes the
observer state inside `geo.mtx` (lines 1068-1083): `state.v.* = 0`,
`state.b.* = 0`, `geo.prev_p/q = (initial pose)`, `geo.prev_vel = 0`. So by
the time `first_opt_done` is flipped, the state is already in the same
condition the `if (!geo.first_opt_done)` block at 2106-2123 would have set
it to. The first scan goes directly to `updateState()` operating on a
properly-initialized observer. The flag flip is correct.

---

### 4. The hessian-condition rejection disjunction is correct

**File**: `gicp_localization/src/localization.cc:2030-2041`

**What it looks like**: a 4-clause OR with three "warn-enabled AND warn-crossed"
sub-clauses plus a fourth "all warns disabled" sub-clause. Easy to convince
yourself the legacy fallback always fires, or that any single threshold
disables the others.

**Why it's actually fine**: trace the truth table.
- The fourth sub-clause `(fitness_warn ≤ 0 && trans_warn ≤ 0 && rot_warn ≤ 0)`
  fires only when **all three** warn thresholds are disabled — that's the
  intentional "legacy hessian-alone" fallback documented in the comment at
  line 2049.
- With YAML defaults (all three thresholds positive: `0.15`, `1.0 m`, `1.5°`),
  the fourth clause is unreachable by design. Rejection then requires hessian
  degenerate AND at least one of the three slide signals crossed.
- A "valid" scan (low fitness, small corrections) never satisfies the
  disjunction even when hessian is high.

The block matches its comment exactly. Don't refactor unless asked.

---

### 5. The URDF rotation between IMU and GNSS antenna is discarded — and that's fine on AV-24

**File**: `GLIM/glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp:107`

**What it looks like**: `t_imu_gnss = T_imu_gnss.translation()` throws away
the rotation part of an SE(3) transform. Looks like a missed correction.

**Why it's actually fine on AV-24**: in `av24.urdf`, both the chassis `pointonenav_joint`
and `gps_antenna_top_joint` are origin-only — no `rpy` attributes — and share the same
parent link `rear_axle_middle`. So `R_imu→antenna = identity`, and the
translation is the same vector regardless of which of those two frames you
express it in. With Atlas firmware projecting both IMU and pose to
`gps_antenna_top` directly, the offset never enters our math at all.

**Watch condition**: if a future URDF adds an `rpy` to either joint, or the
`urdf_imu_frame` is switched to a non-axis-aligned IMU, the lever-arm
correction will become biased. A startup warning was added in commit
`622271f` to flag this — heed it if you see it in the logs.

---

### 6. GNSS lever-arm double-compensation is not a live risk in the current config

**Files**:
- `GLIM/glim/config/config_ros.json` (`extension_modules` list)
- `GLIM/glim/config/config_odometry_ins.json` (`urdf_ins_frame`)
- `GLIM/glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp:361-368`

**What it looks like**: the GNSS extension subtracts `R_world_imu * t_imu_gnss`
from the reported GNSS position. Atlas already outputs its INS **position** at
the antenna phase centre, so if the software also applied a nonzero antenna→body
lever arm, both would compensate, biasing the GNSS prior. (The IMU stream being
body-axis-rotated but device-located — FusionEngine Spec §3.4.1 — is a separate
matter; `gnss_global` touches the position, not the IMU.)

**Why it's actually fine on AV-24 today**:
- **[UPDATED 2026-07-10]** `libgnss_global.so` is now **ENABLED** (the P5
  dual-antenna heading work loads it). The double-compensation stays avoided
  for a different reason: `enable_lever_arm: false` in
  `config_gnss_global.json` — the subtract site exists and is loaded but is
  explicitly switched off because Atlas firmware already reports the INS
  position with its own lever-arm handling. If anyone enables software
  lever-arm compensation, THIS is the double-compensation hazard to re-check.
- The INS-driven odometry frontend (`libodometry_estimation_ins.so`) computes
  its own `T_imu_ins` from URDF but resolves to identity in the current
  config: `urdf_imu_frame: "gps_antenna_top"` and `urdf_ins_frame: "gps_antenna_top"`.
- The localization node (`gicp_localization`) does not apply a GNSS
  lever-arm correction at all. As of the all-P1 single-source design, it
  consumes `/gps_p1/filtered_odom` (Atlas INS, `child_frame_id="gps_antenna_top"`)
  as-is. The lever arm is handled inside Atlas firmware (chassis IMU → antenna
  phase centre). `base_frame`, `imu_frame`, and the gt_odom source are all
  at `gps_antenna_top`, so the in-code TF lookups (`baselink2imu_T`,
  `T_base_gtbody_`) resolve to identity. See
  `gicp_localization/docs/GICP_GNSS_IMU_bug_report.pdf` for the historical
  architectural alternatives that were considered.

**Watch condition**: if `libgnss_global.so` is uncommented, or
`urdf_ins_frame` is changed to a different link than `urdf_imu_frame`,
verify Atlas's firmware projection contract before merging — software
compensation must only be on when the firmware is off, and vice versa.
Setting `urdf_gnss_frame: ""` in `config_gnss_global.json` hard-disables
the software side even if the extension is re-enabled.

---

### 7. RTK gating is per-consumer, not at GT ingestion

**File**: `gicp_localization/src/localization.cc:3359` (`callbackGtOdom`);
gate `gtSampleIsRtkFixed` at `:3487`; consumers `tryRtkCalibrationStep`
(`:3588`) and `maybeSnapPoseToGT` (`:3731`).

**What it looks like**: `callbackGtOdom` ingests **every** `/gps_p1/filtered_odom`
message and pushes it into the buffer with **no** RTK / fix-status check at
ingestion — it only carries the Atlas-reported position covariance
(`cov_pos_xx/yy/zz`) onto the buffered sample. A reviewer skimming the callback
might flag this as a missing guard and assume snap-back can fire from a degraded
GNSS fix. That is by design; the gate moved to the consumers.

**The gate runs per-consumer** (`gtSampleIsRtkFixed`, a covariance check on
`msg->pose.covariance[0,7,14]` vs `localization/rtk_gate/max_pose_var_xy`
default 0.25 m^2 and `.../max_pose_var_z` default 1.0 m^2):
- **Bias calibration / seed** (`tryRtkCalibrationStep`) → **requires RTK-FIXED**.
- **GT divergence cross-check** (`gt_pos_err` diagnostic) → **requires RTK-FIXED**.
- **Snap recovery** (`maybeSnapPoseToGT`) → **intentionally accepts ANY-quality
  Atlas sample** (no `gtSampleIsRtkFixed` call on this path).

The rationale (documented at `localization.cc:3372-3382` and root
`README.md` "Recovery during GICP failures"): Atlas FusionEngine already runs a
coupled GNSS+IMU INS with calibrated sensors, so during RTK loss its degraded
pose is still a better truth source than the node's own software IMU
dead-reckoning when GICP has failed to match the scan. This is the all-P1
replacement for the legacy NovAtel BESTGNSSPOS enum gate — no separate status
topic, no extra subscription. Reference covariances from a known-RTK-fixed AV-24
bag: median cov_xx ≈ 2.8e-5 m^2; RTK-float 1e-2…1e-1 m^2; GPS-only ≥ 1 m^2. The
0.1 s `gt_odom/max_dt` window still applies to all lookups, so a stalled upstream
publisher cleanly defers snap (logged `deferring snap — no GT sample within
max_dt…`) rather than firing on stale data.

**Watch condition**: because snap recovery is deliberately *not* RTK-gated, the
joint failure mode to keep in mind is a low-feature LiDAR stretch coinciding with
an RTK degradation — GICP can't recover geometrically and the snap pulls toward a
degraded GNSS fix. To make snap strictly RTK-gated, raise
`gt_recovery/min_consecutive_failures` or set `gt_recovery/enable=false`.

---

### 8. Things agent reviewers have hallucinated

These were flagged by previous agent runs but **do not exist in the code**.
If you find yourself about to flag one of these, double-check first.

- **"`swapSourceAndTarget` clears the IMU buffer."** False. `swapSourceAndTarget`
  is in `nano_gicp.cc:96-104`; it swaps PCL source/target clouds and clears
  `correspondences_` / `sq_distances_`. It has no relationship to the
  localization node's `imu_buffer`.

- **"`maybeSnapPoseToGT` asserts that `pose_mutex` is held by the caller."**
  False. There is no such assertion in the function (`localization.cc:2404+`).
  The call graph happens to be safe today because the caller is on the LiDAR
  thread, but no compile-time or runtime guard enforces this — flag the
  *real* concern (no explicit lock contract) rather than a fabricated assert.

- **"`Isotropic::Information(asDiagonal())` causes a dimension mismatch /
  silent isotropic noise model."** False — see #1 above.

- **"Aux LiDAR `time_field` precision loss for FLOAT32 is dramatic."** Reality:
  the `+= static_cast<float>(dt)` call works correctly when both `val` and
  `dt` are in seconds (Velodyne convention). The genuine concern is that the
  unit semantic of FLOAT32 time fields is sensor-dependent and undocumented;
  describe that, not a fabricated precision-loss number.

---

## Real issues that are intentional tradeoffs

### A. `localization.yaml` defaults assume default sensors

`use_sim_time: true`, the Luminar primary sensor type, the multi-LiDAR concat
list, and the GT recovery defaults are all calibrated for the AV-24
Cybertruck rig running rosbag-style replay or live with `/clock`. Other rigs
should override via launch args, not by editing the YAML.

### B. The "race-day workaround" commits are a system, not isolated patches

`4a594a9` (slide-along-unconstrained-axis catcher), `83b48a4` (GT-driven
recovery), `d3ec851` (rejection hardening), and `4b74d30` (live-checking
switch to GT) all interact through `performLocalization()` to produce a
robust-but-tangled rejection/recovery state machine. Don't review them in
isolation. A future refactor should consolidate them, but for now treat the
ensemble as load-bearing.

### C. Maps live outside the repo

`localization/map_path` defaults to `dlio_maps/may_4_putnam.pcd` (relative)
but no map is checked in. Production runs override the path via the
working tree or a launch arg pointing at external storage. The `dlio_maps/`
directory does not exist in tree by design.

---

## How to use this file

When reviewing this codebase:
1. Skim this file before flagging an issue.
2. If you find a candidate problem, verify the actual code state matches
   your description before reporting — agent runs in this repo have a
   recurring pattern of fabricated line numbers and inverted conditions.
3. If you confirm a previously-documented non-issue has become a real
   issue (e.g., URDF rpy added per #5 watch-condition), update this file
   in the same commit as the fix.

## Imported Claude Cowork project instructions

Improve the GICP localization algorithm by integrating with a race car platform and two integrated GNSS systems, one for NovAtel, one for vectorNav

### Current scope clarification: single-source Point One Atlas is intentional

The imported objective above mentions two integrated GNSS systems
(NovAtel + VectorNav), but the current `art-jazzy` implementation deliberately
narrowed the localization runtime to the Point One Atlas (`/gps_p1/*`) path.
This is not an accidental omission: the safe race-day design keeps IMU, GT
odom, RTK quality, `base_frame`, and `imu_frame` aligned at `gps_antenna_top`
(Atlas's firmware-projected output point), then gates `/gps_p1/filtered_odom`
on Atlas's own pose covariance before using it for init, cross-check,
calibration, or snap recovery. The NovAtel and VectorNav subscriptions, RTK
status topic, and `novatel_oem7_msgs` dependency were removed.

Do not review the absence of NovAtel or VectorNav fusion as a bug in the
current branch. Treat multi-vendor GNSS integration as future scope requiring
an explicit design for source selection/voting, frame targets, fix-status
semantics, and failure fallbacks before it is enabled in
`gicp_localization`.
