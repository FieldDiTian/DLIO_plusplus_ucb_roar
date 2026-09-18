# gicp_interface

ROS 2 Jazzy LiDAR-to-map localization using external `small_gicp`. The package
provides one standalone node and the equivalent ROS 2 component. Vehicle
bring-up, LiDAR decoding/merging, map building, bag playback, and visualization
belong to the surrounding workspace.

## Build in the existing environment

Use the same ROS 2 Jazzy and dependency overlay as the vehicle workspace.
`small_gicp` and `race_msgs` must already be available there. No `.env` file,
new container, dependency upgrade, or middleware change is required.

```bash
source /opt/ros/jazzy/setup.bash
# Source your existing dependency workspace's install/setup.bash here.
colcon build --packages-select gicp_interface --symlink-install
source install/setup.bash
```

The default build type is Release; an explicit CMake build type is respected.
If the existing overlay does not export the `small_gicp` CMake package, pass
`--cmake-args -Dsmall_gicp_DIR=/absolute/path/to/lib/cmake/small_gicp`.

Dependencies remain C++17, ament, rclcpp/components, ROS sensor/geometry/nav and
race messages, TF2, PCL, Eigen3, OpenMP, libxml2, and external small_gicp. The
repository builds the runtime package; it does not ship a local test suite.

## Start

```bash
ros2 launch gicp_interface localize.launch.py \
  map_path:=/absolute/path/to/map.pcd
```

For rosbag playback, add `use_sim_time:=true` and publish `/clock` from the
player. The launch starts only GICP.

| Interface | Name |
|---|---|
| Package | `gicp_interface` |
| Executable / standalone node name | `gicp_interface_node` / `gicp_interface` |
| Component plugin | `gicp_localizer::GicpLocalizer` |
| Component library | `libgicp_interface_component.so` |
| Default vehicle profile | `param/laguna.param.yaml` |
| Installed vehicle compatibility alias | `param/race_common_laguna_baseline.param.yaml` |

Keep the existing vehicle launcher when running on the car. It owns component
composition, remappings, and `use_intra_process_comms`; this standalone launch
does not create an Iris container or select an RMW implementation.

## Configuration

The standalone launch loads parameters in this order; later values win:

1. `param/gicp_interface.param.yaml`: generic defaults.
2. `config_path`: defaults to the installed `param/laguna.param.yaml`.
3. `config_overlay_path`: optional deployment overrides.
4. Explicit launch overrides, including the required `map_path`.

Use `config_path:=/absolute/path/to/profile.yaml` for another vehicle/track.
An empty `config_path` selects only the generic defaults, which require suitable
frames, extrinsics, and initialization before localization can run.

Tuning arguments default to empty so the YAML remains authoritative. In
particular, `lidar_frame` and `imu_only` preserve the profile unless explicitly
supplied. `use_sim_time` defaults to false. Parameters are read at startup;
restart the node to apply configuration edits. Track-time tuning is a separate,
explicitly enabled mechanism.

```bash
ros2 launch gicp_interface localize.launch.py --show-args
```

For reusable tuning, prefer an overlay YAML to a long command line. Set the
actual reference-odometry input with `gt_odom_topic`; there is no separate
`odom` subscription or `odom/odom_frame` setting.

### Laguna settings

| Setting | Value |
|---|---|
| Output frames | `map` → `cg` |
| IMU / primary LiDAR frame | `cg` / `luminar_front` |
| ENU origin (latitude, longitude, altitude) | `36.5869133, -121.7559026, 231.9349051` |
| Reference translation (`ins_offset`) | `[0, 0, 0]` |
| Map / scan voxel size | 0.25 m / 0.75 m |
| Scan crop / maximum range | 140 m / 140 m |
| Scan point budget | 5,000, spatially balanced |
| Registration mode | `4dof`, full 6DoF every 10 frames |
| GICP workers / callback executor threads | 1 / 1, plus the scan worker |
| Normal and startup outer / inner iterations | 6 / 5 |
| Cooperative solve budget / finalization reserve | 100 ms / 30 ms |
| Hard result-age limit | 1,000 ms |
| Primary scan queue | 8 |
| Registration-rate limit | Disabled (`0`) |
| Internal LiDAR concatenation | Disabled |

The profile also supplies the calibrated `cg <- luminar_front` transform,
IMU settings, initialization/recovery, wheel feedback, and quality gates.
These values are explicit so the file can also be loaded directly by the
vehicle component launcher. Optional internal-merge settings remain for the
existing fallback; they are not needed for front-only operation.

### LiDAR input modes

The default input is `/luminar_front/points`. To consume the externally merged
Iris product:

```bash
ros2 launch gicp_interface localize.launch.py \
  map_path:=/absolute/path/to/map.pcd \
  lidar_concat_enabled:=true
```

This selects `/lidar/points`, defaults its frame to `base_link`, and disables
GICP's internal merge. Override `pointcloud_topic` and `lidar_frame` if the
publisher uses different names. The static front-LiDAR transform is bound to
`luminar_front`; other input frames need the corresponding TF/extrinsic path.

The retained internal merge requires both `lidar_concat_enabled:=true` and
`lidar_concat_source:=gicp_internal_paused`. `require_all_aux` applies to that
mode only. It cannot enforce completeness of a cloud merged by Iris.

## Map and coordinate requirements

Supply a readable, nonempty PCD in the same local ENU coordinate system as the
initial pose/reference odometry. The repository's existing
[Laguna map download](https://drive.google.com/file/d/1v3vX17M8j-TBPCUsUU0VVN9Nmnu_LDgb/view?usp=sharing)
is an external artifact; `map_path` selects the local file to load.

There is no map hash allowlist, checksum authorization, or startup fingerprint
pass. Sparse deterministic voxelization still produces the registration target.
Spatial hash tables used for map indexing are unrelated to authorization.

`localization/require_map_manifest` defaults to false, so a PCD does not require
a sidecar. If `<map>.pcd.manifest.yaml` exists, its coordinate and point-count
checks still apply. Enabling the requirement also requires a complete supported
manifest and matching explicit ENU origin. An explicit
`localization/expected_enu_origin` controls NavSatFix conversion; without an
explicit origin, a usable manifest may supply it. Without either, NavSatFix
publishes `STATUS_NO_FIX`. Setting a datum does not translate the map itself.

## Estimation and outputs

Each scan follows this path:

```text
PointCloud2 → timestamp/frame checks → crop and IMU deskew
           → voxelization and point budget → small_gicp scan-to-map solve
           → support, fitness, motion and degeneracy gates
           → delayed pose fusion → accepted Odometry + NavSatFix
```

GICP minimizes point residuals weighted by local point covariances. The Laguna
4DoF solve estimates translation and yaw while retaining IMU roll/pitch; the
periodic 6DoF solve allows all pose components to change.

The surrounding inertial filter tracks position, velocity, attitude, and gyro
and accelerometer biases. The current node sends accepted **x/y/yaw pose
measurements** to `DelayedGicpFusion`, using configured pose noise. It does not
feed the registration Hessian directly into that filter. The correction is
applied at measurement time and newer inertial/wheel history is replayed.

VKS reference odometry supplies initialization and recovery after the configured
rejection streak. Wheel velocity supplies optional motion feedback and a
stationary gate. Disabling these aids changes the estimator's operating
conditions; the car-tested profile keeps them enabled.

| Default input | ROS message type | Purpose |
|---|---|---|
| `/luminar_front/points` | `sensor_msgs/msg/PointCloud2` | LiDAR scan |
| `/vks/imu` | `sensor_msgs/msg/Imu` | Prediction, deskew, attitude prior |
| `/vks/filtered_odom` | `nav_msgs/msg/Odometry` | Initialization and recovery |
| `/vks/wheels` | `geometry_msgs/msg/TwistWithCovarianceStamped` | Wheel feedback and stationary gate |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Manual initialization/reset |

All input subscriptions request BEST_EFFORT. LiDAR uses VOLATILE, keep-last(5);
IMU uses VOLATILE, keep-last(2000). Optional trajectory commands retain
TRANSIENT_LOCAL durability. Inputs need meaningful timestamps and frames;
per-point acquisition time is needed for motion deskew.

| Output | ROS message type |
|---|---|
| `/localization/gicp/odom` | `nav_msgs/msg/Odometry` |
| `/localization/gicp/navsatfix` | `sensor_msgs/msg/NavSatFix` |

Outputs use BEST_EFFORT, VOLATILE, keep-last(100). Normal operation publishes
both only after an accepted scan. A bounded IMU motion bridge may advance the
pose timestamp by up to `localization/output/max_imu_delay_compensation_s`
(default 0.5 s). Rejected scans produce no new product frame. Explicit
`imu_only:=true` selects a diagnostic mode that bypasses GICP.

## Check data flow

```bash
ros2 topic info /luminar_front/points --verbose
ros2 topic info /vks/imu --verbose
ros2 topic info /localization/gicp/odom --verbose
ros2 topic echo --once --qos-reliability best_effort /localization/gicp/odom
```

For replay, verify `/clock` and a shared `ROS_DOMAIN_ID`. Measure accepted
frames, rejection reasons, `front_overload_dropped`, solve latency, and output
timestamps. A full scan queue drops the oldest queued scan; the 100 ms
cooperative optimizer budget excludes other pipeline work and does not promise
10 Hz output. Same-process composition avoids DDS on those edges, but scan
conversion, deskew, and registration still consume CPU.
