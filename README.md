# DLIO++ — mapping and vehicle localization

This workspace contains GLIM for offline mapping and `gicp_interface` for
vehicle localization. `gicp_interface` replaces the old `gicp_localization`
package and uses the same setup as the standalone
[ART GICP repository](https://github.com/airacingtech/gicp_interface).

The Atlas `adapter` package and its `prep_bag.py` workflow have been removed.
Use the vehicle's existing VKS outputs or replay an existing ROS 2 bag directly.
There is no required normalization, bag preparation, or merge stage here.

## Packages

| Directory | ROS package | Purpose |
|---|---|---|
| `gicp_interface/` | `gicp_interface` | Current vehicle localizer and component |
| `GLIM/glim/` | `glim` | Mapping core |
| `GLIM/glim_ext/` | `glim_ext` | Mapping extensions, including GNSS constraints |
| `GLIM/glim_ros2/` | `glim_ros` | ROS 2 mapping and direct rosbag processing |
| `GICP_plusplus/` | `gicp_plusplus` | Retained older experimental localizer |
| `dlio/` | `dlio` | Metapackage for mapping and current localization |

`GICP_plusplus` remains an independent experimental package with its own
configuration. It is not launched by the current GICP setup.

## Build in the existing environment

Use ROS 2 Jazzy and the existing dependency workspace. Localization needs the
external `small_gicp` and `race_msgs` packages supplied by the vehicle workspace.
GLIM retains its existing GTSAM, gtsam_points, and optional CUDA/viewer
dependencies. No new container or system dependency upgrade is required by this
migration.

```bash
source /opt/ros/jazzy/setup.bash
# Source your existing dependency workspace's install/setup.bash.
# Run from this repository root:
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For a localization-only build in the same environment:

```bash
colcon build --symlink-install --packages-select gicp_interface
source install/setup.bash
```

If `small_gicp` is not exported by the dependency overlay, supply its existing
CMake package directory with
`--cmake-args -Dsmall_gicp_DIR=/absolute/path/to/lib/cmake/small_gicp`.
Use a fresh build/install directory when migrating from the old package layout;
previous install directories can still advertise deleted packages.

## Run localization

```bash
ros2 launch gicp_interface localize.launch.py \
  map_path:=/absolute/path/to/map.pcd
```

The default Laguna profile uses:

| Input/output | Contract |
|---|---|
| `/luminar_front/points` | `sensor_msgs/msg/PointCloud2`, frame `luminar_front` |
| `/vks/imu` | `sensor_msgs/msg/Imu`, frame `cg` |
| `/vks/filtered_odom` | `nav_msgs/msg/Odometry`, `map` → `cg`, initialization/recovery |
| `/vks/wheels` | `geometry_msgs/msg/TwistWithCovarianceStamped`, wheel feedback |
| `/localization/gicp/odom` | Accepted `nav_msgs/msg/Odometry`, `map` → `cg` |
| `/localization/gicp/navsatfix` | Equivalent `sensor_msgs/msg/NavSatFix` |

Inputs and product outputs use BEST_EFFORT reliability. The calibrated
`cg <- luminar_front` transform, local ENU datum, estimator settings, and
quality gates are in `gicp_interface/param/laguna.param.yaml`.
The map must use the same coordinates as the VKS reference. The Laguna datum
is `36.5869133,-121.7559026,231.9349051`, with zero `ins_offset`.
Do not apply these track coordinates to a different map.

The launch starts only the localizer. Keep the existing vehicle bring-up for
sensors, VKS, TF, and Iris component composition. The plugin remains
`gicp_localizer::GicpLocalizer`; the installed profile alias
`race_common_laguna_baseline.param.yaml` remains available.

See [the package README](gicp_interface/README.md) for configuration precedence,
external Iris input, timing, and output semantics. It replaces the old
`localization_with_tf.launch.py` instructions; the new launch does not start
RViz or a robot-state publisher.

## Replay an existing bag directly

Run one bag player and the localizer in separate terminals with the same
workspace sourced and the same `ROS_DOMAIN_ID`:

```bash
ros2 bag play /absolute/path/to/existing_bag --clock
```

```bash
ros2 launch gicp_interface localize.launch.py \
  map_path:=/absolute/path/to/map.pcd use_sim_time:=true
```

The bag must already contain the selected PointCloud2 and VKS input streams.
For external merged Iris clouds, add `lidar_concat_enabled:=true`; this selects
`/lidar/points`. It does not create or merge a bag.

Check delivery and outputs:

```bash
ros2 topic info /luminar_front/points --verbose
ros2 topic info /vks/imu --verbose
ros2 topic echo --once --qos-reliability best_effort /localization/gicp/odom
```

## Build a map from an existing bag

GLIM reads the bag directly with `glim_rosbag`; do not start a second ROS bag
player for this route. The existing configuration generator supports VKS
odometry without an adapter:

```bash
python3 scripts/generate_glim_mapping_config.py --help
```

Its topic defaults are `/vks/imu`, `/vks/filtered_odom` (Odometry), and
`/luminar_front/points`. Supply the actual `T_lidar_imu` calibration and unique
configuration/output directories for the run. Then:

```bash
ros2 run glim_ros glim_rosbag /absolute/path/to/existing_bag --ros-args \
  -p config_path:=/absolute/path/to/generated/config \
  -p dump_path:=/absolute/path/to/output/dump
```

Export the dump using the input VKS datum:

```bash
python3 scripts/export_glim_dump_to_pcd.py \
  /absolute/path/to/output/dump /absolute/path/to/map.pcd \
  --frame enu --enu-origin "LATITUDE,LONGITUDE,ALTITUDE"
```

[GLIM setup](GLIM/README.md) explains the calibration direction, configuration,
and coordinate requirements. Mapping and online localization share the input
coordinate contract but have separate estimator configurations.

## Migration notes

- Build/run `gicp_interface`, not the removed `gicp_localization` or `adapter`.
- Replace old `/gps_p1/*` setup with the existing VKS topics for the current
  Laguna localizer. Topic remapping alone cannot correct a different frame or
  map datum.
- Consumers should use `/localization/gicp/odom` and
  `/localization/gicp/navsatfix`. The old debug, path, and UTM-mirror topics are
  not the current product interface.
- Historical reports under `profiling_logs/` and experimental-package documents
  describe their original runs, not the current vehicle setup.

## Licenses

See the individual package and third-party license files. GLIM, gtsam_points,
GTSAM, the GLIM extensions, and the vehicle packages retain their respective
licenses and attributions.
