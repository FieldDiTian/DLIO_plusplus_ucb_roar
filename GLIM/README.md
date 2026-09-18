# GLIM offline mapping

GLIM reads existing ROS 2 bags directly and exports maps for `gicp_interface`.
The current vehicle path uses VKS IMU/odometry in local ENU. It does not use the
removed Atlas adapter or a bag-preparation/merge command.

## Build

Build from the parent repository root with the existing ROS 2 Jazzy dependency
environment sourced:

```bash
colcon build --symlink-install --packages-up-to glim_ros glim_ext \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Dependencies remain GTSAM, gtsam_points, Eigen3, Boost, OpenMP, spdlog, libxml2,
and the configured CUDA/viewer/OpenCV support. Keep the existing installed
versions and build options. The packages are `glim`, `glim_ext`, and `glim_ros`.

## Configure for the existing sensor inputs

Run the repository's configuration generator from its root:

```bash
python3 scripts/generate_glim_mapping_config.py \
  --output-dir /absolute/path/to/run/config \
  --offload-dir /absolute/path/to/run/offload \
  --t-lidar-imu TX TY TZ QX QY QZ QW
```

Replace the seven calibration placeholders with the actual `T_lidar_imu`
translation and quaternion (x, y, z, w). This maps IMU-frame coordinates into
the primary LiDAR frame. For the Laguna GICP setup, IMU is `cg` and the primary
LiDAR is `luminar_front`, so this is the **inverse** of
`localization/base_lidar_transform` in
[`gicp_interface/param/laguna.param.yaml`](../gicp_interface/param/laguna.param.yaml).
Do not substitute an identity transform or reuse a different vehicle's URDF.

The generator defaults to the current input interfaces:

| Argument | Default |
|---|---|
| `--imu-topic` | `/vks/imu` |
| `--gnss-topic` | `/vks/filtered_odom` |
| `--gnss-msg-type` | `nav_msgs/msg/Odometry` |
| `--points-topic` | `/luminar_front/points` |
| `--primary-frame` | `luminar_front` |

Odometry must already use local ENU coordinates consistent with the selected
map datum. GLIM does not convert raw receiver coordinates to VKS coordinates.
The generated GNSS extension consumes the Odometry directly, so no separate
RTK-filtered bag or normalization node is required. Its covariance weighting,
alignment-quality checks, and mapping health gates remain in the generator.

The checked-in older `glim/config` examples retain their historical Atlas
settings. Use the generated directory above for the VKS setup rather than
editing installed configuration files. Run the generator with `--help` for
explicit topic, point-field, noise, calibration, and multi-LiDAR overrides.

## Process the bag directly

```bash
ros2 run glim_ros glim_rosbag /absolute/path/to/existing_bag --ros-args \
  -p config_path:=/absolute/path/to/run/config \
  -p dump_path:=/absolute/path/to/run/dump
```

This process reads the bag itself; do not launch a ROS bag player alongside it.
The input must contain the selected PointCloud2, Imu, and Odometry messages.
Their timestamps must share a time base. The generator uses front LiDAR only
unless auxiliary inputs and their calibrated extrinsics are explicitly supplied.

Keep unique configuration, offload, and dump directories per run. The generated
profile uses CPU LiDAR-inertial odometry, local/global mapping, and GNSS
constraints. The old INS-only frontend is a different optional configuration;
its requirements should not be applied to this route.

## Export a localization map

```bash
python3 scripts/export_glim_dump_to_pcd.py \
  /absolute/path/to/run/dump /absolute/path/to/map.pcd \
  --frame enu --enu-origin "LATITUDE,LONGITUDE,ALTITUDE"
```

Use the origin belonging to the input VKS local ENU coordinates. For Laguna's
current GICP profile this is `36.5869133,-121.7559026,231.9349051`; other tracks
require their actual origin. The exporter can also read an existing
`enu_origin.txt` from the dump, but direct bag processing does not create that
file automatically, so supplying `--enu-origin` is the explicit route.

The exporter uses `T_world_utm.txt` to transform GLIM's internal world map back
to the input reference frame. The filename is historical: with VKS local ENU
input, the target is ENU, not UTM. Export requires a valid alignment transform;
it should not be bypassed by relabeling world coordinates as `map`.

Load the exported PCD with the current localizer:

```bash
ros2 launch gicp_interface localize.launch.py \
  map_path:=/absolute/path/to/map.pcd
```

See [the root README](../README.md) for direct replay and vehicle bring-up.

## Retained mapping implementation

This fork retains Luminar per-point timestamp decoding, IMU deskew,
auxiliary-LiDAR support, CPU/GPU odometry backends, dense submaps, loop closure,
and the GNSS global-mapping extension. This package migration does not change
those mapping algorithms. The upstream component READMEs in `glim/`,
`glim_ext/`, and `glim_ros2/` remain available for implementation details.

## Credits

This workspace is based on:

- **GLIM** by Kenji Koide
  - Repository: https://github.com/koide3/glim
  - Paper: [Graph-based LiDAR-Inertial Mapping](https://staff.aist.go.jp/k.koide/assets/pdf/koide2024ral.pdf)

- **gtsam_points** by Kenji Koide
  - Repository: https://github.com/koide3/gtsam_points

- **GTSAM** by Georgia Tech
  - Repository: https://github.com/borglab/gtsam

## License

This workspace inherits licenses from its constituent packages:
- GLIM: MIT License
- gtsam_points: MIT License
- GTSAM: BSD License

See individual package directories for full license texts.


## Citation

If you use this work, please cite the original GLIM paper:

```bibtex
@article{koide2024glim,
  title={GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors},
  author={Koide, Kenji and Yokozuka, Masashi and Oishi, Shuji and Banno, Atsuhiko},
  journal={IEEE Robotics and Automation Letters},
  year={2024}
}
```
