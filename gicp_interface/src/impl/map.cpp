#include "gicp_interface/detail/localizer_utils.hpp"

#include "gicp_interface/deterministic_voxelgrid.hpp"

#include <algorithm>
#include <cctype>

bool gicp_localizer::GicpLocalizer::loadMap() {

  if (this->map_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Map path is empty! Please set localization/map_path parameter.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", this->map_path_.c_str());

  // [P3 FIX 2026-07-10] Map-provenance enforcement. The exporter writes
  // <map>.pcd.manifest.yaml recording the coordinate contract and ENU datum;
  // a map built with one origin used against an adapter configured for another
  // is numerically valid but silently incompatible. Support both the original
  // flat frame/enu_origin/points record and the structured
  // coordinate_contract/artifact record exported by the current map pipeline.
  // A missing manifest is accepted only when require_map_manifest is false.
  bool manifest_points_present = false;
  size_t manifest_points = 0;
  {
    const std::string manifest_path = this->map_path_ + ".manifest.yaml";
    std::ifstream mf(manifest_path);
    if (!mf.is_open()) {
      if (this->require_map_manifest_) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Required map manifest is missing at '%s'.",
            manifest_path.c_str());
        return false;
      }
    } else {
      std::string line, mf_frame, mf_world_frame, mf_coordinate_system,
          mf_origin, mf_points;
      std::vector<std::string> enu_origin_lla;
      bool reading_enu_origin_lla = false;
      const auto trim = [](std::string value) {
        const auto begin = value.find_first_not_of(" \t");
        if (begin == std::string::npos) return std::string{};
        const auto end = value.find_last_not_of(" \t");
        return value.substr(begin, end - begin + 1);
      };
      while (std::getline(mf, line)) {
        const std::string trimmed = trim(line);
        auto value_of = [&](const char* key) -> std::string {
          const std::string k(key);
          if (trimmed.rfind(k, 0) != 0) return "";
          std::string v = trim(trimmed.substr(k.size()));
          const size_t h = v.find('#');
          if (h != std::string::npos) v = v.substr(0, h);
          return trim(v);
        };

        if (trimmed.rfind("enu_origin_lla:", 0) == 0) {
          reading_enu_origin_lla = true;
          enu_origin_lla.clear();
          continue;
        }
        if (reading_enu_origin_lla) {
          if (trimmed.rfind("-", 0) == 0) {
            const std::string value = trim(trimmed.substr(1));
            if (!value.empty()) enu_origin_lla.push_back(value);
            if (enu_origin_lla.size() == 3) {
              mf_origin = enu_origin_lla[0] + "," + enu_origin_lla[1] +
                          "," + enu_origin_lla[2];
              reading_enu_origin_lla = false;
            }
            continue;
          }
          if (!trimmed.empty() && trimmed[0] != '#') {
            reading_enu_origin_lla = false;
          }
        }
        if (mf_frame.empty()) { const auto v = value_of("frame:"); if (!v.empty()) mf_frame = v; }
        if (mf_world_frame.empty()) { const auto v = value_of("world_frame:"); if (!v.empty()) mf_world_frame = v; }
        if (mf_coordinate_system.empty()) { const auto v = value_of("coordinate_system:"); if (!v.empty()) mf_coordinate_system = v; }
        if (mf_origin.empty()) { const auto v = value_of("enu_origin:"); if (!v.empty()) mf_origin = v; }
        if (mf_points.empty()) { const auto v = value_of("points:"); if (!v.empty()) mf_points = v; }
        if (mf_points.empty()) { const auto v = value_of("point_count:"); if (!v.empty()) mf_points = v; }
      }
      if (!mf_points.empty()) {
        try {
          size_t parsed = 0;
          const auto value = std::stoull(mf_points, &parsed);
          if (parsed != mf_points.size() || value == 0) {
            throw std::invalid_argument("points must be a positive integer");
          }
          manifest_points = static_cast<size_t>(value);
          manifest_points_present = true;
        } catch (const std::exception& e) {
          RCLCPP_FATAL(
              this->get_logger(),
              "Map manifest points field is invalid ('%s'): %s",
              mf_points.c_str(), e.what());
          return false;
        }
      }
      const bool legacy_coordinate_contract = !mf_frame.empty();
      const bool structured_coordinate_contract =
          !mf_world_frame.empty() && !mf_coordinate_system.empty();
      if (this->require_map_manifest_ &&
          ((!legacy_coordinate_contract && !structured_coordinate_contract) ||
           mf_origin.empty() ||
           mf_origin.rfind("UNSPECIFIED", 0) == 0 ||
           !manifest_points_present)) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Required map manifest is partial: frame='%s' world_frame='%s' "
            "coordinate_system='%s' origin='%s' points='%s'. A complete ENU "
            "provenance record is mandatory.",
            mf_frame.c_str(), mf_world_frame.c_str(),
            mf_coordinate_system.c_str(), mf_origin.c_str(),
            mf_points.c_str());
        return false;
      }
      if (legacy_coordinate_contract && mf_frame != "enu") {
        RCLCPP_FATAL(this->get_logger(),
                     "Map manifest declares frame='%s' (not 'enu'): this map is NOT compatible "
                     "with the ENU reference poses this node consumes directly. Re-export with "
                     "--frame enu.", mf_frame.c_str());
        return false;
      }
      if (!legacy_coordinate_contract && structured_coordinate_contract) {
        std::string coordinate_system = mf_coordinate_system;
        std::transform(coordinate_system.begin(), coordinate_system.end(),
                       coordinate_system.begin(), [](unsigned char value) {
                         return static_cast<char>(std::tolower(value));
                       });
        if (mf_world_frame != "map" ||
            coordinate_system.find("enu") == std::string::npos) {
          RCLCPP_FATAL(
              this->get_logger(),
              "Map manifest coordinate_contract is not local ENU map: "
              "world_frame='%s' coordinate_system='%s'.",
              mf_world_frame.c_str(), mf_coordinate_system.c_str());
          return false;
        }
      }
      if (!this->navsat_origin_valid_ && !mf_origin.empty() &&
          mf_origin.rfind("UNSPECIFIED", 0) != 0) {
        std::string normalized = mf_origin;
        std::replace(normalized.begin(), normalized.end(), ',', ' ');
        std::istringstream stream(normalized);
        navsatfix::EnuOrigin manifest_origin;
        std::string extra;
        if ((stream >> manifest_origin.latitude_deg >>
             manifest_origin.longitude_deg >> manifest_origin.altitude_m) &&
            !(stream >> extra) && manifest_origin.latitude_deg >= -90.0 &&
            manifest_origin.latitude_deg <= 90.0 &&
            manifest_origin.longitude_deg >= -180.0 &&
            manifest_origin.longitude_deg <= 180.0 &&
            std::isfinite(manifest_origin.latitude_deg) &&
            std::isfinite(manifest_origin.longitude_deg) &&
            std::isfinite(manifest_origin.altitude_m)) {
          this->navsat_origin_ = manifest_origin;
          this->navsat_origin_valid_ = true;
          RCLCPP_INFO(this->get_logger(),
                      "Using ENU datum from map manifest for NavSatFix: %s",
                      mf_origin.c_str());
        }
      }
      std::string expected_origin;
      this->get_parameter("localization/expected_enu_origin", expected_origin);
      if (this->require_map_manifest_ && expected_origin.empty()) {
        RCLCPP_FATAL(
            this->get_logger(),
            "localization/require_map_manifest=true also requires "
            "localization/expected_enu_origin; refusing an unverified datum.");
        return false;
      }
      if (!expected_origin.empty()) {
        if (mf_origin.empty() || mf_origin.rfind("UNSPECIFIED", 0) == 0) {
          if (this->require_map_manifest_) {
            RCLCPP_FATAL(
                this->get_logger(),
                "Required map manifest has no usable ENU datum.");
            return false;
          }
          RCLCPP_WARN(this->get_logger(),
                      "localization/expected_enu_origin is set but the map manifest carries no "
                      "datum — origin compatibility CANNOT be verified.");
        } else {
          // [P2 FIX 2026-07-14] Compare the datum NUMERICALLY with tolerance,
          // not by raw string: a raw compare false-rejects on whitespace /
          // precision differences (the exporter now canonicalizes to 8/3 dp,
          // but a hand-set expected_enu_origin need not match byte-for-byte) and
          // says nothing meaningful about how far apart two datums actually are.
          auto parse_origin = [](const std::string& s, double& lat, double& lon, double& alt) {
            std::string norm = s;
            std::replace(norm.begin(), norm.end(), ',', ' ');
            std::istringstream ss(norm);
            return static_cast<bool>(ss >> lat >> lon >> alt);
          };
          double mlat, mlon, malt, elat, elon, ealt;
          const bool mf_ok = parse_origin(mf_origin, mlat, mlon, malt);
          const bool ex_ok = parse_origin(expected_origin, elat, elon, ealt);
          if (!mf_ok || !ex_ok) {
            if (this->require_map_manifest_) {
              RCLCPP_FATAL(this->get_logger(),
                           "ENU datum unparsable (manifest='%s' expected='%s'); expected "
                           "'lat_deg,lon_deg,alt_m'. Aborting.",
                           mf_origin.c_str(), expected_origin.c_str());
              return false;
            }
            RCLCPP_WARN(
                this->get_logger(),
                "Ignoring unparsable optional map-manifest ENU datum '%s'; "
                "using explicit localization/expected_enu_origin='%s' for NavSatFix.",
                mf_origin.c_str(), expected_origin.c_str());
          } else {
            constexpr double kLatLonTolDeg = 1e-6;  // ~0.1 m at this latitude
            constexpr double kAltTolM = 1.0;
            if (std::abs(mlat - elat) > kLatLonTolDeg ||
                std::abs(mlon - elon) > kLatLonTolDeg ||
                std::abs(malt - ealt) > kAltTolM) {
              if (this->require_map_manifest_) {
                RCLCPP_FATAL(this->get_logger(),
                             "ENU datum mismatch: required map manifest origin '%s' != "
                             "expected '%s' (beyond tolerance). Aborting.",
                             mf_origin.c_str(), expected_origin.c_str());
                return false;
              }
              RCLCPP_WARN(
                  this->get_logger(),
                  "Optional map-manifest ENU origin '%s' != explicit expected origin '%s'; "
                  "using the explicit origin for NavSatFix. GICP odom is unchanged.",
                  mf_origin.c_str(), expected_origin.c_str());
            } else {
              RCLCPP_INFO(this->get_logger(),
                          "Map manifest origin matches explicit NavSatFix origin numerically "
                          "(manifest='%s' expected='%s')", mf_origin.c_str(),
                          expected_origin.c_str());
            }
          }
        }
      } else if (legacy_coordinate_contract || structured_coordinate_contract) {
        RCLCPP_INFO(this->get_logger(),
                    "Map manifest: frame=%s world_frame=%s origin=%s (set "
                    "localization/expected_enu_origin to enforce datum matching)",
                    mf_frame.empty() ? "(structured)" : mf_frame.c_str(),
                    mf_world_frame.empty() ? "(legacy)" : mf_world_frame.c_str(),
                    mf_origin.empty() ? "(none)" : mf_origin.c_str());
      }
    }
  }


  // Load PCD file
  if (pcl::io::loadPCDFile<PointType>(this->map_path_, *this->map_cloud) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", this->map_path_.c_str());
    return false;
  }

  if (this->map_cloud->points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Loaded map is empty!");
    return false;
  }
  if (manifest_points_present &&
      manifest_points != this->map_cloud->points.size()) {
    RCLCPP_FATAL(
        this->get_logger(),
        "Map/manifest pair is inconsistent: manifest points=%zu but loaded "
        "PCD points=%zu. Refusing a partially replaced artifact pair.",
        manifest_points, this->map_cloud->points.size());
    return false;
  }

  // Optional static map rotation to correct coordinate-frame differences from source map files.
  if (std::abs(this->map_roll_deg_) > 1e-6 ||
      std::abs(this->map_pitch_deg_) > 1e-6 ||
      std::abs(this->map_yaw_deg_) > 1e-6) {
    constexpr float kDeg2Rad = 0.017453292519943295f;
    const float roll = static_cast<float>(this->map_roll_deg_ * kDeg2Rad);
    const float pitch = static_cast<float>(this->map_pitch_deg_ * kDeg2Rad);
    const float yaw = static_cast<float>(this->map_yaw_deg_ * kDeg2Rad);

    Eigen::Affine3f map_tf = Eigen::Affine3f::Identity();
    map_tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*this->map_cloud, *this->map_cloud, map_tf.matrix());

    RCLCPP_INFO(this->get_logger(),
                "Applied map rotation [roll=%.2f, pitch=%.2f, yaw=%.2f] deg",
                this->map_roll_deg_, this->map_pitch_deg_, this->map_yaw_deg_);
  }

  RCLCPP_INFO(this->get_logger(), "Map loaded successfully with %lu points", this->map_cloud->points.size());

  // Downsample the GICP TARGET map (in place) before it becomes the kd-tree.
  // A dense map (e.g. a ~49M-point GLIM export) otherwise builds a multi-GB
  // kd-tree that exhausts RAM/swap and stalls registration for seconds. Do
  // not use pcl::VoxelGrid here: its dense bounding-box cell-count guard uses
  // a 32-bit product and silently leaves sparse, large-extent Laguna maps
  // unchanged ("integer indices would overflow"). small_gicp's sparse 64-bit
  // voxel keys depend on occupied points instead of the bounding-box volume.
  // Downsampling to ~0.3 m cuts the point count (and kd-tree memory) with
  // negligible accuracy impact at the matching 0.3 m scan voxel. The dense
  // cloud is released as soon as the filter swaps in the downsampled result.
  if (this->map_voxel_size_ > 0.0) {
    const size_t before = this->map_cloud->points.size();
    auto map_ds = gicp_localizer::deterministicVoxelgridSampling(
      *this->map_cloud, this->map_voxel_size_);
    if (map_ds->points.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "sparse map voxelization at %.3f m produced an empty map; "
                   "refusing to build a full-resolution target",
                   this->map_voxel_size_);
      return false;
    }
    this->map_cloud = map_ds;  // releases the dense cloud
    RCLCPP_INFO(this->get_logger(),
                "Downsampled GICP target map deterministically with sparse "
                "64-bit voxel keys: %zu -> %zu points (voxel=%.3f m)",
                before, this->map_cloud->points.size(), this->map_voxel_size_);
  }

  return true;
}

void gicp_localizer::GicpLocalizer::start() {
}
