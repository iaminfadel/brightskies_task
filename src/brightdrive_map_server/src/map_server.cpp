// brightdrive_map_server (ADR 0003, ticket #33): serves the saved map on
// /localization/map_cloud (transient-local, depth 1) in three modes:
//
//   full     the whole map.pcd once, latched -- the fallback and the default
//            for a degraded or unknown pose.
//   windowed tiles whose centres are within R_load of the current pose,
//            loaded on a background thread, merged and published only when
//            the set changes; eviction beyond R_evict (hysteresis).
//   auto     the full map first (breaks the map_server<->localization
//            startup loop), then the windowed set driven by /localization/pose
//            or, until the first topic pose, the seeded initial pose. A stale
//            topic pose falls back to the full map.
//
// The grid itself is not a serving knob: origin and tile size come from the
// builder's map_index.yaml (#32); everything else is a YAML parameter.
#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "brightdrive_map_server/pcd_io.hpp"
#include "brightdrive_map_server/tile_index.hpp"
#include "brightdrive_map_server/window.hpp"

namespace brightdrive::map_server {

namespace {
constexpr auto kPace = std::chrono::milliseconds(200);
}  // namespace

class MapServerNode : public rclcpp::Node {
 public:
  MapServerNode() : Node("map_server") {
    clock_ = get_clock();
    mode_ = declare_parameter<std::string>("serving.mode", "auto");
    map_dir_ = declare_parameter<std::string>("serving.map_dir", "/ws/maps/current/town05-v14-pg");
    pose_topic_ = declare_parameter<std::string>("serving.pose_topic", "/localization/pose");
    lidar_range_m_ = declare_parameter<double>("serving.lidar_range_m", 50.0);
    margin_m_ = declare_parameter<double>("serving.margin_m", 10.0);
    hysteresis_m_ = declare_parameter<double>("serving.hysteresis_m", 15.0);
    downsample_voxel_m_ = declare_parameter<double>("serving.downsample_voxel_m", 0.0);
    publish_rate_hz_ = declare_parameter<double>("serving.publish_rate_hz", 0.0);
    pose_stale_s_ = declare_parameter<double>("serving.pose_stale_s", 2.0);
    initial_pose_x_ = declare_parameter<double>("serving.initial_pose_x", 0.0);
    initial_pose_y_ = declare_parameter<double>("serving.initial_pose_y", 0.0);
    if (mode_ != "full" && mode_ != "windowed" && mode_ != "auto") {
      throw std::runtime_error("serving.mode must be full, windowed or auto, got " + mode_);
    }

    rclcpp::QoS latch(rclcpp::KeepLast(1));
    latch.reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/localization/map_cloud", latch);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          on_pose(*msg);
        });

    window_.load_radius_m = lidar_range_m_ + margin_m_;
    window_.evict_radius_m = window_.load_radius_m + hysteresis_m_;

    // Full mode needs only map.pcd; the grid is required to window. A bundle
    // installed without tiles/ therefore degrades to the full map instead of
    // failing at startup (ADR 0003's fallback promise).
    if (mode_ != "full") {
      try {
        index_ = load_tile_index(map_dir_ + "/map_index.yaml");
        RCLCPP_INFO(get_logger(), "tile grid %s: %zu tiles, %.1f m cells", index_->map_id.c_str(),
                    index_->tiles.size(), index_->tile_size_m);
        frame_ = index_->map_frame;
      } catch (const std::exception& error) {
        RCLCPP_WARN(get_logger(), "%s; serving the full map only", error.what());
        mode_ = "full";
      }
    }
    if (mode_ == "full") {
      frame_ = "map";
    }

    running_ = true;
    loader_ = std::thread([this] { load_loop(); });
    publish_timer_ =
        create_wall_timer(std::chrono::milliseconds(100), [this] { publish_if_due(); });
    RCLCPP_INFO(get_logger(), "serving.mode=%s from %s (R_load=%.0f m, R_evict=%.0f m)",
                mode_.c_str(), map_dir_.c_str(), window_.load_radius_m, window_.evict_radius_m);
  }

  ~MapServerNode() override {
    running_ = false;
    pace_cv_.notify_all();
    if (loader_.joinable()) {
      loader_.join();
    }
  }

 private:
  struct PoseSample {
    double x = 0.0;
    double y = 0.0;
    double stamp_s = 0.0;
  };

  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    const std::lock_guard<std::mutex> lock(pose_mutex_);
    topic_pose_ = {msg.pose.pose.position.x, msg.pose.pose.position.y,
                   stamp_seconds(msg.header.stamp)};
  }

  static double stamp_seconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  // The clock cached at startup: now() is non-const in Humble and this is
  // called from const helpers. Shared across both threads; Clock::now() is
  // safe concurrently in Humble.
  double now_seconds() const { return clock_->now().seconds(); }

  // The pose that drives the window, or nullopt for the full-map fallback.
  // The topic pose wins; in auto the seed stands in until the topic has
  // produced one. The seed is static and never goes stale; a topic pose does
  // (degraded pose -> fallback). A zero stamp is treated as fresh: it means
  // the publisher does not stamp, so there is nothing to judge staleness by.
  std::optional<PoseSample> window_pose() const {
    const std::lock_guard<std::mutex> lock(pose_mutex_);
    if (topic_pose_.has_value()) {
      const PoseSample& pose = *topic_pose_;
      if (pose_stale_s_ <= 0.0 || pose.stamp_s <= 0.0 ||
          now_seconds() - pose.stamp_s <= pose_stale_s_) {
        return pose;
      }
      return std::nullopt;
    }
    if (mode_ == "auto") {
      return PoseSample{initial_pose_x_, initial_pose_y_, 0.0};
    }
    return std::nullopt;
  }

  // One loader iteration: recompute the serving target and rebuild the
  // published cloud only when it changed.
  void serve_step() {
    // ADR 0003: auto publishes the full map first as the transient-local
    // fallback; the windowed set follows on the next step. The gate flips
    // only once the full map is actually published, so a failed first read
    // retries instead of silently skipping the fallback.
    if (mode_ == "auto" && !first_full_done_) {
      swap_full();
      first_full_done_ = true;
      return;
    }

    std::string target = mode_ == "full" ? "full" : "window";
    std::optional<PoseSample> pose;
    if (target == "window") {
      pose = window_pose();
      if (!pose) {
        target = "full";
      }
    }

    // served_kind_/served_set_ belong to the publish state; read them under
    // its mutex.
    std::string served_kind;
    std::set<std::string> served_set;
    {
      const std::lock_guard<std::mutex> lock(publish_mutex_);
      served_kind = served_kind_;
      served_set = served_set_;
    }
    if (target == served_kind) {
      if (target == "window") {
        std::set<std::string> next = serving_set(*index_, pose->x, pose->y, served_set, window_);
        if (next != served_set) {
          swap_window(std::move(next), *pose);
        }
      }
      return;
    }
    if (target == "window") {
      std::set<std::string> next = serving_set(*index_, pose->x, pose->y, {}, window_);
      swap_window(std::move(next), *pose);
    } else {
      swap_full();
    }
  }

  // Loads missing tiles nearest-first, merges, downsamples, publishes. A tile
  // that fails to read degrades the whole window to the full map: a silently
  // holed window would look like a localization problem downstream.
  void swap_window(std::set<std::string> next, const PoseSample& pose) {
    std::vector<const TileEntry*> missing;
    for (const auto& tile : index_->tiles) {
      const std::string key = cell_key(tile.ix, tile.iy);
      if (next.count(key) > 0 && cache_.count(key) == 0) {
        missing.push_back(&tile);
      }
    }
    std::sort(missing.begin(), missing.end(), [&](const TileEntry* a, const TileEntry* b) {
      double ax = 0.0;
      double ay = 0.0;
      double bx = 0.0;
      double by = 0.0;
      tile_center(*index_, a->ix, a->iy, ax, ay);
      tile_center(*index_, b->ix, b->iy, bx, by);
      return std::hypot(ax - pose.x, ay - pose.y) < std::hypot(bx - pose.x, by - pose.y);
    });
    try {
      for (const TileEntry* tile : missing) {
        cache_[cell_key(tile->ix, tile->iy)] = read_pcd_xyz(map_dir_ + "/" + tile->path).points;
      }
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "tile load failed (%s); falling back to the full map",
                   error.what());
      cache_.clear();
      swap_full();
      return;
    }

    // Eviction: tiles outside the serving set are beyond R_evict by
    // construction of serving_set(); drop them from the cache too.
    for (auto it = cache_.begin(); it != cache_.end();) {
      if (next.count(it->first) == 0) {
        it = cache_.erase(it);
      } else {
        ++it;
      }
    }

    std::vector<Eigen::Vector3f> merged;
    std::size_t tiles = 0;
    for (const auto& tile : index_->tiles) {
      const std::string key = cell_key(tile.ix, tile.iy);
      if (next.count(key) == 0) {
        continue;
      }
      const std::vector<Eigen::Vector3f>& points = cache_[key];
      merged.insert(merged.end(), points.begin(), points.end());
      ++tiles;
    }
    if (downsample_voxel_m_ > 0.0) {
      merged = voxel_downsample(merged, downsample_voxel_m_);
    }
    const std::size_t point_count = merged.size();
    swap_published("window", merged, &next);
    RCLCPP_INFO(get_logger(), "window: %zu tiles, %zu pts @ (%.1f, %.1f)", tiles, point_count,
                pose.x, pose.y);
  }

  void swap_full() {
    if (full_cloud_.empty()) {
      full_cloud_ = read_pcd_xyz(map_dir_ + "/map.pcd").points;
      RCLCPP_INFO(get_logger(), "loaded full map: %zu pts", full_cloud_.size());
    }
    swap_published("full", full_cloud_, nullptr);
  }

  void swap_published(const std::string& kind, const std::vector<Eigen::Vector3f>& points,
                      const std::set<std::string>* set) {
    const std::lock_guard<std::mutex> lock(publish_mutex_);
    published_ = points;
    served_kind_ = kind;
    served_set_ = set == nullptr ? std::set<std::string>() : *set;
    dirty_ = true;
  }

  void load_loop() {
    while (running_) {
      std::unique_lock<std::mutex> lock(pace_mutex_);
      pace_cv_.wait_for(lock, kPace);
      lock.unlock();
      if (!running_) {
        break;
      }
      try {
        serve_step();
      } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "serve step failed: %s", error.what());
      }
    }
  }

  void publish_if_due() {
    std::vector<Eigen::Vector3f> points;
    {
      const std::lock_guard<std::mutex> lock(publish_mutex_);
      const double period_s = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.0;
      const bool due = dirty_ || (period_s > 0.0 && now_seconds() - last_publish_s_ >= period_s);
      if (!due) {
        return;
      }
      points = published_;
      dirty_ = false;
      last_publish_s_ = now_seconds();
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(points.size());
    const std::pair<const char*, uint32_t> fields[] = {{"x", 0}, {"y", 4}, {"z", 8}};
    for (const auto& [name, offset] : fields) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      msg.fields.push_back(field);
    }
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
    msg.data.resize(points.size() * 12);
    if (!points.empty()) {
      memcpy(msg.data.data(), points.data(), points.size() * 12);
    }
    map_pub_->publish(msg);
  }

  // Parameters.
  rclcpp::Clock::SharedPtr clock_;
  std::string mode_;
  std::string map_dir_;
  std::string pose_topic_;
  double lidar_range_m_ = 50.0;
  double margin_m_ = 10.0;
  double hysteresis_m_ = 15.0;
  double downsample_voxel_m_ = 0.0;
  double publish_rate_hz_ = 0.0;
  double pose_stale_s_ = 2.0;
  double initial_pose_x_ = 0.0;
  double initial_pose_y_ = 0.0;
  WindowParams window_;

  std::optional<TileIndex> index_;
  std::string frame_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Pose state (ROS thread writes, loader thread reads).
  mutable std::mutex pose_mutex_;
  std::optional<PoseSample> topic_pose_;

  // Published-cloud state (loader thread writes, ROS thread reads).
  std::mutex publish_mutex_;
  std::vector<Eigen::Vector3f> published_;
  std::set<std::string> served_set_;
  std::string served_kind_;
  bool dirty_ = false;
  double last_publish_s_ = 0.0;

  // Loader-thread-private state.
  std::unordered_map<std::string, std::vector<Eigen::Vector3f>> cache_;
  std::vector<Eigen::Vector3f> full_cloud_;
  bool first_full_done_ = false;

  std::thread loader_;
  std::mutex pace_mutex_;
  std::condition_variable pace_cv_;
  std::atomic<bool> running_{false};
};

}  // namespace brightdrive::map_server

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<brightdrive::map_server::MapServerNode>());
  rclcpp::shutdown();
  return 0;
}
