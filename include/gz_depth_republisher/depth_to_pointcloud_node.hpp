#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>

#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// Raw depth frame — holds only what the gz callback copies off the wire
// ---------------------------------------------------------------------------
struct DepthFrame
{
  std::vector<uint8_t> data;
  int      width{0};
  int      height{0};
  bool     is_float32{false};
  int32_t  stamp_sec{0};
  uint32_t stamp_nsec{0};
};

// ---------------------------------------------------------------------------
// Factory: pre-built PointCloud2 skeleton (fields/step set once)
// ---------------------------------------------------------------------------
static sensor_msgs::msg::PointCloud2::UniquePtr
makeCloudShell(const std::string & frame_id);

// ---------------------------------------------------------------------------
// Node declaration
// ---------------------------------------------------------------------------
class DepthToPointCloud : public rclcpp::Node
{
public:
  /// Declare parameters, create publisher, start gz subscriptions and publish thread.
  DepthToPointCloud();

  /// Signal the publish thread to stop and join it.
  ~DepthToPointCloud();

private:
  // ── gz callbacks ──────────────────────────────────────────────────────────

  /**
   * @brief Called on every CameraInfo message from gz-transport.
   *
   * Extracts focal lengths and principal point from the intrinsic matrix K
   * and stores them for use during projection.  Thread-safe via info_mutex_.
   */
  void onGzCameraInfo(const gz::msgs::CameraInfo & msg);

  /**
   * @brief Called on every depth Image message from gz-transport.
   *
   * Performs a raw memcpy of the pixel data into pending_ and wakes the
   * publish thread.  Accepts R_FLOAT32 and L_INT16 pixel formats; frames
   * with any other format are silently counted and dropped.
   * Thread-safe via frame_mutex_.
   */
  void onGzDepth(const gz::msgs::Image & msg);

  // ── Publish thread ────────────────────────────────────────────────────────

  /**
   * @brief Waits for a pending frame, converts it, and publishes the cloud.
   *
   * Uses a double-buffer (cloud_[0] / cloud_[1]) so that the rmw layer can
   * serialise one cloud while the pixel loop fills the other.
   * Runs on publish_thread_.
   */
  void publishLoop();

  // ── Conversion ────────────────────────────────────────────────────────────

  /**
   * @brief Project a DepthFrame into a pre-allocated PointCloud2.
   *
   * Supports both float32 (metres) and uint16 (millimetres → metres) depth.
   * Points whose depth falls in [min_z_, max_z_] or is non-finite are
   * written as NaN (i.e. treated as the "null range" / no-return zone).
   * An optional integer downsampling stride (downsample_) reduces output
   * resolution without resampling.
   *
   * Output layout (per point, 12 bytes):
   *   float x  — forward  (optical axis / depth value)
   *   float y  — left     (-u direction in camera frame)
   *   float z  — up       (-v direction in camera frame)
   *
   * @param cloud  Output message to fill (header, width, height, data).
   * @param f      Source depth frame (pixels + metadata).
   * @param fx     Horizontal focal length [px].
   * @param fy     Vertical   focal length [px].
   * @param cx     Principal point u-coordinate [px].
   * @param cy     Principal point v-coordinate [px].
   */
  void convertInto(sensor_msgs::msg::PointCloud2 & cloud,
                   const DepthFrame              & f,
                   float fx, float fy, float cx, float cy);

  // ── Parameters ────────────────────────────────────────────────────────────
  std::string gz_depth_topic_;        ///< gz-transport depth image topic
  std::string gz_camera_info_topic_;  ///< gz-transport camera-info topic
  std::string output_topic_;          ///< ROS 2 PointCloud2 publisher topic
  float       min_z_{0.0f};           ///< null-range lower bound [m]
  float       max_z_{1.0f};           ///< null-range upper bound [m]
  int         downsample_{1};         ///< pixel stride (1 = full resolution)
  int         strip_width_{10};
  float       danger_threshold_{0.5};

  // ── Camera intrinsics (written by onGzCameraInfo) ─────────────────────────
  std::mutex info_mutex_;
  double fx_{0.0}, fy_{0.0};   ///< focal lengths [px]
  double cx_{0.0}, cy_{0.0};   ///< principal point [px]
  bool   has_info_{false};      ///< true once the first CameraInfo arrives

  // ── Depth frame double-buffer (written by onGzDepth) ──────────────────────
  std::mutex              frame_mutex_;
  std::condition_variable frame_cv_;
  DepthFrame              pending_;      ///< staging area filled by gz callback
  bool                    has_pending_{false};
  bool                    shutdown_{false};

  // ── Double-buffer for output clouds ───────────────────────────────────────
  std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_[2];
  int active_slot_{0};   ///< index of the slot last published

  // ── Diagnostics ───────────────────────────────────────────────────────────
  std::atomic<uint64_t> cloud_count_{0};           ///< total clouds published
  std::atomic<uint64_t> unknown_format_count_{0};  ///< frames with unknown pixel format

  // ── ROS 2 / gz handles ────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  gz::transport::Node gz_node_;
  std::thread         publish_thread_;
};