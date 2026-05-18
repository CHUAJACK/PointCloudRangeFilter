// gz_depth_republisher_node.cpp
// ─────────────────────────────
// Subscribes to a Gazebo gz.msgs.Image depth topic and a gz.msgs.CameraInfo
// topic, then projects each depth pixel through the camera intrinsics to
// produce a sensor_msgs/PointCloud2 suitable for octomap_server.
//
// Ghost-particle removal
// ──────────────────────
//   • Pixels equal to 0.0  → no-return / invalid (Gazebo default fill)
//   • Pixels that are NaN or Inf → beyond sensor range
//   • Pixels where depth < min_range or depth > max_range → out of bounds
//
// Supported pixel formats (gz::msgs::PixelFormatType)
// ────────────────────────────────────────────────────
//   FLOAT32   – 4 bytes/pixel, depth in metres   (most common in Gazebo)
//   FLOAT64   – 8 bytes/pixel, depth in metres
//   L_INT16   – 2 bytes/pixel, depth in millimetres (some sensors)
//
// Parameters
// ──────────
//   gz_image_topic    (string)  gz Image topic         default: /depth_camera/depth_image
//   gz_info_topic     (string)  gz CameraInfo topic    default: /depth_camera/camera_info
//   ros_topic         (string)  ROS 2 PointCloud2      default: /depth_camera/points_filtered
//   min_range         (double)  metres                 default: 0.3
//   max_range         (double)  metres                 default: 10.0
//   frame_id          (string)  header frame_id        default: camera_depth_optical_frame
#include "gz_depth_republisher/gz_depth_republisher_node.hpp"
#include <atomic>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>

// ── pixel helpers ──────────────────────────────────────────────────────────────

namespace
{

/// Read one depth value (in metres) from a raw pixel pointer.
/// Returns NaN for unrecognised formats so the NaN filter discards the point.
inline double read_depth(const uint8_t * px, int pixel_format)
{
  // gz::msgs::PixelFormatType enum values:
  //   FLOAT32 = 16   FLOAT64 = 17   L_INT16 = 6 (depth in mm)
  switch (pixel_format) {
    case 16: {  // FLOAT32 — most common Gazebo depth format
      float v;
      std::memcpy(&v, px, sizeof(float));
      return static_cast<double>(v);
    }
    case 17: {  // FLOAT64
      double v;
      std::memcpy(&v, px, sizeof(double));
      return v;
    }
    case 6: {   // L_INT16 — depth in millimetres
      uint16_t v;
      std::memcpy(&v, px, sizeof(uint16_t));
      return static_cast<double>(v) * 1e-3;
    }
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

/// Byte size per pixel for supported formats.
inline int pixel_step(int pixel_format)
{
  switch (pixel_format) {
    case 16: return 4;   // FLOAT32
    case 17: return 8;   // FLOAT64
    case 6:  return 2;   // L_INT16
    default: return 4;
  }
}

/// Write a little-endian float32 into a byte buffer at the given position.
inline void write_float32(std::vector<uint8_t> & buf, float v)
{
  uint8_t tmp[4];
  std::memcpy(tmp, &v, 4);
  buf.insert(buf.end(), tmp, tmp + 4);
}

} // namespace


// ── node ──────────────────────────────────────────────────────────────────────

class GzDepthRepublisher : public rclcpp::Node
{
public:
  explicit GzDepthRepublisher(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("gz_depth_republisher", opts)
  {
    // ── parameters ───────────────────────────────────────────────────────────
    declare_parameter<std::string>("gz_image_topic", "/depth_camera/depth_image");
    declare_parameter<std::string>("gz_info_topic",  "/depth_camera/camera_info");
    declare_parameter<std::string>("ros_topic",      "/depth_camera/points_filtered");
    declare_parameter<double>("min_range", 0.3);
    declare_parameter<double>("max_range", 10.0);
    declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");

    gz_image_topic_ = get_parameter("gz_image_topic").as_string();
    gz_info_topic_  = get_parameter("gz_info_topic").as_string();
    ros_topic_      = get_parameter("ros_topic").as_string();
    min_range_      = get_parameter("min_range").as_double();
    max_range_      = get_parameter("max_range").as_double();
    frame_id_       = get_parameter("frame_id").as_string();

    RCLCPP_INFO(get_logger(),
      "\n  gz_image_topic : %s"
      "\n  gz_info_topic  : %s"
      "\n  ros_topic      : %s"
      "\n  min_range      : %.3f m"
      "\n  max_range      : %.3f m"
      "\n  frame_id       : %s",
      gz_image_topic_.c_str(), gz_info_topic_.c_str(), ros_topic_.c_str(),
      min_range_, max_range_, frame_id_.c_str());

    // ── ROS 2 publisher ──────────────────────────────────────────────────────
    rclcpp::QoS qos(5);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ros_topic_, qos);

    // ── gz-transport subscribers ─────────────────────────────────────────────
    // CameraInfo first — we need intrinsics before we can project any image.
    std::function<void(const gz::msgs::CameraInfo &)> info_cb =
      std::bind(&GzDepthRepublisher::info_callback, this, std::placeholders::_1);

    std::function<void(const gz::msgs::Image &)> img_cb =
      std::bind(&GzDepthRepublisher::image_callback, this, std::placeholders::_1);

    if (!gz_node_.Subscribe(gz_info_topic_, info_cb)) {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe to gz CameraInfo topic '%s'",
        gz_info_topic_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Subscribed to gz CameraInfo topic '%s'",
        gz_info_topic_.c_str());
    }

    if (!gz_node_.Subscribe(gz_image_topic_, img_cb)) {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe to gz Image topic '%s'",
        gz_image_topic_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Subscribed to gz Image topic '%s'",
        gz_image_topic_.c_str());
    }
  }

private:
  // ── CameraInfo callback ──────────────────────────────────────────────────
  // Store intrinsics. Called infrequently (only when camera params change).
  void info_callback(const gz::msgs::CameraInfo & info)
  {
    // gz-msgs10 API:
    //   info.projection()   → CameraInfo_Projection  with repeated double p()
    //   info.intrinsics()   → CameraInfo_Intrinsics  with repeated double k()
    //
    // Projection matrix P is row-major 3×4:
    //   P = [ fx  0  cx  0 ]
    //       [  0  fy  cy  0 ]
    //       [  0   0   1  0 ]
    // K (intrinsic) is row-major 3×3:
    //   K = [ fx  0  cx ]
    //       [  0  fy  cy ]
    //       [  0   0   1 ]
    if (info.has_projection() && info.projection().p_size() >= 12) {
      std::lock_guard<std::mutex> lk(intrinsics_mutex_);
      fx_ = info.projection().p(0);
      fy_ = info.projection().p(5);
      cx_ = info.projection().p(2);
      cy_ = info.projection().p(6);
    } else if (info.has_intrinsics() && info.intrinsics().k_size() >= 9) {
      RCLCPP_WARN_ONCE(get_logger(),
        "CameraInfo projection matrix unavailable — using intrinsic matrix K.");
      std::lock_guard<std::mutex> lk(intrinsics_mutex_);
      fx_ = info.intrinsics().k(0);
      fy_ = info.intrinsics().k(4);
      cx_ = info.intrinsics().k(2);
      cy_ = info.intrinsics().k(5);
    } else {
      RCLCPP_ERROR_ONCE(get_logger(),
        "CameraInfo has no usable projection or intrinsic matrix. "
        "Check your Gazebo sensor configuration.");
      return;
    }

    intrinsics_ready_ = true;
    RCLCPP_INFO_ONCE(get_logger(),
      "Camera intrinsics received — fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
      fx_, fy_, cx_, cy_);
  }

  // ── Image callback ────────────────────────────────────────────────────────
  void image_callback(const gz::msgs::Image & img)
  {
    if (!intrinsics_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for CameraInfo on '%s' before projecting depth image.",
        gz_info_topic_.c_str());
      return;
    }

    // Snapshot intrinsics under lock
    double fx, fy, cx, cy;
    {
      std::lock_guard<std::mutex> lk(intrinsics_mutex_);
      fx = fx_;  fy = fy_;  cx = cx_;  cy = cy_;
    }

    const int    width  = static_cast<int>(img.width());
    const int    height = static_cast<int>(img.height());
    const int    fmt    = img.pixel_format_type();
    const int    pstep  = pixel_step(fmt);
    const auto * raw    = reinterpret_cast<const uint8_t *>(img.data().data());
    const size_t raw_sz = img.data().size();

    if (static_cast<size_t>(width * height * pstep) > raw_sz) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Depth image buffer too small (%zu bytes) for %dx%d px at %d B/px.",
        raw_sz, width, height, pstep);
      return;
    }

    const double min_sq = min_range_ * min_range_;
    const double max_sq = max_range_ * max_range_;

    // Output PointCloud2 is XYZ FLOAT32 — 12 bytes per point.
    // Pre-allocate worst case (every pixel valid).
    constexpr uint32_t OUT_POINT_STEP = 16u;  // x y z + 4-byte pad for alignment
    std::vector<uint8_t> out_data;
    out_data.reserve(static_cast<size_t>(width * height) * OUT_POINT_STEP);

    uint32_t n_kept = 0;

    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        const uint8_t * px = raw + (row * width + col) * pstep;
        const double depth = read_depth(px, fmt);

        // ── ghost / invalid checks ───────────────────────────────────────────

        // 1. NaN or Inf — beyond sensor range or no return
        if (!std::isfinite(depth)) { continue; }

        // 2. Zero — Gazebo fills no-return pixels with 0
        if (depth == 0.0) { continue; }

        // 3. Range bounds
        if (depth < min_range_ || depth > max_range_) { continue; }

        // ── back-project pixel → 3-D point (pinhole model) ──────────────────
        //   X = (col - cx) * depth / fx
        //   Y = (row - cy) * depth / fy
        //   Z = depth
        const float X = static_cast<float>((col - cx) * depth / fx);
        const float Y = static_cast<float>((row - cy) * depth / fy);
        const float Z = static_cast<float>(depth);

        // Extra distance check after projection (catches oblique near-field)
        const double dist_sq = static_cast<double>(X)*X +
                               static_cast<double>(Y)*Y +
                               static_cast<double>(Z)*Z;
        if (dist_sq < min_sq || dist_sq > max_sq) { continue; }

        write_float32(out_data, X);
        write_float32(out_data, Y);
        write_float32(out_data, Z);
        // 4-byte pad so each point is 16-byte aligned (SSE friendly)
        out_data.insert(out_data.end(), {0, 0, 0, 0});
        ++n_kept;
      }
    }

    // ── build PointCloud2 message ────────────────────────────────────────────
    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();

    out->header.frame_id = frame_id_;
    // gz-msgs10: Header::stamp() returns a single gz::msgs::Time (not repeated)
    if (img.has_header()) {
      out->header.stamp.sec     = static_cast<int32_t>(img.header().stamp().sec());
      out->header.stamp.nanosec = static_cast<uint32_t>(img.header().stamp().nsec());
    } else {
      out->header.stamp = now();
    }

    // Fields: x, y, z (each FLOAT32 at offsets 0, 4, 8)
    auto make_field = [](const std::string & name, uint32_t offset) {
      sensor_msgs::msg::PointField pf;
      pf.name     = name;
      pf.offset   = offset;
      pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
      pf.count    = 1;
      return pf;
    };
    out->fields = {make_field("x", 0), make_field("y", 4), make_field("z", 8)};

    out->point_step   = OUT_POINT_STEP;
    out->height       = 1;          // unorganised
    out->width        = n_kept;
    out->row_step     = OUT_POINT_STEP * n_kept;
    out->is_dense     = true;       // all invalid pixels removed
    out->is_bigendian = false;
    out->data         = std::move(out_data);

    pub_->publish(std::move(*out));

    ++frame_count_;
    if (frame_count_ % 50 == 0) {
      RCLCPP_DEBUG(get_logger(),
        "Frame %u: %d px in → %u pts out (%d dropped)",
        frame_count_, width * height, n_kept,
        width * height - static_cast<int>(n_kept));
    }
  }

  // ── members ─────────────────────────────────────────────────────────────────
  std::string gz_image_topic_;
  std::string gz_info_topic_;
  std::string ros_topic_;
  double      min_range_ {0.3};
  double      max_range_ {10.0};
  std::string frame_id_;

  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // Intrinsics (written by CameraInfo callback, read by Image callback)
  std::mutex   intrinsics_mutex_;
  double       fx_ {0.0}, fy_ {0.0}, cx_ {0.0}, cy_ {0.0};
  std::atomic<bool> intrinsics_ready_ {false};

  uint32_t frame_count_ {0};
};


// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzDepthRepublisher>());
  rclcpp::shutdown();
  return 0;
}
