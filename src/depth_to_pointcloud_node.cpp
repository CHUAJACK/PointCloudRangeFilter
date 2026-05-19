#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// gz-transport / gz-msgs
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>

#include <limits>
#include <mutex>
#include <cmath>

class DepthToPointCloud : public rclcpp::Node
{
public:
  DepthToPointCloud()
  : Node("depth_to_pointcloud")
  {
    // ── Parameters ────────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("gz_depth",       "/depth_camera/depth_image");
    this->declare_parameter<std::string>("gz_camera_info", "/depth_camera/camera_info");
    this->declare_parameter<double>     ("null_range",     0.0);   // placeholder
    this->declare_parameter<std::string>("output_topic",   "/depth_camera_bridged/points");

    gz_depth_topic_       = this->get_parameter("gz_depth").as_string();
    gz_camera_info_topic_ = this->get_parameter("gz_camera_info").as_string();
    null_range_           = this->get_parameter("null_range").as_double();
    output_topic_         = this->get_parameter("output_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "gz_depth       : %s", gz_depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "gz_camera_info : %s", gz_camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "null_range     : %.4f (placeholder)", null_range_);
    RCLCPP_INFO(this->get_logger(), "output_topic   : %s", output_topic_.c_str());

    // ── ROS2 publisher ────────────────────────────────────────────────────────
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    // ── gz-transport subscribers ──────────────────────────────────────────────
    // Camera info: cache the latest intrinsics whenever they arrive
    if (!gz_node_.Subscribe(gz_camera_info_topic_,
          &DepthToPointCloud::onGzCameraInfo, this))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to subscribe to gz camera_info topic: %s", gz_camera_info_topic_.c_str());
    }

    // Depth image: convert + publish on every frame
    if (!gz_node_.Subscribe(gz_depth_topic_,
          &DepthToPointCloud::onGzDepth, this))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to subscribe to gz depth topic: %s", gz_depth_topic_.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "depth_to_pointcloud node started.");
  }

private:
  // ── gz-transport callbacks (called from gz thread) ────────────────────────

  void onGzCameraInfo(const gz::msgs::CameraInfo & msg)
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    // Intrinsic matrix K (row-major, 3x3)
    fx_ = msg.intrinsics().k(0);
    fy_ = msg.intrinsics().k(4);
    cx_ = msg.intrinsics().k(2);
    cy_ = msg.intrinsics().k(5);
    has_info_ = true;
  }

  void onGzDepth(const gz::msgs::Image & msg)
  {
    // We need intrinsics before we can project
    double fx, fy, cx, cy;
    {
      std::lock_guard<std::mutex> lock(info_mutex_);
      if (!has_info_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Waiting for camera_info ...");
        return;
      }
      fx = fx_;  fy = fy_;  cx = cx_;  cy = cy_;
    }

    const int width  = static_cast<int>(msg.width());
    const int height = static_cast<int>(msg.height());

    // gz::msgs::Image pixel formats relevant for depth:
    //   R_FLOAT32  → 4 bytes/pixel, metres
    //   L_INT16    → 2 bytes/pixel, millimetres  (less common)
    const bool is_float32 =
      (msg.pixel_format_type() == gz::msgs::PixelFormatType::R_FLOAT32);
    const bool is_uint16 =
      (msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT16);

    if (!is_float32 && !is_uint16) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Unsupported depth pixel format: %d", static_cast<int>(msg.pixel_format_type()));
      return;
    }

    const auto * raw = reinterpret_cast<const uint8_t *>(msg.data().data());

    // ── Build PointCloud2 ─────────────────────────────────────────────────
    auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

    // Header: use gz timestamp, keep gz frame_id if present
    cloud_msg->header.stamp.sec     = static_cast<int32_t>(msg.header().stamp().sec());
    cloud_msg->header.stamp.nanosec = static_cast<uint32_t>(msg.header().stamp().nsec());
    cloud_msg->header.frame_id      = "depth_camera_link";   // adjust to your tf frame

    cloud_msg->height       = static_cast<uint32_t>(height);
    cloud_msg->width        = static_cast<uint32_t>(width);
    cloud_msg->is_dense     = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");  // x, y, z as float32

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col, ++iter_x, ++iter_y, ++iter_z) {

        float z = getDepthMetres(raw, row, col, width, is_float32);

        if (!std::isfinite(z) || z <= 0.0f) {
          // TODO: apply null_range_ logic here when specified
          *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
          continue;
        }

        *iter_x = static_cast<float>((col - cx) * z / fx);
        *iter_y = static_cast<float>((row - cy) * z / fy);
        *iter_z = z;
      }
    }

    pointcloud_pub_->publish(std::move(cloud_msg));
  }

  // ── Helpers ────────────────────────────────────────────────────────────────

  static float getDepthMetres(const uint8_t * raw, int row, int col,
                               int width, bool is_float32)
  {
    if (is_float32) {
      // 4 bytes per pixel, value already in metres
      const float * fptr = reinterpret_cast<const float *>(raw);
      return fptr[row * width + col];
    } else {
      // 16-bit unsigned, millimetres
      const uint16_t * uptr = reinterpret_cast<const uint16_t *>(raw);
      uint16_t val = uptr[row * width + col];
      return (val == 0) ? std::numeric_limits<float>::quiet_NaN()
                        : static_cast<float>(val) * 1e-3f;
    }
  }

  // ── Members ────────────────────────────────────────────────────────────────
  std::string gz_depth_topic_;
  std::string gz_camera_info_topic_;
  std::string output_topic_;
  double      null_range_;

  // Cached intrinsics (written by gz thread, read by gz thread → mutex)
  std::mutex info_mutex_;
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  bool   has_info_{false};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  gz::transport::Node gz_node_;  // gz-transport node (lives in its own threads)
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthToPointCloud>());
  rclcpp::shutdown();
  return 0;
}