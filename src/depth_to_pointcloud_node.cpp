#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>

class DepthToPointCloud : public rclcpp::Node
{
public:
  DepthToPointCloud()
  : Node("depth_to_pointcloud")
  {
    RCLCPP_INFO(get_logger(), "=== Constructor start ===");

    try {
      // ── Parameters ──────────────────────────────────────────────────────────
      this->declare_parameter<std::string>("gz_depth",       "/depth_camera");
      this->declare_parameter<std::string>("gz_camera_info", "/camera_info");
      this->declare_parameter<double>     ("null_range",     0.0);
      this->declare_parameter<std::string>("output_topic",   "/depth_camera_bridged/points");

      gz_depth_topic_       = this->get_parameter("gz_depth").as_string();
      gz_camera_info_topic_ = this->get_parameter("gz_camera_info").as_string();
      null_range_           = this->get_parameter("null_range").as_double();
      output_topic_         = this->get_parameter("output_topic").as_string();

      RCLCPP_INFO(get_logger(), "gz_depth       : %s", gz_depth_topic_.c_str());
      RCLCPP_INFO(get_logger(), "gz_camera_info : %s", gz_camera_info_topic_.c_str());
      RCLCPP_INFO(get_logger(), "null_range     : %.4f", null_range_);
      RCLCPP_INFO(get_logger(), "output_topic   : %s", output_topic_.c_str());

      // ── Publisher ────────────────────────────────────────────────────────────
      RCLCPP_INFO(get_logger(), "Creating publisher...");
      pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::QoS(10).reliable());
      RCLCPP_INFO(get_logger(), "Publisher created.");

      // ── Diagnostic timer ─────────────────────────────────────────────────────
      RCLCPP_INFO(get_logger(), "Creating diagnostic timer...");
      diag_timer_ = create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&DepthToPointCloud::diagCallback, this));
      RCLCPP_INFO(get_logger(), "Diagnostic timer created.");

      // ── gz-transport subscriptions ───────────────────────────────────────────
      RCLCPP_INFO(get_logger(), "Subscribing to gz topics...");

      bool ok_depth = gz_node_.Subscribe(gz_depth_topic_,
                        &DepthToPointCloud::onGzDepth, this);
      RCLCPP_INFO(get_logger(), "gz Subscribe '%s' -> %s",
        gz_depth_topic_.c_str(), ok_depth ? "OK" : "FAILED");

      bool ok_info = gz_node_.Subscribe(gz_camera_info_topic_,
                       &DepthToPointCloud::onGzCameraInfo, this);
      RCLCPP_INFO(get_logger(), "gz Subscribe '%s' -> %s",
        gz_camera_info_topic_.c_str(), ok_info ? "OK" : "FAILED");

      RCLCPP_INFO(get_logger(), "=== Constructor complete — node is running ===");

    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Exception in constructor: %s", e.what());
      throw;
    } catch (...) {
      RCLCPP_FATAL(get_logger(), "Unknown exception in constructor");
      throw;
    }
  }

private:
  void diagCallback()
  {
    RCLCPP_INFO(get_logger(),
      "Diag | camera_info: %s | depth: %s | clouds published: %lu | unknown_fmt: %lu",
      gz_info_received_  ? "OK" : "WAITING",
      gz_depth_received_ ? "OK" : "WAITING",
      cloud_count_.load(),
      unknown_format_count_.load());

    if (!gz_info_received_) {
      RCLCPP_WARN(get_logger(),
        "No camera_info on '%s'", gz_camera_info_topic_.c_str());
    }
    if (!gz_depth_received_) {
      RCLCPP_WARN(get_logger(),
        "No depth image on '%s'", gz_depth_topic_.c_str());
    }
    if (unknown_format_count_ > 0) {
      RCLCPP_WARN(get_logger(),
        "Received %lu depth frames with unrecognised pixel format — "
        "run `gz topic -i -t %s` and check the message type",
        unknown_format_count_.load(), gz_depth_topic_.c_str());
    }
  }

  // ── gz callbacks (gz internal thread — NO RCLCPP_* macros) ──────────────

  void onGzCameraInfo(const gz::msgs::CameraInfo & msg)
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    fx_ = msg.intrinsics().k(0);
    fy_ = msg.intrinsics().k(4);
    cx_ = msg.intrinsics().k(2);
    cy_ = msg.intrinsics().k(5);
    has_info_         = true;
    gz_info_received_ = true;
  }

  void onGzDepth(const gz::msgs::Image & msg)
  {
    gz_depth_received_ = true;

    double fx, fy, cx, cy;
    {
      std::lock_guard<std::mutex> lock(info_mutex_);
      if (!has_info_) { return; }
      fx = fx_;  fy = fy_;  cx = cx_;  cy = cy_;
    }

    const int width  = static_cast<int>(msg.width());
    const int height = static_cast<int>(msg.height());

    const bool is_float32 =
      (msg.pixel_format_type() == gz::msgs::PixelFormatType::R_FLOAT32);
    const bool is_uint16 =
      (msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT16);

    if (!is_float32 && !is_uint16) {
      ++unknown_format_count_;
      return;
    }

    const auto * raw = reinterpret_cast<const uint8_t *>(msg.data().data());

    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp.sec     = static_cast<int32_t>(msg.header().stamp().sec());
    cloud->header.stamp.nanosec = static_cast<uint32_t>(msg.header().stamp().nsec());
    cloud->header.frame_id      = "camera_link";

    cloud->height       = static_cast<uint32_t>(height);
    cloud->width        = static_cast<uint32_t>(width);
    cloud->is_dense     = false;
    cloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(*cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> ix(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(*cloud, "z");

    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col, ++ix, ++iy, ++iz) {
        float z = readDepth(raw, row, col, width, is_float32);

        if (!std::isfinite(z) || z <= 0.0f) {
          *ix = *iy = *iz = std::numeric_limits<float>::quiet_NaN();
          continue;
        }

        *ix = static_cast<float>((col - cx) * z / fx);
        *iy = static_cast<float>((row - cy) * z / fy);
        *iz = z;
      }
    }

    pointcloud_pub_->publish(std::move(cloud));
    ++cloud_count_;
  }

  static float readDepth(const uint8_t * raw, int row, int col,
                          int width, bool is_float32)
  {
    if (is_float32) {
      float v;
      std::memcpy(&v, raw + (row * width + col) * sizeof(float), sizeof(float));
      return v;
    }
    uint16_t v;
    std::memcpy(&v, raw + (row * width + col) * sizeof(uint16_t), sizeof(uint16_t));
    return (v == 0) ? std::numeric_limits<float>::quiet_NaN()
                    : static_cast<float>(v) * 1e-3f;
  }

  std::string gz_depth_topic_;
  std::string gz_camera_info_topic_;
  std::string output_topic_;
  double      null_range_;

  std::mutex info_mutex_;
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  bool   has_info_{false};

  std::atomic<bool>     gz_info_received_{false};
  std::atomic<bool>     gz_depth_received_{false};
  std::atomic<uint64_t> cloud_count_{0};
  std::atomic<uint64_t> unknown_format_count_{0};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr                                diag_timer_;

  gz::transport::Node gz_node_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<DepthToPointCloud>();

  RCLCPP_INFO(node->get_logger(), "Entering spin...");
  rclcpp::spin(node);
  RCLCPP_INFO(node->get_logger(), "Spin exited.");

  rclcpp::shutdown();
  return 0;
}