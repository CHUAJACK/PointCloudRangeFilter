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
      this->declare_parameter<double>     ("null_range_min",     0.0);
      this->declare_parameter<double>     ("null_range_max",     1.0);
      this->declare_parameter<std::string>("output_topic",   "/depth_camera_bridged/points");

      gz_depth_topic_       = this->get_parameter("gz_depth").as_string();
      gz_camera_info_topic_ = this->get_parameter("gz_camera_info").as_string();
      null_range_min_           = this->get_parameter("null_range_min").as_double();
      null_range_max_           = this->get_parameter("null_range_max").as_double();
      output_topic_         = this->get_parameter("output_topic").as_string();
      min_z  = static_cast<float>(null_range_min_);
      max_z  = static_cast<float>(null_range_max_);

      RCLCPP_INFO(get_logger(), "gz_depth       : %s", gz_depth_topic_.c_str());
      RCLCPP_INFO(get_logger(), "gz_camera_info : %s", gz_camera_info_topic_.c_str());
      RCLCPP_INFO(get_logger(), "null_range_min     : %.4f", null_range_min_);
      RCLCPP_INFO(get_logger(), "null_range_max     : %.4f", null_range_max_);
      RCLCPP_INFO(get_logger(), "output_topic   : %s", output_topic_.c_str());

      // ── Publisher ────────────────────────────────────────────────────────────
      RCLCPP_INFO(get_logger(), "Creating publisher...");
      pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::QoS(10).reliable());
      RCLCPP_INFO(get_logger(), "Publisher created.");



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

    const bool is_float32 = (msg.pixel_format_type() == gz::msgs::PixelFormatType::R_FLOAT32);
    const bool is_uint16  = (msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT16);
    if (!is_float32 && !is_uint16) { ++unknown_format_count_; return; }

    const auto * raw = reinterpret_cast<const uint8_t *>(msg.data().data());

    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp.sec     = static_cast<int32_t>(msg.header().stamp().sec());
    cloud->header.stamp.nanosec = static_cast<uint32_t>(msg.header().stamp().nsec());
    cloud->header.frame_id      = "camera_link";
    cloud->height       = static_cast<uint32_t>(height);
    cloud->width        = static_cast<uint32_t>(width);
    cloud->is_dense     = false;
    cloud->is_bigendian = false;

    // ── Manual field descriptors — skips modifier overhead ───────────────────
    cloud->fields.resize(3);
    cloud->fields[0].name = "x"; cloud->fields[0].offset = 0;
    cloud->fields[1].name = "y"; cloud->fields[1].offset = 4;
    cloud->fields[2].name = "z"; cloud->fields[2].offset = 8;
    for (auto & f : cloud->fields) {
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count    = 1;
    }
    cloud->point_step = 12;                          // 3 × float32
    cloud->row_step   = cloud->point_step * width;
    cloud->data.resize(cloud->row_step * height);

    // ── Single raw output pointer — no iterator overhead ─────────────────────
    float * out = reinterpret_cast<float *>(cloud->data.data());

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inv_fx = static_cast<float>(1.0 / fx);
    const float inv_fy = static_cast<float>(1.0 / fy);
    const float cx_f   = static_cast<float>(cx);
    const float cy_f   = static_cast<float>(cy);

    if (is_float32) {
      const float * depth = reinterpret_cast<const float *>(raw);
      for (int row = 0; row < height; ++row) {
        const float row_cy = static_cast<float>(row) - cy_f;
        for (int col = 0; col < width; ++col) {
          float z = depth[row * width + col];
          float * p = out + (row * width + col) * 3;
          if (!std::isfinite(z) || (z >= min_z && z <= max_z)) {
            p[0] = p[1] = p[2] = nan;
          } else {
            p[0] = z;
            p[1] = -(static_cast<float>(col) - cx_f) * z * inv_fx;
            p[2] = -row_cy                            * z * inv_fy;
          }
        }
      }
    } else {
      const uint16_t * depth = reinterpret_cast<const uint16_t *>(raw);
      for (int row = 0; row < height; ++row) {
        const float row_cy = static_cast<float>(row) - cy_f;
        for (int col = 0; col < width; ++col) {
          uint16_t raw_val = depth[row * width + col];
          float * p = out + (row * width + col) * 3;
          if (raw_val == 0) { p[0] = p[1] = p[2] = nan; continue; }
          float z = static_cast<float>(raw_val) * 1e-3f;
          if (z >= min_z && z <= max_z) {
            p[0] = p[1] = p[2] = nan;
          } else {
            p[0] = z;
            p[1] = -(static_cast<float>(col) - cx_f) * z * inv_fx;
            p[2] = -row_cy                            * z * inv_fy;
          }
        }
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
  double      null_range_min_;
  double      null_range_max_;
  float min_z;
  float max_z;

  std::mutex info_mutex_;
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  bool   has_info_{false};

  std::atomic<bool>     gz_info_received_{false};
  std::atomic<bool>     gz_depth_received_{false};
  std::atomic<uint64_t> cloud_count_{0};
  std::atomic<uint64_t> unknown_format_count_{0};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;


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