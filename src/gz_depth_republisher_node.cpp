#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthToPointCloud : public rclcpp::Node
{
public:
  DepthToPointCloud()
  : Node("depth_to_pointcloud")
  {
    // ── Parameters ────────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("gz_depth",       "/depth_camera/depth/image_raw");
    this->declare_parameter<std::string>("gz_camera_info", "/depth_camera/depth/camera_info");
    this->declare_parameter<double>     ("null_range",     0.0);   // placeholder

    gz_depth_topic_       = this->get_parameter("gz_depth").as_string();
    gz_camera_info_topic_ = this->get_parameter("gz_camera_info").as_string();
    null_range_           = this->get_parameter("null_range").as_double();

    RCLCPP_INFO(this->get_logger(), "gz_depth       : %s", gz_depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "gz_camera_info : %s", gz_camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "null_range     : %.4f (placeholder)", null_range_);

    // ── Publisher ─────────────────────────────────────────────────────────────
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/pointcloud", rclcpp::SensorDataQoS());

    // ── Synchronised subscribers (depth image + camera_info) ──────────────────
    depth_sub_.subscribe(this, gz_depth_topic_,
      rclcpp::SensorDataQoS().get_rmw_qos_profile());
    camera_info_sub_.subscribe(this, gz_camera_info_topic_,
      rclcpp::SensorDataQoS().get_rmw_qos_profile());

    sync_ = std::make_shared<Sync>(SyncPolicy(10), depth_sub_, camera_info_sub_);
    sync_->registerCallback(
      std::bind(&DepthToPointCloud::syncCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "depth_to_pointcloud node started.");
  }

private:
  // ── Types ──────────────────────────────────────────────────────────────────
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  // ── Callback ───────────────────────────────────────────────────────────────
  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr      & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    // Convert ROS image → OpenCV (32FC1 = float metres, 16UC1 = uint16 mm)
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat & depth_image = cv_ptr->image;

    // Retrieve intrinsics from CameraInfo
    // K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    const double fx = info_msg->k[0];
    const double fy = info_msg->k[4];
    const double cx = info_msg->k[2];
    const double cy = info_msg->k[5];

    const int width  = depth_image.cols;
    const int height = depth_image.rows;

    // ── Build PointCloud2 ─────────────────────────────────────────────────
    auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud_msg->header        = depth_msg->header;
    cloud_msg->height        = height;
    cloud_msg->width         = width;
    cloud_msg->is_dense      = false;
    cloud_msg->is_bigendian  = false;

    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");   // adds x, y, z as float32

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col, ++iter_x, ++iter_y, ++iter_z) {

        float z = getDepthMetres(depth_image, row, col);

        if (!std::isfinite(z) || z <= 0.0f) {
          // TODO: apply null_range logic here when specified
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

  /// Return depth in metres regardless of the image encoding.
  static float getDepthMetres(const cv::Mat & img, int row, int col)
  {
    if (img.type() == CV_32FC1) {
      // Gazebo depth cameras publish float32 in metres
      return img.at<float>(row, col);
    } else if (img.type() == CV_16UC1) {
      // Some drivers publish uint16 in millimetres
      uint16_t raw = img.at<uint16_t>(row, col);
      return (raw == 0) ? std::numeric_limits<float>::quiet_NaN()
                        : static_cast<float>(raw) * 1e-3f;
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  // ── Members ────────────────────────────────────────────────────────────────
  std::string gz_depth_topic_;
  std::string gz_camera_info_topic_;
  double      null_range_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  message_filters::Subscriber<sensor_msgs::msg::Image>      depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  std::shared_ptr<Sync>                                      sync_;
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthToPointCloud>());
  rclcpp::shutdown();
  return 0;
}
