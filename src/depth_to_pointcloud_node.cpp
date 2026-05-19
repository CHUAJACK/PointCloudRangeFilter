#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// ---------------------------------------------------------------------------
// Raw depth frame — holds only what the gz callback copies off the wire
// ---------------------------------------------------------------------------
struct DepthFrame {
  std::vector<uint8_t> data;
  int      width{0}, height{0};
  bool     is_float32{false};
  int32_t  stamp_sec{0};
  uint32_t stamp_nsec{0};
};

// ---------------------------------------------------------------------------
// Pre-built PointCloud2 skeleton — fields/step set once, data resized lazily
// ---------------------------------------------------------------------------
static sensor_msgs::msg::PointCloud2::UniquePtr
makeCloudShell(const std::string & frame_id)
{
  auto m = std::make_unique<sensor_msgs::msg::PointCloud2>();
  m->header.frame_id = frame_id;
  m->is_dense        = false;
  m->is_bigendian    = false;
  m->point_step      = 12;   // 3 × float32
  m->fields.resize(3);
  const char * names[] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    m->fields[i].name     = names[i];
    m->fields[i].offset   = static_cast<uint32_t>(i * 4);
    m->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    m->fields[i].count    = 1;
  }
  return m;
}

class DepthToPointCloud : public rclcpp::Node
{
public:
  DepthToPointCloud()
  : Node("depth_to_pointcloud")
  {
    RCLCPP_INFO(get_logger(), "=== Constructor start ===");

    // ── Parameters ────────────────────────────────────────────────────────────
    declare_parameter<std::string>("gz_depth",       "/depth_camera");
    declare_parameter<std::string>("gz_camera_info", "/camera_info");
    declare_parameter<double>     ("null_range_min",  0.0);
    declare_parameter<double>     ("null_range_max",  1.0);
    declare_parameter<std::string>("output_topic",   "/depth_camera_bridged/points");
    declare_parameter<bool>       ("best_effort",     true);
    declare_parameter<int>        ("downsample",       1);   // 1=full res, 2=half, 4=quarter …

    gz_depth_topic_       = get_parameter("gz_depth").as_string();
    gz_camera_info_topic_ = get_parameter("gz_camera_info").as_string();
    min_z_ = static_cast<float>(get_parameter("null_range_min").as_double());
    max_z_ = static_cast<float>(get_parameter("null_range_max").as_double());
    output_topic_         = get_parameter("output_topic").as_string();
    const bool best_effort = get_parameter("best_effort").as_bool();
    downsample_ = static_cast<int>(std::max(int64_t{1}, get_parameter("downsample").as_int()));

    RCLCPP_INFO(get_logger(), "gz_depth       : %s", gz_depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "gz_camera_info : %s", gz_camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "null_range     : [%.4f, %.4f]",
      static_cast<double>(min_z_), static_cast<double>(max_z_));
    RCLCPP_INFO(get_logger(), "output_topic   : %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "QoS            : %s",
      best_effort ? "best_effort" : "reliable");
    RCLCPP_INFO(get_logger(), "downsample     : %d (output res = 1/%d)",
      downsample_, downsample_);

    // ── Publisher ─────────────────────────────────────────────────────────────
    // best_effort avoids rmw blocking on subscriber flow-control.
    auto qos = rclcpp::QoS(10);
    qos.reliable();
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, qos);

    // ── Double-buffer: ping-pong so convert and rmw serialise overlap ─────────
    cloud_[0] = makeCloudShell("camera_link");
    cloud_[1] = makeCloudShell("camera_link");
    active_slot_ = 0;

    // ── Publish thread ────────────────────────────────────────────────────────
    publish_thread_ = std::thread(&DepthToPointCloud::publishLoop, this);

    // ── gz subscriptions ──────────────────────────────────────────────────────
    bool ok_d = gz_node_.Subscribe(gz_depth_topic_,
                  &DepthToPointCloud::onGzDepth, this);
    bool ok_i = gz_node_.Subscribe(gz_camera_info_topic_,
                  &DepthToPointCloud::onGzCameraInfo, this);

    RCLCPP_INFO(get_logger(), "gz depth  '%s' -> %s",
      gz_depth_topic_.c_str(), ok_d ? "OK" : "FAILED");
    RCLCPP_INFO(get_logger(), "gz info   '%s' -> %s",
      gz_camera_info_topic_.c_str(), ok_i ? "OK" : "FAILED");
    RCLCPP_INFO(get_logger(), "=== Constructor complete ===");
  }

  ~DepthToPointCloud()
  {
    {
      std::lock_guard<std::mutex> lk(frame_mutex_);
      shutdown_ = true;
    }
    frame_cv_.notify_one();
    if (publish_thread_.joinable()) publish_thread_.join();
  }

private:
  // ── Camera-info callback (gz thread) ──────────────────────────────────────
  void onGzCameraInfo(const gz::msgs::CameraInfo & msg)
  {
    const auto & k = msg.intrinsics().k();
    std::lock_guard<std::mutex> lk(info_mutex_);
    fx_ = k[0];  cx_ = k[2];
    fy_ = k[4];  cy_ = k[5];
    has_info_ = true;
  }

  // ── Depth callback (gz thread) — memcpy only, no math ────────────────────
  void onGzDepth(const gz::msgs::Image & msg)
  {
    {
      std::lock_guard<std::mutex> lk(info_mutex_);
      if (!has_info_) return;
    }

    const bool is_f32 = (msg.pixel_format_type() ==
                         gz::msgs::PixelFormatType::R_FLOAT32);
    const bool is_u16 = (msg.pixel_format_type() ==
                         gz::msgs::PixelFormatType::L_INT16);
    if (!is_f32 && !is_u16) { ++unknown_format_count_; return; }

    {
      std::lock_guard<std::mutex> lk(frame_mutex_);
      pending_.width      = static_cast<int>(msg.width());
      pending_.height     = static_cast<int>(msg.height());
      pending_.is_float32 = is_f32;
      pending_.stamp_sec  = static_cast<int32_t> (msg.header().stamp().sec());
      pending_.stamp_nsec = static_cast<uint32_t>(msg.header().stamp().nsec());

      const std::size_t bytes = msg.data().size();
      if (pending_.data.size() != bytes) pending_.data.resize(bytes);
      std::memcpy(pending_.data.data(), msg.data().data(), bytes);
      has_pending_ = true;
    }
    frame_cv_.notify_one();
  }

  // ── Publish thread ────────────────────────────────────────────────────────
  void publishLoop()
  {
    DepthFrame work;

    while (true) {
      {
        std::unique_lock<std::mutex> lk(frame_mutex_);
        frame_cv_.wait(lk, [this]{ return has_pending_ || shutdown_; });
        if (shutdown_) break;
        std::swap(work, pending_);
        has_pending_ = false;
      }

      float fx, fy, cx, cy;
      {
        std::lock_guard<std::mutex> lk(info_mutex_);
        fx = static_cast<float>(fx_);
        fy = static_cast<float>(fy_);
        cx = static_cast<float>(cx_);
        cy = static_cast<float>(cy_);
      }

      // Write into the slot that rmw finished with last iteration
      const int slot = active_slot_ ^ 1;
      convertInto(*cloud_[slot], work, fx, fy, cx, cy);
      active_slot_ = slot;

      pointcloud_pub_->publish(*cloud_[slot]);
      ++cloud_count_;

      if ((cloud_count_.load() % 150) == 0) {
        RCLCPP_INFO(get_logger(),
          "published %lu clouds | dropped (unknown fmt) %lu",
          cloud_count_.load(), unknown_format_count_.load());
      }
    }
  }

  // ── Core pixel loop ───────────────────────────────────────────────────────
  void convertInto(sensor_msgs::msg::PointCloud2 & cloud,
                 const DepthFrame & f,
                 float fx, float fy, float cx, float cy)
  {
    const int srcW = f.width;
    const int srcH = f.height;
    const int ds   = downsample_;

    const uint32_t W = static_cast<uint32_t>(srcW / ds);
    const uint32_t H = static_cast<uint32_t>(srcH / ds);

    if (cloud.width != W || cloud.height != H) {
      cloud.width    = W;
      cloud.height   = H;
      cloud.row_step = cloud.point_step * W;
      cloud.data.resize(cloud.row_step * H);
    }
    cloud.header.stamp.sec     = f.stamp_sec;
    cloud.header.stamp.nanosec = f.stamp_nsec;

    float * __restrict__ out =
      reinterpret_cast<float *>(cloud.data.data());

    const float nan    = std::numeric_limits<float>::quiet_NaN();
    const float inv_fx = 1.0f / fx;
    const float inv_fy = 1.0f / fy;
    const float min_z  = min_z_;
    const float max_z  = max_z_;
    const int   iW     = static_cast<int>(W);
    const int   iH     = static_cast<int>(H);

    // ── FOV crop window (in source pixel coordinates) ────────────────────────
    // Crop depth frame so its effective FOV matches the RGB camera (IMX214).
    // Depth HFOV=1.274 rad, RGB HFOV=1.204 rad, same physical pose.
    // Valid columns: [2.75%, 97.25%] of srcW  → trim 2.75% each side
    // Valid rows:    [14.31%, 85.69%] of srcH  → trim 14.31% each side
    const int crop_col_min = static_cast<int>(std::floor(0.0275f * static_cast<float>(srcW)));
    const int crop_col_max = static_cast<int>(std::floor(0.9725f * static_cast<float>(srcW)));
    const int crop_row_min = static_cast<int>(std::floor(0.1431f * static_cast<float>(srcH)));
    const int crop_row_max = static_cast<int>(std::floor(0.8569f * static_cast<float>(srcH)));

    if (f.is_float32) {
      const float * __restrict__ depth =
        reinterpret_cast<const float *>(f.data.data());

      for (int r = 0; r < iH; ++r) {
        const int    src_r = r * ds;
        const float  dy    = static_cast<float>(src_r) - cy;
        const float *d_row = depth + src_r * srcW;
        float       *o_row = out   + r * iW * 3;

        for (int c = 0; c < iW; ++c) {
          const int   src_c = c * ds;
          float * p = o_row + c * 3;

          // Outside FOV crop window → NaN
          if (src_r < crop_row_min || src_r >= crop_row_max ||
              src_c < crop_col_min || src_c >= crop_col_max) {
            p[0] = p[1] = p[2] = nan;
            continue;
          }

          const float z = d_row[src_c];
          if (!std::isfinite(z) || (z >= min_z && z <= max_z)) {
            p[0] = p[1] = p[2] = nan;
          } else {
            p[0] =  z;
            p[1] = -(static_cast<float>(src_c) - cx) * z * inv_fx;
            p[2] = -dy * z * inv_fy;
          }
        }
      }
    } else {
      const uint16_t * __restrict__ depth =
        reinterpret_cast<const uint16_t *>(f.data.data());

      for (int r = 0; r < iH; ++r) {
        const int       src_r = r * ds;
        const float     dy    = static_cast<float>(src_r) - cy;
        const uint16_t *d_row = depth + src_r * srcW;
        float          *o_row = out   + r * iW * 3;

        for (int c = 0; c < iW; ++c) {
          const int      src_c = c * ds;
          float * p = o_row + c * 3;

          // Outside FOV crop window → NaN
          if (src_r < crop_row_min || src_r >= crop_row_max ||
              src_c < crop_col_min || src_c >= crop_col_max) {
            p[0] = p[1] = p[2] = nan;
            continue;
          }

          const uint16_t raw = d_row[src_c];
          if (raw == 0) { p[0] = p[1] = p[2] = nan; continue; }
          const float z = static_cast<float>(raw) * 1e-3f;
          if (z >= min_z && z <= max_z) {
            p[0] = p[1] = p[2] = nan;
          } else {
            p[0] =  z;
            p[1] = -(static_cast<float>(src_c) - cx) * z * inv_fx;
            p[2] = -dy * z * inv_fy;
          }
        }
      }
    }
  }

  // ── Members ───────────────────────────────────────────────────────────────
  std::string gz_depth_topic_, gz_camera_info_topic_, output_topic_;
  float min_z_{0.0f}, max_z_{1.0f};
  int   downsample_{2};

  std::mutex info_mutex_;
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  bool   has_info_{false};

  std::mutex              frame_mutex_;
  std::condition_variable frame_cv_;
  DepthFrame              pending_;
  bool                    has_pending_{false};
  bool                    shutdown_{false};

  // Double-buffer
  std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_[2];
  int active_slot_{0};

  std::atomic<uint64_t> cloud_count_{0};
  std::atomic<uint64_t> unknown_format_count_{0};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  gz::transport::Node gz_node_;
  std::thread         publish_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthToPointCloud>();
  RCLCPP_INFO(node->get_logger(), "Entering spin...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}