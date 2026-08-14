#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SensorHealthMonitor : public rclcpp::Node {
 public:
  SensorHealthMonitor() : Node("sensor_health_monitor"), last_report_(Clock::now()) {
    const auto qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", qos, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { mark(scan_, msg->header.stamp); });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/front_lidar", qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { mark(lidar_, msg->header.stamp); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/front_lidar/imu", qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { mark(imu_, msg->header.stamp); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom/mc_odom", qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { mark(odom_, msg->header.stamp); });
    rtk_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/fix", qos, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          rtk_fix_status_.store(static_cast<int>(msg->status.status), std::memory_order_relaxed);
          const double horizontal_variance = std::max(
              msg->position_covariance[0], msg->position_covariance[4]);
          rtk_horizontal_std_m_.store(
              horizontal_variance >= 0.0 ? std::sqrt(horizontal_variance) : -1.0,
              std::memory_order_relaxed);
          mark(rtk_, msg->header.stamp);
        });
    publisher_ = create_publisher<std_msgs::msg::String>("/sensor_health", 2);
    timer_ = create_wall_timer(1s, std::bind(&SensorHealthMonitor::publish_health, this));
  }

 private:
  using Clock = std::chrono::steady_clock;

  struct Counter {
    std::atomic<uint64_t> count{0};
    std::atomic<int64_t> last_ns{0};
    std::atomic<int64_t> measurement_stamp_ns{0};
  };

  static int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
  }

  static void mark(Counter &counter, const builtin_interfaces::msg::Time &stamp) {
    counter.count.fetch_add(1, std::memory_order_relaxed);
    counter.last_ns.store(now_ns(), std::memory_order_relaxed);
    counter.measurement_stamp_ns.store(
        static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec,
        std::memory_order_relaxed);
  }

  static std::string timestamp_health(Counter &counter) {
    const int64_t stamp_ns = counter.measurement_stamp_ns.load(std::memory_order_relaxed);
    if (stamp_ns < 946684800000000000LL) {
      return ",\"measurement_time_valid\":false,\"measurement_time_offset_ms\":null";
    }
    const int64_t system_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const double offset_ms = (system_ns - stamp_ns) / 1e6;
    std::ostringstream out;
    out << ",\"measurement_time_valid\":"
        << (std::fabs(offset_ms) <= 100.0 ? "true" : "false")
        << ",\"measurement_time_offset_ms\":" << std::fixed << std::setprecision(3)
        << offset_ms;
    return out.str();
  }

  static void append_sensor(
      std::ostringstream &out, const char *name, const char *topic, Counter &counter,
      double elapsed_s, double stale_after_s, const std::string &extra = "") {
    const auto count = counter.count.exchange(0, std::memory_order_relaxed);
    const auto last = counter.last_ns.load(std::memory_order_relaxed);
    const double age = last > 0 ? (now_ns() - last) / 1e9 : -1.0;
    const bool online = age >= 0.0 && age <= stale_after_s;
    out << '"' << name << "\":{";
    out << "\"online\":" << (online ? "true" : "false") << ',';
    out << "\"sample_age_seconds\":";
    if (age >= 0.0) out << std::fixed << std::setprecision(3) << age;
    else out << "null";
    out << ",\"frequency_hz\":" << std::fixed << std::setprecision(1)
        << (elapsed_s > 0.0 ? count / elapsed_s : 0.0);
    out << ",\"topic\":\"" << topic << "\"" << extra << '}';
  }

  void publish_health() {
    const auto now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_report_).count();
    last_report_ = now;
    std::ostringstream out;
    out << '{';
    append_sensor(out, "lidar", "/front_lidar", lidar_, elapsed, 1.0, timestamp_health(lidar_));
    out << ',';
    append_sensor(out, "laser_scan", "/laser_scan", scan_, elapsed, 1.0);
    out << ',';
    append_sensor(out, "imu", "/front_lidar/imu", imu_, elapsed, 0.5, timestamp_health(imu_));
    out << ',';
    append_sensor(out, "odometry", "/odom/mc_odom", odom_, elapsed, 1.0);
    out << ',';
    const int fix = rtk_fix_status_.load(std::memory_order_relaxed);
    const double horizontal_std = rtk_horizontal_std_m_.load(std::memory_order_relaxed);
    const char *quality = fix >= 2 ? "rtk_fixed" : (fix == 1 ? "rtk_float" : (fix == 0 ? "standalone" : "invalid"));
    const char *fusion_mode = fix >= 2 ? "rtk_primary" : (fix == 1 ? "hybrid" : "lidar_fallback");
    std::ostringstream rtk_extra;
    rtk_extra << ",\"fix_status\":" << fix
              << ",\"has_fix\":" << (fix >= 0 ? "true" : "false")
              << ",\"fusion_usable\":" << (fix >= 1 ? "true" : "false")
              << ",\"quality\":\"" << quality << "\""
              << ",\"fusion_mode\":\"" << fusion_mode << "\""
              << ",\"horizontal_std_m\":";
    if (horizontal_std >= 0.0 && std::isfinite(horizontal_std)) rtk_extra << std::fixed << std::setprecision(3) << horizontal_std;
    else rtk_extra << "null";
    append_sensor(
        out, "rtk", "/fix", rtk_, elapsed, 3.0,
        rtk_extra.str() + timestamp_health(rtk_));
    out << '}';
    std_msgs::msg::String message;
    message.data = out.str();
    publisher_->publish(message);
  }

  Counter scan_;
  Counter lidar_;
  Counter imu_;
  Counter odom_;
  Counter rtk_;
  std::atomic<int> rtk_fix_status_{-1};
  std::atomic<double> rtk_horizontal_std_m_{-1.0};
  Clock::time_point last_report_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr rtk_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorHealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
