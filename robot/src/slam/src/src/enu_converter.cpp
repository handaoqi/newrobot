#include "enu_conversion.h"

#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <openssl/evp.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace robot::slam
{
    namespace
    {
        std::string trim(std::string value)
        {
            const auto first = value.find_first_not_of(" \t\r\n\"'");
            if (first == std::string::npos)
                return {};
            const auto last = value.find_last_not_of(" \t\r\n\"'");
            return value.substr(first, last - first + 1);
        }

        std::unordered_map<std::string, std::string> readFlatYaml(const std::string& path)
        {
            std::ifstream input(path);
            if (!input.is_open())
                throw std::runtime_error("cannot open locked origin file: " + path);
            std::unordered_map<std::string, std::string> values;
            std::string line;
            while (std::getline(input, line))
            {
                const auto separator = line.find(':');
                if (separator != std::string::npos)
                    values[trim(line.substr(0, separator))] = trim(line.substr(separator + 1));
            }
            return values;
        }

        std::string sha256File(const std::string& path)
        {
            std::ifstream input(path, std::ios::binary);
            if (!input.is_open())
                throw std::runtime_error("cannot hash locked origin file: " + path);
            EVP_MD_CTX* context = EVP_MD_CTX_new();
            if (!context || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1)
            {
                EVP_MD_CTX_free(context);
                throw std::runtime_error("cannot initialize SHA256");
            }
            std::array<char, 65536> buffer {};
            while (input.good())
            {
                input.read(buffer.data(), buffer.size());
                if (input.gcount() > 0
                    && EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(input.gcount())) != 1)
                {
                    EVP_MD_CTX_free(context);
                    throw std::runtime_error("cannot update SHA256");
                }
            }
            std::array<unsigned char, EVP_MAX_MD_SIZE> digest {};
            unsigned int digest_size = 0;
            if (EVP_DigestFinal_ex(context, digest.data(), &digest_size) != 1)
            {
                EVP_MD_CTX_free(context);
                throw std::runtime_error("cannot finalize SHA256");
            }
            EVP_MD_CTX_free(context);
            std::ostringstream output;
            output << std::hex << std::setfill('0');
            for (unsigned int index = 0; index < digest_size; ++index)
                output << std::setw(2) << static_cast<unsigned int>(digest[index]);
            return output.str();
        }
    }  // namespace

    class EnuConverter final : public rclcpp::Node
    {
    public:
        EnuConverter() : Node("slam_enu_converter")
        {
            origin_.latitude_deg = declare_parameter<double>("lat0", std::numeric_limits<double>::quiet_NaN());
            origin_.longitude_deg = declare_parameter<double>("lon0", std::numeric_limits<double>::quiet_NaN());
            origin_.altitude_m = declare_parameter<double>("alt0", std::numeric_limits<double>::quiet_NaN());
            origin_file_ = declare_parameter<std::string>("origin_file", "");
            origin_session_id_ = declare_parameter<std::string>("origin_session_id", "");
            origin_sha256_ = declare_parameter<std::string>("origin_sha256", "");
            input_topic_ = declare_parameter<std::string>("input_topic", "/fix");
            output_topic_ = declare_parameter<std::string>("output_topic", "/gnss/enu_odom");
            max_age_seconds_ = declare_parameter<double>("max_age_seconds", 1.5);
            min_status_ = declare_parameter<int>("min_status", 1);
            validateConfiguration();
            publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(20));
            subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                input_topic_, rclcpp::QoS(20), std::bind(&EnuConverter::onFix, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "ENU converter ready session=%s sha256=%s",
                origin_session_id_.c_str(), origin_sha256_.c_str());
        }

    private:
        void validateConfiguration()
        {
            validateGeodeticOrigin(origin_);
            if (origin_file_.empty() || origin_session_id_.empty() || origin_sha256_.size() != 64)
                throw std::invalid_argument("origin_file, origin_session_id and origin_sha256 are required");
            if (sha256File(origin_file_) != origin_sha256_)
                throw std::invalid_argument("locked origin SHA256 does not match startup parameters");
            const auto values = readFlatYaml(origin_file_);
            const auto required = [&values](const std::string& key) -> const std::string& {
                const auto item = values.find(key);
                if (item == values.end() || item->second.empty())
                    throw std::invalid_argument("locked origin is missing " + key);
                return item->second;
            };
            const std::string locked_value = required("alignment_locked");
            const bool locked = locked_value == "true" || locked_value == "True" || locked_value == "1";
            if (!locked || required("origin_lock_session_id") != origin_session_id_)
                throw std::invalid_argument("locked origin session does not match startup parameters");
            const GeodeticOrigin file_origin {
                std::stod(required("origin_latitude")), std::stod(required("origin_longitude")),
                std::stod(required("origin_altitude")),
            };
            validateGeodeticOrigin(file_origin);
            if (std::fabs(file_origin.latitude_deg - origin_.latitude_deg) > 1e-10
                || std::fabs(file_origin.longitude_deg - origin_.longitude_deg) > 1e-10
                || std::fabs(file_origin.altitude_m - origin_.altitude_m) > 1e-4)
                throw std::invalid_argument("locked origin coordinates do not match startup parameters");
        }

        void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr fix)
        {
            if (fix->status.status < min_status_ || fix->header.stamp.sec == 0)
                return;
            const double age = std::fabs((get_clock()->now() - rclcpp::Time(fix->header.stamp)).seconds());
            if (!std::isfinite(age) || age > max_age_seconds_)
                return;
            std::array<double, 3> enu;
            try
            {
                enu = geodeticToEnu(fix->latitude, fix->longitude, fix->altitude, origin_);
            }
            catch (const std::exception& error)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Reject invalid GNSS fix: %s", error.what());
                return;
            }
            nav_msgs::msg::Odometry odometry;
            odometry.header.stamp = fix->header.stamp;
            odometry.header.frame_id = "enu";
            odometry.child_frame_id = "gnss_antenna";
            odometry.pose.pose.position.x = enu[0];
            odometry.pose.pose.position.y = enu[1];
            odometry.pose.pose.position.z = enu[2];
            odometry.pose.pose.orientation.w = 1.0;
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    odometry.pose.covariance[row * 6 + column] = fix->position_covariance[row * 3 + column];
            publisher_->publish(odometry);
        }

        GeodeticOrigin origin_;
        std::string origin_file_;
        std::string origin_session_id_;
        std::string origin_sha256_;
        std::string input_topic_;
        std::string output_topic_;
        double max_age_seconds_ = 1.5;
        int min_status_ = 1;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    };
}  // namespace robot::slam

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<robot::slam::EnuConverter>());
    }
    catch (const std::exception& error)
    {
        std::cerr << "slam_enu_converter startup failed: " << error.what() << std::endl;
        rclcpp::shutdown();
        return 2;
    }
    rclcpp::shutdown();
    return 0;
}
