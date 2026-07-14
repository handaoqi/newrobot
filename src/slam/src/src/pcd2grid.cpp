
#include "pcd2grid.h"

namespace robot::slam
{
    Pcd2Grid::Pcd2Grid(const Pcd2GridOptions &options) : options_(options)
    {
    }

    void Pcd2Grid::run(const CloudPtr &pcd_cloud, const std::string &file_name)
    {
        CloudPtr cloud_after_pass_through = CloudPtr(new PointCloudType());
        nav_msgs::msg::OccupancyGrid map_topic_msg;

        PassThroughFilter(pcd_cloud, cloud_after_pass_through);
        SetMapTopicMsg(cloud_after_pass_through, map_topic_msg);
        SavePGMAndYAML(map_topic_msg, file_name);
    }
    void Pcd2Grid::PassThroughFilter(const CloudPtr &pcd_cloud, CloudPtr &cloud_after_pass_through)
    {
        pcl::PassThrough<PointType> passthrough;
        passthrough.setInputCloud(pcd_cloud);
        passthrough.setFilterFieldName("z");
        passthrough.setFilterLimits(options_.thre_z_min, options_.thre_z_max);
        // passthrough.setFilterLimitsNegative(flag_in);
        passthrough.setNegative(bool(options_.flag_pass_through));
        passthrough.filter(*cloud_after_pass_through);
        // pcl::io::savePCDFile<PointType>(options_.file_name + "_filter.pcd",
        //                                 *cloud_after_pass_through);
        // std::cout << "Point cloud size after passthrough filter: "
        //           << cloud_after_pass_through->points.size() << std::endl;
    }

    void Pcd2Grid::SetMapTopicMsg(const CloudPtr cloud, nav_msgs::msg::OccupancyGrid &msg)
    {
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = "map";
        msg.info.map_load_time = rclcpp::Clock().now();
        msg.info.resolution = options_.map_resolution;

        if (cloud->points.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "PCD is empty!");
            return;
        }

        // 单轮遍历：同时找边界和标记栅格
        double x_min = cloud->points[0].x, x_max = x_min;
        double y_min = cloud->points[0].y, y_max = y_min;

        // 第一轮：先找出边界，确定栅格尺寸
        for (size_t i = 1; i < cloud->points.size(); i++)
        {
            double x = cloud->points[i].x, y = cloud->points[i].y;
            if (x < x_min) x_min = x;
            if (x > x_max) x_max = x;
            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
        }

        double res = options_.map_resolution;
        msg.info.origin.position.x = x_min;
        int h = int((y_max - y_min) / res);
        msg.info.origin.position.y = y_max - h * res;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        int width  = int((x_max - x_min) / res);
        int height = int((y_max - y_min) / res);
        msg.info.width  = width;
        msg.info.height = height;
        msg.data.assign(width * height, 0);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grid size: %dx%d = %d cells", width, height, width * height);

        // 第二轮：栅格化，每个点只算一次索引
        for (size_t iter = 0; iter < cloud->points.size(); iter++)
        {
            int i = int((cloud->points[iter].x - x_min) / res);
            if (i < 0 || i >= width) continue;
            int j = int((cloud->points[iter].y - y_min) / res);
            if (j < 0 || j >= height) continue;
            msg.data[i + j * width] = 100;
        }
    }

    void Pcd2Grid::SavePGMAndYAML(const nav_msgs::msg::OccupancyGrid &msg, const std::string &name)
    {
        int width = msg.info.width;
        int height = msg.info.height;

        // Save PGM
        cv::Mat image(height, width, CV_8UC1);

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int8_t data = msg.data[x + y * width];
                if (data == -1)
                {
                    image.at<uchar>(y, x) = 205; // Unknown
                }
                else
                {
                    image.at<uchar>(y, x) = 255 - data * 255 / 100; // Occupied
                }
            }
        }
        cv::flip(image, image, 0); // resolve image mirroring issues
        std::string pgm_file = name + ".pgm";
        std::string pgm_abs_path = pgm_file;  // 使用绝对路径，避免导航找不到文件
        cv::imwrite(pgm_file, image);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Saved PGM file: %s", pgm_file.c_str());

        // Save YAML
        std::string yaml_file = name + ".yaml";
        std::ofstream yaml_output(yaml_file);
        yaml_output << "image: " << pgm_abs_path << std::endl;
        yaml_output << "resolution: " << msg.info.resolution << std::endl;
        yaml_output << "origin: [" << msg.info.origin.position.x << ", "
                    << msg.info.origin.position.y << ", "
                    << msg.info.origin.position.z << "]" << std::endl;
        yaml_output << "negate: 0" << std::endl;
        yaml_output << "occupied_thresh: 0.65" << std::endl;
        yaml_output << "free_thresh: 0.196" << std::endl;
        yaml_output.close();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Saved YAML file: %s", yaml_file.c_str());
    }
}
