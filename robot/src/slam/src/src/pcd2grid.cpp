
#include "pcd2grid.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <limits>
#include <sstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
#pragma pack(push, 1)
    struct BinaryPcdPoint
    {
        float x;
        float y;
        float z;
        float intensity;
        float normal_x;
        float normal_y;
        float normal_z;
        float curvature;
    };
#pragma pack(pop)

    static_assert(sizeof(BinaryPcdPoint) == 32, "unexpected binary PCD point size");

    bool pointPassesHeightFilter(const BinaryPcdPoint& point, const robot::slam::Pcd2GridOptions& options)
    {
        const bool inside = point.z >= options.thre_z_min && point.z <= options.thre_z_max;
        return options.flag_pass_through ? !inside : inside;
    }

    bool pointPassesProjectionFilter(const BinaryPcdPoint& point, const robot::slam::Pcd2GridOptions& options)
    {
        return pointPassesHeightFilter(point, options)
            && (!options.use_xy_bounds
                || (point.x >= options.x_min && point.x <= options.x_max
                    && point.y >= options.y_min && point.y <= options.y_max));
    }
}

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

    bool Pcd2Grid::runFromBinaryPcd(
        const std::string& pcd_file,
        const std::string& file_name,
        const std::function<void(double)>& progress_callback,
        std::string* error)
    {
        auto fail = [&](const std::string& message) {
            if (error)
                *error = message;
            RCLCPP_ERROR(rclcpp::get_logger("pcd2grid"), "%s", message.c_str());
            return false;
        };

        std::ifstream input(pcd_file, std::ios::binary);
        if (!input.is_open())
            return fail("cannot open binary PCD: " + pcd_file);

        std::size_t point_count = 0;
        std::string fields;
        std::string sizes;
        std::streampos data_offset = 0;
        std::string line;
        while (std::getline(input, line))
        {
            if (line.rfind("FIELDS ", 0) == 0)
                fields = line.substr(7);
            else if (line.rfind("SIZE ", 0) == 0)
                sizes = line.substr(5);
            else if (line.rfind("POINTS ", 0) == 0)
                point_count = static_cast<std::size_t>(std::stoull(line.substr(7)));
            else if (line == "DATA binary")
            {
                data_offset = input.tellg();
                break;
            }
        }
        const std::string expected_fields = "x y z intensity normal_x normal_y normal_z curvature";
        const std::string expected_sizes = "4 4 4 4 4 4 4 4";
        if (data_offset <= 0 || point_count == 0 || fields != expected_fields || sizes != expected_sizes)
            return fail("unsupported or empty PCD layout: " + pcd_file);

        constexpr std::size_t chunk_points = 65536;
        std::vector<BinaryPcdPoint> buffer(chunk_points);
        double x_min = std::numeric_limits<double>::infinity();
        double x_max = -std::numeric_limits<double>::infinity();
        double y_min = std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();
        std::size_t scanned = 0;
        std::size_t projected_points = 0;
        input.clear();
        input.seekg(data_offset);
        while (scanned < point_count && input.good())
        {
            const std::size_t requested = std::min(chunk_points, point_count - scanned);
            input.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(requested * sizeof(BinaryPcdPoint)));
            const std::size_t received = static_cast<std::size_t>(input.gcount()) / sizeof(BinaryPcdPoint);
            if (received == 0)
                break;
            for (std::size_t i = 0; i < received; ++i)
            {
                const auto& point = buffer[i];
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)
                    || !pointPassesProjectionFilter(point, options_))
                    continue;
                x_min = std::min(x_min, static_cast<double>(point.x));
                x_max = std::max(x_max, static_cast<double>(point.x));
                y_min = std::min(y_min, static_cast<double>(point.y));
                y_max = std::max(y_max, static_cast<double>(point.y));
                ++projected_points;
            }
            scanned += received;
            if (progress_callback)
                progress_callback(0.20 * static_cast<double>(scanned) / static_cast<double>(point_count));
        }
        if (scanned != point_count)
            return fail("binary PCD ended before declared point count: " + pcd_file);
        if (projected_points == 0 || !std::isfinite(x_min) || !std::isfinite(y_min))
            return fail("no points remain after height filtering");

        const double resolution = options_.map_resolution;
        if (!(resolution > 0.0))
            return fail("map resolution must be positive");
        const auto width64 = static_cast<std::uint64_t>(std::floor((x_max - x_min) / resolution)) + 1;
        const auto height64 = static_cast<std::uint64_t>(std::floor((y_max - y_min) / resolution)) + 1;
        if (width64 == 0 || height64 == 0 || width64 > std::numeric_limits<std::uint32_t>::max()
            || height64 > std::numeric_limits<std::uint32_t>::max()
            || width64 > options_.max_grid_cells / height64)
        {
            std::ostringstream message;
            message << "grid bounds are too large: " << width64 << 'x' << height64
                    << " cells (limit=" << options_.max_grid_cells << ')';
            return fail(message.str());
        }
        const std::size_t width = static_cast<std::size_t>(width64);
        const std::size_t height = static_cast<std::size_t>(height64);
        const std::size_t cell_count = width * height;

        const std::string grid_tmp = file_name + ".grid.tmp";
        const int grid_fd = ::open(grid_tmp.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0600);
        if (grid_fd < 0)
            return fail("cannot create disk-backed grid: " + grid_tmp);
        if (::ftruncate(grid_fd, static_cast<off_t>(cell_count)) != 0)
        {
            ::close(grid_fd);
            std::filesystem::remove(grid_tmp);
            return fail("cannot size disk-backed grid: " + grid_tmp);
        }
        auto* grid = static_cast<std::uint8_t*>(
            ::mmap(nullptr, cell_count, PROT_READ | PROT_WRITE, MAP_SHARED, grid_fd, 0));
        if (grid == MAP_FAILED)
        {
            ::close(grid_fd);
            std::filesystem::remove(grid_tmp);
            return fail("cannot mmap disk-backed grid: " + grid_tmp);
        }

        input.clear();
        input.seekg(data_offset);
        scanned = 0;
        while (scanned < point_count && input.good())
        {
            const std::size_t requested = std::min(chunk_points, point_count - scanned);
            input.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(requested * sizeof(BinaryPcdPoint)));
            const std::size_t received = static_cast<std::size_t>(input.gcount()) / sizeof(BinaryPcdPoint);
            if (received == 0)
                break;
            for (std::size_t i = 0; i < received; ++i)
            {
                const auto& point = buffer[i];
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)
                    || !pointPassesProjectionFilter(point, options_))
                    continue;
                const auto col = static_cast<std::int64_t>(std::floor((point.x - x_min) / resolution));
                const auto row = static_cast<std::int64_t>(std::floor((point.y - y_min) / resolution));
                if (col >= 0 && row >= 0 && static_cast<std::size_t>(col) < width && static_cast<std::size_t>(row) < height)
                {
                    auto& cell = grid[static_cast<std::size_t>(row) * width + static_cast<std::size_t>(col)];
                    if (cell < options_.min_points_per_cell)
                        ++cell;
                }
            }
            scanned += received;
            if (progress_callback)
                progress_callback(0.20 + 0.60 * static_cast<double>(scanned) / static_cast<double>(point_count));
        }
        if (scanned != point_count)
        {
            ::munmap(grid, cell_count);
            ::close(grid_fd);
            std::filesystem::remove(grid_tmp);
            return fail("binary PCD ended during grid pass: " + pcd_file);
        }

        const std::string pgm_file = file_name + ".pgm";
        const std::string pgm_tmp = pgm_file + ".tmp";
        std::ofstream pgm(pgm_tmp, std::ios::binary | std::ios::trunc);
        if (!pgm.is_open())
        {
            ::munmap(grid, cell_count);
            ::close(grid_fd);
            std::filesystem::remove(grid_tmp);
            return fail("cannot create PGM: " + pgm_tmp);
        }
        pgm << "P5\n" << width << ' ' << height << "\n255\n";
        std::vector<std::uint8_t> row_buffer(width);
        for (std::size_t output_row = 0; output_row < height; ++output_row)
        {
            const std::size_t source_row = height - 1 - output_row;
            const auto* source = grid + source_row * width;
            for (std::size_t col = 0; col < width; ++col)
            {
                bool occupied = false;
                if (source[col] > 0)
                {
                    unsigned int support = 0;
                    const std::size_t radius = options_.support_radius_cells;
                    const std::size_t row_min = source_row > radius ? source_row - radius : 0;
                    const std::size_t row_max = std::min(height - 1, source_row + radius);
                    const std::size_t col_min = col > radius ? col - radius : 0;
                    const std::size_t col_max = std::min(width - 1, col + radius);
                    for (std::size_t nearby_row = row_min; nearby_row <= row_max && !occupied; ++nearby_row)
                    {
                        const auto* nearby = grid + nearby_row * width;
                        for (std::size_t nearby_col = col_min; nearby_col <= col_max; ++nearby_col)
                        {
                            support += nearby[nearby_col];
                            if (support >= options_.min_points_per_cell)
                            {
                                occupied = true;
                                break;
                            }
                        }
                    }
                }
                row_buffer[col] = occupied ? 0 : 255;
            }
            pgm.write(reinterpret_cast<const char*>(row_buffer.data()), static_cast<std::streamsize>(row_buffer.size()));
            if (progress_callback && (output_row % 256 == 0 || output_row + 1 == height))
                progress_callback(0.80 + 0.20 * static_cast<double>(output_row + 1) / static_cast<double>(height));
        }
        pgm.close();
        if (!pgm.good())
        {
            ::munmap(grid, cell_count);
            ::close(grid_fd);
            std::filesystem::remove(grid_tmp);
            std::filesystem::remove(pgm_tmp);
            return fail("failed while writing PGM: " + pgm_tmp);
        }
        ::msync(grid, cell_count, MS_ASYNC);
        ::munmap(grid, cell_count);
        ::close(grid_fd);
        std::filesystem::remove(grid_tmp);
        std::error_code filesystem_error;
        std::filesystem::rename(pgm_tmp, pgm_file, filesystem_error);
        if (filesystem_error)
        {
            std::filesystem::remove(pgm_tmp);
            return fail("cannot publish PGM: " + filesystem_error.message());
        }

        const std::string yaml_file = file_name + ".yaml";
        const std::string yaml_tmp = yaml_file + ".tmp";
        std::ofstream yaml(yaml_tmp, std::ios::out | std::ios::trunc);
        if (!yaml.is_open())
            return fail("cannot create YAML: " + yaml_tmp);
        yaml << "image: " << pgm_file << '\n';
        yaml << "resolution: " << resolution << '\n';
        yaml << "origin: [" << x_min << ", " << y_min << ", 0]\n";
        yaml << "negate: 0\n";
        yaml << "occupied_thresh: 0.65\n";
        yaml << "free_thresh: 0.196\n";
        yaml.close();
        if (!yaml.good())
        {
            std::filesystem::remove(yaml_tmp);
            return fail("failed while writing YAML: " + yaml_tmp);
        }
        filesystem_error.clear();
        std::filesystem::rename(yaml_tmp, yaml_file, filesystem_error);
        if (filesystem_error)
        {
            std::filesystem::remove(yaml_tmp);
            return fail("cannot publish YAML: " + filesystem_error.message());
        }

        RCLCPP_INFO(rclcpp::get_logger("pcd2grid"),
            "Saved disk-backed grid %zux%zu (%zu cells, %zu projected points, min_support=%u radius=%u)",
            width, height, cell_count, projected_points,
            options_.min_points_per_cell, options_.support_radius_cells);
        return true;
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
