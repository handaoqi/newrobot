#include "pcd2grid.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/resource.h>

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 9)
    {
        std::cerr << "Usage: pcd2grid_streaming INPUT_PCD OUTPUT_PREFIX "
                     "[RESOLUTION] [Z_MIN] [Z_MAX] [MAX_CELLS] [MIN_POINTS] [SUPPORT_RADIUS]\n";
        return 2;
    }

    robot::slam::Pcd2GridOptions options;
    if (argc > 3)
        options.map_resolution = std::stod(argv[3]);
    if (argc > 4)
        options.thre_z_min = std::stod(argv[4]);
    if (argc > 5)
        options.thre_z_max = std::stod(argv[5]);
    if (argc > 6)
        options.max_grid_cells = static_cast<std::size_t>(std::stoull(argv[6]));
    if (argc > 7)
        options.min_points_per_cell = static_cast<std::uint8_t>(std::clamp(std::stoi(argv[7]), 1, 255));
    if (argc > 8)
        options.support_radius_cells = static_cast<std::uint8_t>(std::clamp(std::stoi(argv[8]), 0, 8));

    rclcpp::init(argc, argv);
    robot::slam::Pcd2Grid converter(options);
    std::string error;
    const bool ok = converter.runFromBinaryPcd(
        argv[1], argv[2], [](double progress) {
            static int last_percent = -1;
            const int percent = static_cast<int>(progress * 100.0);
            if (percent != last_percent)
            {
                last_percent = percent;
                std::cout << "progress=" << percent << "%\r" << std::flush;
            }
        }, &error);
    rclcpp::shutdown();
    if (!ok)
    {
        std::cerr << "\nerror: " << error << '\n';
        return 1;
    }
    struct rusage usage {};
    getrusage(RUSAGE_SELF, &usage);
    std::cout << "progress=100% peak_rss_kib=" << usage.ru_maxrss << '\n';
    return 0;
}
