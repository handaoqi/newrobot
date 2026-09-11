#pragma once

#include <cstddef>
#include <deque>

namespace robot::slam
{
    template <typename CloudPtrT>
    struct TimedLidarScan
    {
        double    stamp = 0.0;
        CloudPtrT cloud;
    };

    // Navigation consumes only the newest scan after a short overload.  When
    // syncData() has already selected the front scan and is waiting for IMU,
    // that in-flight scan must remain intact; every later queued scan is stale.
    template <typename ScanT>
    inline void discardSupersededNavigationScans(std::deque<ScanT>& scans, bool has_inflight_scan)
    {
        const std::size_t retained = has_inflight_scan ? 1U : 0U;
        while (scans.size() > retained)
            scans.pop_back();
    }

    inline bool needsWorldPointCloud(bool publish_world_points, bool mapping_capture_enabled,
        bool keyframe_recording_enabled)
    {
        return publish_world_points || (mapping_capture_enabled && keyframe_recording_enabled);
    }
}  // namespace robot::slam
