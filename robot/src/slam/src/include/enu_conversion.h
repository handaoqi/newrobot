#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace robot::slam
{
    struct GeodeticOrigin
    {
        double latitude_deg = 0.0;
        double longitude_deg = 0.0;
        double altitude_m = 0.0;
    };

    inline void validateGeodeticOrigin(const GeodeticOrigin& origin)
    {
        if (!std::isfinite(origin.latitude_deg) || !std::isfinite(origin.longitude_deg)
            || !std::isfinite(origin.altitude_m))
            throw std::invalid_argument("ENU origin contains a non-finite value");
        if (origin.latitude_deg < -90.0 || origin.latitude_deg > 90.0)
            throw std::invalid_argument("ENU latitude is outside [-90, 90]");
        if (origin.longitude_deg < -180.0 || origin.longitude_deg > 180.0)
            throw std::invalid_argument("ENU longitude is outside [-180, 180]");
        if (std::fabs(origin.latitude_deg) < 1e-12 && std::fabs(origin.longitude_deg) < 1e-12
            && std::fabs(origin.altitude_m) < 1e-9)
            throw std::invalid_argument("ENU origin cannot be all zero");
    }

    inline std::array<double, 3> geodeticToEcef(
        double latitude_deg, double longitude_deg, double altitude_m)
    {
        constexpr double kA = 6378137.0;
        constexpr double kInvF = 298.257223563;
        constexpr double kF = 1.0 / kInvF;
        constexpr double kE2 = kF * (2.0 - kF);
        constexpr double kDegToRad = M_PI / 180.0;
        const double latitude = latitude_deg * kDegToRad;
        const double longitude = longitude_deg * kDegToRad;
        const double sin_latitude = std::sin(latitude);
        const double cos_latitude = std::cos(latitude);
        const double prime_vertical = kA / std::sqrt(1.0 - kE2 * sin_latitude * sin_latitude);
        return {
            (prime_vertical + altitude_m) * cos_latitude * std::cos(longitude),
            (prime_vertical + altitude_m) * cos_latitude * std::sin(longitude),
            (prime_vertical * (1.0 - kE2) + altitude_m) * sin_latitude,
        };
    }

    inline std::array<double, 3> geodeticToEnu(
        double latitude_deg, double longitude_deg, double altitude_m, const GeodeticOrigin& origin)
    {
        validateGeodeticOrigin(origin);
        if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) || !std::isfinite(altitude_m)
            || latitude_deg < -90.0 || latitude_deg > 90.0
            || longitude_deg < -180.0 || longitude_deg > 180.0)
            throw std::invalid_argument("GNSS fix is outside valid geodetic bounds");
        constexpr double kDegToRad = M_PI / 180.0;
        const auto point = geodeticToEcef(latitude_deg, longitude_deg, altitude_m);
        const auto anchor = geodeticToEcef(origin.latitude_deg, origin.longitude_deg, origin.altitude_m);
        const double dx = point[0] - anchor[0];
        const double dy = point[1] - anchor[1];
        const double dz = point[2] - anchor[2];
        const double latitude = origin.latitude_deg * kDegToRad;
        const double longitude = origin.longitude_deg * kDegToRad;
        const double sin_latitude = std::sin(latitude);
        const double cos_latitude = std::cos(latitude);
        const double sin_longitude = std::sin(longitude);
        const double cos_longitude = std::cos(longitude);
        return {
            -sin_longitude * dx + cos_longitude * dy,
            -sin_latitude * cos_longitude * dx - sin_latitude * sin_longitude * dy + cos_latitude * dz,
            cos_latitude * cos_longitude * dx + cos_latitude * sin_longitude * dy + sin_latitude * dz,
        };
    }
}  // namespace robot::slam
