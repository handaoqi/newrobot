#include "enu_conversion.h"

#include <gtest/gtest.h>

using robot::slam::GeodeticOrigin;
using robot::slam::geodeticToEnu;
using robot::slam::validateGeodeticOrigin;

TEST(EnuConversionTest, OriginMapsToZero)
{
    const GeodeticOrigin origin { 39.9, 116.4, 52.0 };
    const auto enu = geodeticToEnu(39.9, 116.4, 52.0, origin);
    EXPECT_NEAR(enu[0], 0.0, 1e-6);
    EXPECT_NEAR(enu[1], 0.0, 1e-6);
    EXPECT_NEAR(enu[2], 0.0, 1e-6);
}

TEST(EnuConversionTest, EastNorthUpDirectionsArePreserved)
{
    const GeodeticOrigin origin { 39.9, 116.4, 52.0 };
    const auto east = geodeticToEnu(39.9, 116.40001, 52.0, origin);
    const auto north = geodeticToEnu(39.90001, 116.4, 52.0, origin);
    const auto up = geodeticToEnu(39.9, 116.4, 53.0, origin);
    EXPECT_GT(east[0], 0.8);
    EXPECT_NEAR(east[1], 0.0, 0.02);
    EXPECT_GT(north[1], 1.0);
    EXPECT_NEAR(north[0], 0.0, 0.02);
    EXPECT_NEAR(up[2], 1.0, 1e-5);
}

TEST(EnuConversionTest, RejectsMissingAndOutOfRangeOrigins)
{
    EXPECT_THROW(validateGeodeticOrigin({ 0.0, 0.0, 0.0 }), std::invalid_argument);
    EXPECT_THROW(validateGeodeticOrigin({ 91.0, 116.4, 0.0 }), std::invalid_argument);
    EXPECT_THROW(validateGeodeticOrigin({ 39.9, 181.0, 0.0 }), std::invalid_argument);
}
