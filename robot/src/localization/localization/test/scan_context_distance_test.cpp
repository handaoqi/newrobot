#include "localization/scan_context_db.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace {

TEST(ScanContextDistance, SectorCosineIsZeroForIdenticalDescriptors) {
  localization::ScanContextDatabase database;
  const auto& params = database.params();
  std::vector<float> descriptor(static_cast<std::size_t>(params.cells()), 0.0f);
  for (int sector = 0; sector < params.sectors; ++sector) {
    descriptor[static_cast<std::size_t>(sector % params.rings) * params.sectors + sector] =
      static_cast<float>(sector + 1);
  }
  EXPECT_NEAR(database.descriptorCosineDistance(descriptor, descriptor, 0), 0.0, 1e-9);
}

TEST(ScanContextDistance, SectorCosineRecoversCircularShift) {
  localization::ScanContextDatabase database;
  const auto& params = database.params();
  constexpr int shift = 7;
  std::vector<float> query(static_cast<std::size_t>(params.cells()), 0.0f);
  std::vector<float> target(static_cast<std::size_t>(params.cells()), 0.0f);
  for (int sector = 0; sector < params.sectors; ++sector) {
    const int ring = sector % params.rings;
    query[static_cast<std::size_t>(ring) * params.sectors + sector] = 1.0f;
    const int shifted_sector = (sector + shift) % params.sectors;
    target[static_cast<std::size_t>(ring) * params.sectors + shifted_sector] = 1.0f;
  }
  EXPECT_NEAR(database.descriptorCosineDistance(query, target, shift), 0.0, 1e-9);
  EXPECT_GT(database.descriptorCosineDistance(query, target, 0), 0.9);
}

}  // namespace
