/**
 * @file grid_geometry_test.cpp
 * @brief Grid geometry test responsibilities.
 *
 * @details This file exercises grid geometry test behavior for automated
 * verification and regression testing. It centers on
 * `SupportsNegativeCoordinatesNonzeroOriginAndFractionalCells`,
 * `ExpandsInEveryDirectionAndPreservesWorldCellCenters`,
 * `EnforcesMaximumExtentAndMemoryLimits`,
 * `SerializationRestoresAllGeometryMetadata`,
 * `FixedExtentRejectsExpansionAndOutsideQueries`. Its package-relative
 * location is `test/unit/grid_geometry_test.cpp`.
 */
#include <gtest/gtest.h>

#include <semaforr/domain/grid_geometry.hpp>

namespace {

using semaforr::domain::GridExtentMode;
using semaforr::domain::GridExtentSource;
using semaforr::domain::GridGeometry;
using semaforr::domain::GridOutOfBoundsBehavior;

/**
 * @brief Performs the expandable geometry operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `GridGeometry` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
GridGeometry expandableGeometry() {
  return GridGeometry::fromBounds(
      "odom", {-2.5, -1.5}, {2.5, 3.5}, 0.25,
      GridExtentMode::Expandable,
      GridExtentSource::ConfiguredMaplessInitialBounds,
      GridOutOfBoundsBehavior::ExpandBeforeInsert);
}

}  // namespace

TEST(GridGeometry, SupportsNegativeCoordinatesNonzeroOriginAndFractionalCells) {
  const auto geometry = expandableGeometry();
  ASSERT_TRUE(geometry.valid());
  const auto index = geometry.index({-2.375, -1.375});
  ASSERT_TRUE(index.has_value());
  EXPECT_EQ(geometry.center(*index),
            (semaforr::domain::Point2D{-2.375, -1.375}));
  EXPECT_FALSE(geometry.index(geometry.maximum).has_value());
  EXPECT_TRUE(geometry.index(geometry.minimum).has_value());
}

TEST(GridGeometry, ExpandsInEveryDirectionAndPreservesWorldCellCenters) {
  const auto original = expandableGeometry();
  const auto remembered = original.center(*original.index({0.0, 0.0}));
  semaforr::domain::GridExpansionPolicy policy;
  policy.margin_m = 0.5;
  policy.increment_cells = 8U;
  auto positive = semaforr::domain::expandToInclude(original, {8.0, 9.0}, policy);
  ASSERT_TRUE(positive.expanded);
  auto both = semaforr::domain::expandToInclude(positive.geometry,
                                                {-9.0, -8.0}, policy);
  ASSERT_TRUE(both.expanded);
  EXPECT_LT(both.geometry.minimum.x_m, original.minimum.x_m);
  EXPECT_GT(both.geometry.maximum.y_m, original.maximum.y_m);
  ASSERT_TRUE(both.geometry.index(remembered).has_value());
  EXPECT_EQ(both.geometry.center(*both.geometry.index(remembered)), remembered);
  EXPECT_GT(both.geometry.geometry_revision, original.geometry_revision);
  EXPECT_EQ(both.geometry.extent_source,
            GridExtentSource::SensorDerivedExpansion);
}

TEST(GridGeometry, EnforcesMaximumExtentAndMemoryLimits) {
  const auto original = expandableGeometry();
  semaforr::domain::GridExpansionPolicy policy;
  policy.maximum_width_m = original.widthMeters();
  policy.maximum_height_m = original.heightMeters();
  policy.memory_limit_cells = original.cellCount();
  const auto result =
      semaforr::domain::expandToInclude(original, {100.0, 100.0}, policy);
  EXPECT_FALSE(result.expanded);
  EXPECT_TRUE(result.resource_limited);
  EXPECT_FALSE(result.diagnostic.empty());
}

TEST(GridGeometry, SerializationRestoresAllGeometryMetadata) {
  auto original = expandableGeometry();
  original.geometry_revision = 17U;
  original.map_identifier = "map-checksum-123";
  const auto restored = semaforr::domain::deserializeGridGeometry(
      semaforr::domain::serializeGridGeometry(original));
  EXPECT_EQ(restored, original);
}

TEST(GridGeometry, FixedExtentRejectsExpansionAndOutsideQueries) {
  const auto fixed = GridGeometry::fromBounds(
      "map", {-10.0, -5.0}, {3.0, 2.0}, 0.5, GridExtentMode::Fixed,
      GridExtentSource::StaticMapBounds,
      GridOutOfBoundsBehavior::NonTraversable);
  const auto result = semaforr::domain::expandToInclude(fixed, {4.0, 0.0}, {});
  EXPECT_FALSE(result.expanded);
  EXPECT_FALSE(fixed.index({4.0, 0.0}).has_value());
  EXPECT_NE(result.diagnostic.find("fixed grid"), std::string::npos);
}
