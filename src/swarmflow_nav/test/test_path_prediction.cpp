#include <gtest/gtest.h>
#include <cmath>
#include "swarmflow_nav/path_prediction.hpp"

using swarmflow_nav::posesAlongPath;
using P = std::vector<std::pair<double, double>>;

TEST(PathPrediction, StraightPathAtSpeed)
{
  const P path{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {10, 0}};
  auto p = posesAlongPath(path, 0.5, 0.1, 0.5, 0.4, 3, 0.5);  // 0.2 m per step from x = 0.5
  ASSERT_EQ(p.size(), 3u);
  EXPECT_NEAR(p[0].x, 0.7, 1e-9);
  EXPECT_NEAR(p[2].x, 1.1, 1e-9);
  EXPECT_NEAR(p[2].y, 0.0, 1e-9);
  EXPECT_NEAR(p[2].yaw, 0.0, 1e-9);
}

TEST(PathPrediction, FollowsTheCorner)  // east 1 m, then north: a constant-heading model would go on east
{
  const P path{{0, 0}, {1, 0}, {1, 1}, {1, 3}};
  auto p = posesAlongPath(path, 0.0, 0.0, 0.5, 1.0, 4, 0.5);  // 0.5 m per step
  ASSERT_EQ(p.size(), 4u);
  EXPECT_NEAR(p[1].x, 1.0, 1e-9);
  EXPECT_NEAR(p[1].y, 0.0, 1e-9);
  EXPECT_NEAR(p[2].x, 1.0, 1e-9);
  EXPECT_NEAR(p[2].y, 0.5, 1e-9);
  EXPECT_NEAR(p[2].yaw, M_PI / 2, 1e-9);
  EXPECT_NEAR(p[3].y, 1.0, 1e-9);
}

TEST(PathPrediction, StopsAtTheGoal)
{
  const P path{{0, 0}, {1, 0}};
  auto p = posesAlongPath(path, 0.0, 0.0, 0.6, 1.0, 5, 0.5);
  ASSERT_EQ(p.size(), 2u);
  EXPECT_NEAR(p[1].x, 1.0, 1e-9);
}

TEST(PathPrediction, FallsBackWhenOffPathOrNotMovingForward)
{
  const P path{{0, 0}, {1, 0}, {2, 0}};
  EXPECT_TRUE(posesAlongPath(path, 1.0, 0.8, 0.5, 0.4, 5, 0.5).empty());   // 0.8 m off the path
  EXPECT_TRUE(posesAlongPath(path, 1.0, 0.0, 0.0, 0.4, 5, 0.5).empty());   // standing
  EXPECT_TRUE(posesAlongPath(path, 1.0, 0.0, -0.1, 0.4, 5, 0.5).empty());  // reversing
  EXPECT_TRUE(posesAlongPath(P{{0, 0}}, 0.0, 0.0, 0.5, 0.4, 5, 0.5).empty());
}

TEST(PathPrediction, StartsFromTheNearestPointNotTheFirst)  // a plan published 1 s ago: the robot is already along it
{
  const P path{{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  auto p = posesAlongPath(path, 2.0, 0.05, 0.5, 0.4, 1, 0.5);
  ASSERT_EQ(p.size(), 1u);
  EXPECT_NEAR(p[0].x, 2.2, 1e-9);
}
