#include <gtest/gtest.h>
#include "swarmflow_nav/prediction_cost.hpp"

using swarmflow_nav::predictedCost;

TEST(PredictionCost, FirstTwoSecondsUnchanged)  // 200 -> 100 over 2 s, as in the recorded benchmarks
{
  EXPECT_EQ(predictedCost(0.4, 2.0, 2.0, 200, 20), 180);
  EXPECT_EQ(predictedCost(2.0, 2.0, 2.0, 200, 20), 100);
  EXPECT_EQ(predictedCost(1.2, 2.0, 5.0, 200, 20), predictedCost(1.2, 2.0, 2.0, 200, 20));
}

TEST(PredictionCost, FadesToFarCostAtHorizon)
{
  EXPECT_EQ(predictedCost(2.0, 2.0, 5.0, 200, 20), 100);
  EXPECT_EQ(predictedCost(3.5, 2.0, 5.0, 200, 20), 60);
  EXPECT_EQ(predictedCost(5.0, 2.0, 5.0, 200, 20), 20);
  EXPECT_LT(predictedCost(4.0, 2.0, 5.0, 200, 20), predictedCost(3.0, 2.0, 5.0, 200, 20));
}

TEST(PredictionCost, NeverLethalNorFree)
{
  EXPECT_LE(predictedCost(0.0, 2.0, 5.0, 252, 0), 252);
  EXPECT_GE(predictedCost(5.0, 2.0, 5.0, 200, 0), 1);
}
