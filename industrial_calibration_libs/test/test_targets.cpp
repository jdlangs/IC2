#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <gtest/gtest.h>

TEST(Targets, temp_test)
{
  EXPECT_TRUE(industrial_calibration_libs::number == 5);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}