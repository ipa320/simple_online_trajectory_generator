#include <gtest/gtest.h>

size_t sotgTest();

TEST(TrajectoryGeneratorTestSuit, jsonFileTest) { ASSERT_EQ(sotgTest(), 0); }

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
