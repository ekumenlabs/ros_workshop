#include <gmock/gmock.h>

namespace {

TEST(Example, Should_Pass) {
  ASSERT_TRUE(true);
}

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
