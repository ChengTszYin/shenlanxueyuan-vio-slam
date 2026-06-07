#include <iostream>
#include "hello.h"
#include <gtest/gtest.h>

TEST(HelloTest, test_sayhello)
{
  EXPECT_NO_THROW(sayHello());
}

int main( int argc, char** argv ) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
