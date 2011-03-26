/*
 *  ART steering servo controller device interface unit test
 *
 *  Copyright (C) 2008 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <gtest/gtest.h>
#include "devsteer.h"

void compare_ticks_and_degrees(int32_t center)
{
  devsteer dev(center);                 // initializes center_ticks_

  EXPECT_EQ(0.0, dev.ticks2degrees(center));
  EXPECT_EQ(center, dev.degrees2ticks(0.0));

  for (int32_t tick = -300000; tick <= 300000; tick += 100)
    {
      EXPECT_EQ(tick, dev.degrees2ticks(dev.ticks2degrees(tick)));
    }
}

TEST(Conversions, ticks2degreesCentered)
{
  compare_ticks_and_degrees(0);
}

TEST(Conversions, ticks2degreesOffsetLeft)
{
  compare_ticks_and_degrees(-130);
}

TEST(Conversions, ticks2degreesOffsetRight)
{
  compare_ticks_and_degrees(1024);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
