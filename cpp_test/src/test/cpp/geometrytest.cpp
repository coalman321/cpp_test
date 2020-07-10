
#pragma once
#include "lib/geometry/Pose2d.h"
#include "cstring"
#include "gtest/gtest.h"

TEST(CSVTest, Pose2dCSV){
    Pose2d geometry = Pose2d();
    //printf("%s\n", geometry.toCSV().c_str());
    EXPECT_EQ(geometry.toCSV().length(), strlen("0.000000, 0.000000, 0.000000"));
    EXPECT_EQ(geometry.getNumFields(), 3);
}

TEST(CSVTest, Translation2dCSV){
    Translation2d geometry = Translation2d();
    //printf("%s\n", geometry.toCSV().c_str());
    EXPECT_EQ(geometry.toCSV().length(), strlen("0.000000, 0.000000"));
    EXPECT_EQ(geometry.getNumFields(), 2);
}

TEST(CSVTest, Rotation2dCSV){
    Rotation2d geometry = Rotation2d();
    //printf("%s\n", geometry.toCSV().c_str());
    EXPECT_EQ(geometry.toCSV().length(), strlen("0.000000"));
    EXPECT_EQ(geometry.getNumFields(), 1);
}