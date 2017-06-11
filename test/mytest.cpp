#include <ros/ros.h>
#include <gtest/gtest.h>
#include <thread>
#include "my_math.h"

TEST(MyTest, knobelTest) {
    MyMath mymath;
    EXPECT_EQ(1, mymath.knobel(7,13)) << "gcd(7,13) = 1";
    ASSERT_EQ(27, mymath.knobel(27,27)) << "gcd(27,27) = 27";
    EXPECT_EQ(6, mymath.knobel(12,18)) << "gcd(12,18) = 6";
    EXPECT_EQ(5, mymath.knobel(5,0)) << "gcd(5,0) = 5";
    EXPECT_THROW(mymath.knobel(12,-18), std::invalid_argument) 
        << "Our gcd isn't defined for negative values";
}

class MyMathTestSuite : public ::testing::Test, public MyMath {
public:
    MyMathTestSuite() {
    	mLastBase = 5;
    }
};

TEST_F(MyMathTestSuite, nextSquareTest) {
	EXPECT_EQ(5, mLastBase);
	EXPECT_EQ(36, nextSquare()) << "Square of 6 should be 36";
	EXPECT_EQ(6, mLastBase);
	EXPECT_EQ(49, nextSquare()) << "Square of 7 should be 49";
	EXPECT_EQ(7, mLastBase);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "MyTestNode");
    testing::InitGoogleTest(&argc, argv);
    ros::NodeHandle roshandle;
    std::thread t([]{while(ros::ok()) ros::spin();});
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
    	ros::console::levels::Info) ) {
       ros::console::notifyLoggerLevelsChanged();
    }
    // ::testing::GTEST_FLAG(filter) = "MyTest.evaluationTimeTest";
    auto res = RUN_ALL_TESTS();
    ros::shutdown();
    return res;
}