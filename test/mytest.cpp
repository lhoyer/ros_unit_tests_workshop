#include <ros/ros.h>
#include <gtest/gtest.h>
#include <thread>
#include "my_math.h"

TEST(MyTest, knobelTest) {
    MyMath mymath;
    EXPECT_EQ(1, mymath.knobel(7,13));
    EXPECT_EQ(27, mymath.knobel(27,27));
    EXPECT_EQ(6, mymath.knobel(12,18));
    EXPECT_EQ(5, mymath.knobel(5,0));
    EXPECT_THROW(mymath.knobel(12,-18), std::invalid_argument);
}

class MyMathTestSuite : public ::testing::Test, public MyMath {
public:
    MyMathTestSuite() {
    	mLastSquare = 5;
    }
};

TEST_F(MyMathTestSuite, nextSquareTest) {
	EXPECT_EQ(5, mLastSquare);
	EXPECT_EQ(36, nextSquare());
	EXPECT_EQ(6, mLastSquare);
	EXPECT_EQ(49, nextSquare());
	EXPECT_EQ(7, mLastSquare);
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