#ifndef SUTURO_MANIPULATION_TEST_REFLEXXES_INTERPOLATOR
#define SUTURO_MANIPULATION_TEST_REFLEXXES_INTERPOLATOR

#include "suturo_manipulation_reflexxes_interpolator.h"

class Suturo_Manipulation_Test_Reflexxes_Interpolator : public Suturo_Manipulation_Reflexxes_Interpolator
{
public:
	Suturo_Manipulation_Test_Reflexxes_Interpolator();

    ~Suturo_Manipulation_Test_Reflexxes_Interpolator();

    bool setInputFromOutput();

    bool interpolate(geometry_msgs::PoseStamped robotPose, geometry_msgs::PoseStamped targetPose);
};

#endif