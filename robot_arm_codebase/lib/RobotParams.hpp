#ifdef ARDUINO
#include <Vector.h>
#else
#include <vector>
#endif

#include "types.hpp"

/*
    Following are the parameters of the "ideal" geometry of the arm. That is, joints are all centred and exactly in-line/parallel.
    This allows the geometric 3DoF IK approach to work as expected and be accurate enough. This way we can use the DH matrix and
    the method mentioned in the section 5.2 of the Robot Modeling and Control, 2nd Edition textbook to be implemented properly.
    Of course, this means the final position is not exact as the robot body has some offsets.
*/
const vector<DHParams> globalJointParams =
    {
        DHParams(
            167.719, /// 1000.0,
            0.0,
            0.0,
            0.0,
            {rad(0), rad(180)}),
        DHParams(
            0,//19.898, // 1000.0, // shoulder offset denoted as d in the Figure 5.6 of the book
            0,
            rad(90),
            0.0,
            {rad(0), rad(180)}),
        DHParams(
            0,
            250.201, /// 1000.0,
            0,
            0.0,
            {rad(-40), rad(220)}),
        DHParams(
            231.03, /// 1000.0,
            0,
            rad(90),
            0.0,
            {rad(-180), rad(180)}),
        DHParams(
            0,
            0,
            -rad(90),
            0.0,
            {rad(-100), rad(100)}),
        DHParams(
            156.927, /// 1000.0, // centre of joint to centre of ball
            0,
            rad(90),
            0.0,
            {rad(-180), rad(180)}),
};

/*
    The following are the exact or precise joint parameters. These include the real-world offsets from fusion that
    allow for exact calculations. The probe/DIFF mode uses these prameters to operate to showcase the precision of
    the system. These will later be used for itterative IK as well since the itterative approach allows for any DH
    parameters to be used.
*/
const vector<DHParams> preciseJointParams =
    {
        DHParams(
            167.719 / 1000.0,
            0.0,
            0.0,
            0.0),
        DHParams(
            19.898 / 1000.0,
            0,
            rad(90),
            0.0),
        DHParams(
            -0.39 / 1000.0,
            250.201 / 1000.0,
            0,
            0.0),
        DHParams(
            231.03 / 1000.0,
            0,
            rad(90),
            0.0),
        DHParams(
            2.785 / 1000.0,
            0,
            -rad(90),
            0.0),
        DHParams(
            156.927 / 1000.0,
            0,
            rad(90),
            0.0),
};