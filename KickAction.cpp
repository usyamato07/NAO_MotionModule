
/*
 * This example demonstrates how to get images from the robot remotely and how
 * to display them on your screen using opencv.
 *
 * Copyright Aldebaran Robotics
 */

// Aldebaran includes.

#include <alproxies/almotionproxy.h>


#include <iostream>
#include <string>
using namespace AL;

class KickAction
{
private:
  ALPtr<ALMotionProxy> motionProxy;
public:
  KickAction()
  {
    const std::string robotIp("127.0.0.1");
    motionProxy = makeALPtr(new ALMotionProxy(robotIp, 9559));
  }
  ~KickAction()
  {
  }


  void initialize()
  {
    ALValue angles;
    ALValue names;

    // Create a proxy to ALVideoDevice on the robot.


    float fractionMaxSpeed = 0.1f;

    names = ALValue::array("HeadYaw","HeadPitch","LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll");
    angles = ALValue::array(0.0f, -0.0f, 1.4f, 0.35f, -1.4f, -1.05f);
    motionProxy->setAngles(names, angles, fractionMaxSpeed);

    names = ALValue::array("LHipYawPitch","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll");
    angles = ALValue::array(0.0f, -0.0f, -0.44f, 0.7f, -0.35, 0.0f);
    motionProxy->setAngles(names, angles, fractionMaxSpeed);

    names = ALValue::array("RHipYawPitch","RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll");
    angles = ALValue::array(0.0f, -0.0f, -0.44f, 0.7f, -0.35f, 0.0f);
    motionProxy->setAngles(names, angles, fractionMaxSpeed);

    names = ALValue::array("RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll");
    angles = ALValue::array(1.4f, -0.35f, 1.4f, 1.05f);
    motionProxy->setAngles(names, angles, fractionMaxSpeed);
  }

  void shoot()
  {
    ALValue angles;
    ALValue names;
 
   //Right Foot support 
    float fractionMaxSpeed = 0.07f;
    names = ALValue::array("RAnkleRoll","LAnkleRoll","RHipRoll","LHipRoll");
    angles = ALValue::array(-0.33f, -0.33f, 0.25f, 0.33f);
    motionProxy->angleInterpolationWithSpeed(names, angles, fractionMaxSpeed);

    //Rise Left Foot
    names = ALValue::array("LHipPitch", "LKneePitch","LAnklePitch");
    angles = ALValue::array(-0.523f, 1.398f, -0.875f);
    motionProxy->angleInterpolationWithSpeed(names, angles, fractionMaxSpeed);

    //Get Left Leg Position
    std::string effector = "LLeg"; 
    int space = 2; 
    bool useSensorValues = true; 
    std::vector<float> legPos = motionProxy->getPosition(effector, space, useSensorValues); 

    //Move Left Leg Forward
    legPos[0] += 0.20f;
    legPos[1] -= 0.05f;
    legPos[2] -= 0.05f;
    ALValue	times      = 0.5f; // seconds 
    bool isAbsolute = false; 
    int axisMask = 63;
    motionProxy->positionInterpolation(effector, space, legPos, axisMask, times, isAbsolute); 

    //Get Left Leg Position
    legPos = motionProxy->getPosition(effector, space, useSensorValues);

    //Move Left Leg Backward
    legPos[0] -= 0.2f;
    legPos[2] -= 0.095f;
    legPos[4] -= 0.4f;
    times      = 2.0f; // seconds  
    motionProxy->positionInterpolation(effector, space, legPos, axisMask, times, isAbsolute); 
  }
};

int main(int argc, char* argv[])
{
  KickAction kickAction;
  kickAction.initialize();
  kickAction.shoot();
  kickAction.initialize();
  return 0;
}
