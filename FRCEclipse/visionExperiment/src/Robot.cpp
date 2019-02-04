#include <iostream>
#include <string>
#include <memory>
#include <stdlib.h>
#include <cmath>
//Main header files.
#include <Robot.h>
#include "WPILib.h"
//Peripheral header files.
#include "Doublesolenoid.h"
#include "Timer.h"
#include <AnalogGyro.h>
#include <cmath>
#include <ADXRS450_Gyro.h>
#include <ctre/Phoenix.h>
//Vision header files
#include "CameraServer.h"
#include "vision/VisionPipeline.h"
#include "vision/VisionRunner.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "C:\Users\GoldA\GRIP\GripPipeline.h"
#include "C:\Users\GoldA\GRIP\GripPipeline.cpp"

using namespace cs;
using namespace std;

grip::GripPipeline *visionPipeline;

frc::VisionRunner *visionThread;

const int imageHeight = 480;
const int imageWidth = 640;

void Robot::RobotInit() {
    UsbCamera robotVision;
    robotVision.SetResolution(imageWidth, imageHeight);
    robotVision.SetFPS(25);
    visionPipeline = new grip::GripPipeline();

    cv::Mat videoSource = robotVision;

    visionThread->RunForever();
    visionThread->frc::VisionRunner(robotVision, visionPipeline());

    cv::Mat videoOutput;

}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {

}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot)
