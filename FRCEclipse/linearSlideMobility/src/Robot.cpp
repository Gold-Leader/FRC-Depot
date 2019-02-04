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

using namespace std;

TalonSRX *elevatorMasterMotor;
TalonSRX *elevatorSlaveMotor;

Joystick *leftJoystick;
Joystick *rightJoystick;

double elevatorThrottle;
double elevatorTarget;

int elevatorState;

bool elevatorIsOverride;
bool elevatorSoloTest;

const int hatchOffsetDistance = 1043.037835 * 28.0;
const int hatchInitialDistiance = 1043.037835 * 19.0;
const int portOffsetDistance = 1043.037835 * 28.0;
const int portInitiallDistance = 1043.037835 * 27.5;

const double elevatorData[7] = {hatchInitialDistiance,
		(portInitiallDistance),
		(hatchInitialDistiance) + (hatchOffsetDistance * 1),
		(portInitiallDistance) + (portOffsetDistance * 1),
		(hatchInitialDistiance) + (hatchOffsetDistance * 2),
		(portInitiallDistance) + (portOffsetDistance * 2),
};

void Robot::RobotInit() {

	elevatorThrottle = 0;
	elevatorTarget = 0;

	elevatorState = 1;

	elevatorIsOverride = true;
	elevatorSoloTest = true;

	elevatorMasterMotor = new TalonSRX(5);
	elevatorSlaveMotor = new TalonSRX(7);

	elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);
	elevatorSlaveMotor->Set(ControlMode::Follower, 5);

	elevatorMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	elevatorMasterMotor->SetSensorPhase(true);

//	elevatorMasterMotor->Config_kP(1, 1, 10);
//	elevatorMasterMotor->Config_kI(1, 1, 10);
//	elevatorMasterMotor->Config_kD(1, 1, 10);

//	elevatorMasterMotor->ConfigForwardSoftLimitEnable(true);
//	elevatorMasterMotor->ConfigForwardSoftLimitThreshold(((portInitiallDistance) + (portOffsetDistance * 2)) + 100);
//	elevatorMasterMotor->ConfigReverseSoftLimitEnable(false);
//	elevatorMasterMotor->ConfigReverseSoftLimitThreshold(hatchInitialDistiance - 100);

}

void MANUAL_OVERRIDE() {
	elevatorIsOverride =! elevatorIsOverride;
	elevatorSoloTest = false;
	SmartDashboard::PutString("Elevator Control State", "Is in MANUAL");
}
void AUTO_OVERRIDE() {
	elevatorIsOverride =! elevatorIsOverride;
	elevatorSoloTest = true;
	SmartDashboard::PutString("Elevator Control State", "Is in AUTO");
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
	MANUAL_OVERRIDE();
	elevatorState = 1;
	elevatorMasterMotor->SetSelectedSensorPosition(0, 1, 10);

}

void Robot::TeleopPeriodic() {
	//Every push of state up activates the next elevator state
	//Reversely, push of state down activates the previous elevator state
	//Seven unique states
	//Must constrain between 1 and 7, one is base, 7 is max
	//Must also alternate between manual and auto
	//Manual initially
	//Will need to reset encoders
	if(leftJoystick->GetRawButton(1) && elevatorSoloTest) {
		if(elevatorIsOverride) {
			AUTO_OVERRIDE();
			SmartDashboard::PutString("Mode", "In Automatic!");
		} else if(!elevatorIsOverride) {
			MANUAL_OVERRIDE();
			SmartDashboard::PutString("Mode", "In Manual!");
		}
		elevatorSoloTest = false;
	}

	if(!leftJoystick->GetRawButton(1)) {
		elevatorSoloTest = true;
	}

	if(elevatorIsOverride) {
		elevatorThrottle = leftJoystick->GetRawAxis(1);
		// elevatorAlpha->Set(ControlMode::PercentOutput, elevatorThrottle);
	} else if(!elevatorIsOverride) {
		if(leftJoystick->GetRawButton(3) && (elevatorState >= 1) && (elevatorState <= 7)) {
			elevatorState += 1;
			elevatorTarget = elevatorData[elevatorState];
			// elevatorAlpha->Set(ControlMode::Position, elevatorTarget);
		} else if(leftJoystick->GetRawButton(4) && (elevatorState >= 1) && (elevatorState <= 7)) {
			elevatorState -= 1;
			elevatorTarget = elevatorData[elevatorState];
			// elevatorAlpha->Set(ControlMode::Position, elevatorTarget);
		}

		if(elevatorState > 7) {
			elevatorState = 7;
		} else if(elevatorState < 1) {
			elevatorState = 1;
		}
	}

	SmartDashboard::PutNumber("Elevator Codeside Target", elevatorState);
	SmartDashboard::PutNumber("Elevator Talonside Target", elevatorMasterMotor->GetClosedLoopTarget(1));
	SmartDashboard::PutNumber("Elevator Talonside Error", elevatorMasterMotor->GetClosedLoopTarget(1));
	SmartDashboard::PutNumber("TalonSRX Position", elevatorMasterMotor->GetSelectedSensorPosition(1));
}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot)
