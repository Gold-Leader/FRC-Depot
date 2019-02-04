#include <iostream>
#include <string>
#include <memory>
#include <stdlib.h>
#include <cmath>
//Main header files.
#include <Robot.h>
#include "frc/WPILib.h"
//Peripheral header files.
#include "frc/Doublesolenoid.h"
#include "frc/Timer.h"
#include <frc/AnalogGyro.h>
#include <cmath>
#include <frc/ADXRS450_Gyro.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>

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

	// elevatorAlpha->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 1, 10);
	// elevatorAlpha->SetSensorPhase(true);

	// elevatorAlpha->Config_kP(1, 1, 10);
	// elevatorAlpha->Config_kI(1, 1, 10);
	// elevatorAlpha->Config_kD(1, 1, 10);

	// elevatorMasterMotor->ConfigForwardSoftLimitEnable(true);
	// elevatorMasterMotor->ConfigForwardSoftLimitThreshold((618.0775 * 7) + 100);
	// elevatorMasterMotor->ConfigReverseSoftLimitEnable(false);
	// elevatorMasterMotor->ConfigReverseSoftLimitThreshold((618.0775 * 1) - 100);

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
	// elevatorAlpha->SetSelectedSensorPosition(0, 1, 10);
	MANUAL_OVERRIDE();
}

void Robot::TeleopPeriodic() {
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

	// SmartDashboard::PutNumber("Linear Slide Encoder Value", elevatorAlpha->GetSelectedSensorPosition());
	// SmartDashboard::PutNumber("Linear Slide Error", elevatorAlpha->GetClosedLoopError(1));
	// SmartDashboard::PutNumber("Linear Slide Target", elevatorAlpha->GetClosedLoopTarget(1));
	// SmartDashboard::PutNumber("Code Target", elevatorTarget);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
