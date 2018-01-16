/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MOLib.h"

class Robot : public frc::SampleRobot {
public:
	Robot() {
		mtr_L_Drive.SetInverted(false);
		mtr_R_Drive.SetInverted(true);
	}

	void RobotInit() {

	}

	void Autonomous() {

	}

	void OperatorControl() override {
		while (IsOperatorControl() && IsEnabled()) {
			rbt_Drivetrain.Update();
			frc::Wait(0.005);
		}
	}

	void Test() override {}

private:
	CTRLib::TalonSRX mtr_L_Drive{0};
	CTRLib::TalonSRX mtr_R_Drive{1};
	WPILib::Solenoid sol_Shifter{0};
	MOLib::Drivetrain::TankScheme::TankDrivetrain rbt_Drivetrain{&mtr_L_Drive, &mtr_R_Drive, &sol_Shifter};
};

START_ROBOT_CLASS(Robot)
