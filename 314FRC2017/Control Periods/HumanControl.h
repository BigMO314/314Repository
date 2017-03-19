#pragma once

#include "MOLib.h"
#include "Robot Specifications.h"
#include "BallManagement.h"
#include "Gear.h"
#include "Climber.h"

namespace ControlPeriod{
	class HumanControl{
	private:
		//Defines the joysricks,drivetrains,shooter,Intakes,and gear thingeys.
		MOLib::XBoxController* ctl_Driver;
		MOLib::XBoxController* ctl_Operator;
		MOLib::TankDrivetrain* rbt_Drivetrain;
		Robot::BallManagement* rbt_BallManagement;
		Robot::Gear* rbt_Gear;
		Robot::Climber* rbt_Climber;

	public:
		HumanControl(
				MOLib::XBoxController* ref_Driver, MOLib::XBoxController* ref_Operator,
				MOLib::TankDrivetrain* ref_Drivetrain, Robot::BallManagement* ref_BallManagement,
				Robot::Gear* ref_Gear, Robot::Climber* ref_Climber){
			this->ctl_Driver = ref_Driver;
			this->ctl_Operator = ref_Operator;

			this->rbt_Drivetrain = ref_Drivetrain;
			this->rbt_BallManagement = ref_BallManagement;
			this->rbt_Gear = ref_Gear;
			this->rbt_Climber = ref_Climber;
		}

		void Update(){
			//TODO Complete Control Finalizing

			//Drivetrain Controls																																							.

			rbt_Drivetrain->SetTankDrive(ctl_Driver->GetY(MOLib::XBoxController::kLeftHand), ctl_Driver->GetY(MOLib::XBoxController::kRightHand));
			rbt_Drivetrain->SetShift((ctl_Driver->GetBumper(MOLib::XBoxController::kLeftHand) ? MOLib::ShiftState::kHighSpeed : MOLib::ShiftState::kLowSpeed));

			//BallManagement Controls																																						.
			if(ctl_Operator->GetBButton()) { rbt_BallManagement->ExtendIntake(); }
			else if(ctl_Operator->GetYButton()) { rbt_BallManagement->RetractIntake(); }

			if(ctl_Operator->GetBumper(MOLib::XBoxController::kRightHand)) { rbt_BallManagement->SetIntakePower(1.0); }
			else if(ctl_Operator->GetAButton()) { rbt_BallManagement->SetIntakePower(1.0); }
			else { rbt_BallManagement->SetIntakePower(ctl_Operator->GetTriggerAxis(MOLib::XBoxController::kRightHand)); }

			if(ctl_Operator->GetAButton()){
				rbt_BallManagement->SetBeltingPower(1.0);
				rbt_BallManagement->SetKickerPower(1.0);
				rbt_BallManagement->SetUptakePower(1.0);
			}
			else{
				rbt_BallManagement->SetBeltingPower(0.0);
				rbt_BallManagement->SetKickerPower(0.0);
				rbt_BallManagement->SetUptakePower(0.0);
			}

			if(ctl_Driver->GetBButton()) { rbt_BallManagement->RaiseHood(); }
			else if(ctl_Driver->GetYButton()) { rbt_BallManagement->LowerHood(); }

			if(ctl_Operator->GetX(MOLib::XBoxController::kLeftHand) > 0.40) { rbt_BallManagement->RaiseHood(); }
			if(ctl_Operator->GetX(MOLib::XBoxController::kLeftHand) < -0.40) { rbt_BallManagement->LowerHood(); }


			//if(ctl_Driver->GetAButton()) { rbt_Drivetrain->AlignToGoal(); }
			//if(ctl_Driver->GetBButton()) { rbt_Drivetrain->StopVisionPID(); }
			//Gear Controls																																								.

			if(ctl_Operator->GetY(MOLib::XBoxController::kLeftHand) > 0.40) { rbt_Gear->RetractSlide(); }
			if(ctl_Operator->GetY(MOLib::XBoxController::kLeftHand) < -0.40) { rbt_Gear->ExtendSlide(); }
			if(ctl_Operator->GetTriggerAxis(MOLib::XBoxController::kLeftHand) > 0.1){ rbt_Gear->OpenHolder(); }
			else { rbt_Gear->CloseHolder(); }

			//Climber Controls																																								.

			rbt_Climber->SetPower(ctl_Driver->GetTriggerAxis(MOLib::XBoxController::kRightHand));
		}
	};
}
