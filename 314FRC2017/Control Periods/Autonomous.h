#pragma once
#include "MOLib.h"
#include "BallManagement.h"
#include "Gear.h"
#include "Drivetrain2.h"

#define StageMachine	switch(AutonStage)
#define Stage			case
#define NextStage		AutonStage++; break

namespace ControlPeriod{

	class Autonomous{
		//TODO Tune DriveDistance, DriveStraight, and DriveAngle
		//TODO Tune Autonomous
		//TODO Redo Shooter Speed Distances
		//TODO Test Climber Control -- After Fixed
		//TODO Impliment DriveStraight into HumanControl
	private:
		MOLib::TankDrivetrain* rbt_Drivetrain;
		Robot::BallManagement* rbt_BallManagement;
		Robot::Gear* rbt_Gear;

		WPILib::Timer tmr_Autonomous;

		int runCount = 0;
		int AutonStage;
		int m_SelectedAlliance = 1;
		std::string m_SelectedAuton;



	public:
		WPILib::SendableChooser<std::string>	chs_Alliance;

		WPILib::SendableChooser<std::string>	chs_Auton;
		const std::string kDoNothing		= "Do Nothing";
		const std::string kNearHopperShot	= "Near Hopper Shot";
		const std::string kGearSide			= "Gear Side";
		const std::string kTest				= "Test";

		Autonomous(MOLib::TankDrivetrain* ref_Drivetrain, Robot::BallManagement* ref_BallManagement, Robot::Gear* ref_GearManagement){
			this->rbt_Drivetrain = ref_Drivetrain;
			this->rbt_BallManagement = ref_BallManagement;
			this->rbt_Gear = ref_GearManagement;
			this->AutonStage = 0;

			chs_Alliance.AddDefault("Red Alliance", "Red Alliance");
			chs_Alliance.AddObject("Blue Alliance", "Blue Alliance");

			//SmartDashboard::PutData("Selected Alliance", &chs_Alliance);

			chs_Auton.AddDefault(kDoNothing, kDoNothing);
			chs_Auton.AddObject(kNearHopperShot, kNearHopperShot);
			chs_Auton.AddObject(kGearSide, kGearSide);
			chs_Auton.AddObject(kTest, kTest);

			//SmartDashboard::PutData("Selected Auton", &chs_Auton);
		}
		void AutonomousInit(){
			AutonStage = 0;
			m_SelectedAuton		= chs_Auton.GetSelected();
			m_SelectedAlliance	= (chs_Alliance.GetSelected() == "Red Alliance" ? 1 : -1);

			tmr_Autonomous.Reset();
			tmr_Autonomous.Start();

			rbt_Drivetrain->StopAnglePID();
			rbt_Drivetrain->StopDistancePID();
			rbt_Drivetrain->ResetDistance();
			rbt_Drivetrain->ResetAngle();

			rbt_BallManagement->StopShooterPID();
			rbt_BallManagement->SetShooterPower(0, 0);
		}

		void Update(){
					 if(m_SelectedAuton == kDoNothing)		Auton_DoNothing();
				else if(m_SelectedAuton == kNearHopperShot)	Auton_NearHopperShot();
				else if(m_SelectedAuton == kGearSide)		Auton_GearSide();
				else if(m_SelectedAuton == kTest)			Auton_Test();
				else{ Print("Error: Unknown or no auton selected"); }
			rbt_Drivetrain->Update();
			rbt_BallManagement->Update();
		}

	private:
		void Auton_DoNothing(){
			StageMachine{
				Stage  0: Print("Starting Autonomous [Do Nothing]");																				NextStage;
				Stage  1: Print("Autonomous Complete [Do Nothing]");																				NextStage;
			}
		};

		void Auton_NearHopperShot(){
			StageMachine{
				Stage  0: Print("Starting Autonomous [Near Hopper Shot]");																			NextStage;

				Stage  1: Print("--Extending Intake"); Print("--Driving to 10ft 8in");																NextStage;
				Stage  2: rbt_BallManagement->ExtendIntake(); rbt_Drivetrain->GoToDistance(10.0_ft + 8.0_in); tmr_Autonomous.Reset();				NextStage;
				Stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 6.0)															NextStage;

				Stage  4: Print("--Turning 90deg");																								NextStage;
				Stage  5: rbt_Drivetrain->GoToAngle(90.0 * m_SelectedAlliance); tmr_Autonomous.Reset();												NextStage;
				Stage  6: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 6.0)												AutonStage = 10; break;

				Stage  7: Print("--Driving to 4ft 0in");																							NextStage;
				Stage  8: rbt_Drivetrain->GoToDistance(4.0_ft); tmr_Autonomous.Reset();																NextStage;
				Stage  9: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 6.0)															NextStage;

				Stage 10: Print("--Driving forward at 40% for 1.5s");																					NextStage;
				Stage 11: rbt_Drivetrain->StopDistancePID(); rbt_Drivetrain->SetTankDrive(0.4, 0.4); tmr_Autonomous.Reset();						NextStage;
				Stage 12: if(tmr_Autonomous.Get() > 1.5)																							NextStage;

				Stage 13: Print("--Driving backwards at 40% for 0.5s");																				NextStage;
				Stage 14: rbt_Drivetrain->SetTankDrive(-0.4, -0.4);  tmr_Autonomous.Reset();														NextStage;
				Stage 15: if(tmr_Autonomous.Get() > 0.5)																							NextStage;

				Stage 16: Print("--Turning 80deg");																									NextStage;
				Stage 17: rbt_Drivetrain->GoToAngle(80.0 * m_SelectedAlliance); tmr_Autonomous.Reset();												NextStage;
				Stage 18: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 6.0)																NextStage;

				Stage 19: Print("--Enabling shooter at 5350rpm");  Print("--Lowering Hood");														NextStage;
				Stage 20: rbt_BallManagement->SetShooterSpeed(5350, 5350); rbt_BallManagement->LowerHood(); tmr_Autonomous.Reset();					NextStage;
				Stage 21: if(tmr_Autonomous.Get() > 2.0)																							NextStage;

				Stage 22: Print("--Enabling Indexer for 10s");																						NextStage;
				Stage 23: rbt_BallManagement->EnableIndexer(); tmr_Autonomous.Reset();																NextStage;
				Stage 24: if(tmr_Autonomous.Get() > 10.0)																							NextStage;

				Stage 25: Print("Autonomous Completed [Near Hopper Shot]"); Print("--Stopping all systems");										NextStage;
				Stage 26: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  rbt_BallManagement->SetShooterSpeed(0.0, 0.0);
						  rbt_BallManagement->DisableIndexer();																						NextStage;
			}
		}

		void Auton_GearSide(){
			StageMachine{
				Stage  0: Print("Starting Autonomous [GearSide]");																					NextStage;

				Stage  1: Print("--Driving backwards to 10 feet");																					NextStage;
				Stage  2: rbt_Drivetrain->GoToDistance(10.0_ft); tmr_Autonomous.Reset();															NextStage;
				Stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 5)															NextStage;

				stage  4: Print("--Turning to 45 Degrees");																							NextStage;
				stage  5: rbt_Drivetrain->GoToAngle(-45.0 * m_SelectedAlliance); tmr_Autonomous.Reset();											NextStage;
				Stage  6: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 5)																NextStage;

				stage  7: Print("--Drive 3 feet into Gear Hook");																					NextStage;
				stage  8: rbt_Drivetrain->GoToDistance(3.0_ft); tmr_Autonomous.Reset();																NextStage;
				stage  9: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 5)															NextStage;

				stage 10: Print("--Release gear and drive forward 2 feet");																			NextStage;
				stage 11: rbt_Gear->OpenHolder(); rbt_Drivetrain->GoToDistance(2.0_ft); tmr_Autonomous.Reset();										NextStage;
				stage 12: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 5)															NextStage;

				stage 13: Print("Autonomous Completed [GearSide]"); Print("--Closing gear holder"); Print("--Stopping all systems");				NextStage;
				stage 14: rbt_Gear->CloseHolder(); rbt_Drivetrain->SetTankDrive(0.0, 0.0); rbt_BallManagement->SetShooterSpeed(0.0, 0.0);			NextStage;
			}
		}

		void Auton_Test(){
			StageMachine{
				Stage  0: rbt_Drivetrain->GoToDistance(125); tmr_Autonomous.Reset();		NextStage;

				Stage  1: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 6)	NextStage;

				Stage  2: rbt_Drivetrain->SetArcadeDrive(0, 0);
			}
		}
	};
}
