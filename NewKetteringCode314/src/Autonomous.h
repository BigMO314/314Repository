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
		const std::string kGearCenter		= "Gear Center";
		const std::string kCrossBaseLine	= "Cross Base Line";
		const std::string kCenterGearThenShoot = "Center Gear Then Shoot";
		const std::string kBoilerSideGear			= "Boiler Side Gear";
		const std::string kRetrievalSideGear		= "Retrieval Side Gear";

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
			chs_Auton.AddObject(kGearCenter, kGearCenter);
			chs_Auton.AddObject(kCrossBaseLine, kCrossBaseLine);
			chs_Auton.AddObject(kCenterGearThenShoot, kCenterGearThenShoot);
			chs_Auton.AddObject(kRetrievalSideGear, kRetrievalSideGear);
			chs_Auton.AddObject(kBoilerSideGear, kBoilerSideGear);

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

			rbt_BallManagement->DisableIndexer();

			rbt_BallManagement->RetractIntake();

			rbt_Gear->CloseHolder();

			rbt_Drivetrain->SetShift(MOLib::ShiftState::kLowSpeed);
		}

		void Update(){
					 if(m_SelectedAuton == kDoNothing)				Auton_DoNothing();
				else if(m_SelectedAuton == kNearHopperShot)			Auton_NearHopperShot();
				else if(m_SelectedAuton == kGearCenter)				Auton_CenterGear();
				else if(m_SelectedAuton == kCrossBaseLine)			Auton_CrossBaseLine();
				else if(m_SelectedAuton == kCenterGearThenShoot)	Auton_CenterGearThenShoot();
				else if(m_SelectedAuton == kBoilerSideGear)			Auton_BoilerSideGear();
				else if(m_SelectedAuton == kRetrievalSideGear)		Auton_RetrievalSideGear();
				else{ Print("Error: Unknown or no auton selected"); }
			rbt_Drivetrain->Update();
			rbt_BallManagement->Update();
			rbt_Gear->Update();
		}

	private:
		void Auton_DoNothing(){
			StageMachine{
				Stage  0: rbt_Drivetrain->SetTankDrive(0,0); Print("Starting Autonomous [Do Nothing]");												NextStage;
				Stage  1: Print("Autonomous Complete [Do Nothing]");																				NextStage;
			}
		};

		void Auton_NearHopperShot(){
			StageMachine{
				Stage  0: Print("Starting Autonomous [Near Hopper Shot]"); tmr_Autonomous.Reset();													NextStage;
				Stage  1: if(tmr_Autonomous.Get() > 0.25)																							NextStage;

				Stage  2: Print("--Extending Intake"); Print("--Driving to 10ft 8in");																NextStage;
				Stage  3: rbt_BallManagement->ExtendIntake(); rbt_Drivetrain->GoToDistance(11.0_ft + 1.0_in); tmr_Autonomous.Reset();				NextStage;//10.0_ft + 9.0_in blue
				Stage  4: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.5)															NextStage;

				Stage  5: Print("--Turning 90deg");																									NextStage;//90 deg blue
				Stage  6: rbt_Drivetrain->GoToAngle(88.0 * m_SelectedAlliance); tmr_Autonomous.Reset();												NextStage;
				Stage  7: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 3.0) AutonStage = 11;												break;

				Stage  8: Print("--Driving to 4ft 0in");																							NextStage;
				Stage  9: rbt_Drivetrain->GoToDistance(4.0_ft); tmr_Autonomous.Reset();																NextStage;
				Stage 10: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 4.0)															NextStage;

				Stage 11: Print("--Driving forward at 40% for 2.5s");																				NextStage;
				Stage 12: rbt_Drivetrain->StopDistancePID(); rbt_Drivetrain->SetTankDrive(0.8, 0.8);
				rbt_BallManagement->SetShooterSpeed(5300, 5300); tmr_Autonomous.Reset();															NextStage;//5400 blue
				Stage 13: if(tmr_Autonomous.Get() > 2.5)																							NextStage;

				Stage 14: Print("--Driving backwards at 40% for 0.5s");																				NextStage;
				Stage 15: rbt_Drivetrain->SetTankDrive(-0.4, -0.4);  tmr_Autonomous.Reset();														NextStage;
				Stage 16: if(tmr_Autonomous.Get() > 0.5)																							NextStage;

				Stage 17: Print("--Enabling shooter at 5400rpm");  Print("--Lowering Hood");														NextStage;
				Stage 18:  rbt_BallManagement->RaiseHood();																							NextStage;

				Stage 19: Print("--Turning 73deg");																									NextStage;
				Stage 20: rbt_Drivetrain->GoToAngle(76.0 * m_SelectedAlliance); tmr_Autonomous.Reset();												NextStage;//76 blue
				Stage 21: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 2.2)																NextStage;

				Stage 22: Print("--Enabling Indexer for 10s");																						NextStage;
				Stage 23:  tmr_Autonomous.Reset();																									NextStage;
				Stage 24: rbt_BallManagement->EnableIndexer(); if(tmr_Autonomous.Get() > 10.0)																							NextStage;

				Stage 25: Print("Autonomous Completed [Near Hopper Shot]"); Print("--Stopping all systems");										NextStage;
				Stage 26: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  rbt_BallManagement->SetShooterSpeed(0.0, 0.0);
						  rbt_BallManagement->DisableIndexer();																						NextStage;
			}
		}

		void Auton_RetrievalSideGear(){
			StageMachine{
				Stage  0: Print("Starting Autonomous [Retrieval Side Gear]");																		NextStage;

				Stage  1: Print("--Driving backwards to 6 feet and 6 inches");																		NextStage;
				Stage  2: rbt_Drivetrain->GoToDistance(-(6.0_ft + 6.0_in)); tmr_Autonomous.Reset();													NextStage;
				Stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)															NextStage;

				stage  4: Print("--Turning to 45 Degrees");																							NextStage;
				stage  5: rbt_Drivetrain->GoToAngle(60.0 * m_SelectedAlliance); tmr_Autonomous.Reset();												NextStage;
				Stage  6: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 2.0)																NextStage;

				stage  7: Print("--Drive 5 feet into Gear Hook");																					NextStage;
				stage  8: rbt_Drivetrain->GoToDistance(-69); tmr_Autonomous.Reset();																NextStage;
				stage  9: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)															NextStage;

				Stage 10: Print("--Wiggling for 0.8 seconds");																						NextStage;
				Stage 11: rbt_Drivetrain->StopDistancePID(); rbt_Drivetrain->SetTankDrive(0.3, -0.3); tmr_Autonomous.Reset();						NextStage;
				Stage 12: if(tmr_Autonomous.Get() > 0.4)																							NextStage;
				Stage 13: rbt_Drivetrain->SetTankDrive(-0.3, 0.3); tmr_Autonomous.Reset();															NextStage;
				Stage 14: if(tmr_Autonomous.Get() > 0.4)																							NextStage;
				Stage 15: rbt_Drivetrain->SetTankDrive(0.0, 0.0); tmr_Autonomous.Reset();															NextStage;

				stage 16: Print("--Release gear and drive forward 2 feet");																			NextStage;
				stage 17: rbt_Gear->OpenHolder(); tmr_Autonomous.Reset();																			NextStage;
				stage 18: if(tmr_Autonomous.Get() > 0.5)																							NextStage;

				stage 19: rbt_Drivetrain->StopDistancePID(); rbt_Drivetrain->GoToDistance(2.0_ft); tmr_Autonomous.Reset();																NextStage;
				stage 20: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 3.0)															NextStage;

				stage 21: Print("Autonomous Completed [GearSide]"); Print("--Closing gear holder"); Print("--Stopping all systems");				NextStage;
				stage 22: rbt_Gear->CloseHolder(); rbt_Drivetrain->SetTankDrive(0.0, 0.0); rbt_BallManagement->SetShooterSpeed(0.0, 0.0);			NextStage;
			}
		}

		void Auton_BoilerSideGear(){
			StageMachine{
				Stage  0: Print("Starting Autonomous [GearSide]");																					NextStage;

				Stage  1: Print("--Driving backwards to 6 feet");																					NextStage;
				Stage  2: rbt_Drivetrain->GoToDistance(-81); tmr_Autonomous.Reset();																NextStage;
				Stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)															NextStage;

				stage  4: Print("--Turning to 45 Degrees");																							NextStage;
				stage  5: rbt_Drivetrain->GoToAngle(-60.0 * m_SelectedAlliance); tmr_Autonomous.Reset();											NextStage;
				Stage  6: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1.2)																NextStage;

				stage  7: Print("--Drive 5 feet into Gear Hook");																					NextStage;
				stage  8: rbt_Drivetrain->GoToDistance(-68.0); tmr_Autonomous.Reset();																NextStage;
				stage  9: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)															NextStage;

				stage 10: Print("--Release gear and drive forward 2 feet");																			NextStage;
				stage 11: rbt_Gear->OpenHolder(); tmr_Autonomous.Reset();																			NextStage;
				stage 12: if(tmr_Autonomous.Get() > 0.5)																							NextStage;
				Stage 13: rbt_BallManagement->SetShooterSpeed(5500, 5500); rbt_BallManagement->LowerHood();											NextStage;
				stage 14: rbt_Drivetrain->StopDistancePID(); rbt_Drivetrain->GoToDistance(2.0_ft); tmr_Autonomous.Reset();																NextStage;
				stage 15: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.3)															NextStage;

				Stage 16: rbt_Drivetrain->GoToAngle(25.0 * m_SelectedAlliance); tmr_Autonomous.Reset();													NextStage;
				Stage 17: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 2.0)																NextStage;
				Stage 18: rbt_BallManagement->EnableIndexer(); 																						NextStage;

				stage 19: Print("Autonomous Completed [GearSide]"); Print("--Closing gear holder"); Print("--Stopping all systems");				NextStage;
				stage 20: rbt_Gear->CloseHolder(); rbt_Drivetrain->SetTankDrive(0.0, 0.0); rbt_BallManagement->SetShooterSpeed(0.0, 0.0);			NextStage;
			}
		}

		void Auton_CenterGear(){
			StageMachine{
				//TODO Move to appropriate steps
				Stage  0: Print("Starting Autonomous [Center Gear]");														NextStage;

				Stage  1: Print("--Driving backwards to 7ft 3in");															NextStage;
				Stage  2: rbt_Drivetrain->GoToDistance(-7.0_ft + -3.0_in); tmr_Autonomous.Reset();							NextStage;
				Stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)									NextStage;

				Stage  4: Print("--Wiggling for 0.8 seconds");																NextStage;
				Stage  5: rbt_Drivetrain->SetTankDrive(0.3, -0.3); tmr_Autonomous.Reset();									NextStage;
				Stage  6: if(tmr_Autonomous.Get() > 0.4)																	NextStage;
				Stage  7: rbt_Drivetrain->SetTankDrive(-0.3, 0.3); tmr_Autonomous.Reset();									NextStage;
				Stage  8: if(tmr_Autonomous.Get() > 0.4)																	NextStage;
				Stage  9: rbt_Drivetrain->SetTankDrive(0.0, 0.0); tmr_Autonomous.Reset();									NextStage;

				Stage 10: Print("--Releasing Gear");																		NextStage;
				Stage 11: rbt_Gear->OpenHolder(); tmr_Autonomous.Reset();													NextStage;
				Stage 12: if(tmr_Autonomous.Get() > 0.5) 																	NextStage;

				Stage 13: Print("--Driving forward to 3ft");																NextStage;
				Stage 14: rbt_Drivetrain->StopDistancePID(); rbt_Drivetrain->GoToDistance(3.0_ft); tmr_Autonomous.Reset();	NextStage;
				Stage 15: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0) rbt_Gear->CloseHolder();			NextStage;

				Stage 16: Print("Autonomous Completed [Center Gear]");														NextStage;
			}
		}

		void Auton_CenterGearThenShoot(){
			StageMachine{
				//TODO Move to appropriate steps
				Stage  0: Print("Starting Autonomous [Center Gear then Shoot]");							NextStage;

				Stage  1: Print("--Driving backwards to 7ft 3in");											NextStage;
				Stage  2: rbt_Drivetrain->GoToDistance(-7.0_ft + -3.0_in); tmr_Autonomous.Reset();			NextStage;
				Stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)					NextStage;

				Stage  4: Print("--Releasing Gear");														NextStage;
				Stage  5: rbt_Gear->OpenHolder(); tmr_Autonomous.Reset();									NextStage;
				Stage  6: if(tmr_Autonomous.Get() > 0.5)													NextStage;
				Stage  7: rbt_Drivetrain->StopDistancePID(); rbt_Drivetrain->GoToDistance(3.0_ft);			NextStage;

				Stage  8: Print("--Driving forward to 3ft and enabling shooter at 5000rpm");				//NextLine
				rbt_BallManagement->SetShooterSpeed(5000, 5000); tmr_Autonomous.Reset();					NextStage;
				Stage  9: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)					NextStage;

				Stage 10: Print("--Closing Gear and turning to 70deg");										NextStage;
				Stage 11: rbt_Gear->CloseHolder(); rbt_Drivetrain->GoToAngle(-65*m_SelectedAlliance); tmr_Autonomous.Reset();	NextStage;
				Stage 12: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 2)						NextStage;

				Stage 13: Print("--Enbling Indexer for the remainder of Auton");							NextStage;
				Stage 14: rbt_BallManagement->EnableIndexer();												NextStage;

				Stage 15: Print("Autonomous Completed [Center Gear then Shoot]");							NextStage;
			}
		}

		void Auton_CrossBaseLine(){
			StageMachine{
				Stage  0: rbt_Drivetrain->GoToDistance(9.0_ft); tmr_Autonomous.Reset();		NextStage;

				Stage  1: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 10)	NextStage;

				Stage  2: rbt_Drivetrain->SetTankDrive(0, 0);
			}
		}
		//TODO Finish up 2 Gear Auton
		void Auton_DoubleGear(){
			StageMachine{
				Stage  0: Print("Starting Autonomous [DoubleGear]");																NextStage;

				Stage  1: Print("--Driving backwards to 7ft 3in");																	NextStage;
				Stage  2: rbt_Drivetrain->GoToDistance(-7.0_ft + -3.0_in); tmr_Autonomous.Reset();									NextStage;
				Stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)											NextStage;

				Stage  4: Print("--Releasing First Gear");																			NextStage;
				Stage  5: rbt_Gear->OpenHolder(); tmr_Autonomous.Reset();															NextStage;
				Stage  6: if(tmr_Autonomous.Get() > 0.5)																			NextStage;

				Stage  7: Print("--Driving forward to 6ft");																		NextStage;
				Stage  8: rbt_Drivetrain->StopDistancePID(); rbt_Gear->CloseHolder(); rbt_Drivetrain->GoToDistance(6.0_ft);			NextStage;
				Stage  9: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)											NextStage;

				Stage 13: Print("--Retrieving Second Gear");																		NextStage;
				Stage 14: rbt_Gear->GearIntakeDown(); rbt_Gear->SetGearIntake(1.0); tmr_Autonomous.Reset();							NextStage;
				Stage 15: if(tmr_Autonomous.Get() > 0.125)																			NextStage;
				Stage 16: rbt_Drivetrain->GoToDistance(1.0_ft); tmr_Autonomous.Reset();												NextStage;
				Stage 17: if(tmr_Autonomous.Get() > 0.50)																			NextStage;
				Stage 16: rbt_Gear->GearIntakeUp(); rbt_Gear->SetGearIntake(0.0); tmr_Autonomous.Reset();							NextStage;
				Stage 17: if(tmr_Autonomous.Get() > 0.5)																			NextStage;

				Stage 10: Print("--Driving backwards to 7ft");																		NextStage;
				Stage 11: rbt_Drivetrain->GoToDistance(-5.0_ft); tmr_Autonomous.Reset();											NextStage;
				Stage 12: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.5)											NextStage;
				Stage 13: rbt_Drivetrain->GoToAngle(180); tmr_Autonomous.Reset();													NextStage;
				Stage 14: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 0.5)												NextStage;

				Stage 15: Print("--Placing Second Gear");																			NextStage;
				Stage 16: rbt_Drivetrain->GoToDistance(2.0_ft); tmr_Autonomous.Reset();												NextStage;
				Stage 17: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 0.75)											NextStage;
				Stage 18: rbt_Gear->GearIntakeDown(); tmr_Autonomous.Reset();														NextStage;
				Stage 19: if(tmr_Autonomous.Get() > 0.5)																			NextStage;

				Stage 20: Print("--Driving Backwards to 4ft");																		NextStage;
				Stage 21: rbt_Drivetrain->GoToDistance(-4.0_ft); tmr_Autonomous.Reset();											NextStage;
				Stage 22: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.0)											NextStage;

				Stage 23: Print("--Setting for 'HumanControl'");																	NextStage;
				Stage 24: rbt_Gear->GearIntakeUp(); rbt_Gear->SetGearIntake(0.0); tmr_Autonomous.Reset();							NextStage;
				Stage 25: if(tmr_Autonomous.Get() > 0.5)																			NextStage;

				Stage 26: Print("--Autonomous [DoubleGear] Complete");																NextStage;

			}
		}
	};
}
