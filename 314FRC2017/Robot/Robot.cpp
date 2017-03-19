#include "MOLib.h"
#include "Utility.h"
#include "Robot Specifications.h"

#include "BallManagement.h"
#include "Gear.h"
#include "Climber.h"
#include "HumanControl.h"
#include "Autonomous.h"
#include "Vision.h"
#include "CanTalonSRX.h"

//TODO Create Simple Program to output all Encoder Values onto dashboard

class Onslaught: public frc::SampleRobot {
private:
	long double runCount = 0;

	//DriveTrain
	WPILib::Victor*		  			mtr_L_Drive_1;
	WPILib::Victor*					mtr_L_Drive_2;
	WPILib::Victor*					mtr_R_Drive_1;
	WPILib::Victor*					mtr_R_Drive_2;
	WPILib::Solenoid*				sol_Shifter;
	WPILib::Encoder*				enc_L_DriveDistance;
	WPILib::Encoder*				enc_R_DriveDistance;
	MOLib::AnalogGyro*				gyr_DriveAngle;

	MOLib::TankDrivetrain*			rbt_Drivetrain;

	//Ball Management
	WPILib::Victor*					mtr_Intake;
	CanTalonSRX*					mtr_L_Belting;
	CanTalonSRX*					mtr_R_Belting;
	WPILib::Solenoid*				sol_Agitator;
	WPILib::Victor*					mtr_Kicker;
	WPILib::Victor*					mtr_L_Uptake;
	WPILib::Victor*					mtr_R_Uptake;
	WPILib::Victor*					mtr_T_Shooter;
	WPILib::Victor*					mtr_B_Shooter_1;
	WPILib::Victor*					mtr_B_Shooter_2;
	WPILib::Solenoid*				sol_Arm;
	WPILib::Solenoid*				sol_Hood;
	WPILib::Encoder*				enc_T_ShooterSpeed;
	WPILib::Encoder*				enc_B_ShooterSpeed;

	MOLib::PIDLoop*					pid_T_ShooterSpeed;
	MOLib::PIDLoop*					pid_B_ShooterSpeed;

	Robot::BallManagement*			rbt_BallManagement;

	//Gear Mechanism
	WPILib::Solenoid*				sol_GearSlide;
	WPILib::Solenoid*				sol_GearHolder;

	Robot::Gear* 					rbt_Gear;

	//Climber
	CanTalonSRX*					mtr_L_Climber;
	CanTalonSRX*					mtr_R_Climber;

	Robot::Climber*					rbt_Climber;

	//Vision
	MOLib::Vision::Target*			vis_Boiler;
	Solenoid*						led_RingLight;

	//Controllers
	MOLib::XBoxController*			ctl_Driver;
	MOLib::XBoxController*			ctl_Operator;

	ControlPeriod::HumanControl*	prd_HumanControl;

	ControlPeriod::Autonomous*		prd_Autonomous;

	MOLib::Dashboard::Indicator*	dsh_DriveDistance_PIDEnabled;
	MOLib::Dashboard::Indicator*	dsh_DriveDistance_PIDOnTarget;
	MOLib::Dashboard::Number*		dsh_DriveDistance_Distance;
	MOLib::Dashboard::Number*		dsh_DriveDistance_Plot;
	MOLib::Dashboard::Number*		dsh_DriveDistance_P;
	MOLib::Dashboard::Number*		dsh_DriveDistance_I;
	MOLib::Dashboard::Number*		dsh_DriveDistance_D;

	MOLib::Dashboard::Indicator*	dsh_DriveStraight_PIDEnabled;
	MOLib::Dashboard::Indicator*	dsh_DriveStraight_PIDOnTarget;
	MOLib::Dashboard::Number*		dsh_DriveStraight_P;
	MOLib::Dashboard::Number*		dsh_DriveStraight_I;
	MOLib::Dashboard::Number*		dsh_DriveStraight_D;

	MOLib::Dashboard::Indicator*	dsh_DriveAngle_PIDEnabled;
	MOLib::Dashboard::Indicator*	dsh_DriveAngle_PIDOnTarget;
	MOLib::Dashboard::Number*		dsh_DriveAngle_Angle;
	MOLib::Dashboard::Number*		dsh_DriveAngle_Plot;
	MOLib::Dashboard::Number*		dsh_DriveAngle_P;
	MOLib::Dashboard::Number*		dsh_DriveAngle_I;
	MOLib::Dashboard::Number*		dsh_DriveAngle_D;

	MOLib::Dashboard::Indicator*	dsh_DriveVision_PIDEnabled;
	MOLib::Dashboard::Indicator*	dsh_DriveVision_PIDOnTarget;
	MOLib::Dashboard::Number*		dsh_DriveVision_OffCenter;
	MOLib::Dashboard::Number*		dsh_DriveVision_Plot;
	MOLib::Dashboard::Number*		dsh_DriveVision_P;
	MOLib::Dashboard::Number*		dsh_DriveVision_I;
	MOLib::Dashboard::Number*		dsh_DriveVision_D;

	MOLib::Dashboard::Number*		dsh_T_ShooterSpeed;
	MOLib::Dashboard::Number*		dsh_B_ShooterSpeed;

	//MOLib::Dashboard::Number*		dsh_VisionX;

public:
	Onslaught() {
		//Drivetrain
		this->mtr_L_Drive_1				= new WPILib::Victor(0);
		this->mtr_L_Drive_2				= new WPILib::Victor(1);
		this->mtr_R_Drive_1				= new WPILib::Victor(2);
		this->mtr_R_Drive_2				= new WPILib::Victor(3);
		this->sol_Shifter				= new WPILib::Solenoid(0);
		this->enc_L_DriveDistance		= new WPILib::Encoder(0,1);
		this->enc_R_DriveDistance		= new WPILib::Encoder(2,3);
		this->gyr_DriveAngle			= new MOLib::AnalogGyro(0);
		this->vis_Boiler				= new MOLib::Vision::Target();

		rbt_Drivetrain					= new MOLib::TankDrivetrain(
			mtr_L_Drive_1, mtr_L_Drive_2, NullPtr,
			mtr_R_Drive_1, mtr_R_Drive_2, NullPtr,
			sol_Shifter, MOLib::ShiftState::kHighSpeed,
			enc_L_DriveDistance, gyr_DriveAngle, vis_Boiler
		);

		//Ball Management
		this->mtr_Intake				= new WPILib::Victor(4);
		this->sol_Arm					= new WPILib::Solenoid(1);
		this->mtr_L_Belting				= new CanTalonSRX(1);
		this->mtr_R_Belting				= new CanTalonSRX(2);
		this->sol_Agitator				= new WPILib::Solenoid(5);
		this->mtr_Kicker				= new WPILib::Victor(6);
		this->mtr_L_Uptake				= new WPILib::Victor(7);
		this->mtr_R_Uptake				= new WPILib::Victor(8);
		this->mtr_T_Shooter				= new WPILib::Victor(9);
		this->mtr_B_Shooter_1			= new WPILib::Victor(10);
		this->mtr_B_Shooter_2			= new WPILib::Victor(11);
		this->sol_Hood					= new WPILib::Solenoid(2);
		this->enc_T_ShooterSpeed		= new WPILib::Encoder(4,5);
		this->enc_B_ShooterSpeed		= new WPILib::Encoder(6,7);
		this->pid_T_ShooterSpeed		= new MOLib::PID::EncLoop(0.0, 0.0, 0.0, enc_T_ShooterSpeed);
		this->pid_B_ShooterSpeed		= new MOLib::PID::EncLoop(0.0, 0.0, 0.0, enc_B_ShooterSpeed);

		rbt_BallManagement				= new Robot::BallManagement(
			mtr_Intake, sol_Arm, mtr_L_Belting, mtr_R_Belting, sol_Agitator,
			mtr_Kicker, mtr_L_Uptake, mtr_R_Uptake,
			mtr_T_Shooter, mtr_B_Shooter_1, mtr_B_Shooter_2, sol_Hood,
			enc_T_ShooterSpeed, enc_B_ShooterSpeed,
			pid_T_ShooterSpeed, pid_B_ShooterSpeed
		);

		//Defining the Gear Mechanism.
		this->sol_GearSlide				= new WPILib::Solenoid(4);
		this->sol_GearHolder			= new WPILib::Solenoid(3);

		rbt_Gear						= new Robot::Gear(
			sol_GearSlide, sol_GearHolder
		);

		vis_Boiler						= new MOLib::Vision::Target();
		led_RingLight					= new Solenoid(6);

		//Defining Climber.
		this->mtr_L_Climber				= new CanTalonSRX(3);
		this->mtr_R_Climber				= new CanTalonSRX(4);

		rbt_Climber						= new Robot::Climber(
			mtr_L_Climber, mtr_R_Climber
		);

		//Defining Controllers.
		this->ctl_Driver				= new MOLib::XBoxController(0, 0.1);
		this->ctl_Operator				= new MOLib::XBoxController(1, 0.1);

		prd_HumanControl				= new ControlPeriod::HumanControl(
			ctl_Driver, ctl_Operator,
			rbt_Drivetrain, rbt_BallManagement, rbt_Gear, rbt_Climber
		);

		prd_Autonomous					= new ControlPeriod::Autonomous(rbt_Drivetrain, rbt_BallManagement, rbt_Gear);

		NetworkTable::GlobalDeleteAll();

		dsh_T_ShooterSpeed				= new MOLib::Dashboard::Number("[T ShooterSpeed] T ShooterSpeed");
		dsh_B_ShooterSpeed				= new MOLib::Dashboard::Number("[Shooter] B ShooterSpeed");

		dsh_DriveDistance_PIDEnabled	= new MOLib::Dashboard::Indicator("[DriveDistance] PID Enabled");
		dsh_DriveDistance_PIDOnTarget	= new MOLib::Dashboard::Indicator("[DriveDistance] PID on Target");
		dsh_DriveDistance_Distance		= new MOLib::Dashboard::Number("[DriveDistance] Distance");
		dsh_DriveDistance_Plot			= new MOLib::Dashboard::Number("[DriveDistance] Plot");
		dsh_DriveDistance_P				= new MOLib::Dashboard::Number("[DriveDistance] P", 0.023);
		dsh_DriveDistance_I				= new MOLib::Dashboard::Number("[DriveDistance] I", 1.0e-13);
		dsh_DriveDistance_D				= new MOLib::Dashboard::Number("[DriveDistance] D", 0.038);

		dsh_DriveStraight_PIDEnabled	= new MOLib::Dashboard::Indicator("[DriveStraight] PID Enabled");
		dsh_DriveStraight_PIDOnTarget	= new MOLib::Dashboard::Indicator("[DriveStraight] PID on Target");
		dsh_DriveStraight_P				= new MOLib::Dashboard::Number("[DriveStraight] P", 0.08);
		dsh_DriveStraight_I				= new MOLib::Dashboard::Number("[DriveStraight] I", 0.0);
		dsh_DriveStraight_D				= new MOLib::Dashboard::Number("[DriveStraight] D", 0.0);

		dsh_DriveAngle_PIDEnabled		= new MOLib::Dashboard::Indicator("[DriveAngle] PID Enabled");
		dsh_DriveAngle_PIDOnTarget		= new MOLib::Dashboard::Indicator("[DriveAngle] PID on Target");
		dsh_DriveAngle_Angle			= new MOLib::Dashboard::Number("[DriveAngle] Angle");
		dsh_DriveAngle_Plot				= new MOLib::Dashboard::Number("[DriveAngle] Plot");
		dsh_DriveAngle_P				= new MOLib::Dashboard::Number("[DriveAngle] P", 0.04);
		dsh_DriveAngle_I				= new MOLib::Dashboard::Number("[DriveAngle] I", 0.0);
		dsh_DriveAngle_D				= new MOLib::Dashboard::Number("[DriveAngle] D", 0.04);

		dsh_DriveVision_PIDEnabled		= new MOLib::Dashboard::Indicator("[DriveVision] PID Enabled");
		dsh_DriveVision_PIDOnTarget		= new MOLib::Dashboard::Indicator("[DriveVision] PID on Target");
		dsh_DriveVision_OffCenter		= new MOLib::Dashboard::Number("[DriveVision] Angle");
		dsh_DriveVision_Plot			= new MOLib::Dashboard::Number("[DriveVision] Plot");
		dsh_DriveVision_P				= new MOLib::Dashboard::Number("[DriveVision] P", 0.04);
		dsh_DriveVision_I				= new MOLib::Dashboard::Number("[DriveVision] I", 0.0);
		dsh_DriveVision_D				= new MOLib::Dashboard::Number("[DriveVision] D", 0.04);


		SmartDashboard::PutData("Selected Alliance", &prd_Autonomous->chs_Alliance);
		SmartDashboard::PutData("Selected Auton", &prd_Autonomous->chs_Auton);
	}

	void UpdateDashboard(){
		dsh_DriveDistance_PIDEnabled->Set(rbt_Drivetrain->pid_DriveDistance->IsEnabled());
		dsh_DriveDistance_PIDOnTarget->Set(rbt_Drivetrain->pid_DriveDistance->OnTarget());
		dsh_DriveDistance_Distance->Set(enc_L_DriveDistance->GetDistance());
		dsh_DriveDistance_Plot->Set(enc_L_DriveDistance->GetDistance());

		dsh_DriveStraight_PIDEnabled->Set(rbt_Drivetrain->pid_DriveStraight->IsEnabled());
		dsh_DriveStraight_PIDOnTarget->Set(rbt_Drivetrain->pid_DriveStraight->OnTarget());

		dsh_DriveAngle_PIDEnabled->Set(rbt_Drivetrain->pid_DriveAngle->IsEnabled());
		dsh_DriveAngle_PIDOnTarget->Set(rbt_Drivetrain->pid_DriveAngle->OnTarget());
		dsh_DriveAngle_Angle->Set(gyr_DriveAngle->GetAngle());
		dsh_DriveAngle_Plot->Set(gyr_DriveAngle->GetAngle());

		dsh_DriveVision_PIDEnabled->Set(rbt_Drivetrain->IsVisionPIDEnabled());
		dsh_DriveVision_PIDOnTarget->Set(rbt_Drivetrain->IsVisionPIDAlignedToGoal());
		dsh_DriveVision_OffCenter->Set(rbt_Drivetrain->GetVisionPIDError());
		dsh_DriveVision_Plot->Set(rbt_Drivetrain->GetVisionPIDError());

		rbt_Drivetrain->pid_DriveDistance->SetPID(dsh_DriveDistance_P->Get(), dsh_DriveDistance_I->Get(), dsh_DriveDistance_D->Get());
		rbt_Drivetrain->pid_DriveStraight->SetPID(dsh_DriveStraight_P->Get(), dsh_DriveStraight_I->Get(), dsh_DriveStraight_D->Get());
		rbt_Drivetrain->pid_DriveAngle->SetPID(dsh_DriveAngle_P->Get(), dsh_DriveAngle_I->Get(), dsh_DriveAngle_D->Get());
		rbt_Drivetrain->pid_GoalAngle->SetPID(dsh_DriveVision_P->Get(), dsh_DriveVision_I->Get(), dsh_DriveVision_D->Get());
	}

	void RobotInit() {
		//Inverting the Right Side of the DriveTrain.
		mtr_L_Drive_1->SetInverted(false);
		mtr_L_Drive_2->SetInverted(false);
		mtr_R_Drive_1->SetInverted(true);
		mtr_R_Drive_2->SetInverted(true);

		rbt_Drivetrain->SetWheelDiameter(Robot::WheelDiameter);
		rbt_Drivetrain->SetGearRatio(Robot::LowGearRatio);
		rbt_Drivetrain->SetScale(0.91, 1.0, 1.0, 0.95);

		enc_R_DriveDistance->SetDistancePerPulse((Circumference(4.0) / 2048) * Robot::LowGearRatio);

		enc_L_DriveDistance->SetReverseDirection(false);

		enc_R_DriveDistance->SetReverseDirection(true);

		gyr_DriveAngle->Calibrate();
		gyr_DriveAngle->SetAngleScale(90.0 / 88.0);

		rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.9, 0.9);
		rbt_Drivetrain->pid_DriveDistance->SetPID(0.023, 1.0e-13, 0.038);
		rbt_Drivetrain->pid_DriveDistance->SetTargetTime(0.2);
		rbt_Drivetrain->pid_DriveDistance->SetAbsoluteTolerance(1.0);

		rbt_Drivetrain->pid_DriveStraight->SetOutputRange(-0.7, 0.7);
		rbt_Drivetrain->pid_DriveStraight->SetPID(0.08, 0.0, 0.0);
		rbt_Drivetrain->pid_DriveStraight->SetTargetTime(0.2);
		rbt_Drivetrain->pid_DriveStraight->SetAbsoluteTolerance(1.0);

		rbt_Drivetrain->pid_DriveAngle->SetOutputRange(-0.7, 0.7);
		rbt_Drivetrain->pid_DriveAngle->SetPID(0.04, 0.0, 0.04);
		rbt_Drivetrain->pid_DriveAngle->SetTargetTime(0.1);
		rbt_Drivetrain->pid_DriveAngle->SetAbsoluteTolerance(1.5);

		rbt_Drivetrain->pid_GoalAngle->SetOutputRange(0,0);
		rbt_Drivetrain->pid_GoalAngle->SetPID(0,0,0);//NOT SET!!                       .
		rbt_Drivetrain->pid_GoalAngle->SetTargetTime(0.1);
		rbt_Drivetrain->pid_GoalAngle->SetAbsoluteTolerance(1);

		mtr_Intake->SetInverted(true);
		mtr_Kicker->SetInverted(true);
		mtr_L_Uptake->SetInverted(false);
		mtr_R_Uptake->SetInverted(true);

		mtr_T_Shooter->SetInverted(true);
		mtr_B_Shooter_1->SetInverted(false);
		mtr_B_Shooter_2->SetInverted(true);

		enc_T_ShooterSpeed->SetReverseDirection(false);
		enc_T_ShooterSpeed->SetDistancePerPulse(60.0/512);
		pid_T_ShooterSpeed->SetPID(0.00001, 0.001, 0.00001);
		pid_T_ShooterSpeed->SetPIDSourceType(PIDSourceType::kRate);
		pid_T_ShooterSpeed->SetOutputRange(-1.0, 1.0);
		pid_T_ShooterSpeed->SetTargetTime(0.1);
		pid_T_ShooterSpeed->SetAbsoluteTolerance(0.1);

		enc_B_ShooterSpeed->SetReverseDirection(true);
		enc_B_ShooterSpeed->SetDistancePerPulse(60.0/512);
		pid_B_ShooterSpeed->SetPID(0.00001, 0.001, 0.00001);
		pid_B_ShooterSpeed->SetPIDSourceType(PIDSourceType::kRate);
		pid_B_ShooterSpeed->SetOutputRange(-1.0, 1.0);
		pid_B_ShooterSpeed->SetTargetTime(0.1);
		pid_B_ShooterSpeed->SetAbsoluteTolerance(0.1);

		vis_Boiler->SetResoloution(320, 240);
		vis_Boiler->SetTargetSize(15, 10);
		vis_Boiler->SetFOV(58.5, 45.6);

	}

	void Disabled() { Print("Robot Disabled"); }

	void Autonomous() {
		prd_Autonomous->AutonomousInit();
		while (IsAutonomous() && IsEnabled()) {
			UpdateDashboard();
			prd_Autonomous->Update();
		}
	}

	void OperatorControl() override {
		enc_L_DriveDistance->Reset();
		enc_R_DriveDistance->Reset();
		gyr_DriveAngle->Reset();
		rbt_Drivetrain->StopAnglePID();
		rbt_Drivetrain->StopDistancePID();
		rbt_BallManagement->StopShooterPID();
		rbt_BallManagement->SetShooterPower(0,0);

		rbt_Drivetrain->SetShift(MOLib::ShiftState::kLowSpeed);
		while (IsOperatorControl() && IsEnabled()) {
			UpdateDashboard();
			//rbt_BallManagement->SetShooterSpeed(dsh_T_ShooterSpeed->Get(), dsh_B_ShooterSpeed->Get());
			prd_HumanControl->Update();
			rbt_Drivetrain->Update();
			rbt_BallManagement->Update();
			rbt_Gear->Update();
			rbt_Climber->Update();
			frc::Wait(0.005);
		}
	}

	void Test() override {}
};
START_ROBOT_CLASS(Onslaught)
