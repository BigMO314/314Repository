#pragma once

#include "MOLib.h"

class Drivetrain2 {
public:
	//No shifters
	Drivetrain2(
			WPILib::Victor* ref_L_Drive_1, WPILib::Victor* ref_L_Drive_2,
			WPILib::Victor* ref_R_Drive_1, WPILib::Victor* ref_R_Drive_2,
			WPILib::Encoder* ref_DriveDistance, WPILib::GyroBase* ref_DriveAngle) {
		this->mtr_L_Drive_1 = ref_L_Drive_1;
		this->mtr_L_Drive_2 = ref_L_Drive_2;
		this->mtr_R_Drive_1 = ref_R_Drive_1;
		this->mtr_R_Drive_2 = ref_R_Drive_2;

		enc_DriveDistance = ref_DriveDistance;
		gyr_DriveAngle = ref_DriveAngle;

		pid_DriveDistance = new MOLib::PID::EncLoop(0.0, 0.0, 0.0, enc_DriveDistance);
		pid_DriveDistance->SetOutputRange(-1.0, 1.0);
		pid_DriveDistance->SetTargetTime(0.1);
		pid_DriveDistance->SetAbsoluteTolerance(2);

		pid_DriveAngle = new MOLib::PID::GyrLoop(0.0, 0.0, 0.0, gyr_DriveAngle);
		pid_DriveAngle->SetOutputRange(-0.75, 0.75);
		pid_DriveAngle->SetTargetTime(0.1);
		pid_DriveAngle->SetAbsoluteTolerance(3);
	}

	MOLib::PIDLoop* DistancePID() { return pid_DriveDistance; }

	bool IsDistancePIDEnabled() const { return pid_DriveDistance->IsEnabled(); }

	void SetDistanceOutputRange(double min, double max){ pid_DriveDistance->SetOutputRange(min, max); }
	void SetDistancePID(double P, double I, double D){ pid_DriveDistance->SetPID(P, I, D); }

	bool IsAtDistance() const { return pid_DriveDistance->OnTarget(); }

	double GetDistanceError() const { return pid_DriveDistance->GetError(); }

	double GetDistanceTarget() const { return pid_DriveDistance->GetSetpoint(); }

	double GetDistancePIDOut() const { return pid_DriveDistance->Get(); }

	void GoToDistance(double inches, bool resetDistance = true) {
		if(!IsDistancePIDEnabled()){
			pid_DriveAngle->SetSetpoint(0);
			pid_DriveAngle->Enable();
			pid_DriveDistance->SetSetpoint(inches);
			pid_DriveDistance->Enable();
			if(resetDistance){
				pid_DriveDistance->ResetSource();
				pid_DriveAngle->ResetSource();
			}
		}
	}

	void StopDistancePID() { pid_DriveDistance->Disable(); pid_DriveAngle->Disable(); }

	//---------------------------------------------------------------------------------------------------------------------------------------------------|

	MOLib::PIDLoop* AnglePID() { return pid_DriveAngle; }

	bool IsAnglePIDEnabled() const { return pid_DriveAngle->IsEnabled(); }

	void SetAnglePID(double P, double I, double D){ pid_DriveAngle->SetPID(P, I, D); }

	bool IsAtAngle() const { return pid_DriveAngle->OnTarget(); }
	double GetAngleError() const { return pid_DriveAngle->GetError(); }
	double GetAngleTarget() const { return pid_DriveAngle->GetSetpoint(); }
	double GetAnglePIDOut() const { return pid_DriveAngle->Get(); }

	void StopAnglePID() { pid_DriveAngle->Disable(); }

	//---------------------------------------------------------------------------------------------------------------------------------------------------|

	void SetTankDrive(float lPower, float rPower){ SetDrive(lPower, rPower); }

	void SetArcadeDrive(float throttle, float steering){ SetDrive(throttle+steering, throttle-steering); }

	void ResetDistance(){
		enc_DriveDistance->Reset();
		std::cout << "Reset Distance" << std::endl;
	}

	void ResetAngle(){ gyr_DriveAngle->Reset();
	std::cout << "Reset Angle" << std::endl;
	}
	void CalibrateAngle(){ gyr_DriveAngle->Calibrate(); }

	double GetDistance() const { return enc_DriveDistance->GetDistance(); }
	double GetAngle() const { return gyr_DriveAngle->GetAngle(); }

	void SetWheelDiameter(double diameter){
		m_WheelDiameter = diameter;
		enc_DriveDistance->SetDistancePerPulse(((m_WheelDiameter * Pi) * m_GearRatio) / 2048);
	}

	void SetGearRatio(double ratio){
		m_GearRatio = ratio;
		enc_DriveDistance->SetDistancePerPulse(((m_WheelDiameter * Pi) * m_GearRatio) / 2048);
	}
	void SetDrive(float lPower, float rPower) { m_LPower = lPower; m_RPower = rPower; }

	void Update(){
		if(pid_DriveDistance->IsEnabled()){
			if(pid_DriveDistance->OnTarget()){ pid_DriveDistance->Disable(); pid_DriveAngle->Disable(); }
			SetArcadeDrive(pid_DriveDistance->Get(), pid_DriveAngle->Get());
		}

		mtr_L_Drive_1->Set(m_LPower);
		mtr_L_Drive_2->Set(m_LPower);
		mtr_R_Drive_1->Set(m_RPower);
		mtr_R_Drive_2->Set(m_RPower);
	}

private:
	WPILib::Victor* mtr_L_Drive_1;
	WPILib::Victor* mtr_L_Drive_2;
	WPILib::Victor* mtr_R_Drive_1;
	WPILib::Victor* mtr_R_Drive_2;

	WPILib::Encoder* enc_DriveDistance;
	WPILib::GyroBase* gyr_DriveAngle;

	MOLib::PIDLoop* pid_DriveDistance;
	MOLib::PIDLoop* pid_DriveAngle;

	double m_WheelDiameter = 0.0;
	double m_GearRatio = 1.0;

	float m_LPower	= 0.0;
	float m_RPower	= 0.0;

	bool IsUsingPID() const { return gyr_DriveAngle && enc_DriveDistance; }
};
