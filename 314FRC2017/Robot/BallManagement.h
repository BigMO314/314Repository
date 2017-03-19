#pragma once
#include "MOLib.h"
#include "CanTalonSRX.h"

namespace Robot{
	class BallManagement(
	private:
		//Shooter
		WPILib::Victor*		mtr_Intake;
		WPILib::Solenoid*	sol_Arm;
		CanTalonSRX*		mtr_L_Belting;
		CanTalonSRX*		mtr_R_Belting;
		WPILib::Solenoid*	sol_Agitator;
		WPILib::Victor*		mtr_Kicker;
		WPILib::Victor*		mtr_L_Uptake;
		WPILib::Victor*		mtr_R_Uptake;

		WPILib::Victor*		mtr_T_Shooter;
		WPILib::Victor*		mtr_B_Shooter_1;
		WPILib::Victor*		mtr_B_Shooter_2;

		WPILib::Solenoid*	sol_Hood;

		WPILib::Encoder*	enc_T_ShooterSpeed;
		WPILib::Encoder*	enc_B_ShooterSpeed;
		MOLib::PIDLoop*		pid_T_ShooterSpeed;
		MOLib::PIDLoop*		pid_B_ShooterSpeed;

		WPILib::Timer		tmr_Agitator;

		float				m_IntakePower		= 0.0;
		bool				m_IntakeExtended	= false;

		float				m_L_Belting			= 0.0;
		float				m_R_Belting			= 0.0;

		bool				m_Agitator			= false;

		float				m_KickerPower		= 0.0;
		float				m_UptakePower		= 0.0;

		float				m_T_ShooterPower	= 0.0;
		float				m_B_ShooterPower	= 0.0;

		bool				m_HoodLowered		= false;

	public:
		/**
		 *
		 * @param ref_T_Shooter-making a reference for the Top Shooter.
		 * @param ref_B_Shooter_1-making a reference for one of the bottom shooter motors.
		 * @param ref_B_Shooter_2-making a reference for one of the bottom shooter motors.
		 * @param ref_R_Indexer-making a reference for the right indexeing motor.
		 * @param ref_L_Indexer-making a reference for the left indexeing motor.
		 * @param ref_C_Indexer-making a reference for the center indexeing motor.
		 */
		BallManagement(
				Victor* ref_Intake, Solenoid* ref_Arm, CanTalonSRX* ref_L_Belting, CanTalonSRX* ref_R_Belting, Solenoid* ref_Agitator,
				Victor* ref_Kicker, Victor* ref_L_Uptake, Victor* ref_R_Uptake,
				Victor* ref_T_Shooter, Victor* ref_B_Shooter_1 , Victor* ref_B_Shooter_2,
				Solenoid* ref_Hood, Encoder* ref_T_ShooterSensor, Encoder* ref_B_ShooterSensor,
				MOLib::PIDLoop* ref_T_ShooterSpeed, MOLib::PIDLoop* ref_B_ShooterSpeed
				){
			this->mtr_Intake			= ref_Intake;
			this->sol_Arm				= ref_Arm;
			this->mtr_L_Belting			= ref_L_Belting;
			this->mtr_R_Belting			= ref_R_Belting;
			this->sol_Agitator			= ref_Agitator;
			this->mtr_Kicker			= ref_Kicker;
			this->mtr_L_Uptake			= ref_L_Uptake;
			this->mtr_R_Uptake			= ref_R_Uptake;

			this->mtr_T_Shooter			= ref_T_Shooter;
			this->mtr_B_Shooter_1		= ref_B_Shooter_1;
			this->mtr_B_Shooter_2		= ref_B_Shooter_2;

			this->sol_Hood				= ref_Hood;

			this->enc_T_ShooterSpeed	= ref_T_ShooterSensor;
			this->enc_B_ShooterSpeed	= ref_B_ShooterSensor;

			this->pid_T_ShooterSpeed	= ref_T_ShooterSpeed;
			this->pid_B_ShooterSpeed	= ref_B_ShooterSpeed;

			tmr_Agitator.Start();
		}

		void SetIntakePower(float power){ m_IntakePower = power; }
		void ExtendIntake(){ m_IntakeExtended = true; }
		void RetractIntake(){ m_IntakeExtended = false; }

		void SetBeltingPower(float power){ m_L_Belting = power; m_R_Belting = power; }

		void ExtendAgitator() { m_Agitator = true; }
		void RetractAgitator() { m_Agitator = false; }
		void ToggleAgitator() { m_Agitator = !m_Agitator; }

		void SetKickerPower(float power) { m_KickerPower = power; }
		void SetUptakePower(float power) { m_UptakePower = power; }

		void EnableIndexer(){
			SetBeltingPower(1.0);
			SetKickerPower(1.0);
			SetUptakePower(0.75);
			if(tmr_Agitator.Get() > 2) { ToggleAgitator(); tmr_Agitator.Reset(); }
		}
		void DisableIndexer(){
			SetBeltingPower(0.0);
			SetKickerPower(0.0);
			SetUptakePower(0.0);
			RetractAgitator();
		}

		void RaiseHood(){ m_HoodLowered = false; }
		void LowerHood(){ m_HoodLowered = true; }

		bool IsHoodLowered(){ return m_HoodLowered; }

		void SetTShooterPower(float tPower){ m_T_ShooterPower = tPower; }
		void SetBShooterPower(float bPower){ m_B_ShooterPower = bPower; }
		void SetShooterPower(float tPower, float bPower){ m_T_ShooterPower = tPower; m_B_ShooterPower = bPower; }

		void SetTShooterSpeed(float tSpeed){ pid_T_ShooterSpeed->SetSetpoint(tSpeed); pid_T_ShooterSpeed->Enable(); }
		void SetBShooterSpeed(float bSpeed){ pid_B_ShooterSpeed->SetSetpoint(bSpeed); pid_B_ShooterSpeed->Enable(); }
		void SetShooterSpeed(float tSpeed, float bSpeed){
			pid_T_ShooterSpeed->SetSetpoint(tSpeed); pid_T_ShooterSpeed->Enable();
			pid_B_ShooterSpeed->SetSetpoint(bSpeed); pid_B_ShooterSpeed->Enable();
		}

		bool IsTShooterPIDEnabled(){ return pid_T_ShooterSpeed->IsEnabled(); }
		bool IsBShooterPIDEnabled(){ return pid_B_ShooterSpeed->IsEnabled(); }
		bool IsShooterPIDEnabled(){ return pid_T_ShooterSpeed->IsEnabled() && pid_B_ShooterSpeed->IsEnabled(); }

		void SetTShooterPID(double P, double I, double D){ pid_T_ShooterSpeed->SetPID(P, I, D); }
		void SetBShooterPID(double P, double I, double D){ pid_B_ShooterSpeed->SetPID(P, I, D); }

		void StopShooterPID(){ pid_T_ShooterSpeed->Disable(); pid_B_ShooterSpeed->Disable(); }

		double GetTShooterSpeed(){ return enc_T_ShooterSpeed->GetRate(); }
		double GetBShooterSpeed(){ return enc_B_ShooterSpeed->GetRate(); }

		void Update(){
			if(pid_T_ShooterSpeed->IsEnabled() || pid_B_ShooterSpeed->IsEnabled()){
				SetShooterPower(pid_T_ShooterSpeed->Get(), pid_B_ShooterSpeed->Get());
				//if(pid_T_Shooter_PID->OnTarget()) pid_T_Shooter_PID->Disable();
				//if(pid_B_Shooter_PID->OnTarget()) pid_B_Shooter_PID->Disable();
			}

			mtr_Intake->Set(m_IntakePower);
			sol_Arm->Set(m_IntakeExtended);
			mtr_L_Belting->Set(m_L_Belting);
			mtr_R_Belting->Set(m_R_Belting);
			sol_Agitator->Set(m_Agitator);
			mtr_Kicker->Set(m_KickerPower);
			mtr_L_Uptake->Set(m_UptakePower);
			mtr_R_Uptake->Set(m_UptakePower);

			mtr_T_Shooter->Set(m_T_ShooterPower);
			mtr_B_Shooter_1->Set(m_B_ShooterPower);
			mtr_B_Shooter_2->Set(m_B_ShooterPower);

			sol_Hood->Set(m_HoodLowered);

		}
	};
}
