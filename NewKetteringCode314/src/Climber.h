#pragma once
#include "WPILib.h"
#include "CanTalonSRX.h"
#include "Robot Specifications.h"

namespace Robot{
	class Climber{
	private:
		CanTalonSRX* mtr_L_Climber;
		CanTalonSRX* mtr_R_Climber;

		float m_Power = 0;

	public:
		Climber(CanTalonSRX* ref_L_Climber, CanTalonSRX* ref_R_Climber){
			this->mtr_L_Climber = ref_L_Climber;
			this->mtr_R_Climber = ref_R_Climber;
		}
		void Enable() { m_Power = 1.0; }
		void Disable() { m_Power = 0.0; }
		void SetPower(float power) { m_Power = power; }

		void Update(){
			mtr_L_Climber->Set(m_Power);
			mtr_R_Climber->Set(m_Power);
		}
	};
}
