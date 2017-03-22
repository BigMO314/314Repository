#pragma once
#include "MOlib.h"

namespace Robot{
	class Gear{
	private:
		//Defineing the gear Solenoids.
		Solenoid* sol_Slide;
		Solenoid* sol_Holder;
		WPILib::Victor* mtr_GearIntakeWheel;
		WPILib::Solenoid* sol_GearIntake;

		bool m_SlideExtended = false;
		bool m_HolderOpened = false;
		bool m_GearIntake = false;

	public:
		Gear(Solenoid* ref_Slide, Solenoid* ref_Holder, WPILib::Victor* ref_GearIntakeWheel, WPILib::Solenoid* ref_GearIntake){
			this->sol_Slide = ref_Slide;
			this->sol_Holder = ref_Holder;
			this->mtr_GearIntakeWheel = ref_GearIntakeWheel;
			this->sol_GearIntake = ref_GearIntake;
		}
		//Setting the anagle for the gear things.
		void ExtendSlide(){ m_SlideExtended = true; }
		void RetractSlide(){ m_SlideExtended = false; }

		void OpenHolder(){ m_HolderOpened = true; }
		void CloseHolder(){ m_HolderOpened = false; }

		void GearIntakeUp(){ m_GearIntake = false; }
		void GearIntakeDown(){ m_GearIntake = true; }
		void SetGearIntake(float gearintake){ mtr_GearIntakeWheel->Set(gearintake); }

		void Update(){
			sol_Slide->Set(m_SlideExtended);
			sol_Holder->Set(m_HolderOpened);
			sol_GearIntake->Set(m_GearIntake);
		}
	};
}
