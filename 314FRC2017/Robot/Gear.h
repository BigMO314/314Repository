#pragma once
#include "MOlib.h"

namespace Robot{
	class Gear{
	private:
		//Defineing the gear Solenoids.
		Solenoid* sol_Slide;
		Solenoid* sol_Holder;

		bool m_SlideExtended = false;
		bool m_HolderOpened = false;
	public:
		Gear(Solenoid* ref_Slide, Solenoid* ref_Holder){
			this->sol_Slide = ref_Slide;
			this->sol_Holder = ref_Holder;
		}
		//Setting the anagle for the gear things.
		void ExtendSlide(){ m_SlideExtended = true; }
		void RetractSlide(){ m_SlideExtended = false; }

		void OpenHolder(){ m_HolderOpened = true; }
		void CloseHolder(){ m_HolderOpened = false; }

		void Update(){
			sol_Slide->Set(m_SlideExtended);
			sol_Holder->Set(m_HolderOpened);
		}
	};
}
