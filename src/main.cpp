#include <Arduino.h>


#define T2_MSFM
#ifdef T2_MSFM
	#include "T2_MSFM_250/T200_Main_250.h"

#endif

// #define T4_MC
#ifdef T4_MC
	#include "T4_MC_013/013/T400_Main_013.hpp"
#endif


void setup() {
	// delay(5000);

	Serial.begin(115200);
	Serial.print("setup: ");


	#ifdef T2_MSFM
		T2_init();
	#endif

	#ifdef T4_MC
		T4_init();
	#endif


}

void loop() {

	#ifdef T2_MSFM
		T2_run();
	#endif

	#ifdef T4_MC
		T4_run();
	#endif
}
