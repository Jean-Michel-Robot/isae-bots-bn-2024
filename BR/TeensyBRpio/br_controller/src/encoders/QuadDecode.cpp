#ifdef ARDUINO

#include "encoders/QuadDecode.h"

// ISR routine for FlexTimer1 Module
extern "C" void ftm1_isr(void) {
    if ((FTM1_SC & FTM_SC_TOF) != 0) { // read the timer overflow flag (TOF in FTM1_SC)
        FTM1_SC &= ~FTM_SC_TOF;        // if set, clear overflow flag
        if (QuadDecode::instance()
                .first_interrupt_1) { // When FTM is initialized, it generate an interrupt. We escape it because it is not an proper overflow
            QuadDecode::instance().first_interrupt_1 = false;
            return;
        }
        if (FTM1_CNT < 0x8000) { // distinguish if the overflow is by the min value or the max
            QuadDecode::instance().retenue1++;
        } else {
            QuadDecode::instance().retenue1--;
        }
    }
}

// ISR routine for FlexTimer1 Module
extern "C" void ftm2_isr(void) {
    if ((FTM2_SC & FTM_SC_TOF) != 0) { // read the timer overflow flag (TOF in FTM1_SC)
        FTM2_SC &= ~FTM_SC_TOF;        // if set, clear overflow flag
        if (QuadDecode::instance()
                .first_interrupt_2) { // When FTM is initialized, it generate an interrupt. We escape it because it is not an proper overflow
            QuadDecode::instance().first_interrupt_2 = false;
            return;
        }
        if (FTM2_CNT < 0x8000) { // distinguish if the overflow is by the min value or the max
            QuadDecode::instance().retenue2++;
        } else {
            QuadDecode::instance().retenue2--;
        }
    }
}

#endif