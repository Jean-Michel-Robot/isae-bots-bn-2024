#ifndef QUADDECODE_H
#define QUADDECODE_H

#include <stdint.h>
#include <Arduino.h>
/*
 *  "Ecrit" par Etienne ARLAUD

 *  Le code a salement été pompé sur ces 2 sites :
 *  Pour le fonctionnement du FTM :
 *  https://forum.pjrc.com/threads/26803-Hardware-Quadrature-Code-for-Teensy-3-x/page3
 *
 *  Pour la mise en place d'interruptions :
 *  https://forum.pjrc.com/threads/40173-FTM-Channel-compare-interrupts-(Teensy-3-5)
 *
 *
 *  Finalement, le plus gros a été pompé d'ici :
 *  https://forum.pjrc.com/threads/40825-T3-6-Using-an-external-clock-for-FTM-timers
 *
 *  Pour rappel la datasheet du proco de teensy 3.2 :
 *  https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
 *
 */
 
 // classe permettant de dechiffrer les odometres

class QuadDecode_t {
  public:
    int16_t retenue1;
    int16_t retenue2;

    bool first_interrupt_1;
    bool first_interrupt_2;

    QuadDecode_t() {
      retenue1 = 0;
      retenue2 = 0;

      first_interrupt_1 = true;
      first_interrupt_2 = true;

      //initialise FTM_CLKIN0 and Flextimer 1 CH0
      FTM1_MODE = 0x05;   //set write-protect disable (WPDIS) bit to modify other registers
                          //FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1(no restriction FTM)
      FTM1_SC = 0x00;     //set FTM1 Status/Control to zero = disabled (enabled later in setup)
      FTM1_CNT = 0x0000;  //reset count to zero
      FTM1_MOD = 0xFFFF; //Maximum value of the counter
      FTM1_C0SC = 0x14;   // CHF=0, CHIE=0 (disable interrupt, use software polling), MSB=0 MSA=1, ELSB=0 ELSA=1 (output compare - toggle), 0, DMA=0

      FTM2_MODE = 0x05;   //set write-protect disable (WPDIS) bit to modify other registers
                          //FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1(no restriction FTM)
      FTM2_SC = 0x00;     //set FTM1 Status/Control to zero = disabled (enabled later in setup)
      FTM2_CNT = 0x0000;  //reset count to zero
      FTM2_MOD = 0xFFFF; //Maximum value of the counter
      FTM2_C0SC = 0x14;   // CHF=0, CHIE=0 (disable interrupt, use software polling), MSB=0 MSA=1, ELSB=0 ELSA=1 (output compare - toggle), 0, DMA=0

      // Set registers to count quadrature
      //FTM1_FILTER=0x22;	// 2x4 clock filters on both channels
      FTM1_CNTIN=0x0000;
      FTM1_QDCTRL=0b11000001;

      //FTM2_FILTER=0x22;	// 2x4 clock filters on both channels
      FTM2_CNTIN=0x0000;
      FTM2_QDCTRL=0b11000001;

      //enable FTM interrupt within NVIC table
      NVIC_ENABLE_IRQ(IRQ_FTM1);
      NVIC_ENABLE_IRQ(IRQ_FTM2);

      //configure Teensy port pins
      PORTA_PCR12 = 0x00000712;   //Alt7-QD_FTM1,FilterEnable,Pulldown
      PORTA_PCR13 = 0x00000712;
      //configure Teensy port pins
      PORTB_PCR18 = 0x00000612;   //Alt7-QD_FTM1,FilterEnable,Pulldown
      PORTB_PCR19 = 0x00000612;

      //enable external clock (10 MHz), no prescale
      FTM1_C0V = 0;   //compare value = 0
      FTM1_SC = 0x58; //(Note - FTM1_SC [TOF=0 TOIE=1 CPWMS=0 CLKS=11 (external clocks enabled) PS=000 [no prescale divide])

      //enable external clock (10 MHz), no prescale
      FTM2_C0V = 0;   //compare value = 0
      FTM2_SC = 0x58; //(Note - FTM1_SC [TOF=0 TOIE=1 CPWMS=0 CLKS=11 (external clocks enabled) PS=000 [no prescale divide])

    }

    void resetCounter1( ) {
      FTM1_CNT = 0xFFFF; //When we try to write CNT, CNT take the value of CNTIN (aka 0x0000 here) !
      retenue1 = 0;
    }

    void resetCounter2( ) {
      FTM2_CNT = 0xFFFF; //When we try to write CNT, CNT take the value of CNTIN (aka 0x0000 here) !
      retenue2 = 0;
    }

    int32_t getCounter1( )const {
      int32_t c = FTM1_CNT +  0x10000 * retenue1;
      return c;
    }

    int32_t getCounter2( ) const{
      int32_t c = (uint32_t) FTM2_CNT + (uint32_t) 0x10000 * retenue2;

      return c;
    }
};


QuadDecode_t QuadDecode;

//ISR routine for FlexTimer1 Module
extern "C" void ftm1_isr(void) {
  if( (FTM1_SC & FTM_SC_TOF) != 0) {  //read the timer overflow flag (TOF in FTM1_SC)
    FTM1_SC &= ~FTM_SC_TOF;           //if set, clear overflow flag
    if(QuadDecode.first_interrupt_1) { //When FTM is initialized, it generate an interrupt. We escape it because it is not an proper overflow
      QuadDecode.first_interrupt_1 = false;
      return;
    }
    if(FTM1_CNT < 0x8000) {   //distinguish if the overflow is by the min value or the max
      QuadDecode.retenue1++;
    } else {
      QuadDecode.retenue1--;
    }
  }
}

//ISR routine for FlexTimer1 Module
extern "C" void ftm2_isr(void) {
  if( (FTM2_SC & FTM_SC_TOF) != 0) {  //read the timer overflow flag (TOF in FTM1_SC)
    FTM2_SC &= ~FTM_SC_TOF;           //if set, clear overflow flag
    if(QuadDecode.first_interrupt_2) { //When FTM is initialized, it generate an interrupt. We escape it because it is not an proper overflow
      QuadDecode.first_interrupt_2 = false;
      return;
    }
    if(FTM2_CNT < 0x8000) {   //distinguish if the overflow is by the min value or the max
      QuadDecode.retenue2++;
    } else {
      QuadDecode.retenue2--;
    }
  }
}

#endif
