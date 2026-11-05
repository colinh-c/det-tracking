#pragma once
#include <cstdint>
#include "Arduino.h"
// Stub mirrors the subset of TMCStepper the firmware uses. Getters return
// plausible healthy values so DIAG can be exercised in simulation; the real
// library's readback comes off the TMC2209 over UART.
struct TMC2209Stepper {
  TMC2209Stepper(SerialStub*, float, uint8_t){}
  void begin(){}
  void blank_time(uint8_t){}
  void mstep_reg_select(bool){}
  void pwm_autoscale(bool){}
  void TPWMTHRS(uint32_t){}
  void iholddelay(uint8_t){}
  void TPOWERDOWN(uint8_t){}
  void rms_current(uint16_t ma, float){ _ma = ma; }

  // setter / getter pairs, as in the real library
  void toff(uint8_t v){ _toff = v; }
  uint8_t toff(){ return _toff; }
  void microsteps(uint16_t v){ _mres = v; }
  uint16_t microsteps(){ return _mres; }
  void en_spreadCycle(bool v){ _spread = v; }
  bool en_spreadCycle(){ return _spread; }

  uint8_t  test_connection(){ return 0; }
  uint32_t status = 0xC0010000u;
  unsigned status_reads = 0;
  uint32_t DRV_STATUS(){ ++status_reads; return status; }
  uint16_t irun(){ return 16; }
  uint16_t ihold(){ return 8; }
  bool ola(){ return (DRV_STATUS() >> 6) & 1; }  bool olb(){ return (DRV_STATUS() >> 7) & 1; }
  bool s2ga(){ return (DRV_STATUS() >> 2) & 1; } bool s2gb(){ return (DRV_STATUS() >> 3) & 1; }
  bool ot(){ return (DRV_STATUS() >> 1) & 1; } bool otpw(){ return DRV_STATUS() & 1; }
  bool stst(){ return (DRV_STATUS() >> 31) & 1; }
  uint16_t cs_actual(){ return (DRV_STATUS() >> 16) & 0x1F; }
private:
  uint8_t  _toff = 0;
  uint16_t _mres = 16, _ma = 0;
  bool     _spread = false;
};
