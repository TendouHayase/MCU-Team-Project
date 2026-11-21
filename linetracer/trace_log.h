#pragma once
#include <stdint.h>

// 주행 중 마지막 몇 초를 RAM에 계속 덮어쓰며 기록하고, 멈출 때 EEPROM에 옮긴다.
// USB 시리얼을 열면 보드가 리셋돼서 RAM에 둔 기록은 남지 않는다

void trace_sample(uint8_t line, uint8_t mode, int32_t y_um, int32_t psi_cdeg,
                  int16_t pwm_l, int16_t pwm_r);

// RAM 기록을 EEPROM에 저장한다. 바뀐 바이트만 쓰지만 최대 3초쯤 걸린다.
// 주행 시작 3초 안에는 저장하지 않는다 (USB를 꽂아 켜질 때 덮어쓰지 않게)
void trace_flush();

// EEPROM 기록을 CSV로 시리얼에 출력한다. Serial.begin() 뒤에 부른다
void trace_dump();
