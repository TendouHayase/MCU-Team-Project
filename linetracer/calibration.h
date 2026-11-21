#pragma once

// 측정 모드. 전원을 켜고 3초 뒤 calibration.cpp의 TEST_NUMBER 테스트를 한 번
// 달리고 멈춘다. 다시 하려면 리셋한다
void calibration_setup();
void calibration_loop();
