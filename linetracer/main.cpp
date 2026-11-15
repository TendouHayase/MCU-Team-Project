/* 이 소스는 에듀이노(Eduino)에 의해서 번역, 수정, 작성되었고 소유권 또한
 * 에듀이노의 것입니다. 소유권자의 허락을 받지 않고 무단으로 수정, 삭제하여
 * 배포할 시 법적인 처벌을 받을 수도 있습니다.
 * 에듀이노 메탈 라인트레이서 기본예제
 */

#include "src/AFMotor/AFMotor.h"
#include "src/range_sensor.h"

#include <Arduino.h>

constexpr uint8_t VELOCITY = 100;

AF_DCMotor motor_l(1); // 모터드라이버 L293D  1: M1에 연결,  4: M4에 연결
AF_DCMotor motor_r(4);

// 주행 제어는 drive.cpp에 두고, 이 파일은 Arduino 진입 함수만 유지한다.
void cornerRecoveryStep();

void setup()
{
    motor_l.setSpeed(VELOCITY); // 왼쪽 모터의 속도
    motor_l.run(RELEASE);
    motor_r.setSpeed(VELOCITY); // 오른쪽 모터의 속도
    motor_r.run(RELEASE);
    // 벽 감지는 다음 단계에서 완성한다. 현재 거리 센서 코드는 송신 펄스와
    // Timer1 비교 인터럽트 처리 함수가 없어 라인 주행 중에는 시작하지 않는다.
    // RangeSensor::begin();
}

void loop()
{
    // 센서 판정과 단계별 조향은 drive.cpp에서 처리한다.
    cornerRecoveryStep();
}

ISR(PCINT0_vect) { RangeSensor::echo_edge_isr(); }
