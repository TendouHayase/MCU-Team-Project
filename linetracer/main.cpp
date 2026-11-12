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

void setup()
{
    motor_l.setSpeed(VELOCITY); // 왼쪽 모터의 속도
    motor_l.run(RELEASE);
    motor_r.setSpeed(VELOCITY); // 오른쪽 모터의 속도
    motor_r.run(RELEASE);
    RangeSensor::begin();
}
void cornerRecoveryStep();

void loop()
{
    cornerRecoveryStep();
}

ISR(PCINT0_vect) { RangeSensor::echo_edge_isr(); }
