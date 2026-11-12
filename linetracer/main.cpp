/* 이 소스는 에듀이노(Eduino)에 의해서 번역, 수정, 작성되었고 소유권 또한
 * 에듀이노의 것입니다. 소유권자의 허락을 받지 않고 무단으로 수정, 삭제하여
 * 배포할 시 법적인 처벌을 받을 수도 있습니다.
 * 에듀이노 메탈 라인트레이서 기본예제
 */

#include "src/AFMotor/AFMotor.h"
#include "src/range_sensor.h"

#include <Arduino.h>

constexpr uint8_t VELOCITY = 230;

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

void loop()
{
    int val1 = digitalRead(A0); // 라인센서1
    int val2 = digitalRead(A5); // 라인센서2

    if (val1 == 0 && val2 == 0)
    { // 직진
        motor_l.run(FORWARD);
        motor_r.run(FORWARD);
    }
    else if (val1 == 0 && val2 == 1)
    { // 우회전
        motor_l.run(FORWARD);
        motor_r.run(RELEASE);
    }
    else if (val1 == 1 && val2 == 0)
    { // 좌회전
        motor_l.run(RELEASE);
        motor_r.run(FORWARD);
    }
    else if (val1 == 1 && val2 == 1)
    { // 정지
        motor_l.run(RELEASE);
        motor_r.run(RELEASE);
    }
}

ISR(PCINT0_vect) { RangeSensor::echo_edge_isr(); }
