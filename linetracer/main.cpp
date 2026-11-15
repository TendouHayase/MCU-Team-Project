/* 이 소스는 에듀이노(Eduino)에 의해서 번역, 수정, 작성되었고 소유권 또한
 * 에듀이노의 것입니다. 소유권자의 허락을 받지 않고 무단으로 수정, 삭제하여
 * 배포할 시 법적인 처벌을 받을 수도 있습니다.
 * 에듀이노 메탈 라인트레이서 기본예제
 */
#pragma once
#include "src/AFMotor/AFMotor.h"
#include "src/range/range_sensor.h"

#include <Arduino.h>

constexpr uint8_t VELOCITY = 100;

AF_DCMotor motor_l(1); // 모터드라이버 L293D  1: M1에 연결,  4: M4에 연결
AF_DCMotor motor_r(4);

void cornerRecoveryStep();
void invalidateDriveCommandCache();

// RangeSensor::read()의 거리는 mm 단위다.
// 15cm 이내에서 멈추고, 20cm 이상으로 멀어지면 다시 출발한다.
constexpr uint16_t WALL_STOP_MM = 150;
constexpr uint16_t WALL_RESUME_MM = 200;

void setup()
{
    motor_l.setSpeed(VELOCITY); // 왼쪽 모터의 속도
    motor_l.run(RELEASE);
    motor_r.setSpeed(VELOCITY); // 오른쪽 모터의 속도
    motor_r.run(RELEASE);
    // 초음파 센서의 주기 측정과 에코 인터럽트를 시작한다.
    RangeSensor::begin();

}

void loop()
{
    // 거리값을 한 번만 읽고, 이전에 벽 때문에 멈췄는지도 기억한다.
    static bool stoppedForWall = false;
    const RangeStatus range = RangeSensor::read();
    const bool wasStopped = stoppedForWall;

    if (range.status == RangeStatus::UNKNOWN) {
        // 240ms 이상 새 에코가 없으면 마지막 근접 거리값은 오래된 값이다.
        // 벽이 치워져 에코가 없어졌을 때도 재출발할 수 있게 해제한다.
        stoppedForWall = false;
    } else if (stoppedForWall) {
        // 정지와 출발 사이에 5cm 차이를 두어 경계에서 덜컥거리지 않게 한다.
        if (range.distance >= WALL_RESUME_MM)
            stoppedForWall = false;
    } else if (range.distance <= WALL_STOP_MM) {
        // 기존의 50mm 하한을 제거한다. 벽이 5cm보다 가까워져도 멈춰야 한다.
        stoppedForWall = true;
    }

    if (stoppedForWall) {
        if (!wasStopped) {
            // AFMotor의 RELEASE는 모터 출력을 끈다. BRAKE는 이 라이브러리의
            // run()에서 처리되지 않는다. 상태가 바뀔 때 한 번만 명령한다.
            motor_l.run(RELEASE);
            motor_r.run(RELEASE);
            // drive.cpp가 기억한 이전 전진/회전 명령은 이제 실제 상태와 다르다.
            invalidateDriveCommandCache();
        }
        return;
    }

    // 벽이 멀어지면 drive.cpp가 모터 속도와 방향을 다시 명령해 출발한다.
    cornerRecoveryStep();
}
