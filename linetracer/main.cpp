/* 이 소스는 에듀이노(Eduino)에 의해서 번역, 수정, 작성되었고 소유권 또한
 * 에듀이노의 것입니다. 소유권자의 허락을 받지 않고 무단으로 수정, 삭제하여
 * 배포할 시 법적인 처벌을 받을 수도 있습니다.
 * 에듀이노 메탈 라인트레이서 기본예제
 */
#include <Arduino.h>
#include "src/AFMotor/AFMotor.h"
#include "src/range/range_sensor.h"

AF_DCMotor motor_l(1); // 모터드라이버 L293D  1: M1에 연결,  4: M4에 연결
AF_DCMotor motor_r(4);

void cornerRecoveryStep();
void resetDriveStateAfterWallStop();

namespace {

// setup()에서 모터를 정지 상태로 준비할 때만 사용한다.
// 실제 주행 PWM은 drive.cpp에서 설정한다.
constexpr uint8_t INITIAL_MOTOR_PWM = 100;

// RangeSensor::read()의 거리는 mm 단위다.
// 15cm 이내에서 멈추고, 20cm 이상으로 멀어지면 다시 출발한다.
constexpr uint16_t WALL_STOP_MM = 150;
constexpr uint16_t WALL_RESUME_MM = 200;

bool shouldStopForWall(const RangeStatus& range, bool alreadyStopped)
{
    // 센서값이 오래 갱신되지 않으면 예전의 근접 거리로 계속 멈춰 있지 않는다.
    if (range.status == RangeStatus::UNKNOWN)
        return false;

    // 정지 중에는 20cm까지 멀어져야 재출발한다. 주행 중에는 15cm에서 멈춘다.
    return alreadyStopped ? range.distance < WALL_RESUME_MM
                          : range.distance <= WALL_STOP_MM;
}

} // namespace

void setup()
{
    motor_l.setSpeed(INITIAL_MOTOR_PWM);
    motor_l.run(RELEASE);
    motor_r.setSpeed(INITIAL_MOTOR_PWM);
    motor_r.run(RELEASE);

    // 초음파 센서의 주기 측정과 에코 인터럽트를 시작한다.
    RangeSensor::begin();
}

void loop()
{
    // 거리값을 한 번만 읽고 벽 앞에서 멈춘 상태를 유지한다.
    static bool stoppedForWall = false;
    const RangeStatus range = RangeSensor::read();
    const bool wasStopped = stoppedForWall;
    stoppedForWall = shouldStopForWall(range, wasStopped);

    if (stoppedForWall) {
        if (!wasStopped) {
            // AFMotor의 RELEASE는 모터 출력을 끈다. BRAKE는 이 라이브러리의
            // run()에서 처리되지 않는다. 상태가 바뀔 때 한 번만 명령한다.
            motor_l.run(RELEASE);
            motor_r.run(RELEASE);
            // 주행 코드의 모터 명령 기억과 코너 상태를 초기화한다.
            resetDriveStateAfterWallStop();
        }
        return;
    }

    // 벽이 멀어지면 drive.cpp가 모터 속도와 방향을 다시 명령해 출발한다.
    cornerRecoveryStep();
}
