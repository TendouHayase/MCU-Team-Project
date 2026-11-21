#include "calibration.h"
#include "drive.h"

#include "src/AFMotor/AFMotor.h"
#include "src/range/range_sensor.h"

#include <Arduino.h>

AF_DCMotor motor_l(1); // L293D 실드 M1: 왼쪽 바퀴
AF_DCMotor motor_r(4); // M4: 오른쪽 바퀴

namespace
{
// true로 올리면 주행 대신 측정 모드(calibration.cpp)를 돈다
const bool CALIBRATION_MODE = false;

// 거리는 mm. 이 안이면 멈추고, RESUME 이상 멀어지면(벽을 치우면) 다시 출발한다.
// 평가는 벽에서 10cm +-5cm라 한가운데를 노린다. 실차에서 차체 맨 앞이
// 135일 때 6~7cm, 170일 때 평균 13cm, 155일 때 평균 11cm에 섰다
const uint16_t WALL_STOP_MM = 155;
const uint16_t WALL_RESUME_MM = 200;
} // namespace

void setup()
{
    drive_stop();
    RangeSensor::begin(); // 초음파 측정(60ms 주기)과 에코 인터럽트 시작
    if (CALIBRATION_MODE)
        calibration_setup();
}

void loop()
{
    if (CALIBRATION_MODE)
    {
        calibration_loop();
        return;
    }

    static bool stopped = false;
    const RangeStatus range = RangeSensor::read();
    if (range.status == RangeStatus::UNKNOWN)
        stopped = false; // 센서 응답이 없으면 멈춰 있지 않는다
    else if (stopped)
        stopped = range.distance < WALL_RESUME_MM;
    else
        stopped = range.distance <= WALL_STOP_MM;

    if (stopped)
        drive_stop();
    else
        line_follow_step();
}
