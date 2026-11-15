#include "src/AFMotor/AFMotor.h"
#include "src/range/range_sensor.h"

#include <Arduino.h>

AF_DCMotor motor_l(1); // 모터드라이버 L293D  1: M1에 연결,  4: M4에 연결
AF_DCMotor motor_r(4);

// drive.cpp
void lineFollowStep();
void driveStop();

// 거리는 mm 단위. 15cm 이내면 멈추고, 20cm 이상 멀어지면 다시 출발한다.
const uint16_t WALL_STOP_MM = 150;
const uint16_t WALL_RESUME_MM = 200;

void setup()
{
    driveStop();
    RangeSensor::begin(); // 초음파 측정(60ms 주기)과 에코 인터럽트 시작
}

void loop()
{
    static bool stopped = false;

    const RangeStatus range = RangeSensor::read();
    if (range.status == RangeStatus::UNKNOWN)
        stopped = false; // 센서 응답이 없으면 멈춰 있지 않는다
    else if (stopped)
        stopped = range.distance < WALL_RESUME_MM;
    else
        stopped = range.distance <= WALL_STOP_MM;

    if (stopped)
        driveStop();
    else
        lineFollowStep();
}
