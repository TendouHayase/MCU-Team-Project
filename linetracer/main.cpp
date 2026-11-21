#include "trace_log.h"

#include "src/AFMotor/AFMotor.h"
#include "src/range/range_sensor.h"

#include <Arduino.h>

AF_DCMotor motor_l(1); // 모터드라이버 L293D  1: M1에 연결,  4: M4에 연결
AF_DCMotor motor_r(4);

// drive.cpp
void line_follow_step();
void drive_stop();

// calibration.cpp. true로 올리면 주행 대신 바퀴 속도 측정 순서를 돈다
const bool CALIBRATION_MODE = false;
void calibration_setup();
void calibration_loop();

// 거리는 mm 단위. 이 안이면 멈추고, 20cm 이상 멀어지면 다시 출발한다.
// 평가는 벽에서 10cm +-5cm라 한가운데를 노린다. 실차에서 차체 맨 앞이
// 135일 때 6~7cm, 170일 때 평균 13cm에 섰다. 기준 1mm에 정지 위치가 약
// 1.9mm씩 움직여서(원인 미확인) 두 점 사이를 이어 10cm가 되는 값을 골랐다.
// 155에서 평균 11cm에 섰다
const uint16_t WALL_STOP_MM = 155;
const uint16_t WALL_RESUME_MM = 200;
// 기록 저장은 EEPROM 쓰기라 최대 3초 멈춘다. 주변 물건 때문에 잠깐 서는 것까지
// 저장하면 그때마다 3초씩 서 버려서, 이만큼 계속 서 있을 때만 저장한다
const unsigned long SAVE_AFTER_STOP_MS = 1000;

void setup()
{
    drive_stop();
    // USB를 꽂아 리셋될 때마다 지난 주행 기록을 내보낸다
    Serial.begin(115200);
    trace_dump();
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
    static unsigned long stopped_at = 0;
    static bool saved = false;
    const bool was_stopped = stopped;

    const RangeStatus range = RangeSensor::read();
    if (range.status == RangeStatus::UNKNOWN)
        stopped = false; // 센서 응답이 없으면 멈춰 있지 않는다
    else if (stopped)
        stopped = range.distance < WALL_RESUME_MM;
    else
        stopped = range.distance <= WALL_STOP_MM;

    if (stopped)
    {
        drive_stop();
        if (!was_stopped)
        {
            stopped_at = millis();
            saved = false;
        }
        // 손을 대서 1초 넘게 세우면 직전 몇 초가 저장된다. 서 있는 동안은
        // 기록이 쌓이지 않아서 늦게 저장해도 같은 구간이다
        if (!saved && millis() - stopped_at >= SAVE_AFTER_STOP_MS)
        {
            trace_flush();
            saved = true;
        }
    }
    else
    {
        line_follow_step();
    }
}
