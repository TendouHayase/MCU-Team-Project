// 2개 디지털 라인 센서의 단계별 코너 복구 제어기.
// src/main.cpp의 loop()에서 cornerRecoveryStep()을 호출해야 동작한다.
#include <Arduino.h>
#include "src/AFMotor/AFMotor.h"

// 기존 main.cpp에서 만든 모터를 공유한다. 여기서 다시 만들지 않는다.
extern AF_DCMotor motor_l; // 실드 M1: 왼쪽 바퀴
extern AF_DCMotor motor_r; // 실드 M4: 오른쪽 바퀴

namespace {
// setSpeed()의 범위는 0~255. 바닥과 배터리에 맞춰 조정할 값들이다.
const uint8_t STRAIGHT_SPEED = 100; // 직진 속도
const uint8_t GENTLE_OUTER = 110;   // 1단계: 바깥 바퀴
const uint8_t GENTLE_INNER = 65;    // 1단계: 안쪽 바퀴
const uint8_t STRONG_OUTER = 125;   // 2단계: 바깥 바퀴
const uint8_t STRONG_INNER = 25;    // 2단계: 안쪽 바퀴
const uint8_t PIVOT_SPEED = 90;     // 3단계: 제자리 회전 속도

// 같은 방향으로 계속 보정하면 시간에 따라 회전을 강화한다.
const unsigned long STRONG_AFTER_MS = 180;
const unsigned long PIVOT_AFTER_MS = 450;
// 양쪽 센서가 HIGH인 상태가 계속되면 무한 회전하지 않도록 멈춘다.
const unsigned long BOTH_HIGH_TIMEOUT_MS = 1500;
// 최근 방향 기록이 이 시간보다 오래되면 기본 탐색 방향을 사용한다.
const unsigned long RECENT_DIRECTION_MS = 700;
const int8_t DEFAULT_SEARCH_DIRECTION = 1; // +1 오른쪽, -1 왼쪽
const bool ENABLE_PIVOT = true;            // false면 후진 단계 비활성화
const bool PRINT_SENSOR_STATES = false;     // true면 시리얼 모니터에 0/1 출력

int8_t turnDirection = 0;             // +1 우회전, -1 좌회전, 0 미결정
unsigned long correctionStartedAt = 0; // 현재 방향 보정을 시작한 시각
unsigned long directionSeenAt = 0;      // 한쪽 센서가 방향을 알려준 마지막 시각
unsigned long bothHighStartedAt = 0;    // 양쪽 HIGH가 시작된 시각
bool correcting = false;
bool bothHighActive = false;

// 두 바퀴의 PWM 속도와 회전 방향을 함께 지정한다.
void drive(uint8_t leftSpeed, uint8_t leftMode,
           uint8_t rightSpeed, uint8_t rightMode)
{
    motor_l.setSpeed(leftSpeed);
    motor_r.setSpeed(rightSpeed);
    motor_l.run(leftMode);
    motor_r.run(rightMode);
}
} // namespace

// delay() 없이 반복 호출하므로 매번 센서 상태를 다시 확인할 수 있다.
void cornerRecoveryStep()
{
    const unsigned long now = millis();
    const int left = digitalRead(A0);
    const int right = digitalRead(A5);

    // 검정색에서 LED가 꺼진다는 사실만으로 전기적 HIGH/LOW를 단정할 수 없다.
    // 실제 값을 확인하려면 PRINT_SENSOR_STATES를 true로 바꾸고
    // 시리얼 모니터를 9600 baud로 열어 0/0 또는 1/1을 확인한다.
    if (PRINT_SENSOR_STATES) {
        static bool serialStarted = false;
        static unsigned long lastPrintedAt = 0;
        if (!serialStarted) {
            Serial.begin(9600);
            serialStarted = true;
        }
        if (now - lastPrintedAt >= 200) {
            lastPrintedAt = now;
            Serial.print("A0=");
            Serial.print(left);
            Serial.print(" A5=");
            Serial.println(right);
        }
    }

    // 기존 main.cpp의 상태표: 0/0 직진, 0/1 우회전, 1/0 좌회전.
    // LED 꺼짐과 digitalRead()의 HIGH/LOW 관계는 실측으로 확인해야 한다.
    if (left == LOW && right == LOW) {
        // 직진 상태로 복귀하면 이전 코너의 단계 타이머를 초기화한다.
        correcting = false;
        bothHighActive = false;
        drive(STRAIGHT_SPEED, FORWARD, STRAIGHT_SPEED, FORWARD);
        return;
    }

    int8_t observedDirection = 0;
    if (left == LOW && right == HIGH)
        observedDirection = 1; // 오른쪽 센서 반응 → 우회전
    else if (left == HIGH && right == LOW)
        observedDirection = -1; // 왼쪽 센서 반응 → 좌회전

    if (observedDirection != 0) {
        // 한쪽 센서가 방향을 알려줬으므로 기록한다.
        bothHighActive = false;
        directionSeenAt = now;
        // 시작 또는 방향 전환 시에만 타이머를 리셋해야 2·3단계에 도달한다.
        if (!correcting || observedDirection != turnDirection) {
            correctionStartedAt = now;
            correcting = true;
        }
        turnDirection = observedDirection;
    } else {
        // HIGH/HIGH: 한쪽 센서만의 방향 정보가 없어 탐색 방향을 고른다.
        // 이전 코드는 방향 기록이 없으면 정지해서 역회전에 도달하지 못했다.
        if (!bothHighActive) {
            bothHighActive = true;
            bothHighStartedAt = now;
            if (turnDirection == 0 ||
                now - directionSeenAt > RECENT_DIRECTION_MS) {
                // 최근 방향이 없으면 기본적으로 오른쪽을 탐색한다.
                turnDirection = DEFAULT_SEARCH_DIRECTION;
                correctionStartedAt = now;
                correcting = true;
            } else if (!correcting) {
                correctionStartedAt = now;
                correcting = true;
            }
        }
        if (now - bothHighStartedAt >= BOTH_HIGH_TIMEOUT_MS) {
            // 충분히 탐색했는데도 복귀하지 못하면 두 모터를 놓고 정지.
            correcting = false;
            turnDirection = 0;
            drive(0, RELEASE, 0, RELEASE);
            return;
        }
    }

    const unsigned long elapsed = now - correctionStartedAt;
    uint8_t outer = GENTLE_OUTER;
    uint8_t inner = GENTLE_INNER;
    uint8_t innerMode = FORWARD;

    if (ENABLE_PIVOT && elapsed >= PIVOT_AFTER_MS) {
        // 3단계: 바깥 바퀴 전진 + 안쪽 바퀴 후진으로 제자리 회전.
        outer = PIVOT_SPEED;
        inner = PIVOT_SPEED;
        innerMode = BACKWARD;
    } else if (elapsed >= STRONG_AFTER_MS) {
        // 2단계: 두 바퀴 모두 전진하면서 속도 차이를 키운다.
        outer = STRONG_OUTER;
        inner = STRONG_INNER;
    }
    // 180ms 전에는 1단계: 작은 속도 차이로 부드럽게 조향한다.

    if (turnDirection > 0)
        drive(outer, FORWARD, inner, innerMode); // 우회전: 오른쪽이 안쪽
    else
        drive(inner, innerMode, outer, FORWARD); // 좌회전: 왼쪽이 안쪽
}
