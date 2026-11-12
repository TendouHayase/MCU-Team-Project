#include <Arduino.h>
#include "src/AFMotor/AFMotor.h"

extern AF_DCMotor motor_l; // M1: 왼쪽 바퀴
extern AF_DCMotor motor_r; // M4: 오른쪽 바퀴

namespace {
constexpr uint8_t STRAIGHT_SPEED = 100;
constexpr uint8_t GENTLE_OUTER = 110;
constexpr uint8_t GENTLE_INNER = 65;
constexpr uint8_t STRONG_OUTER = 125;
constexpr uint8_t STRONG_INNER = 25;
constexpr uint8_t PIVOT_SPEED = 90;

constexpr unsigned long STRONG_AFTER_MS = 180;
constexpr unsigned long PIVOT_AFTER_MS = 450;
constexpr unsigned long LOST_TIMEOUT_MS = 1500;
constexpr bool ENABLE_PIVOT = true;

int8_t lastTurn = 0; // +1 우회전, -1 좌회전
unsigned long turnStartedAt = 0;
unsigned long lastLineSeenAt = 0;
bool correcting = false;

void drive(uint8_t leftSpeed, uint8_t leftDirection,
           uint8_t rightSpeed, uint8_t rightDirection)
{
    motor_l.setSpeed(leftSpeed);
    motor_r.setSpeed(rightSpeed);
    motor_l.run(leftDirection);
    motor_r.run(rightDirection);
}
}

void cornerRecoveryStep()
{
    const unsigned long now = millis();
    const int left = digitalRead(A0);
    const int right = digitalRead(A5);

    // 기존 코드의 센서 판정: 0/0 직진, 0/1 우회전, 1/0 좌회전
    if (left == LOW && right == LOW) {
        correcting = false;
        drive(STRAIGHT_SPEED, FORWARD, STRAIGHT_SPEED, FORWARD);
        return;
    }

    int8_t detectedTurn = 0;
    if (left == LOW && right == HIGH) detectedTurn = 1;
    if (left == HIGH && right == LOW) detectedTurn = -1;

    if (detectedTurn != 0) {
        lastLineSeenAt = now;

        if (!correcting || detectedTurn != lastTurn) {
            turnStartedAt = now;
            correcting = true;
        }
        lastTurn = detectedTurn;
    } else {
        // 1/1: 양쪽 모두 라인을 감지하지 못했다고 가정
        if (lastTurn == 0 || now - lastLineSeenAt > LOST_TIMEOUT_MS) {
            correcting = false;
            lastTurn = 0;
            drive(0, RELEASE, 0, RELEASE);
            return;
        }

        if (!correcting) {
            turnStartedAt = now;
            correcting = true;
        }
    }

    const unsigned long elapsed = now - turnStartedAt;
    uint8_t outerSpeed = GENTLE_OUTER;
    uint8_t innerSpeed = GENTLE_INNER;
    uint8_t innerDirection = FORWARD;

    if (ENABLE_PIVOT && elapsed >= PIVOT_AFTER_MS) {
        outerSpeed = PIVOT_SPEED;
        innerSpeed = PIVOT_SPEED;
        innerDirection = BACKWARD;
    } else if (elapsed >= STRONG_AFTER_MS) {
        outerSpeed = STRONG_OUTER;
        innerSpeed = STRONG_INNER;
    }

    if (lastTurn > 0) {
        drive(outerSpeed, FORWARD, innerSpeed, innerDirection);
    } else {
        drive(innerSpeed, innerDirection, outerSpeed, FORWARD);
    }
}