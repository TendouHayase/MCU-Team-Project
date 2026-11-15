#include <Arduino.h>
#include "src/AFMotor/AFMotor.h"
#include "src/range/range_sensor.h"


// 모터 객체와 setup()/loop()는 기존 main.cpp에 그대로 둔다.
extern AF_DCMotor motor_l; // M1: 왼쪽 바퀴
extern AF_DCMotor motor_r; // M4: 오른쪽 바퀴

namespace {

// 직진에서 오른쪽으로 조금 휘므로 왼쪽 바퀴만 5 낮춰 시험한다.
// 현재 비교값: 왼쪽 145 / 오른쪽 150. 더 조정할 때는 왼쪽 값만 바꾼다.
// PWM은 전압 측정값이 아니라 모터에 주는 0~255 속도 명령이다.
const uint8_t LINE_STRAIGHT_LEFT_PWM = 200;
const uint8_t LINE_STRAIGHT_RIGHT_PWM = 240;
// 코너의 강한 제자리 회전은 가장 잘 되던 양쪽 150을 유지한다.
const uint8_t LINE_CORNER_PWM = 150;

// 성공했던 코너 코드의 시간 판정은 유지한다.
// 한쪽 검정을 보는 즉시 회전을 시작하고, 20~80ms에는 연속 제자리 회전한다.
const unsigned long LINE_PIVOT_START_MS = 20;
const unsigned long LINE_PIVOT_END_MS = 80;
const unsigned long LINE_CORNER_ARM_MS = 20;
const unsigned long LINE_CORNER_HOLD_MS = 40;

// 코너 뒤 직선 진입을 판정하는 시간도 성공했던 코드의 값으로 되돌린다.
// 안정화 중 같은 쪽 검정이 50ms 이어지면 새 코너로 인정한다.
const unsigned long LINE_SETTLE_MAX_MS = 1500;
const unsigned long LINE_CENTER_STABLE_MS = 200;
const unsigned long LINE_SETTLE_NEW_CORNER_MS = 50;

// 짧은 제자리 회전과 직선 전진을 번갈아 완만하게 조향한다.
// 회전 펄스의 시간은 그대로 두고, 직선 전진의 좌우 비율만 조정한다.
const unsigned long LINE_GENTLE_CYCLE_MS = 30;
const unsigned long LINE_GENTLE_PIVOT_MS = 12;
const unsigned long LINE_SETTLE_SHORT_PIVOT_MS = 8;
const unsigned long LINE_SETTLE_LONG_PIVOT_MS = 12;
const unsigned long LINE_SETTLE_STRONG_AFTER_MS = 35;

int8_t lastTurn = 0; // +1 우회전, -1 좌회전, 0 방향 기록 없음
unsigned long turnStartedAt = 0;
unsigned long whiteStartedAt = 0;
bool whiteAfterTurn = false;
bool cornerCommitted = false;

bool settling = false;
unsigned long settleStartedAt = 0;
bool settleSawSide = false;
bool settleCenterActive = false;
unsigned long settleCenterStartedAt = 0;
int8_t settleSideDirection = 0;
unsigned long settleSideStartedAt = 0;

void drive(uint8_t leftSpeed, uint8_t leftMode,
           uint8_t rightSpeed, uint8_t rightMode)
{
    // 속도나 방향이 바뀌었을 때만 모터 실드에 새 명령을 보낸다.
    static uint8_t previousLeftSpeed = 255;
    static uint8_t previousRightSpeed = 255;
    static uint8_t previousLeftMode = 0;
    static uint8_t previousRightMode = 0;

    if (leftSpeed != previousLeftSpeed) {
        motor_l.setSpeed(leftSpeed);
        previousLeftSpeed = leftSpeed;
    }
    if (rightSpeed != previousRightSpeed) {
        motor_r.setSpeed(rightSpeed);
        previousRightSpeed = rightSpeed;
    }
    if (leftMode != previousLeftMode) {
        motor_l.run(leftMode);
        previousLeftMode = leftMode;
    }
    if (rightMode != previousRightMode) {
        motor_r.run(rightMode);
        previousRightMode = rightMode;
    }
}

void straight()
{
    drive(LINE_STRAIGHT_LEFT_PWM, FORWARD,
          LINE_STRAIGHT_RIGHT_PWM, FORWARD);
}

void pivotToward(int8_t direction)
{
    // 우회전: 왼쪽 전진·오른쪽 후진. 좌회전: 왼쪽 후진·오른쪽 전진.
    // 코너에서는 두 바퀴 모두 PWM 150이며 방향만 반대로 준다.
    if (direction > 0)
        drive(LINE_CORNER_PWM, FORWARD,
              LINE_CORNER_PWM, BACKWARD);
    else
        drive(LINE_CORNER_PWM, BACKWARD,
              LINE_CORNER_PWM, FORWARD);
}

void gentleTurn(int8_t direction, unsigned long elapsed,
                unsigned long pivotMs)
{
    // 검정 쪽으로 계속 강하게 돌면 선을 지나칠 수 있다.
    // 한 주기의 앞부분만 짧게 회전하고 나머지는 두 바퀴를 전진시킨다.
    // 코너 조향 중 전진 부분은 성공했던 150/150을 유지한다.
    // 145/150은 센서가 직진 상태를 가리킬 때만 적용한다.
    if (elapsed % LINE_GENTLE_CYCLE_MS < pivotMs)
        pivotToward(direction);
    else
        drive(LINE_CORNER_PWM, FORWARD,
              LINE_CORNER_PWM, FORWARD);
}

void beginSettling(unsigned long now)
{
    settling = true;
    settleStartedAt = now;
    settleSawSide = false;
    settleCenterActive = false;
    settleSideDirection = 0;
    lastTurn = 0;
    whiteAfterTurn = false;
    cornerCommitted = false;
}

void settleOnLine(int leftSensor, int rightSensor, unsigned long now)
{
    int8_t direction = 0;
    if (leftSensor == LOW && rightSensor == HIGH)
        direction = 1;
    else if (leftSensor == HIGH && rightSensor == LOW)
        direction = -1;

    // 양쪽 센서가 같은 색이면 현재 시험 중인 145/150 비율로 전진한다.
    if (direction == 0) {
        straight();
        return;
    }

    // beginSettling() 직후 한쪽 검정으로 이 함수에 들어올 수도 있다.
    // 그 경우에도 회전 시작 시각을 이쪽에서 잡아 주기를 일정하게 한다.
    if (direction != settleSideDirection) {
        settleSideDirection = direction;
        settleSideStartedAt = now;
    }
    const unsigned long sideElapsed = now - settleSideStartedAt;
    const unsigned long pulseMs =
        sideElapsed >= LINE_SETTLE_STRONG_AFTER_MS ?
        LINE_SETTLE_LONG_PIVOT_MS : LINE_SETTLE_SHORT_PIVOT_MS;
    gentleTurn(direction, sideElapsed, pulseMs);
}

void turn(int8_t direction, bool strong, unsigned long now)
{
    // 이전 성공 코드처럼 처음에는 완만하게, 20~80ms에는 강하게 회전한다.
    // 한 바퀴만 놓던 동작은 좌우 PWM을 동일하게 유지하기 위해 짧은 회전
    // 펄스로 대체한다. 검정이 사라지면 즉시 다음 센서 상태를 판단한다.
    if (strong)
        pivotToward(direction);
    else
        gentleTurn(direction, now - turnStartedAt, LINE_GENTLE_PIVOT_MS);
}

} // namespace

// main.cpp의 loop()에서 반복 호출한다. delay() 없이 매번 센서를 다시 읽는다.
void cornerRecoveryStep()
{
    const unsigned long now = millis();
    const int leftSensor = digitalRead(A0);
    const int rightSensor = digitalRead(A5);

    if (settling) {
        int8_t sideDirection = 0;
        if (leftSensor == LOW && rightSensor == HIGH)
            sideDirection = 1;
        else if (leftSensor == HIGH && rightSensor == LOW)
            sideDirection = -1;
        const bool bothWhite = leftSensor == LOW && rightSensor == LOW;

        if (sideDirection != 0) {
            settleSawSide = true;
            settleCenterActive = false;
            if (sideDirection != settleSideDirection) {
                settleSideDirection = sideDirection;
                settleSideStartedAt = now;
            }
        } else if (bothWhite && settleSawSide) {
            settleSideDirection = 0;
            if (!settleCenterActive) {
                settleCenterStartedAt = now;
                settleCenterActive = true;
            }
        } else {
            settleSideDirection = 0;
            settleCenterActive = false;
        }

        // 이전 성공 코드의 새 코너 감지를 복구한다.
        // 1.5초 안정화 시간이 남았어도 한쪽 검정이 50ms 지속되면 회전한다.
        const bool centered = settleCenterActive &&
            now - settleCenterStartedAt >= LINE_CENTER_STABLE_MS;
        const bool timedOut = now - settleStartedAt >= LINE_SETTLE_MAX_MS;
        const bool newCorner = settleSideDirection != 0 &&
            now - settleSideStartedAt >= LINE_SETTLE_NEW_CORNER_MS;
        if (!newCorner && !centered && !timedOut) {
            settleOnLine(leftSensor, rightSensor, now);
            return;
        }
        settling = false;
        if (newCorner) {
            lastTurn = settleSideDirection;
            turnStartedAt = settleSideStartedAt;
            whiteAfterTurn = false;
            cornerCommitted = false;
        }
        if (centered) {
            straight();
            return;
        }
    }

    // A0 왼쪽 / A5 오른쪽: LOW=흰색, HIGH=검정.
    // 0/0 양쪽 흰색: 직진 또는 짧은 코너 유지.
    // 0/1 오른쪽 검정: 우회전.
    // 1/0 왼쪽 검정: 좌회전.
    // 1/1 양쪽 검정: 직진 또는 짧은 코너 유지.
    if (leftSensor == LOW && rightSensor == LOW) {
        if (lastTurn != 0 && now - turnStartedAt >= LINE_CORNER_ARM_MS) {
            if (!whiteAfterTurn) {
                whiteStartedAt = now;
                whiteAfterTurn = true;
            }
            if (now - whiteStartedAt < LINE_CORNER_HOLD_MS) {
                turn(lastTurn, true, now);
                return;
            }
            beginSettling(now);
            settleOnLine(leftSensor, rightSensor, now);
            return;
        }
        lastTurn = 0;
        whiteAfterTurn = false;
        straight();
        return;
    }

    if (leftSensor == HIGH && rightSensor == HIGH) {
        if (cornerCommitted) {
            if (!whiteAfterTurn) {
                whiteStartedAt = now;
                whiteAfterTurn = true;
            }
            if (now - whiteStartedAt < LINE_CORNER_HOLD_MS) {
                turn(lastTurn, true, now);
                return;
            }
            beginSettling(now);
            settleOnLine(leftSensor, rightSensor, now);
            return;
        }
        lastTurn = 0;
        whiteAfterTurn = false;
        straight();
        return;
    }

    const int8_t direction =
        (leftSensor == LOW && rightSensor == HIGH) ? 1 : -1;
    if (cornerCommitted && direction != lastTurn) {
        // 코너 직후 반대쪽 센서가 검정을 보면 즉시 반대로 돌지 않는다.
        beginSettling(now);
        settleOnLine(leftSensor, rightSensor, now);
        return;
    }
    if (direction != lastTurn) {
        // 직선에서 검정을 보는 첫 순간부터 회전한다. 12ms 지연은 제거했다.
        lastTurn = direction;
        turnStartedAt = now;
        cornerCommitted = false;
    }
    whiteAfterTurn = false;

    const unsigned long turnElapsed = now - turnStartedAt;
    if (turnElapsed >= LINE_CORNER_ARM_MS)
        cornerCommitted = true;
    const bool strong = turnElapsed >= LINE_PIVOT_START_MS &&
                        turnElapsed < LINE_PIVOT_END_MS;
    turn(direction, strong, now);
}
