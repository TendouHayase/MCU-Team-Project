#include "src/AFMotor/AFMotor.h"

#include <Arduino.h>

// 모터 객체와 setup()/loop()는 main.cpp에 있다.
extern AF_DCMotor motor_l; // M1: 왼쪽 바퀴
extern AF_DCMotor motor_r; // M4: 오른쪽 바퀴

namespace
{
// 직진에서 오른쪽으로 휘므로 오른쪽 바퀴를 더 세게 준다. 1차 성공본의
// 200/240 비율을 옮긴 시작값이며, 라인 없이 직진시켜 편차가 0이 되게 맞춘다.
// PWM과 바퀴 속도는 비례하지 않으므로 기본 속도를 바꾸면 다시 맞춰야 한다.
const uint8_t STRAIGHT_LEFT_PWM = 150;
const uint8_t STRAIGHT_RIGHT_PWM = 180;
// 회전은 제자리 회전(한쪽 전진, 반대쪽 후진). 너무 많이 돌면 이 값을 낮춘다.
const uint8_t TURN_PWM = 150;
// RECOVER에서 dir쪽으로 휘게 할 때 안쪽 바퀴에서 빼는 값.
// 선이 가운데 있었다면 LOST_MS 안에 어느 한쪽 센서가 선을 밟을 만큼은 휘어야
// 한다. 너무 크면 직선에서 흔들린다.
const uint8_t RECOVER_BIAS_PWM = 40;

// RECOVER에서 이 시간 동안 검정이 없으면 선을 놓친 것으로 본다.
// 직각을 지나친 뒤 구동축이 코너까지 오는 시간(센서-구동축 5~10cm / 속도)보다
// 길어야 SEARCH의 제자리 회전 후 선 위에 정렬된다.
const unsigned long LOST_MS = 250;
const unsigned long SEARCH_MAX_MS = 2000;
// 선 경계에서 비교기 출력이 떨리는 것을 거른다.
const unsigned long DEBOUNCE_US = 2000;
// 호출 간격이 이보다 길면 벽 정지로 멈춰 있던 것으로 보고 그만큼 상태 시각을
// 민다. 그러지 않으면 재출발 직후 LOST_MS/SEARCH_MAX_MS가 지난 것으로 판정된다.
const unsigned long PAUSE_GAP_MS = 20;

// CENTER  : 선이 두 센서 사이에 있다고 확신 (시작 상태)
// STEER   : dir쪽 센서가 지금 검정
// RECOVER : 양쪽 흰색, 직전 측면 이벤트가 dir쪽. 선은 가운데 또는 dir쪽 바깥
// SEARCH  : 선을 놓침. 선은 dir쪽 바깥
// 2센서로는 "가운데로 돌아왔다"를 확인할 수 없으므로 CENTER로 돌아가는 전이는
// 없다.
enum class State : uint8_t
{
    CENTER,
    STEER,
    RECOVER,
    SEARCH
};

State state = State::CENTER;
int8_t dir = 0; // -1 왼쪽, +1 오른쪽
unsigned long enteredAt = 0;

void enter(State next, int8_t nextDir, unsigned long now)
{
    state = next;
    dir = nextDir;
    enteredAt = now;
}

void drive(uint8_t leftSpeed, uint8_t leftMode, uint8_t rightSpeed,
           uint8_t rightMode)
{
    // 모터 실드 명령은 느리므로(약 150us) 바뀐 값만 보낸다.
    static uint8_t lastLeftSpeed = 255, lastRightSpeed = 255;
    static uint8_t lastLeftMode = 0, lastRightMode = 0;

    if (leftSpeed != lastLeftSpeed)
    {
        motor_l.setSpeed(leftSpeed);
        lastLeftSpeed = leftSpeed;
    }
    if (rightSpeed != lastRightSpeed)
    {
        motor_r.setSpeed(rightSpeed);
        lastRightSpeed = rightSpeed;
    }
    if (leftMode != lastLeftMode)
    {
        motor_l.run(leftMode);
        lastLeftMode = leftMode;
    }
    if (rightMode != lastRightMode)
    {
        motor_r.run(rightMode);
        lastRightMode = rightMode;
    }
}

void straight()
{
    drive(STRAIGHT_LEFT_PWM, FORWARD, STRAIGHT_RIGHT_PWM, FORWARD);
}

void pivot(int8_t d)
{
    if (d < 0)
        drive(TURN_PWM, BACKWARD, TURN_PWM, FORWARD);
    else
        drive(TURN_PWM, FORWARD, TURN_PWM, BACKWARD);
}

void arc(int8_t d)
{
    if (d < 0)
        drive(STRAIGHT_LEFT_PWM - RECOVER_BIAS_PWM, FORWARD, STRAIGHT_RIGHT_PWM,
              FORWARD);
    else
        drive(STRAIGHT_LEFT_PWM, FORWARD, STRAIGHT_RIGHT_PWM - RECOVER_BIAS_PWM,
              FORWARD);
}

// 라인 센서 A0(왼쪽), A5(오른쪽): LOW=흰색, HIGH=검정.
// bit0 = 왼쪽, bit1 = 오른쪽. DEBOUNCE_US 동안 같은 값이어야 반영한다.
uint8_t readLine()
{
    static uint8_t stable = 0, candidate = 0;
    static unsigned long candidateAt = 0;

    const unsigned long now = micros();
    const uint8_t raw =
        (digitalRead(A0) == HIGH ? 1 : 0) | (digitalRead(A5) == HIGH ? 2 : 0);
    if (raw != candidate)
    {
        candidate = raw;
        candidateAt = now;
    }
    else if (now - candidateAt >= DEBOUNCE_US)
    {
        stable = candidate;
    }
    return stable;
}

} // namespace

// 모터 출력을 끈다. (AFMotor 의 run()은 BRAKE 를 처리하지 않는다)
void driveStop() { drive(0, RELEASE, 0, RELEASE); }

void lineFollowStep()
{
    static unsigned long lastCallAt = 0;

    const unsigned long now = millis();
    if (now - lastCallAt >= PAUSE_GAP_MS)
        enteredAt += now - lastCallAt;
    lastCallAt = now;

    const uint8_t line = readLine();
    const bool leftBlack = line & 1;
    const bool rightBlack = line & 2;
    const bool bothBlack = leftBlack && rightBlack;
    int8_t side = 0;
    if (leftBlack != rightBlack)
        side = leftBlack ? -1 : 1;

    switch (state)
    {
    case State::CENTER:
        if (side != 0)
            enter(State::STEER, side, now);
        break;
    case State::STEER:
        // 양쪽 검정은 기운 채 직각에 진입한 경우이므로 회전을 유지한다.
        if (side != 0 && side != dir)
            enter(State::STEER, side, now);
        else if (side == 0 && !bothBlack)
            enter(State::RECOVER, dir, now);
        break;
    case State::RECOVER:
    case State::SEARCH:
        if (side != 0)
            enter(State::STEER, side, now);
        else if (bothBlack)
            enter(State::STEER, dir, now);
        else if (state == State::RECOVER && now - enteredAt >= LOST_MS)
            enter(State::SEARCH, dir, now);
        break;
    }

    const unsigned long elapsed = now - enteredAt;
    switch (state)
    {
    case State::CENTER:
        straight();
        break;
    case State::STEER:
        pivot(dir);
        break;
    case State::RECOVER:
        // 검정이 사라진 뒤 더 도는 hold는 두지 않는다. 직각에서 덜 돈 것은
        // SEARCH가 처리하고, hold는 접촉마다 전진을 멈추게 해 느려진다.
        arc(dir);
        break;
    case State::SEARCH:
        if (elapsed < SEARCH_MAX_MS)
            pivot(dir);
        else
            driveStop();
        break;
    }
}
