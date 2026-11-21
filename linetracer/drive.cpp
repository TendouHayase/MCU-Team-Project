#include "drive.h"

#include "src/AFMotor/AFMotor.h"

#include <Arduino.h>

extern AF_DCMotor motor_l; // M1: 왼쪽 바퀴
extern AF_DCMotor motor_r; // M4: 오른쪽 바퀴

namespace
{
// 같은 PWM이면 오른쪽으로 휜다. 바퀴는 데드밴드(왼 39, 오 47)를 넘어야 돌고,
// 그 위로 1 : 1.2로 줘야 직진한다. 실차에서 170/204까지 모퉁이를 돌았고
// 180/216부터 놓쳤다
const uint8_t LINE_STRAIGHT_LEFT_PWM = 170;
const uint8_t LINE_STRAIGHT_RIGHT_PWM = 204;
const uint8_t LINE_CORNER_PWM = 150;

// 한쪽 검정을 본 지 20~80ms 사이에만 계속 제자리 회전한다. 그 앞은 직선에서
// 스친 것일 수 있고, 그 뒤까지 세게 돌면 선을 지나친다
const unsigned long LINE_PIVOT_START_MS = 20;
const unsigned long LINE_PIVOT_END_MS = 80;
// 한쪽으로 이만큼 돌았으면 코너로 본다
const unsigned long LINE_CORNER_ARM_MS = 20;
// 코너에서 선이 사라진 뒤에도 이만큼 더 돈다. 급한 곡선에서는 센서가 선을
// 바깥으로 지나쳐 버린다
const unsigned long LINE_CORNER_HOLD_MS = 40;

// 코너 뒤 안정화. 양쪽 흰색이 CENTER_STABLE 이어지거나 SETTLE_MAX가 지나면
// 끝난다. 같은 쪽 검정이 NEW_CORNER 이어지면 다음 코너로 보고 바로 돈다
const unsigned long LINE_SETTLE_MAX_MS = 1500;
const unsigned long LINE_CENTER_STABLE_MS = 200;
const unsigned long LINE_SETTLE_NEW_CORNER_MS = 50;

// 완만한 조향은 주기마다 앞부분만 제자리 회전하고 나머지는 전진한다.
// 회전 시간의 비율이 꺾는 세기다
const unsigned long LINE_GENTLE_CYCLE_MS = 30;
const unsigned long LINE_GENTLE_PIVOT_MS = 12;
// 안정화 중에는 더 약하게 돌다가, 같은 쪽이 STRONG_AFTER 넘게 이어지면 키운다
const unsigned long LINE_SETTLE_SHORT_PIVOT_MS = 8;
const unsigned long LINE_SETTLE_LONG_PIVOT_MS = 12;
const unsigned long LINE_SETTLE_STRONG_AFTER_MS = 35;

int8_t last_turn = 0; // +1 우회전, -1 좌회전, 0 돌고 있지 않음
unsigned long turn_started_at = 0;
// 코너 중 선이 사라진(또는 양쪽 검정이 된) 시각
unsigned long white_started_at = 0;
bool white_after_turn = false;
bool corner_committed = false;

bool settling = false;
unsigned long settle_started_at = 0;
bool settle_saw_side = false;
bool settle_center_active = false;
unsigned long settle_center_started_at = 0;
int8_t settle_side = 0;
unsigned long settle_side_started_at = 0;

void drive(uint8_t left_speed, uint8_t left_mode, uint8_t right_speed,
           uint8_t right_mode)
{
    // 실드 명령 한 번에 150us 정도 걸려서 바뀐 값만 보낸다
    static uint8_t last_left_speed = 255, last_right_speed = 255;
    static uint8_t last_left_mode = 0, last_right_mode = 0;

    if (left_speed != last_left_speed)
    {
        motor_l.setSpeed(left_speed);
        last_left_speed = left_speed;
    }
    if (right_speed != last_right_speed)
    {
        motor_r.setSpeed(right_speed);
        last_right_speed = right_speed;
    }
    if (left_mode != last_left_mode)
    {
        motor_l.run(left_mode);
        last_left_mode = left_mode;
    }
    if (right_mode != last_right_mode)
    {
        motor_r.run(right_mode);
        last_right_mode = right_mode;
    }
}

void straight()
{
    drive(LINE_STRAIGHT_LEFT_PWM, FORWARD, LINE_STRAIGHT_RIGHT_PWM, FORWARD);
}

void pivot_toward(int8_t direction)
{
    if (direction > 0)
        drive(LINE_CORNER_PWM, FORWARD, LINE_CORNER_PWM, BACKWARD);
    else
        drive(LINE_CORNER_PWM, BACKWARD, LINE_CORNER_PWM, FORWARD);
}

void gentle_turn(int8_t direction, unsigned long elapsed,
                 unsigned long pivot_ms)
{
    if (elapsed % LINE_GENTLE_CYCLE_MS < pivot_ms)
        pivot_toward(direction);
    else
        drive(LINE_CORNER_PWM, FORWARD, LINE_CORNER_PWM, FORWARD);
}

// A0 왼쪽 / A5 오른쪽, HIGH = 검정. +1 오른쪽만 검정, -1 왼쪽만 검정,
// 0 양쪽 같은 색
int8_t one_side_black(int left, int right)
{
    if (left == LOW && right == HIGH)
        return 1;
    if (left == HIGH && right == LOW)
        return -1;
    return 0;
}

void begin_settling(unsigned long now)
{
    settling = true;
    settle_started_at = now;
    settle_saw_side = false;
    settle_center_active = false;
    settle_side = 0;
    last_turn = 0;
    white_after_turn = false;
    corner_committed = false;
}

void settle_on_line(int left, int right, unsigned long now)
{
    const int8_t direction = one_side_black(left, right);
    if (direction == 0)
    {
        straight();
        return;
    }

    // begin_settling() 직후 한쪽 검정으로 바로 들어올 수도 있어서 회전 시작
    // 시각을 여기서도 잡아 펄스 주기를 맞춘다
    if (direction != settle_side)
    {
        settle_side = direction;
        settle_side_started_at = now;
    }
    const unsigned long side_elapsed = now - settle_side_started_at;
    const unsigned long pulse_ms = side_elapsed >= LINE_SETTLE_STRONG_AFTER_MS
                                       ? LINE_SETTLE_LONG_PIVOT_MS
                                       : LINE_SETTLE_SHORT_PIVOT_MS;
    gentle_turn(direction, side_elapsed, pulse_ms);
}

// 안정화를 한 단계 진행한다. 이번 단계를 여기서 처리했으면 true,
// 안정화가 끝나 평소 주행으로 넘겨야 하면 false
bool settle_step(int left, int right, unsigned long now)
{
    const int8_t side = one_side_black(left, right);
    if (side != 0)
    {
        settle_saw_side = true;
        settle_center_active = false;
        if (side != settle_side)
        {
            settle_side = side;
            settle_side_started_at = now;
        }
    }
    else if (left == LOW && right == LOW && settle_saw_side)
    {
        settle_side = 0;
        if (!settle_center_active)
        {
            settle_center_started_at = now;
            settle_center_active = true;
        }
    }
    else
    {
        settle_side = 0;
        settle_center_active = false;
    }

    const unsigned long center_ms = now - settle_center_started_at;
    const unsigned long side_ms = now - settle_side_started_at;
    const bool centered =
        settle_center_active && center_ms >= LINE_CENTER_STABLE_MS;
    const bool timed_out = now - settle_started_at >= LINE_SETTLE_MAX_MS;
    const bool new_corner =
        settle_side != 0 && side_ms >= LINE_SETTLE_NEW_CORNER_MS;
    if (!new_corner && !centered && !timed_out)
    {
        settle_on_line(left, right, now);
        return true;
    }

    settling = false;
    if (new_corner)
    {
        last_turn = settle_side;
        turn_started_at = settle_side_started_at;
        white_after_turn = false;
        corner_committed = false;
    }
    if (centered)
    {
        straight();
        return true;
    }
    return false;
}

// 코너 중 선이 사라졌다. LINE_CORNER_HOLD_MS 동안 같은 쪽으로 더 돈 뒤
// 안정화로 넘어간다
void hold_then_settle(int left, int right, unsigned long now)
{
    if (!white_after_turn)
    {
        white_started_at = now;
        white_after_turn = true;
    }
    if (now - white_started_at < LINE_CORNER_HOLD_MS)
    {
        pivot_toward(last_turn);
        return;
    }
    begin_settling(now);
    settle_on_line(left, right, now);
}

} // namespace

// AFMotor run()은 BRAKE를 처리 안 해서 RELEASE로 끈다. drive()를 거쳐야
// 마지막 명령 캐시가 맞아서 다시 출발할 때 명령이 빠지지 않는다
void drive_stop() { drive(0, RELEASE, 0, RELEASE); }

void line_follow_step()
{
    const unsigned long now = millis();
    const int left = digitalRead(A0);
    const int right = digitalRead(A5);

    if (settling && settle_step(left, right, now))
        return;

    if (left == LOW && right == LOW)
    {
        if (last_turn != 0 && now - turn_started_at >= LINE_CORNER_ARM_MS)
        {
            hold_then_settle(left, right, now);
            return;
        }
        last_turn = 0;
        white_after_turn = false;
        straight();
        return;
    }

    if (left == HIGH && right == HIGH)
    {
        if (corner_committed)
        {
            hold_then_settle(left, right, now);
            return;
        }
        last_turn = 0;
        white_after_turn = false;
        straight();
        return;
    }

    const int8_t direction = one_side_black(left, right);
    if (corner_committed && direction != last_turn)
    {
        // 코너 직후 반대쪽을 밟으면 반대로 세게 돌지 않고 안정화로 고친다
        begin_settling(now);
        settle_on_line(left, right, now);
        return;
    }
    if (direction != last_turn)
    {
        last_turn = direction;
        turn_started_at = now;
        corner_committed = false;
    }
    white_after_turn = false;

    const unsigned long turn_elapsed = now - turn_started_at;
    if (turn_elapsed >= LINE_CORNER_ARM_MS)
        corner_committed = true;
    if (turn_elapsed >= LINE_PIVOT_START_MS && turn_elapsed < LINE_PIVOT_END_MS)
        pivot_toward(direction);
    else
        gentle_turn(direction, turn_elapsed, LINE_GENTLE_PIVOT_MS);
}
