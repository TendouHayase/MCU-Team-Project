#include "drive.h"

#include "src/AFMotor/AFMotor.h"

#include <Arduino.h>

namespace
{

AF_DCMotor motor_l(1); // 모터 실드 M1: 왼쪽 바퀴
AF_DCMotor motor_r(4); // 모터 실드 M4: 오른쪽 바퀴

// 라인 센서: LOW=흰색, HIGH=검정
constexpr uint8_t LEFT_LINE_PIN = A0;
constexpr uint8_t RIGHT_LINE_PIN = A5;

// 직진과 코너 회전의 PWM. PWM은 측정 전압이 아닌 0~255 속도 명령이다.
constexpr uint8_t LINE_STRAIGHT_LEFT_PWM = 150;
constexpr uint8_t LINE_STRAIGHT_RIGHT_PWM = 150;
constexpr uint8_t LINE_CORNER_PWM = 150;

// 한쪽 검정을 보는 즉시 회전을 시작한다. 20~80ms에는 연속 제자리 회전한다.
// 20ms 이상 돌면 코너로 확정하고, 그 뒤 선을 놓치면 40ms 더 돈다.
constexpr unsigned long LINE_PIVOT_START_MS = 20;
constexpr unsigned long LINE_PIVOT_END_MS = 80;
constexpr unsigned long LINE_CORNER_ARM_MS = 20;
constexpr unsigned long LINE_CORNER_HOLD_MS = 40;

// 코너 뒤 안정화: 중앙(양쪽 흰색)이 200ms 이어지거나 1.5초가 지나면 끝낸다.
// 안정화 중에도 한쪽 검정이 50ms 이어지면 새 코너로 인정한다.
constexpr unsigned long LINE_SETTLE_MAX_MS = 1500;
constexpr unsigned long LINE_CENTER_STABLE_MS = 200;
constexpr unsigned long LINE_SETTLE_NEW_CORNER_MS = 50;

// 짧은 제자리 회전과 전진을 번갈아 완만하게 조향한다.
constexpr unsigned long LINE_GENTLE_CYCLE_MS = 30;
constexpr unsigned long LINE_GENTLE_PIVOT_MS = 12;
constexpr unsigned long LINE_SETTLE_SHORT_PIVOT_MS = 8;
constexpr unsigned long LINE_SETTLE_LONG_PIVOT_MS = 12;
constexpr unsigned long LINE_SETTLE_STRONG_AFTER_MS = 35;

enum class TurnDirection : int8_t
{
    Left = -1,
    None = 0,
    Right = 1
};

struct LineReading
{
    bool left_black;
    bool right_black;

    bool both_white() const { return !left_black && !right_black; }
    bool both_black() const { return left_black && right_black; }

    /// 한쪽만 검정이면 그쪽 방향, 아니면 None
    TurnDirection black_side() const
    {
        if (!left_black && right_black)
            return TurnDirection::Right;
        if (left_black && !right_black)
            return TurnDirection::Left;
        return TurnDirection::None;
    }
};

// 코너 회전 상태
struct CornerState
{
    TurnDirection last_turn = TurnDirection::None;
    unsigned long turn_started_at = 0;
    bool committed = false;        // LINE_CORNER_ARM_MS 이상 돌아 코너로 확정
    bool white_after_turn = false; // 확정 뒤 선을 놓친 구간에 들어왔는지
    unsigned long white_started_at = 0;
};

// 코너 뒤 직선으로 돌아오는 안정화 상태
struct SettleState
{
    bool active = false;
    unsigned long started_at = 0;
    bool saw_side = false; // 안정화 중 한쪽 검정을 본 적이 있는지
    bool center_active = false;
    unsigned long center_started_at = 0;
    TurnDirection side_direction = TurnDirection::None;
    unsigned long side_started_at = 0;
};

// 마지막으로 모터 실드에 보낸 명령. 바뀐 값만 다시 보낸다.
// stale 이면(시작 직후, 벽 앞 정지 뒤) 실제 모터 상태와 다를 수 있으므로
// 모든 값을 다시 보낸다.
struct SentCommand
{
    bool stale = true;
    uint8_t left_speed = 0;
    uint8_t right_speed = 0;
    uint8_t left_mode = 0;
    uint8_t right_mode = 0;
};

CornerState corner;
SettleState settle;
SentCommand sent;

LineReading read_line_sensors()
{
    LineReading reading;
    reading.left_black = digitalRead(LEFT_LINE_PIN) == HIGH;
    reading.right_black = digitalRead(RIGHT_LINE_PIN) == HIGH;
    return reading;
}

void drive(uint8_t left_speed, uint8_t left_mode, uint8_t right_speed,
           uint8_t right_mode)
{
    if (sent.stale || left_speed != sent.left_speed)
    {
        motor_l.setSpeed(left_speed);
        sent.left_speed = left_speed;
    }
    if (sent.stale || right_speed != sent.right_speed)
    {
        motor_r.setSpeed(right_speed);
        sent.right_speed = right_speed;
    }
    if (sent.stale || left_mode != sent.left_mode)
    {
        motor_l.run(left_mode);
        sent.left_mode = left_mode;
    }
    if (sent.stale || right_mode != sent.right_mode)
    {
        motor_r.run(right_mode);
        sent.right_mode = right_mode;
    }
    sent.stale = false;
}

void straight()
{
    drive(LINE_STRAIGHT_LEFT_PWM, FORWARD, LINE_STRAIGHT_RIGHT_PWM, FORWARD);
}

void pivot_toward(TurnDirection direction)
{
    // 우회전: 왼쪽 전진·오른쪽 후진. 좌회전: 왼쪽 후진·오른쪽 전진.
    if (direction == TurnDirection::Right)
        drive(LINE_CORNER_PWM, FORWARD, LINE_CORNER_PWM, BACKWARD);
    else if (direction == TurnDirection::Left)
        drive(LINE_CORNER_PWM, BACKWARD, LINE_CORNER_PWM, FORWARD);
}

void gentle_turn(TurnDirection direction, unsigned long elapsed,
                 unsigned long pivot_ms)
{
    // 검정 쪽으로 계속 강하게 돌면 선을 지나칠 수 있다.
    // 한 주기의 앞부분만 짧게 회전하고 나머지는 두 바퀴를 전진시킨다.
    if (elapsed % LINE_GENTLE_CYCLE_MS < pivot_ms)
        pivot_toward(direction);
    else
        drive(LINE_CORNER_PWM, FORWARD, LINE_CORNER_PWM, FORWARD);
}

void forget_turn_and_go_straight()
{
    corner.last_turn = TurnDirection::None;
    corner.white_after_turn = false;
    straight();
}

void begin_settling(unsigned long now)
{
    corner = CornerState();
    settle = SettleState();
    settle.active = true;
    settle.started_at = now;
}

void settle_on_line(const LineReading& reading, unsigned long now)
{
    const TurnDirection direction = reading.black_side();
    if (direction == TurnDirection::None)
    {
        straight();
        return;
    }

    // begin_settling() 직후 한쪽 검정으로 들어올 수도 있으므로
    // 회전 시작 시각을 여기서도 잡아 주기를 일정하게 한다.
    if (direction != settle.side_direction)
    {
        settle.side_direction = direction;
        settle.side_started_at = now;
    }
    const unsigned long side_elapsed = now - settle.side_started_at;
    const unsigned long pulse_ms = side_elapsed >= LINE_SETTLE_STRONG_AFTER_MS
                                       ? LINE_SETTLE_LONG_PIVOT_MS
                                       : LINE_SETTLE_SHORT_PIVOT_MS;
    gentle_turn(direction, side_elapsed, pulse_ms);
}

/// 코너 회전 중 선을 놓치면(양쪽 같은 색) LINE_CORNER_HOLD_MS 동안 같은
/// 방향으로 더 돈 뒤 안정화로 넘어간다.
void hold_corner_then_settle(const LineReading& reading, unsigned long now)
{
    if (!corner.white_after_turn)
    {
        corner.white_started_at = now;
        corner.white_after_turn = true;
    }
    if (now - corner.white_started_at < LINE_CORNER_HOLD_MS)
    {
        pivot_toward(corner.last_turn);
        return;
    }
    begin_settling(now);
    settle_on_line(reading, now);
}

/// 안정화 중 한 단계. 이번 단계를 처리했으면 true, 안정화가 끝나 일반
/// 판정으로 넘겨야 하면 false 를 반환한다.
bool settle_step(const LineReading& reading, unsigned long now)
{
    const TurnDirection side = reading.black_side();
    if (side != TurnDirection::None)
    {
        settle.saw_side = true;
        settle.center_active = false;
        if (side != settle.side_direction)
        {
            settle.side_direction = side;
            settle.side_started_at = now;
        }
    }
    else if (reading.both_white() && settle.saw_side)
    {
        settle.side_direction = TurnDirection::None;
        if (!settle.center_active)
        {
            settle.center_started_at = now;
            settle.center_active = true;
        }
    }
    else
    {
        settle.side_direction = TurnDirection::None;
        settle.center_active = false;
    }

    const bool centered =
        settle.center_active &&
        now - settle.center_started_at >= LINE_CENTER_STABLE_MS;
    const bool timed_out = now - settle.started_at >= LINE_SETTLE_MAX_MS;
    const bool new_corner =
        settle.side_direction != TurnDirection::None &&
        now - settle.side_started_at >= LINE_SETTLE_NEW_CORNER_MS;

    if (!new_corner && !centered && !timed_out)
    {
        settle_on_line(reading, now);
        return true;
    }

    settle.active = false;
    if (new_corner)
    {
        corner.last_turn = settle.side_direction;
        corner.turn_started_at = settle.side_started_at;
        corner.white_after_turn = false;
        corner.committed = false;
    }
    if (centered)
    {
        straight();
        return true;
    }
    return false;
}

} // namespace

void drive_begin()
{
    motor_l.run(RELEASE);
    motor_r.run(RELEASE);
}

void drive_stop()
{
    // AFMotor 의 run()은 BRAKE 를 처리하지 않으므로 RELEASE 로 출력을 끈다.
    motor_l.run(RELEASE);
    motor_r.run(RELEASE);
    sent.stale = true;
    corner = CornerState();
    settle = SettleState();
}

void line_follow_step()
{
    const unsigned long now = millis();
    const LineReading reading = read_line_sensors();

    if (settle.active && settle_step(reading, now))
        return;

    // 양쪽 흰색: 코너 회전 중이었으면 회전을 유지하고, 아니면 직진한다.
    if (reading.both_white())
    {
        if (corner.last_turn != TurnDirection::None &&
            now - corner.turn_started_at >= LINE_CORNER_ARM_MS)
            hold_corner_then_settle(reading, now);
        else
            forget_turn_and_go_straight();
        return;
    }

    // 양쪽 검정: 확정된 코너 회전 중이었으면 유지하고, 아니면 직진한다.
    if (reading.both_black())
    {
        if (corner.committed)
            hold_corner_then_settle(reading, now);
        else
            forget_turn_and_go_straight();
        return;
    }

    // 한쪽 검정: 그쪽으로 회전한다.
    const TurnDirection direction = reading.black_side();
    if (corner.committed && direction != corner.last_turn)
    {
        // 코너 직후 반대쪽 센서가 검정을 보면 바로 반대로 돌지 않는다.
        begin_settling(now);
        settle_on_line(reading, now);
        return;
    }
    if (direction != corner.last_turn)
    {
        corner.last_turn = direction;
        corner.turn_started_at = now;
        corner.committed = false;
    }
    corner.white_after_turn = false;

    const unsigned long turn_elapsed = now - corner.turn_started_at;
    if (turn_elapsed >= LINE_CORNER_ARM_MS)
        corner.committed = true;

    if (turn_elapsed >= LINE_PIVOT_START_MS && turn_elapsed < LINE_PIVOT_END_MS)
        pivot_toward(direction);
    else
        gentle_turn(direction, turn_elapsed, LINE_GENTLE_PIVOT_MS);
}
