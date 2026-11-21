#include "src/AFMotor/AFMotor.h"

#include <Arduino.h>

extern AF_DCMotor motor_l;
extern AF_DCMotor motor_r;

namespace
{
struct Test
{
    int16_t left_pwm; // 음수 = 후진
    int16_t right_pwm;
    unsigned long ms;
};

// 직진 두 번은 PWM이 달라야 데드밴드와 기울기를 따로 구할 수 있다
const Test TESTS[] = {
    {150, 180, 2000},  // 1: 직진, 간 거리를 잰다
    {200, 240, 2000},  // 2: 직진, 간 거리를 잰다
    {150, -150, 3000}, // 3: 제자리 우회전, 돈 각도를 잰다
    // 4: 1초 직진. 1번(2초)과의 거리 차가 정속 1초 분량이라, 둘을 비교하면
    //    출발 때 가속으로 잃는 시간(모터 지연)이 나온다
    {150, 180, 1000},
    {254, 0, 2000}, // 5: 안쪽 바퀴를 세운 원호 우회전, 돈 각도를 잰다
    {0, 254, 2000}, // 6: 안쪽 바퀴를 세운 원호 좌회전, 돈 각도를 잰다
};

// 할 테스트 번호. 바꾸고 다시 업로드한다
const uint8_t TEST_NUMBER = 4;
static_assert(TEST_NUMBER >= 1 &&
                  TEST_NUMBER <= sizeof(TESTS) / sizeof(TESTS[0]),
              "TEST_NUMBER out of range");

// 전원을 켜고 차를 내려놓을 시간
const unsigned long START_DELAY_MS = 3000;

void set_motor(AF_DCMotor& motor, int16_t pwm)
{
    motor.setSpeed(static_cast<uint8_t>(pwm < 0 ? -pwm : pwm));
    motor.run(pwm < 0 ? BACKWARD : FORWARD);
}

void stop_motors()
{
    motor_l.setSpeed(0);
    motor_r.setSpeed(0);
    motor_l.run(RELEASE);
    motor_r.run(RELEASE);
}

} // namespace

void calibration_setup()
{
    stop_motors();
    delay(START_DELAY_MS);

    const Test& test = TESTS[TEST_NUMBER - 1];
    set_motor(motor_l, test.left_pwm);
    set_motor(motor_r, test.right_pwm);
    delay(test.ms);
    stop_motors();
}

// 한 번 하고 멈춘다. 다시 하려면 리셋
void calibration_loop() {}
