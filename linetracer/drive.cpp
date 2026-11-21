#include "trace_log.h"

#include "src/AFMotor/AFMotor.h"
#include "src/math/sin_table.h"

#include <Arduino.h>

extern AF_DCMotor motor_l; // M1: 왼쪽 바퀴
extern AF_DCMotor motor_r; // M4: 오른쪽 바퀴

namespace
{
// ---- 실측 값 ----
const int16_t SENSOR_GAP_MM = 50; // 두 센서 중심 사이
// 센서가 검정으로 판정하는 폭. 테이프는 17mm인데 센서 LED로 재면 5mm쯤 넓다
const int16_t LINE_WIDTH_MM = 19;
const int16_t SENSOR_TO_AXLE_MM = 85; // 센서 막대 중앙 ~ 구동축
// 구동바퀴 접지 중심 사이는 115mm인데 바퀴가 미끄러져 덜 돈다. 직선 주행 기록을
// 재생해 맞춘 값(170, 아래 모터 지연과 함께 맞춤). 제자리 회전 실측(3초 320도)
// 으로는 230쯤이라 제자리 회전은 이보다 더 미끄러진다
const int16_t EFFECTIVE_TRACK_MM = 170;
// 좌우가 반대로 도는 제자리 회전은 더 미끄러진다. 측정 3번
// (150/-150, 3초 320도)을 모터 지연까지 넣어 풀면 228.
// 회전이 제자리에 가까운 정도로 둘 사이를 섞는다
const int16_t PIVOT_TRACK_MM = 228;
// 명령 PWM이 바뀐 뒤 바퀴 속도가 따라오는 1차 지연.
// 같은 기록에서 0.125~0.15초가 가장 잘 맞고 0.1초 아래나
// 0.2초 위는 크게 어긋났다. 0이면 관측기가 실제보다 먼저
// 다 돌았다고 판단해서 선을 가로질러 반대쪽까지 넘어간다
const int32_t MOTOR_TAU_MS = 130;

// 바퀴 속도 모델: v[mm/s] = (|pwm| - DEADBAND) * MM_S_PER_PWM_Q8 / 256
// 150/180으로 2초에 50cm, 200/240으로 2초에 72.5cm 직진한 값에서 구했다
const int16_t DEADBAND_L = 39;
const int16_t DEADBAND_R = 47;
const int16_t MM_S_PER_PWM_Q8_L = 576; // 2.25 mm/s per PWM
const int16_t MM_S_PER_PWM_Q8_R = 480; // 1.875 mm/s per PWM

// ---- 튜닝 값 ----
// 직진에서 오른쪽으로 휘어서 오른쪽을 더 세게 준다. 라인 없이 직진시켜 맞춘 값
const int16_t STRAIGHT_LEFT_PWM = 150;
const int16_t STRAIGHT_RIGHT_PWM = 180;
const int16_t PWM_MAX = 255;
// 많이 꺾어야 할수록 기본 속도를 낮춘다. 한쪽 바퀴를 세워도 이 차가 돌 수 있는
// 반경엔 한계가 있어서(시뮬레이션에서 반경 120mm 아래부터 놓침) 급한 곡선은
// 느리게 들어가야 한다. 조향량(PWM) 1당 기본 PWM을 이만큼(/256) 낮춘다.
// 시뮬레이션에서 64(0.25)로 반경 100mm까지 통과, 128부터는 느리기만 하다
const int32_t SLOW_PER_STEER_Q8 = 64;
// 낮춘 속도는 바로 회복하지 않고 2^8 스텝(~0.25 s)에 걸쳐 돌아온다
const uint8_t SLOW_RELEASE_SHIFT = 8;
const int16_t BASE_MIN_PWM = 70;
// 반대쪽 센서가 선을 보기 전까지 마지막으로 닿은 쪽의 반대로는 이만큼까지만
// 조향한다. 급한 곡선에서 관측기는 "돈 만큼 선이 반대로 흘러갔다"고 잘못
// 예측해서, 실차 기록에서 오른쪽 센서만 계속 닿는데도 왼쪽으로 -190/254까지
// 꺾기를 반복했다. 0으로 막으면 직선에서 좌우 센서를 오가며 지그재그가 된다.
// 시뮬레이션에서 100까지는 직선·곡선이 그대로고 60 아래부터 곡선이 나빠졌다
const int32_t OPPOSITE_STEER_MAX = 100;
// 일반 주행에서 바퀴를 후진시키지 않는다. 회전 모델은 한쪽 바퀴가 서는 원호
// 회전 기록으로 맞춘 것이고, 안쪽 바퀴를 후진시키는 제자리 회전형 명령은 훨씬
// 더 미끄러져서 모델이 크게 틀린다(급한 곡선에서 좌우로 꺾기를 반복한 원인)
const bool NO_REVERSE = false;

// 회전 반응이 느리고 약해서 크게 준다. 측정 오차를 섞은 시뮬레이션에서 4~8은
// 자주 놓치고, 급한 곡선(반경 100~200mm)은 속도 조절과 함께 16이 가장 안정
const int32_t KP_Q8 = 16 * 256; // PWM per mm
const int32_t KD_Q8 = 2 * 256;  // PWM per degree
// 지금 도는 빠르기(지연을 거친 좌우 바퀴 속도 차)로 감쇠를 건다. psi는 적분이라
// 곡선에서 20~30도씩 틀리지만 이건 지금 명령만으로 정해져 오차가 쌓이지 않는다.
// 시뮬레이션에서는 넣어도 나아지지 않아서 0
const int32_t KW_Q8 = 0; // PWM per (mm/s) * 256

// y를 센서 범위로 잘라낸 양(mm)만큼 psi를 반대로 고친다 (centi-degree per mm)
// 크게 주면 모델 오차가 클 때 직선에서 진동한다 (시뮬레이션에서 50부터 깨짐)
const int32_t PSI_PER_MM_CDEG = 25;
// 센서 하나의 값이 바뀐 순간에는 선 경계가 그 센서 위에 있어서 y가
// 경계값 하나로 정해진다. 지난 경계부터 쌓인 y 예측 오차는 대부분
// psi 오차 * 속도 * 시간이라 그 시간으로 나눠 psi를 고친다.
// 위의 고정 비율은 짧은 접촉(0.1초)에서는 10배쯤 약하고 긴 흰색 구간
// 뒤에는 너무 세서, 곡선에서 psi가 20~30도씩 틀렸다.
// 오차를 psi 탓으로 보는 비율(/256). 오차가 회전 모델에서 생겼다면 psi
// 오차는 구간 동안 0부터 커진 것이라 1보다 크게 준다. 시뮬레이션에서
// 1.5가 곡선 끝의 비틀거림을 1/3 줄였고 3부터는 오히려 깨졌다
const int32_t EDGE_PSI_GAIN_Q8 = 384;
// 이보다 짧은 간격은 경계에서 떨린 것으로 보고 psi를 안 고친다
const unsigned long EDGE_MIN_US = 20000;
const int32_t EDGE_MIN_MM_S = 50;
const int32_t EDGE_PSI_MAX_CDEG = 2000;
// 같은 양만큼 선의 곡률도 고친다 (1/km per mm). 곡선에서는 같은 쪽으로 계속
// 잘라내게 되므로 이게 쌓여 곡률이 된다. 그런데 psi 추정이 곡선에서 20~30도씩
// 틀려서, 잘라낸 양은 대부분 psi 오차인데 곡률로 배운다. 시뮬레이션에서 켜면
// 선을 가로지르는 횟수가 3배가 돼서 끈다
const int32_t KAPPA_PER_MM = 0;
// 한 스텝에 psi, 곡률을 고치는 데 쓰는 잘라낸 양의 상한. 추정이 크게 틀어진
// 순간(회전 직후, 재출발)의 한 번 큰 차이가 방향 추정을 망치지 않게 한다
const int32_t INNOVATION_CAP_UM = 1000;
// 선은 평균적으로 곧다고 보고 psi를 0으로 천천히 당긴다. 2^10 스텝 ~ 1 s
const uint8_t PSI_LEAK_SHIFT = 10;
const int16_t PSI_MAX_CDEG = 6000;
// 곡선이 끝나면 곡률 추정이 0으로 돌아오게 한다. 2^11 스텝 ~ 2 s
const uint8_t KAPPA_LEAK_SHIFT = 11;
const int32_t KAPPA_MAX_PER_KM = 8000; // 반경 125mm
// 좌우 바퀴의 미세한 차이(직진 보정값 오차)로 생기는 일정한 쏠림을 바퀴 속도 차
// 오프셋으로 배운다. 잘라낸 양 1mm당 (mm/s)/256. 오른쪽 바퀴 +-10% 오차를 넣은
// 시뮬레이션에서 직선 접촉이 1/3로 줄었다
const int32_t TRIM_PER_MM_Q8 = 256;
const int32_t TRIM_MAX_MM_S = 60;
// 곡선에서 잘라낸 양도 같은 쪽으로 쌓여서 그대로 배우면 곡선이 끝난 뒤 반대로
// 쏠린다(시뮬레이션에서 곡선에서 선을 가로지르는 횟수가 3배). 조향량을 0.5초쯤
// 평균한 값이 이보다 클 때(곡선)는 배우지 않는다. 직선에서 스치는 접촉은 짧아서
// 평균이 작다. 30~60은 직선에서도 거의 안 배우고, 100이 곡선을 안 깨는 선에서
// 직선 접촉을 가장 많이 줄였다
const int32_t TRIM_GATE_PWM = 100;
const uint8_t STEER_AVG_SHIFT = 9; // 2^9 스텝 ~ 0.5 s

// 접촉이 길면 직각으로 보던 판정. 실차에서는 평범한 접촉도 90~230ms였고
// 곡선에서는 400ms도 넘어서 접촉 시간으로는 직각을 가를 수 없다.
// 직각 기록을 받아 판정 방법을 새로 정하기 전까지 끈다
const bool CORNER_DETECT = false;
const unsigned long CORNER_MIN_US = 400000;
// 직각을 본 뒤 구동축이 코너 쪽으로 가도록 더 가는 시간 (센서-축 거리 / 속도
// = 340ms 근처). 짧으면 회전 후 선이 옆에 남고, 길면 코너를 지나쳐 90도를
// 넘게 돌아야 선이 보인다. 시뮬레이션에서 300ms가 가장 견고했다
const unsigned long CORNER_ADVANCE_US = 300000;
// 회전 시작 직후 아직 옛 선을 보고 있을 수 있어서 이 시간은 검정을 무시한다
const unsigned long PIVOT_MIN_US = 50000;
const int16_t TURN_PWM = 150;

// 직각에서는 꺾인 선이 진행 방향과 직각이라 센서가 선 폭(22mm)을 90~120ms면
// 지나가 버리고, 모터 지연(130ms) 때문에 그 안에 거의 못 돈다(시뮬레이션에서
// 6도). 2센서로는 "선이 가운데"와 "선을 놓침"이 같은 신호라, 마지막 접촉 뒤
// 이 거리를 가도록 아무 센서도 검정을 못 보면 놓친 것으로 보고 마지막으로
// 닿은 쪽으로 제자리 회전하며 찾는다. 회전 중심(구동축)이 꼭짓점 근처여야
// 센서가 꺾인 선을 쓸고 지나간다(센서 쪽 85mm 앞). 시뮬레이션에서 80~120이
// 직각을 거의 다 찾았고, 짧을수록 직선·곡선에서 잘못 걸려 느려진다.
// 원호로 돌며 찾기, 찾은 뒤 회전 중 적분한 psi를 이어 쓰기는 둘 다 더 나빴다.
// 0이면 끈다
const int32_t LOST_SEARCH_MM = 110;
// 선을 가운데가 아니라 마지막으로 닿은 센서 쪽으로 이만큼(mm) 치우치게
// 따라간다. 그쪽 센서에 자주 닿아서 직선에서 흰색이 길게 이어지지 않으니
// 위의 찾기가 덜 잘못 걸린다(시뮬레이션 직선에서 43번 -> 18번, 주행 시간
// +40% -> +15%). 16부터는 곡선을 놓친다.
// 목표는 2^HUG_SHIFT 스텝(~0.5 s)에 걸쳐 옮겨서
// 쪽이 바뀌어도 조향이 튀지 않는다(바로 옮기면 12mm * KP = 192 PWM이 한 번에)
const int32_t HUG_MM = 12;
const uint8_t HUG_SHIFT = 9;

const unsigned long LOST_STOP_US = 2000000;

// ---- 타이밍 ----
// 선 경계에서 센서 출력이 떨리는 걸 거른다
const unsigned long DEBOUNCE_US = 2000;
const unsigned long DT_US = 1000; // 1 kHz
// 이보다 밀리면(벽 정지 등) 밀린 스텝을 따라잡지 않고 기준 시각을 다시 잡는다
const unsigned long RESYNC_US = 10 * DT_US;

static_assert(DT_US % 1000 == 0, "DT_US must be a whole number of ms");
const int32_t DT_MS = DT_US / 1000;
// 시간은 스텝 수로 센다. 벽 정지 중에는 스텝이 안 돌아서 같이 멈춘다
const uint16_t CORNER_MIN_STEPS = CORNER_MIN_US / DT_US;
const uint16_t CORNER_ADVANCE_STEPS = CORNER_ADVANCE_US / DT_US;
const uint16_t PIVOT_MIN_STEPS = PIVOT_MIN_US / DT_US;
const uint16_t LOST_STOP_STEPS = LOST_STOP_US / DT_US;
const uint16_t EDGE_MIN_STEPS = EDGE_MIN_US / DT_US;
const uint8_t TRACE_EVERY_STEPS = 25000 / DT_US;

// ---- 고정소수점 ----
// y는 um * 256, psi는 centi-degree * 256 로 들고 있는다
constexpr int32_t mm_to_q8(int32_t mm) { return mm * 1000 * 256; }

// 선 중심 y가 이 범위에 있으면 그 센서가 검정을 본다
const int32_t Y_INNER = mm_to_q8(SENSOR_GAP_MM - LINE_WIDTH_MM) / 2;
const int32_t Y_SENSOR = mm_to_q8(SENSOR_GAP_MM) / 2;
const int32_t Y_OUTER = mm_to_q8(SENSOR_GAP_MM + LINE_WIDTH_MM) / 2;
const int32_t Y_LIMIT = mm_to_q8(100);
const int32_t PSI_MAX_Q8 = static_cast<int32_t>(PSI_MAX_CDEG) * 256;

// dv/T [rad/s] -> centi-degree * 256 per ms
const int32_t PSI_RATE_K =
    static_cast<int32_t>(180.0 / 3.14159265 * 100 * 256 / 1000 + 0.5);

// 값은 bit0 = 왼쪽 검정, bit1 = 오른쪽 검정. read_line()이 이 배치로 캐스트한다
enum class LineState : uint8_t
{
    BOTH_WHITE = 0,
    LEFT_BLACK = 1,
    RIGHT_BLACK = 2,
    BOTH_BLACK = 3
};

enum class Mode : uint8_t
{
    TRACK,   // 관측기 + PD
    ADVANCE, // 직각을 봤다. 구동축이 코너에 올 때까지 직진
    PIVOT    // 직각 쪽으로 제자리 회전하며 새 선을 찾는다
};

// 센서 막대 중심에서 본 선 중심 위치. + = 선이 오른쪽
int32_t y_q8 = 0;
// 선 방향에 대한 차체 방향. + = 차가 오른쪽(시계방향)으로 틀어짐
int32_t psi_q8 = 0;
// 지난 스텝에 낸 PWM. 음수 = 후진. 예측은 이 값으로 한다
int16_t pwm_l = 0;
int16_t pwm_r = 0;
// 모터 지연을 거친 바퀴 속도 추정 (mm/s * 256)
int32_t vl_q8 = 0;
int32_t vr_q8 = 0;
int32_t v_mm_s = 0;
int32_t dv_mm_s = 0; // + = 시계방향으로 도는 중
int32_t slow_q8 = 0; // 기본 PWM을 낮추는 양 * 256
// 모델보다 실제로 더 시계방향으로 도는 양 (mm/s * 256)
int32_t trim_q8 = 0;
int32_t steer_avg_q8 = 0; // |조향량| 평균 * 256
int8_t last_side = 0;     // 마지막으로 검정을 본 센서. -1 왼쪽, +1 오른쪽
int32_t white_um = 0;     // 마지막으로 검정을 본 뒤 간 거리
// 흰색에서 검정으로 새로 닿으면 켜지고, 찾기를 한 번 하면 꺼진다.
// 찾기 끝에 선을 다시 밟은 건 새 접촉으로 치지 않는다. 안 그러면
// 직선에서 찾기가 되풀이된다
bool search_armed = false;
bool prev_black = false;
// 선의 곡률 (1/km). + = 선이 오른쪽으로 휜다
int32_t kappa_per_km = 0;
int32_t y_ref_q8 = 0; // 따라갈 선 위치
LineState edge_line = LineState::BOTH_WHITE;
uint16_t since_edge_steps = 0;

Mode mode = Mode::TRACK;
uint16_t mode_steps = 0;
int8_t corner_dir = 0; // -1 왼쪽, +1 오른쪽
// 흰색에서 시작한 한쪽 검정이 몇 스텝째인지. 0 = 접촉 없음
int8_t touch_side = 0;
uint16_t touch_steps = 0;
uint16_t lost_steps = 0;

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

int16_t clamp_pwm(int32_t pwm)
{
    if (pwm > PWM_MAX)
        return PWM_MAX;
    if (pwm < -PWM_MAX)
        return -PWM_MAX;
    return static_cast<int16_t>(pwm);
}

void set_wheels(int32_t left, int32_t right)
{
    pwm_l = clamp_pwm(left);
    pwm_r = clamp_pwm(right);
    drive(static_cast<uint8_t>(pwm_l < 0 ? -pwm_l : pwm_l),
          pwm_l < 0 ? BACKWARD : FORWARD,
          static_cast<uint8_t>(pwm_r < 0 ? -pwm_r : pwm_r),
          pwm_r < 0 ? BACKWARD : FORWARD);
}

int32_t wheel_speed(int16_t pwm, int16_t deadband, int16_t mm_s_per_pwm_q8)
{
    const int32_t mag = (pwm < 0 ? -pwm : pwm) - deadband;
    if (mag <= 0)
        return 0;
    const int32_t v = (mag * mm_s_per_pwm_q8) >> 8;
    return pwm < 0 ? -v : v;
}

// centi-degree -> Q15. 1도 테이블 사이를 선형 보간한다
int32_t sin_cdeg(int16_t cdeg)
{
    const bool negative = cdeg < 0;
    const uint16_t a = negative ? -cdeg : cdeg;
    const uint8_t deg = a / 100;
    const uint8_t frac = a % 100;
    const int32_t s0 = trig::sin_q15(deg);
    const int32_t s1 = trig::sin_q15(deg + 1);
    const int32_t s = s0 + (s1 - s0) * frac / 100;
    return negative ? -s : s;
}

int32_t clamp(int32_t x, int32_t lo, int32_t hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

// 지난 스텝의 PWM으로 dt 동안 y, psi가 얼마나 변했는지 적분한다
void predict()
{
    const int32_t target_l = wheel_speed(pwm_l, DEADBAND_L, MM_S_PER_PWM_Q8_L);
    const int32_t target_r = wheel_speed(pwm_r, DEADBAND_R, MM_S_PER_PWM_Q8_R);
    vl_q8 += (target_l * 256 - vl_q8) * DT_MS / MOTOR_TAU_MS;
    vr_q8 += (target_r * 256 - vr_q8) * DT_MS / MOTOR_TAU_MS;
    const int32_t vl = vl_q8 >> 8;
    const int32_t vr = vr_q8 >> 8;
    const int32_t v = (vl + vr) / 2;
    const int32_t dv = vl - vr + (trim_q8 >> 8); // + = 시계방향 회전
    v_mm_s = v;
    dv_mm_s = dv;

    // 한쪽 바퀴가 서면(|v| = |dv|/2) 0, 제자리 회전(v = 0)이면 256
    const int32_t abs_v = v < 0 ? -v : v;
    const int32_t abs_dv = dv < 0 ? -dv : dv;
    const int32_t pivot_q8 =
        abs_dv > 0 ? clamp(256 - 512 * abs_v / abs_dv, 0, 256) : 0;
    const int32_t track =
        EFFECTIVE_TRACK_MM +
        (PIVOT_TRACK_MM - EFFECTIVE_TRACK_MM) * pivot_q8 / 256;

    // 선이 센서에 대해 옆으로 움직이는 속도 = 진행에 의한 것 + 센서가
    // 축보다 앞에 있어서 회전할 때 옆으로 쓸리는 것
    // (int32 >> 는 avr-gcc에서 산술 시프트라 음수도 그대로 된다)
    const int16_t psi_cdeg = static_cast<int16_t>(psi_q8 >> 8);
    const int32_t by_heading = (v * sin_cdeg(psi_cdeg)) >> 7;
    const int32_t by_rotation = SENSOR_TO_AXLE_MM * dv * 256 / track;
    y_q8 = clamp(y_q8 - (by_heading + by_rotation) * DT_MS, -Y_LIMIT, Y_LIMIT);

    // 선이 휘는 만큼은 차가 돌아도 선에 대한 방향 오차가 생기지 않는다
    psi_q8 += dv * PSI_RATE_K * DT_MS / track;
    psi_q8 -= (v * kappa_per_km / 1000) * PSI_RATE_K / 1000 * DT_MS;
    psi_q8 -= psi_q8 >> PSI_LEAK_SHIFT;
    psi_q8 = clamp(psi_q8, -PSI_MAX_Q8, PSI_MAX_Q8);

    kappa_per_km -= kappa_per_km >> KAPPA_LEAK_SHIFT;
}

// 센서가 보는 값과 맞도록 y를 가능한 범위로 자르고, learn이면 자른 만큼
// psi와 곡률도 고친다
void correct(LineState line, bool learn)
{
    const uint8_t changed =
        static_cast<uint8_t>(line) ^ static_cast<uint8_t>(edge_line);
    // 양쪽 검정은 곧은 선으로는 안 나오는 값이라 거기 드나드는 경계는 안 쓴다
    const bool single_edge = (changed == 1 || changed == 2) &&
                             line != LineState::BOTH_BLACK &&
                             edge_line != LineState::BOTH_BLACK;
    edge_line = line;
    if (since_edge_steps < UINT16_MAX)
        ++since_edge_steps;
    if (single_edge)
    {
        // 경계 둘(안쪽, 바깥쪽) 중 예측에 가까운 쪽을 지났다고 본다
        const int32_t side = changed == 2 ? 1 : -1;
        const int32_t fix =
            side * (y_q8 * side >= Y_SENSOR ? Y_OUTER : Y_INNER);
        if (learn && since_edge_steps >= EDGE_MIN_STEPS &&
            v_mm_s >= EDGE_MIN_MM_S)
        {
            // um*256 / (mm/s * ms) = rad*256. 57.3*100 = cdeg
            const int32_t miss_cap = mm_to_q8(40); // * 57 이 int32를 넘지 않게
            const int32_t miss = clamp(fix - y_q8, -miss_cap, miss_cap);
            const int32_t elapsed_ms = since_edge_steps * DT_MS;
            const int32_t psi_err =
                clamp(miss * 57 / (v_mm_s * elapsed_ms) * 100,
                      -EDGE_PSI_MAX_CDEG * 256, EDGE_PSI_MAX_CDEG * 256);
            psi_q8 -= psi_err * EDGE_PSI_GAIN_Q8 / 256;
        }
        y_q8 = fix;
    }
    if (changed != 0)
        since_edge_steps = 0;

    int32_t lo = 0;
    int32_t hi = 0;
    switch (line)
    {
    case LineState::LEFT_BLACK:
        lo = -Y_OUTER;
        hi = -Y_INNER;
        break;
    case LineState::RIGHT_BLACK:
        lo = Y_INNER;
        hi = Y_OUTER;
        break;
    case LineState::BOTH_WHITE:
        // 선이 센서를 어느 쪽으로 빠져나갔는지는 추정치가 센서의 안쪽이냐
        // 바깥쪽이냐로 정한다
        if (y_q8 >= Y_SENSOR)
        {
            lo = Y_OUTER;
            hi = Y_LIMIT;
        }
        else if (y_q8 <= -Y_SENSOR)
        {
            lo = -Y_LIMIT;
            hi = -Y_OUTER;
        }
        else
        {
            lo = -Y_INNER;
            hi = Y_INNER;
        }
        break;
    case LineState::BOTH_BLACK:
        // 곧은 선으로는 나올 수 없는 값(기운 채 직각 진입 등). 추정을 믿는다
        return;
    }

    const int32_t clamped = clamp(y_q8, lo, hi);
    if (learn)
    {
        // 선이 예측보다 오른쪽에 있었다 = 왼쪽으로 덜 흘렀다 = psi가 실제보다
        // 컸거나 선이 오른쪽으로 휘고 있다
        const int32_t cap = INNOVATION_CAP_UM * 256;
        const int32_t innovation = clamp(clamped - y_q8, -cap, cap);
        psi_q8 -= innovation * PSI_PER_MM_CDEG / 1000;
        kappa_per_km += (innovation >> 8) * KAPPA_PER_MM / 1000;
        kappa_per_km = clamp(kappa_per_km, -KAPPA_MAX_PER_KM, KAPPA_MAX_PER_KM);
        // 선이 예측보다 오른쪽 = 실제로는 덜 시계방향으로 돌았다
        if (steer_avg_q8 < TRIM_GATE_PWM * 256)
            trim_q8 -= (innovation >> 8) * TRIM_PER_MM_Q8 / 1000;
        trim_q8 = clamp(trim_q8, -TRIM_MAX_MM_S * 256, TRIM_MAX_MM_S * 256);
    }
    y_q8 = clamped;
}

// 선이 오른쪽(y > 0)이면 오른쪽으로 돈다. 이미 오른쪽으로 틀어져
// 있으면(psi > 0) 곧 선이 돌아오므로 덜 돈다. 선이 휘어 있으면 그만큼
// 처음부터 돈다
void control()
{
    y_ref_q8 += (last_side * mm_to_q8(HUG_MM) - y_ref_q8) >> HUG_SHIFT;
    const int32_t y_um = (y_q8 - y_ref_q8) >> 8;
    const int32_t psi_cdeg = psi_q8 >> 8;
    // 곡률만큼 돌려면 바퀴 속도 차 dv = v * kappa * T. PWM 1당 좌우 속도 차는
    // 두 바퀴 기울기의 합이다
    const int32_t dv_ff =
        (v_mm_s * kappa_per_km / 1000) * EFFECTIVE_TRACK_MM / 1000;
    // 쏠림만큼은 반대로 미리 준다
    const int32_t u_ff =
        (dv_ff * 256 - trim_q8) / (MM_S_PER_PWM_Q8_L + MM_S_PER_PWM_Q8_R);
    int32_t u =
        ((KP_Q8 * y_um / 1000 - KD_Q8 * psi_cdeg / 100 - KW_Q8 * dv_mm_s) >>
         8) +
        u_ff;
    if (u * last_side < -OPPOSITE_STEER_MAX)
        u = -last_side * OPPOSITE_STEER_MAX;

    const int32_t steer = u < 0 ? -u : u;
    steer_avg_q8 += (steer * 256 - steer_avg_q8) >> STEER_AVG_SHIFT;

    // 낮출 때는 바로, 회복은 천천히
    const int32_t want = steer * SLOW_PER_STEER_Q8;
    slow_q8 -= slow_q8 >> SLOW_RELEASE_SHIFT;
    if (want > slow_q8)
        slow_q8 = want;
    // 두 바퀴 속도가 같은 만큼 줄도록 오른쪽은 기울기 비(576/480)만큼 더 낮춘다
    const int32_t slow_l = slow_q8 >> 8;
    const int32_t slow_r = slow_l * MM_S_PER_PWM_Q8_L / MM_S_PER_PWM_Q8_R;
    int32_t base_l = STRAIGHT_LEFT_PWM - slow_l;
    int32_t base_r = STRAIGHT_RIGHT_PWM - slow_r;
    if (base_l < BASE_MIN_PWM)
        base_l = BASE_MIN_PWM;
    if (base_r < BASE_MIN_PWM)
        base_r = BASE_MIN_PWM;
    int32_t left = base_l + u;
    int32_t right = base_r - u;
    if (NO_REVERSE)
    {
        if (left < 0)
            left = 0;
        if (right < 0)
            right = 0;
    }
    set_wheels(left, right);
}

void enter(Mode next)
{
    mode = next;
    mode_steps = 0;
}

// 흰색에서 시작한 한쪽 접촉의 길이를 잰다. 직각을 밟고 지나가는 동안
// 양쪽 검정이 잠깐 섞여도 같은 접촉으로 본다
void track_touch(LineState line)
{
    switch (line)
    {
    case LineState::BOTH_WHITE:
        touch_side = 0;
        touch_steps = 0;
        return;
    case LineState::LEFT_BLACK:
    case LineState::RIGHT_BLACK: {
        const int8_t side = line == LineState::LEFT_BLACK ? -1 : 1;
        if (touch_side == 0 && touch_steps == 0)
            touch_side = side;
        else if (side != touch_side)
            touch_side = 0; // 반대쪽으로 넘어갔다. 흰색을 볼 때까지 안 센다
        break;
    }
    case LineState::BOTH_BLACK:
        break;
    }
    if (touch_side != 0 && touch_steps < UINT16_MAX)
        ++touch_steps;
}

void track(LineState line)
{
    correct(line, true);
    if (line == LineState::LEFT_BLACK)
        last_side = -1;
    else if (line == LineState::RIGHT_BLACK)
        last_side = 1;
    track_touch(line);
    if (CORNER_DETECT && touch_side != 0 && touch_steps >= CORNER_MIN_STEPS)
    {
        corner_dir = touch_side;
        enter(Mode::ADVANCE);
        return;
    }

    const bool black = line != LineState::BOTH_WHITE;
    if (black && !prev_black)
        search_armed = true;
    prev_black = black;
    if (black)
        white_um = 0;
    else if (white_um < INT32_MAX / 2)
        white_um += (v_mm_s < 0 ? -v_mm_s : v_mm_s) * DT_MS; // mm/s * ms = um
    if (LOST_SEARCH_MM > 0 && search_armed && last_side != 0 &&
        white_um >= LOST_SEARCH_MM * 1000)
    {
        corner_dir = last_side;
        search_armed = false;
        enter(Mode::PIVOT);
        return;
    }

    const bool outside = y_q8 >= Y_OUTER || y_q8 <= -Y_OUTER;
    if (line == LineState::BOTH_WHITE && outside)
    {
        if (lost_steps < LOST_STOP_STEPS)
            ++lost_steps;
    }
    else
    {
        lost_steps = 0;
    }

    if (lost_steps >= LOST_STOP_STEPS)
    {
        set_wheels(0, 0);
        if (lost_steps == LOST_STOP_STEPS)
        {
            trace_flush();
            ++lost_steps; // 한 번만 저장한다
        }
    }
    else
    {
        control();
    }
}

void pivot(LineState line)
{
    if (mode_steps >= PIVOT_MIN_STEPS && line != LineState::BOTH_WHITE)
    {
        // 새 선을 밟았다. 방향 오차는 모르니 0에서 다시 시작하고, 위치는
        // 지금 센서 값으로 잡는다. 회전 중 y 추정은 의미가 없어서 잘라낸 양으로
        // psi를 고치면 안 된다(실차에서 이것 때문에 psi가 -28도로 시작했다).
        // 이 접촉은 직각 판정에 넣지 않는다
        psi_q8 = 0;
        kappa_per_km = 0;
        correct(line, false);
        touch_side = 0;
        touch_steps = 1;
        white_um = 0;
        prev_black = true;
        enter(Mode::TRACK);
        control();
        return;
    }
    if (mode_steps >= LOST_STOP_STEPS)
    {
        set_wheels(0, 0);
        if (mode_steps == LOST_STOP_STEPS)
            trace_flush();
        return;
    }
    set_wheels(corner_dir * TURN_PWM, -corner_dir * TURN_PWM);
}

// A0 왼쪽, A5 오른쪽. HIGH = 검정
LineState read_line()
{
    static LineState stable = LineState::BOTH_WHITE;
    static LineState candidate = LineState::BOTH_WHITE;
    static unsigned long candidate_at = 0;

    const unsigned long now = micros();
    const LineState raw = static_cast<LineState>(
        (digitalRead(A0) == HIGH ? 1 : 0) | (digitalRead(A5) == HIGH ? 2 : 0));
    if (raw != candidate)
    {
        candidate = raw;
        candidate_at = now;
    }
    else if (now - candidate_at >= DEBOUNCE_US)
    {
        stable = candidate;
    }
    return stable;
}

} // namespace

// AFMotor run()은 BRAKE를 처리 안 해서 RELEASE로 끈다
void drive_stop() { drive(0, RELEASE, 0, RELEASE); }

void line_follow_step()
{
    static unsigned long next_at = 0;
    const unsigned long now = micros();
    if (static_cast<long>(now - next_at) < 0)
        return;
    if (now - next_at >= RESYNC_US)
    {
        // 벽 정지 등으로 멈춰 있다 다시 불렸다. 바퀴는 서 있다
        next_at = now;
        vl_q8 = 0;
        vr_q8 = 0;
    }
    next_at += DT_US;

    const LineState line = read_line();
    predict();
    if (mode_steps < UINT16_MAX)
        ++mode_steps;

    switch (mode)
    {
    case Mode::TRACK:
        track(line);
        break;
    case Mode::ADVANCE:
        // 매 스텝 다시 내야 벽 정지(drive_stop) 뒤에도 다시 움직인다
        set_wheels(STRAIGHT_LEFT_PWM, STRAIGHT_RIGHT_PWM);
        if (mode_steps >= CORNER_ADVANCE_STEPS)
            enter(Mode::PIVOT);
        break;
    case Mode::PIVOT:
        pivot(line);
        break;
    }

    // 센서나 모드가 바뀔 때마다, 그 밖에는 TRACE_EVERY_STEPS마다 기록한다
    static LineState traced_line = LineState::BOTH_WHITE;
    static Mode traced_mode = Mode::TRACK;
    static uint8_t since_trace = 0;
    if (line != traced_line || mode != traced_mode ||
        ++since_trace >= TRACE_EVERY_STEPS)
    {
        trace_sample(static_cast<uint8_t>(line), static_cast<uint8_t>(mode),
                     y_q8 >> 8, psi_q8 >> 8, pwm_l, pwm_r);
        traced_line = line;
        traced_mode = mode;
        since_trace = 0;
    }
}
