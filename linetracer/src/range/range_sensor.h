#pragma once
#include <Arduino.h>

namespace timer1
{

constexpr uint8_t WGM_A_MASK = _BV(WGM11) | _BV(WGM10);
constexpr uint8_t WGM_B_MASK = _BV(WGM13) | _BV(WGM12);
constexpr uint8_t WGM_A_FAST_PWM_ICR1 = _BV(WGM11);
constexpr uint8_t WGM_B_FAST_PWM_ICR1 = _BV(WGM13) | _BV(WGM12);

constexpr uint8_t COM1A_MASK = _BV(COM1A1) | _BV(COM1A0);
constexpr uint8_t COM1B_MASK = _BV(COM1B1) | _BV(COM1B0);
constexpr uint8_t COM1A_NON_INVERTING = _BV(COM1A1);

constexpr uint8_t CS_MASK = _BV(CS12) | _BV(CS11) | _BV(CS10);
constexpr uint8_t CS_DIV64 = _BV(CS11) | _BV(CS10);

constexpr uint32_t PRESCALER = 64;
constexpr uint32_t TICK_US = PRESCALER * 1000000UL / F_CPU; // 16 MHz -> 4 us,
constexpr uint32_t PERIOD_US = 60000;                       // 60 ms
constexpr uint32_t PERIOD_TICKS = PERIOD_US / TICK_US;

// 트리거 펄스: 센서 요구 최소 10 us 이상. HIGH 구간 = (OCR1A + 1) 틱
constexpr uint32_t TRIGGER_MIN_US = 10;
constexpr uint32_t TRIGGER_TICKS =
    (TRIGGER_MIN_US + TICK_US - 1) / TICK_US; // 올림: 3 틱 = 12 us

static_assert(
    (PRESCALER * 1000000UL) % F_CPU == 0,
    "Timer1 tick is not an integer number of microseconds for this F_CPU");
static_assert(PERIOD_US % TICK_US == 0,
              "PERIOD_US is not a multiple of the Timer1 tick");
static_assert(PERIOD_TICKS >= 1 && PERIOD_TICKS <= 65536UL,
              "PERIOD_TICKS out of 16-bit Timer1 range");
static_assert(TRIGGER_TICKS >= 1 && TRIGGER_TICKS < PERIOD_TICKS,
              "trigger pulse must be shorter than the period");
static_assert(TRIGGER_TICKS * TICK_US >= TRIGGER_MIN_US,
              "trigger pulse shorter than the sensor minimum");

constexpr uint16_t ICR1_TOP = static_cast<uint16_t>(PERIOD_TICKS - 1);
constexpr uint16_t OCR1A_TRIGGER = static_cast<uint16_t>(TRIGGER_TICKS - 1);
} // namespace timer1

constexpr uint64_t pow2_fraction_abs_diff(uint64_t a, uint64_t b)
{
    return a > b ? a - b : b - a;
}

struct Pow2Fraction
{
    uint16_t num;     // 분자
    uint8_t log2_den; // 분모의 지수

    constexpr uint16_t apply(uint16_t x) const
    {
        return static_cast<uint16_t>(static_cast<uint16_t>(x * num) >>
                                     log2_den);
    }

    constexpr uint16_t max_input() const
    {
        return static_cast<uint16_t>(UINT16_MAX / num);
    }
};

constexpr Pow2Fraction make_pow2_fraction(uint32_t p, uint32_t q,
                                          uint8_t log2_den)
{
    return Pow2Fraction{
        static_cast<uint16_t>(((p << log2_den) * 2u + q) / (2u * q)), log2_den};
}

struct RangeStatus
{
    enum
    {
        NOT_CHANGED,
        CHANGED,
        UNKNOWN
    } status;
    uint16_t distance;
};

class RangeSensor
{
  private:
    // 에코 1틱당 물체까지의 거리 = 343 m/s * 4 us / 2 = 1372/2000 mm
    static constexpr uint32_t MM_PER_TICK_P = 1372;
    static constexpr uint32_t MM_PER_TICK_Q = 2000;
    static constexpr Pow2Fraction TICK_TO_MM =
        make_pow2_fraction(MM_PER_TICK_P, MM_PER_TICK_Q, 4);

    // 유효 펄스 폭 상한: 센서 측정 한계와 16비트 변환 한계 중 작은 쪽
    static constexpr uint32_t SENSOR_MAX_MM = 4000;
    static constexpr uint32_t SENSOR_MAX_TICKS =
        SENSOR_MAX_MM * MM_PER_TICK_Q / MM_PER_TICK_P; // 약 5830 틱
    static constexpr uint16_t MAX_VALID_TICKS = static_cast<uint16_t>(
        SENSOR_MAX_TICKS < TICK_TO_MM.max_input() ? SENSOR_MAX_TICKS
                                                  : TICK_TO_MM.max_input());
    static_assert(MAX_VALID_TICKS <= TICK_TO_MM.max_input(),
                  "valid ticks must not overflow TICK_TO_MM.apply()");

    // 무반사(장애물 없음)를 range_ticks 에 기록할 때 쓰는 표시값
    static constexpr uint16_t NO_ECHO_TICKS = UINT16_MAX;
    static_assert(NO_ECHO_TICKS > MAX_VALID_TICKS,
                  "NO_ECHO_TICKS must not collide with a valid width");

    // 이 시간 동안 새 측정이 없으면 센서 무응답으로 보고 UNKNOWN 을 반환
    static constexpr uint32_t STALE_PERIODS = 4;
    static constexpr unsigned long STALE_MS =
        timer1::PERIOD_US * STALE_PERIODS / 1000UL; // 240 ms

    static constexpr int SENDER = PB1;
    static constexpr int RECEIVER = PB2;

  public:
    RangeSensor() = delete;

    RangeSensor(const RangeSensor&) = delete;
    RangeSensor(RangeSensor&&) = delete;

    RangeSensor& operator=(const RangeSensor&) = delete;
    RangeSensor& operator=(RangeSensor&&) = delete;

    static void begin();

    /// 최신 거리(mm)와 그 상태를 반환
    /// - CHANGED     : 새로 측정된 값 (약 60 ms 마다).
    /// 장애물이 없으면 mm == FAR_MM
    /// - NOT_CHANGED : 새로운 측정 없음. mm 는 직전 값
    /// - UNKNOWN     : 측정 전이거나 STALE_MS 이상 갱신 없음 (센서 이상).
    ///                 mm 는 마지막으로 알던 값, 측정 전에는 FAR_MM
    static RangeStatus read();

    /// ISR 전용, 호출금지
    static inline void __attribute__((always_inline)) echo_edge_isr();

    static constexpr uint16_t FAR_MM = UINT16_MAX;

  private:
    static void timer1_init();

    static void setup_distance_sensor();

    // 인터럽트 활성화
    static void enable_pcint0();

    static inline void __attribute__((always_inline)) write(uint16_t value);

    static void calculate_distance(uint16_t tick);

    static volatile bool is_renew;
    static volatile uint16_t range_ticks;
    static uint16_t distance;
};