#pragma once
#include <Arduino.h>

namespace timer1
{

constexpr uint8_t WGM_A_MASK = _BV(WGM11) | _BV(WGM10);
constexpr uint8_t WGM_B_MASK = _BV(WGM13) | _BV(WGM12);
constexpr uint8_t WGM_B_CTC = _BV(WGM12); // WGM = 0100 (CTC, TOP = OCR1A)
constexpr uint8_t CS_MASK = _BV(CS12) | _BV(CS11) | _BV(CS10);
constexpr uint8_t CS_DIV64 = _BV(CS11) | _BV(CS10);

constexpr uint32_t PRESCALER = 64;
constexpr uint32_t TICK_US = PRESCALER * 1000000UL / F_CPU; // 16 MHz -> 4 us,
constexpr uint32_t PERIOD_US = 60000;                       // 60 ms
constexpr uint32_t PERIOD_TICKS = PERIOD_US / TICK_US;

static_assert(
    (PRESCALER * 1000000UL) % F_CPU == 0,
    "Timer1 tick is not an integer number of microseconds for this F_CPU");
static_assert(PERIOD_US % TICK_US == 0,
              "PERIOD_US is not a multiple of the Timer1 tick");
static_assert(PERIOD_TICKS >= 1 && PERIOD_TICKS <= 65536UL,
              "PERIOD_TICKS out of 16-bit Timer1 range");

constexpr uint16_t OCR1A_TOP = static_cast<uint16_t>(PERIOD_TICKS - 1);
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
    uint16_t mm;
};

class RangeSensor
{
  private:
    static constexpr Pow2Fraction TICK_TO_MM =
        make_pow2_fraction(1372, 2000, 4); // 343 * 4 / 2 = 1372/2000

    static constexpr int SENDER = PB1;
    static constexpr int RECEIVER = PB2;

  public:
    constexpr RangeSensor() = default;

    RangeSensor(const RangeSensor&) = delete;
    RangeSensor(RangeSensor&&) = delete;

    RangeSensor& operator=(const RangeSensor&) = delete;
    RangeSensor& operator=(RangeSensor&&) = delete;

    static void begin()
    {
        setup_distance_sensor();
        timer1_init();
        enable_pcint0();
        enable_timer1a();
    }

    /// mm 거리 반환
    static RangeStatus read()
    {
        if (!is_renew)
            return {RangeStatus::NOT_CHANGED, distance};
        distance = calculate_distance(range_ticks);
        is_renew = false;
        return {RangeStatus::CHANGED, distance};
    }

    static __attribute__((always_inline)) void echo_edge_isr()
    {

        static uint16_t tick = 0;
        const int recive_value = (PINB & _BV(RECEIVER));
        if (recive_value)
            tick = TCNT1;
        else
            RangeSensor::write(TCNT1 - tick);
    }

  private:
    static void timer1_init()
    {
        using namespace timer1;
        TCCR1B &= ~CS_MASK;

        TCCR1A &= ~WGM_A_MASK;
        TCCR1B = (TCCR1B & ~WGM_B_MASK) | WGM_B_CTC;

        TCNT1 = 0;

        OCR1A = OCR1A_TOP;

        TIFR1 = _BV(OCF1A);

        TCCR1B = (TCCR1B & ~CS_MASK) | CS_DIV64;
    }

    static void enable_timer1a() { TIMSK1 |= _BV(OCIE1A); }
    static void disable_timer1a()
    {
        TIMSK1 &= ~_BV(OCIE1A);
        TIFR1 = _BV(OCF1A);
    }

    static void setup_distance_sensor()
    {
        PORTB = 0;
        DDRB |= _BV(SENDER);
        DDRB &= ~_BV(RECEIVER);
    }

    // 인터럽트 활성화
    static void enable_pcint0()
    {
        PCMSK0 |= _BV(RECEIVER);
        PCIFR = PCIF0;
        PCICR |= _BV(PCIE0);
    }

    static __attribute__((always_inline)) void write(uint16_t value)
    {
        range_ticks = value;
        is_renew = true;
    }

    static uint16_t calculate_distance(uint16_t tick)
    {
        is_renew = true;
        return TICK_TO_MM.apply(tick);
    }

    static bool is_renew;
    static uint16_t range_ticks;
    static uint16_t distance;
};