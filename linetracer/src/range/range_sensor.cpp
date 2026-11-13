#include "range_sensor.h"

volatile bool RangeSensor::is_renew = false;
volatile uint16_t RangeSensor::range_ticks = 0;
// 측정 전에는 "장애물 없음"으로 둔다. 0 이면 "아주 가까움"으로 해석되어
// 출발하자마자 정지할 수 있다.
uint16_t RangeSensor::distance = RangeSensor::FAR_MM;

constexpr Pow2Fraction RangeSensor::TICK_TO_MM;
constexpr uint16_t RangeSensor::FAR_MM; // 공개 상수: 참조로 넘겨질 때 대비

void RangeSensor::begin()
{
    setup_distance_sensor();
    timer1_init();
    enable_pcint0();
}

RangeStatus RangeSensor::read()
{
    static bool has_value = false;
    static unsigned long last_update_ms = 0;

    if (!is_renew)
    {
        if (!has_value || millis() - last_update_ms > STALE_MS)
            return {RangeStatus::UNKNOWN, distance};
        return {RangeStatus::NOT_CHANGED, distance};
    }

    cli();
    is_renew = false;
    auto tick = range_ticks;
    sei();

    calculate_distance(tick);

    has_value = true;
    last_update_ms = millis();
    return {RangeStatus::CHANGED, distance};
}

inline void __attribute__((always_inline)) RangeSensor::echo_edge_isr()
{
    static uint16_t tick = 0;
    static bool rising_seen = false;
    const int receive_value = (PINB & _BV(RECEIVER));
    if (receive_value)
    {
        tick = TCNT1;
        rising_seen = true;
        return;
    }

    // 상승을 보지 못한 하강
    if (!rising_seen)
        return;
    rising_seen = false;

    // TCNT1 은 ICR1(TOP)에서 0으로 돌아가므로, 하강 < 상승 이면 주기 경계를
    // 넘은 것
    const uint16_t now = TCNT1;
    if (now < tick)
        return;

    const uint16_t width = now - tick;
    // 센서 타임아웃 펄스 등 상한을 넘는 폭은 무반사로 기록
    RangeSensor::write(width > MAX_VALID_TICKS ? NO_ECHO_TICKS : width);
}

void RangeSensor::timer1_init()
{
    using namespace timer1;
    TCCR1B &= ~CS_MASK;

    TCCR1A = (TCCR1A & ~(WGM_A_MASK | COM1A_MASK | COM1B_MASK)) |
             WGM_A_FAST_PWM_ICR1 | COM1A_NON_INVERTING;
    TCCR1B = (TCCR1B & ~WGM_B_MASK) | WGM_B_FAST_PWM_ICR1;

    ICR1 = ICR1_TOP;       // 주기: (ICR1 + 1) 틱 = 60 ms
    OCR1A = OCR1A_TRIGGER; // 트리거 폭: (OCR1A + 1) 틱 = 12 us

    TCNT1 = 0;

    TIFR1 = _BV(OCF1A);

    TCCR1B = (TCCR1B & ~CS_MASK) | CS_DIV64;
}

void RangeSensor::setup_distance_sensor()
{
    PORTB &= ~(_BV(RECEIVER) | _BV(SENDER));
    DDRB |= _BV(SENDER);
    DDRB &= ~_BV(RECEIVER);
}

void RangeSensor::enable_pcint0()
{
    PCMSK0 |= _BV(RECEIVER);
    PCIFR = _BV(PCIF0);
    PCICR |= _BV(PCIE0);
}

inline void __attribute__((always_inline)) RangeSensor::write(uint16_t value)
{
    range_ticks = value;
    is_renew = true;
}

void RangeSensor::calculate_distance(uint16_t tick)
{
    distance = (tick == NO_ECHO_TICKS) ? FAR_MM : TICK_TO_MM.apply(tick);
}

ISR(PCINT0_vect) { RangeSensor::echo_edge_isr(); }