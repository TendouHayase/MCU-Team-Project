#include "range_sensor.h"

bool RangeSensor::is_renew = false;
uint16_t RangeSensor::range_ticks = 0;
uint16_t RangeSensor::distance = 0;

constexpr Pow2Fraction RangeSensor::TICK_TO_MM;

ISR(PCINT0_vect) { RangeSensor::echo_edge_isr(); }