#include "range_sensor.h"
inline void test_init()
{
    Serial.begin(115200);
    RangeSensor::begin();
    Serial.println(F("RangeSensor test: ms  dt_ms  mm"));
}

inline void test_main()
{
    static bool was_unknown = false;
    static unsigned long last_print_ms = 0;

    const RangeStatus r = RangeSensor::read();

    // UNKNOWN 은 들어가는 순간 한 번만 출력한다
    if (r.status == RangeStatus::UNKNOWN)
    {
        if (!was_unknown)
        {
            Serial.print(millis());
            Serial.print(F("  UNKNOWN  last="));
            Serial.println(r.distance);
        }
        was_unknown = true;
        return;
    }
    was_unknown = false;

    // 새 측정일 때만 출력한다 (약 60 ms 마다 한 줄).
    // 매 루프 출력하면 송신 버퍼(64 바이트)가 차서 Serial.print 가 블로킹된다.
    if (r.status != RangeStatus::CHANGED)
        return;

    const unsigned long now = millis();
    Serial.print(now);
    Serial.print(F("  "));
    Serial.print(now - last_print_ms); // 측정 간격 확인용
    Serial.print(F("  "));
    if (r.distance == RangeSensor::FAR_MM)
        Serial.println(F("FAR"));
    else
        Serial.println(r.distance);
    last_print_ms = now;
}