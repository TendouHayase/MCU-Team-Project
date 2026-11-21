#include "trace_log.h"

#include <Arduino.h>
#include <avr/eeprom.h>

namespace
{
// EEPROM에 그대로 쓰므로 패딩이 없어야 한다
struct __attribute__((packed)) Record
{
    uint16_t t_ms;
    uint8_t state; // bit0~1 = 센서, bit2~3 = 모드
    int8_t y_mm;
    int8_t psi_deg;
    int8_t pwm_l_half; // PWM / 2
    int8_t pwm_r_half;
};
static_assert(sizeof(Record) == 7, "Record must stay 7 bytes");

struct __attribute__((packed)) Header
{
    uint16_t magic;
    uint8_t count;
};

const uint16_t MAGIC = 0x4C54;
// RAM 2KB 중 900B. 25ms마다 + 센서/모드가 바뀔 때마다 기록해서 2~3초 분량
const uint8_t RECORD_COUNT = 128;
const uint16_t HEADER_ADDR = 0;
const uint16_t RECORDS_ADDR = 4;
static_assert(RECORDS_ADDR + RECORD_COUNT * sizeof(Record) <= E2END + 1,
              "trace does not fit in EEPROM");

// USB를 꽂으면 보드가 켜지면서 주행을 시작하고, 손에 든 채라 초음파가
// 바로 정지를 걸어 직전 주행 기록을 덮어쓴다. 이보다 짧게 달린 구간은
// 저장하지 않는다
const unsigned long MIN_RUN_MS = 3000;

Record records[RECORD_COUNT];
uint8_t head = 0; // 다음에 쓸 자리
uint8_t count = 0;
bool started = false;
unsigned long started_at = 0;

int8_t saturate(int32_t v)
{
    return static_cast<int8_t>(v > 127 ? 127 : (v < -127 ? -127 : v));
}

void* eeprom_addr(uint16_t addr) { return reinterpret_cast<void*>(addr); }

} // namespace

void trace_sample(uint8_t line, uint8_t mode, int32_t y_um, int32_t psi_cdeg,
                  int16_t pwm_l, int16_t pwm_r)
{
    const unsigned long now = millis();
    if (!started)
    {
        started = true;
        started_at = now;
    }

    Record& r = records[head];
    r.t_ms = static_cast<uint16_t>(now);
    r.state = static_cast<uint8_t>((line & 3) | ((mode & 3) << 2));
    r.y_mm = saturate(y_um / 1000);
    r.psi_deg = saturate(psi_cdeg / 100);
    r.pwm_l_half = saturate(pwm_l / 2);
    r.pwm_r_half = saturate(pwm_r / 2);

    head = static_cast<uint8_t>((head + 1) % RECORD_COUNT);
    if (count < RECORD_COUNT)
        ++count;
}

void trace_flush()
{
    if (count == 0 || !started || millis() - started_at < MIN_RUN_MS)
        return;

    // 쓰는 도중 전원이 꺼지면 옛 헤더가 새 기록을 가리키게 되므로 먼저 지운다
    const Header invalid = {0, 0};
    eeprom_update_block(&invalid, eeprom_addr(HEADER_ADDR), sizeof(invalid));

    const uint8_t start =
        static_cast<uint8_t>((head + RECORD_COUNT - count) % RECORD_COUNT);
    for (uint8_t i = 0; i < count; ++i)
    {
        const Record& r = records[(start + i) % RECORD_COUNT];
        eeprom_update_block(&r, eeprom_addr(RECORDS_ADDR + i * sizeof(Record)),
                            sizeof(Record));
    }

    const Header header = {MAGIC, count};
    eeprom_update_block(&header, eeprom_addr(HEADER_ADDR), sizeof(header));

    // 같은 구간을 다음 정지 때 또 저장하지 않는다
    head = 0;
    count = 0;
}

void trace_dump()
{
    Header header;
    eeprom_read_block(&header, eeprom_addr(HEADER_ADDR), sizeof(header));
    if (header.magic != MAGIC || header.count > RECORD_COUNT)
    {
        Serial.println(F("# no trace"));
        return;
    }

    Serial.println(F("# trace begin"));
    Serial.println(F("t_ms,line,mode,y_mm,psi_deg,pwm_l,pwm_r"));
    for (uint8_t i = 0; i < header.count; ++i)
    {
        Record r;
        eeprom_read_block(&r, eeprom_addr(RECORDS_ADDR + i * sizeof(Record)),
                          sizeof(Record));
        Serial.print(r.t_ms);
        Serial.print(',');
        Serial.print(r.state & 3);
        Serial.print(',');
        Serial.print((r.state >> 2) & 3);
        Serial.print(',');
        Serial.print(r.y_mm);
        Serial.print(',');
        Serial.print(r.psi_deg);
        Serial.print(',');
        Serial.print(r.pwm_l_half * 2);
        Serial.print(',');
        Serial.println(r.pwm_r_half * 2);
    }
    Serial.println(F("# trace end"));
}
