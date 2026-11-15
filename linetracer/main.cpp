/* 이 소스는 에듀이노(Eduino)에 의해서 번역, 수정, 작성되었고 소유권 또한
 * 에듀이노의 것입니다. 소유권자의 허락을 받지 않고 무단으로 수정, 삭제하여
 * 배포할 시 법적인 처벌을 받을 수도 있습니다.
 * 에듀이노 메탈 라인트레이서 기본예제
 */
#include "drive.h"

#include "src/range/range_sensor.h"

#include <Arduino.h>

namespace
{

// RangeSensor::read()의 거리는 mm 단위다.
// 15cm 이내에서 멈추고, 20cm 이상으로 멀어지면 다시 출발한다.
constexpr uint16_t WALL_STOP_MM = 150;
constexpr uint16_t WALL_RESUME_MM = 200;

bool should_stop_for_wall(const RangeStatus& range, bool already_stopped)
{
    // 센서값이 오래 갱신되지 않으면 예전의 근접 거리로 계속 멈춰 있지 않는다.
    if (range.status == RangeStatus::UNKNOWN)
        return false;

    // 정지와 출발 기준을 달리해 경계에서 덜컥거리지 않게 한다.
    return already_stopped ? range.distance < WALL_RESUME_MM
                           : range.distance <= WALL_STOP_MM;
}

} // namespace

void setup()
{
    drive_begin();
    // 초음파 센서의 주기 측정과 에코 인터럽트를 시작한다.
    RangeSensor::begin();
}

void loop()
{
    static bool stopped_for_wall = false;

    const bool was_stopped = stopped_for_wall;
    stopped_for_wall = should_stop_for_wall(RangeSensor::read(), was_stopped);

    if (stopped_for_wall)
    {
        if (!was_stopped)
            drive_stop(); // 멈추는 순간에 한 번만 명령한다
        return;
    }

    line_follow_step();
}
