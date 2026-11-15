#pragma once

// 라인 주행 제어. 모터 객체와 코너 판정 상태는 drive.cpp 안에 둔다.

/// 모터를 정지 상태로 준비한다. setup()에서 한 번 호출한다.
void drive_begin();

/// 라인 센서를 읽고 조향을 한 단계 진행한다. loop()에서 매번 호출한다.
/// delay() 없이 바로 반환한다.
void line_follow_step();

/// 모터 출력을 끄고 코너 판정 상태를 버린다. 벽 앞에서 정지할 때 한 번
/// 호출한다. 다음 line_follow_step()이 속도와 방향을 다시 보내 출발한다.
void drive_stop();
