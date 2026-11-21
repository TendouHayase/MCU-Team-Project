#pragma once

// 라인 센서를 읽고 조향을 한 단계 진행한다. loop()에서 매번 부른다.
// delay() 없이 바로 돌아온다
void line_follow_step();

// 모터를 끈다. 다음 line_follow_step()이 명령을 다시 보내 출발한다
void drive_stop();
