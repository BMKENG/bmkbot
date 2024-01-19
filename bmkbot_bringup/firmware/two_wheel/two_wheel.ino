
#include "PID.h"

void setup() {
  setting();
  Serial.begin(115200);  
  // timer 10ms = 1초에 100번
  
  Serial.setTimeout(10);
  Serial.println("m1_ref_spd, m1speed");
  
  Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
  // 1000us(1ms) 에 한번씩 타이머 발생하게 설정
  Timer2.setPeriod(10000);  //50,000 us = 50ms
  Timer2.attachCompare1Interrupt(t2_ISR);
}


void loop() {
  // 시리얼 버퍼에 데이터가 있는지 확인
  if (Serial.available() > 0) {
    // 목표 속도값을 받아옴
    char c[100];
    // 목표속도에 대한 문자열을 char 형태로 저장합니다.
    Serial.readBytesUntil(')', c, 100); // ')' 문자를 만날 때까지 읽습니다.
    c[99] = '\0'; // 마지막에 널 문자를 추가하여 문자열을 종료시킵니다.

    // '(' 문자를 찾아 데이터의 시작 위치를 조정합니다.
    char *start = strchr(c, '(');
    if (start != NULL) {
      // strtok 함수를 통해 문자열을 자르고, 자른 문자열을 정수형으로 변환합니다.
      char *token = strtok(start + 1, ","); // 첫 번째 숫자 추출
      if (token != NULL) {
        m1_ref_spd = atoi(token); // 첫 번째 숫자를 정수로 변환
        token = strtok(NULL, ","); // 두 번째 숫자 추출
        if (token != NULL) {
          m2_ref_spd = atoi(token); // 두 번째 숫자를 정수로 변환
        }
      }
    }
  }
    
 //   m1_ref_spd += m2_ref_spd // 31; 
} 
