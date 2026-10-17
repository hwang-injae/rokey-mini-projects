# 제스처 인식 결과를 전달한다.
# 명령값은 2주차 회의에서 합의한 uint8 상수를 사용한다.

std_msgs/Header header

uint8 NONE=0
uint8 START=1
uint8 STOP=2

uint8 command
float32 confidence
