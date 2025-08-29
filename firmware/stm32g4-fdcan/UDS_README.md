# UDS (Unified Diagnostic Services) 구현

## 개요
이 프로젝트에 UDS 시퀀스 처리 기능을 추가했습니다. 특정 진단 요청에 대해 멀티프레임 응답을 자동으로 처리합니다.

## UDS 시퀀스

### 1. 진단 요청 수신
- **CAN ID**: `0x7D4`
- **데이터**: `03 22 01 01 55 55 55 55`
- **설명**: ReadDataByIdentifier 서비스 요청

### 2. First Frame 응답
- **CAN ID**: `0x7DC`
- **데이터**: `10 17 62 01 01 FF F0 00`
- **설명**: 총 23바이트 데이터의 첫 번째 프레임 (MULTI-CHECK)

### 3. Flow Control 대기
- **대기 시간**: 100ms 이내
- **예상 수신**: `0x7D4 | 30 08 02 55 55 55 55 55`
- **설명**: Continue To Send (CTS) 플로우 컨트롤

### 4. Consecutive Frames 전송
차례로 3개의 연속 프레임 전송:
- **CF1**: `0x7DC | 21 00 8A 89 F1 00 E6 FF`
- **CF2**: `0x7DC | 22 55 E5 00 00 00 10 10`
- **CF3**: `0x7DC | 23 25 01 00 AA AA AA AA`

## 구현된 파일들

### 새로 추가된 파일
- `uds_handler.h` - UDS 처리 헤더 파일
- `uds_handler.c` - UDS 처리 구현 파일

### 수정된 파일
- `can_tx.h` - RX 메시지 처리 함수 추가
- `can_tx.c` - UDS 기능 통합

## 주요 기능

### UDS 상태 관리
- `UDS_STATE_IDLE`: 대기 상태
- `UDS_STATE_WAITING_FLOW_CONTROL`: Flow Control 대기
- `UDS_STATE_SENDING_CONSECUTIVE`: 연속 프레임 전송
- `UDS_STATE_ERROR`: 에러 상태

### 타이밍 제어
- Flow Control 응답 타임아웃: 100ms
- 연속 프레임 간 지연: 1ms

### 디버그 출력
`DEBUG_CAN_PROTOCOL = 1`로 설정되어 있어 모든 UDS 활동이 UART로 출력됩니다.

## 사용법

1. **빌드**: 프로젝트를 빌드합니다
2. **플래시**: STM32G4 보드에 업로드합니다
3. **TX 모드 설정**: `CAN_TEST_MODE_TX = true`로 설정되어 있는지 확인
4. **테스트**: 
   - CAN 툴에서 `0x7D4 | 03 22 01 01 55 55 55 55` 전송
   - UDS 응답 시퀀스 확인

## 디버그 메시지 예시
```
[UDS] Valid request received!
[UDS] Request ID: 0x7D4 | Length: 8 | Data: 03 22 01 01 55 55 55 55
[UDS] First Frame sent successfully
[UDS] Response FF ID: 0x7DC | Length: 8 | Data: 10 17 62 01 01 FF F0 00
[UDS] Valid Flow Control received!
[UDS] Flow Control ID: 0x7D4 | Length: 8 | Data: 30 08 02 55 55 55 55 55
[UDS] Consecutive Frame 1 sent
[UDS] Response CF1 ID: 0x7DC | Length: 8 | Data: 21 00 8A 89 F1 00 E6 FF
[UDS] Consecutive Frame 2 sent
[UDS] Response CF2 ID: 0x7DC | Length: 8 | Data: 22 55 E5 00 00 00 10 10
[UDS] Consecutive Frame 3 sent
[UDS] Response CF3 ID: 0x7DC | Length: 8 | Data: 23 25 01 00 AA AA AA AA
[UDS] All consecutive frames sent successfully
```

## 설정

### DEBUG 활성화/비활성화
```c
#define DEBUG_CAN_PROTOCOL  1  // 1: 활성화, 0: 비활성화
```

### UDS 타임아웃 변경
```c
#define UDS_TIMEOUT_MS      100  // milliseconds
```
