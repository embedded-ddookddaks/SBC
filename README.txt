딜리봇 장애물 감지 시스템
========================================

[하드웨어]
- SBC: 라즈베리파이 5
- LiDAR: YDLidar X4 Pro
- 초음파: HY-SRF05 × 3개


[목적]
LiDAR + 초음파 센서로 장애물 감지
정량 지표 기반 위험도 판단


[정량 지표]
- LiDAR 3m 이하 → 즉시 정지 신호
- 초음파 1m 이하 → 즉시 정지 신호
- 3.5m 이상 → 재주행 허용


[파일 구성]
config.py              정량 지표 설정
lidar_reader.py        LiDAR 센서
ultrasonic_reader.py   초음파 센서  
obstacle_detector.py   장애물 감지 (핵심)
main.py                실행 프로그램
test_sensors.py        센서 테스트
requirements.txt       필요 패키지


[설치]
# YDLidar SDK 설치 (필수)
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK/build
cmake ..
make
sudo make install

# Python 패키지
pip3 install -r requirements.txt

# GPIO (라즈베리파이 5)
sudo apt-get install python3-lgpio

# 권한 설정
sudo usermod -a -G dialout $USER
sudo reboot


[사용]
1. 테스트
   ./test_sensors.py
   
2. 실행
   ./run.sh
   
3. 출력 예시
   [ 5.2s] ✓ 안전 | 전방: 5.23m 좌: 3.45m 우: 4.12m
   [10.7s] ⚠️ 경고 | 전방: 3.78m 좌: 3.21m 우: 4.05m
   [15.3s] 🚨 위험 | 전방: 2.45m 좌: 3.10m 우: 3.89m
          🚨 front 방향 장애물 감지!


[설정 변경]
config.py에서:
- LIDAR_DANGER_DISTANCE = 3.0
- ULTRASONIC_DANGER_DISTANCE = 1.0  
- SAFE_RESUME_DISTANCE = 3.5
- GPIO 핀 번호


[문제 해결]
Q: LiDAR 연결 안 됨
A: ls -l /dev/ttyUSB*
   시뮬레이션 모드로 계속 진행

Q: 초음파 값이 999.0
A: GPIO 배선 확인
   시뮬레이션 모드일 수 있음
