"""
main.py

딜리봇 장애물 감지 시스템
- LiDAR + 초음파 센서 읽기
- 장애물 감지 (정량 지표 기반)
- 결과 출력
"""

import time
import config
from lidar_reader import LidarReader
from ultrasonic_reader import UltrasonicReader
from obstacle_detector import ObstacleDetector


class ObstacleDetectionSystem:
    def __init__(self):
        print("장애물 감지 시스템 초기화 중...")
        
        # 센서 초기화
        self.lidar = LidarReader()
        self.ultrasonic = UltrasonicReader()
        self.detector = ObstacleDetector(self.lidar, self.ultrasonic)
        
        self.running = False
        self.start_time = 0
        
        print("초기화 완료\n")
    
    def start(self):
        """시스템 시작"""
        print("시스템 시작 중...\n")
        
        # 센서 시작
        self.lidar.start()
        self.ultrasonic.start()
        
        # 안정화
        time.sleep(2.0)
        
        print("시스템 시작 완료\n")
        self.running = True
        self.start_time = time.time()
        
        return True
    
    def stop(self):
        """시스템 정지"""
        print("\n시스템 종료 중...")
        
        self.running = False
        self.lidar.stop()
        self.ultrasonic.stop()
        
        print("종료 완료")
    
    def run(self):
        """메인 루프"""
        last_update_time = 0
        last_status_time = 0
        
        print("장애물 감지 중... (Ctrl+C로 종료)\n")
        
        try:
            while self.running:
                current_time = time.time()
                
                # 장애물 감지 업데이트 (20Hz)
                if current_time - last_update_time >= config.SENSOR_READ_INTERVAL:
                    last_update_time = current_time
                    self.detector.update()
                
                # 상태 출력 (1Hz)
                if current_time - last_status_time >= config.STATUS_PRINT_INTERVAL:
                    last_status_time = current_time
                    self.print_status()
                
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\n사용자 중단")
    
    def print_status(self):
        """상태 출력"""
        status = self.detector.get_status()
        
        # 위험도 표시
        icons = {0: '✓', 1: '⚠️', 2: '🚨'}
        icon = icons[status['danger_level']]
        
        # 위험 레벨에 따른 텍스트
        level_text = {
            0: '안전',
            1: '경고',
            2: '위험'
        }[status['danger_level']]
        
        print(f"[{time.time() - self.start_time:5.1f}s] "
              f"{icon} {level_text:4s} | "
              f"전방:{status['front_distance']:5.2f}m "
              f"좌:{status['left_distance']:5.2f}m "
              f"우:{status['right_distance']:5.2f}m")
        
        # 위험 시 경고
        if status['danger_level'] == 2:
            print(f"       🚨 {status['danger_direction']} 방향 장애물 감지!")
        elif status['danger_level'] == 1:
            print(f"       ⚠️  {status['danger_direction']} 방향 접근 중")


def main():
    system = ObstacleDetectionSystem()
    
    if system.start():
        system.run()
    
    system.stop()


if __name__ == '__main__':
    main()
