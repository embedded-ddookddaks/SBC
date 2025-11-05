"""
main.py

딜리봇 장애물 감지 시스템
- LiDAR + 초음파 센서 읽기
- 장애물 감지 (정량 지표 기반)
- 결과 출력
"""

import time
import threading
import queue
import sys
import pygame
import argparse
import serial
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
        # UART 초기 설정
        ap = argparse.ArgumentParser(description="Joystick -> UART (no parsing)")
        ap.add_argument("--port", default="/dev/ttyAMA0")
        ap.add_argument("--baud", type=int, default=115200)
        ap.add_argument("--hz", type=int, default=120)
        args = ap.parse_args()
        
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.2)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print(f"UART open: {args.port} @ {args.baud}")
        except Exception as e:
            print("UART open failed:", e)
            sys.exit(1)
            
        # Pygame/조이스틱
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick found")
            sys.exit(1)
        js = pygame.joystick.Joystick(0)
        js.init()
        print("Name:", js.get_name(), "Axes:", js.get_numaxes(), "Buttons:", js.get_numbuttons())

        clock = pygame.time.Clock()
        """메인 루프"""
        last_update_time = 0
        last_status_time = 0
        
        print("장애물 감지 중... (Ctrl+C로 종료)\n")
        
        try:
            while self.running:
                current_time = time.time()
                for e in pygame.event.get():
                    if e.type == pygame.JOYBUTTONDOWN:
                        btn = e.button
                        print(f"Button {btn} DOWN -> TX 1 byte")
                        # 버튼 인덱스를 '그대로' 1바이트로 전송
                        ser.write(bytes([btn]))
                        ser.flush()

                # 수신 버퍼에 쌓인 건 그냥 전부 읽어서 헥사로 출력
                n = ser.in_waiting
                if n:
                    data = ser.read(n)
                    # 보기 편하게 공백 구분 헥사 출력
                    print("[RX]", data.hex(" "))

                clock.tick(args.hz)            
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
        finally:
            try:
                js.quit()
                pygame.joystick.quit()
                pygame.quit()
                ser.close()
            except:
                pass

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
