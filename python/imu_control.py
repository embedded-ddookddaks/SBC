import serial
import time
import board
import busio
import pygame 
import adafruit_bno055 

# --- 1. 통신 및 센서 초기 설정 ---
# STM32 UART 설정
try:
    ser = serial.Serial(
        port='/dev/ttyS0',      # 라즈베리 파이의 UART 포트
        baudrate=115200,        
        timeout=0.1
    )
except serial.SerialException as e:
    print(f"UART 포트 오류: {e}"); exit()

# BNO055 IMU 초기화 (I2C 통신)
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c)
except ValueError:
    print("BNO055 센서를 찾을 수 없습니다. 연결을 확인하세요."); exit()

# Pygame 및 조이스틱 초기화
pygame.init(); pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("조이스틱이 감지되지 않았습니다."); exit()
joystick = pygame.joystick.Joystick(0); joystick.init()


# --- 2. 제어 상수 ---
TARGET_HEADING = 0  # 목표 방향 (Yaw)
Kp = 0.5            # PID 비례 게인


# --- 3. 명령 전송 함수 ---
def send_motor_command(speed_l, speed_r):
    """ STM32의 UART 규격에 맞게 4바이트 패킷을 구성하여 전송합니다. """
    
    # 속도를 0~100 범위와 방향(1/-1)으로 분리
    def get_motor_params(speed):
        if speed >= 0:
            direction = 1  # 정회전
            magnitude = min(100, int(round(speed)))
        else:
            direction = -1 # 역회전
            magnitude = min(100, int(round(-speed)))
        return direction, magnitude

    dir_l, mag_l = get_motor_params(speed_l)
    dir_r, mag_r = get_motor_params(speed_r)

    # 4바이트 패킷: [DIR_L, Speed_L(0~100), DIR_R, Speed_R(0~100)]
    packet = [dir_l & 0xFF, mag_l & 0xFF, dir_r & 0xFF, mag_r & 0xFF]
    
    # 바이너리 데이터로 전송
    ser.write(bytes(packet))


# --- 4. 메인 루프 (IMU 읽기 및 제어) ---
try:
    while True:
        # A. IMU 데이터 읽기
        yaw_pitch_roll = sensor.euler 
        current_yaw = yaw_pitch_roll[0] if yaw_pitch_roll[0] is not None else 0
        
        # B. 조이스틱 입력 처리
        pygame.event.pump() 
        throttle_input = -joystick.get_axis(1) * 100 # Y축 (전/후진)
        steer_input    = joystick.get_axis(2) * 50  # X축 (좌/우 조향)
        
        # C. IMU 보정값 계산 (PID P-Gain 적용)
        steer_correction = 0
        if abs(throttle_input) > 10: # 움직이는 중일 때만 보정
            heading_error = TARGET_HEADING - current_yaw
            # 180도 오차 보정
            if heading_error > 180: heading_error -= 360
            if heading_error < -180: heading_error += 360
            steer_correction = Kp * heading_error 

        # D. 최종 모터 속도 계산
        final_steer = steer_input + steer_correction
        
        speed_L = throttle_input + final_steer
        speed_R = throttle_input - final_steer

        # E. 명령 전송
        send_motor_command(speed_L, speed_R)
        
        time.sleep(0.01) # 100Hz 루프

except KeyboardInterrupt:
    print("\n제어 루프 종료. 모터 정지 명령 전송.")
    send_motor_command(0, 0)
    ser.close()
    pygame.quit()