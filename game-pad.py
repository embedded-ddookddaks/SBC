import sys, argparse
import pygame
import serial

def main():
    ap = argparse.ArgumentParser(description="Joystick -> UART (no parsing)")
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=int, default=120)
    args = ap.parse_args()

    # UART 열기: 논블로킹 읽기(timeout=0)
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

    try:
        while True:
            for e in pygame.event.get():
                if e.type == pygame.JOYBUTTONDOWN:
                    btn = e.button
                    print(f"Button {btn } DOWN -> TX 1 byte")
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

    except KeyboardInterrupt:
        print("\nBye")
    finally:
        try:
            js.quit()
            pygame.joystick.quit()
            pygame.quit()
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()
