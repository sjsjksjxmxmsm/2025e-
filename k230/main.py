import os
import gc
import time
import cv2
import image
import ustruct

from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ==================== 与 STM32 的串口通信 ====================
# 庐山派 GH1.25-4P 串口2： T = GPIO11 (UART2_TXD)   R = GPIO12 (UART2_RXD)
# 接线：K230 的 T  ->  STM32 的 PA3
#       K230 的 G  ->  STM32 的 GND（必须共地）
#       K230 的 V(5V) 不要接（板子已经另外供电）
UART_TX_PIN = 11
UART_RX_PIN = 12
UART_BAUD = 115200

uart = None
last_sent = [0, 0, 0]      # 最近一次真正发出去的值，只用来打印


def uart_init():
    """把 GPIO11/12 复用成 UART2 并初始化。"""
    global uart
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    uart = UART(UART.UART2, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)
    return uart


def send_target(dx, dy, area):
    """
    给 STM32 发一帧 8 字节：  0x7B  XH XL  YH YL  AH AL  0x7D
        dx / dy : 目标相对画面中心的偏移（像素，有符号）
        area    : 目标面积 / 10，发给 0 表示"没有目标"
    帧内所有值都是无符号，所以偏移要 +32768 再拆成两个字节。
    """
    global last_sent
    last_sent = [dx, dy, area]

    if uart is None:
        return

    vx = dx + 32768
    vy = dy + 32768
    if vx < 0:
        vx = 0
    elif vx > 65535:
        vx = 65535
    if vy < 0:
        vy = 0
    elif vy > 65535:
        vy = 65535
    if area < 0:
        area = 0
    elif area > 65535:
        area = 65535

    try:
        uart.write(b'\x7b' + ustruct.pack('>HHH', vx, vy, area) + b'\x7d')
    except Exception as e:
        print("uart write error:", e)


# ==================== 图像参数 ====================
# 摄像头处理分辨率
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# LCD 分辨率
LCD_WIDTH = 800
LCD_HEIGHT = 480

# 常见 LCD：ST7701
# 如果你的屏不是这个，改成对应面板类型，比如 Display.NT35516 / Display.GC9503
DISPLAY_TYPE = Display.ST7701

# 屏幕还没调通就改成 False：完全跳过显示，只跑「检测 + 串口发送」。
# 屏一旦初始化失败会把整个程序带崩，串口数据也就断了，调云台阶段建议先关掉。
USE_LCD = True

# Canny thresholds
CANNY_LOW = 50
CANNY_HIGH = 150

# Contour approximation accuracy
APPROX_RATIO = 0.02

# Rectangle filter
MIN_AREA = 800
MAX_AREA = 60000
MIN_RECT_WIDTH = 40
MIN_RECT_HEIGHT = 40


def is_target_rectangle(approx, area):
    x, y, w, h = cv2.boundingRect(approx)
    if w <= 0 or h <= 0:
        return False

    if w < MIN_RECT_WIDTH or h < MIN_RECT_HEIGHT:
        return False

    if area < MIN_AREA or area > MAX_AREA:
        return False

    rect_area = w * h
    if rect_area <= 0:
        return False

    fill_ratio = area / rect_area
    if fill_ratio < 0.50:
        return False

    return True


def main():
    sensor = None
    clock = time.clock()
    print_timer = time.ticks_ms()

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)

        sensor = Sensor(width=1280, height=960, fps=90)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB888)

        if USE_LCD:
            Display.init(DISPLAY_TYPE, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
        MediaManager.init()
        uart_init()                      # ★ 启动和 STM32 的串口
        print("uart ok, 115200 8N1")

        sensor.run()
        time.sleep(0.5)

        while True:
            os.exitpoint()
            clock.tick()

            img = sensor.snapshot()
            frame = img.to_numpy_ref()

            # 检测用灰度图
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)

            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            rect_count = 0
            best_area = 0
            best_pts = None
            best_rect = None

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA or area > MAX_AREA:
                    continue

                epsilon = APPROX_RATIO * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                if len(approx) != 4:
                    continue

                if not is_target_rectangle(approx, area):
                    continue

                if area > best_area:
                    best_area = area
                    best_pts = approx
                    best_rect = cv2.boundingRect(approx)

            if best_pts is not None:
                rect_count = 1
                cv2.drawContours(frame, [best_pts], -1, (0, 255, 0), 2)

                x, y, rw, rh = best_rect
                center_x = x + rw // 2
                center_y = y + rh // 2
                img_center_x = FRAME_WIDTH // 2
                img_center_y = FRAME_HEIGHT // 2
                offset_x = center_x - img_center_x
                offset_y = center_y - img_center_y

                cv2.circle(frame, (center_x, center_y), 6, (0, 0, 255), -1)
                cv2.line(frame, (img_center_x, img_center_y), (center_x, center_y), (0, 255, 255), 2)
                cv2.putText(frame, "Center:(%d,%d)" % (center_x, center_y),
                            (max(center_x - 80, 10), max(center_y - 12, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(frame, "Offset:(%d,%d)" % (offset_x, offset_y),
                            (30, 145),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # ★ 有目标：发偏移 + 面积/10（至少 1，保证 STM32 那边不为 0）
                # contourArea 返回的是浮点数，先转 int 再整除，ustruct 只吃整数
                area10 = int(best_area) // 10
                if area10 < 1:
                    area10 = 1
                send_target(offset_x, offset_y, area10)
            else:
                # ★ 没目标：发面积 0，STM32 会认为目标丢失
                send_target(0, 0, 0)

            fps = clock.fps()
            cv2.putText(frame, "FPS: %.1f" % fps, (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.putText(frame,
                        "Rectangles:%d Area:%d-%d Size>%dx%d" % (
                            rect_count, MIN_AREA, MAX_AREA, MIN_RECT_WIDTH, MIN_RECT_HEIGHT
                        ),
                        (30, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # 显示部分单独包一层：屏一旦报错也不会把整个循环带崩（否则串口就断了）
            if USE_LCD:
                try:
                    # 为了铺满 800x480 LCD，这里把 640x480 放大显示
                    show_np = cv2.resize(frame, (LCD_WIDTH, LCD_HEIGHT))
                    show_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.RGB888,
                                           alloc=image.ALLOC_REF, data=show_np)
                    Display.show_image(show_img)
                except Exception as e:
                    print("display error:", e)

            # 每秒在 IDE 终端打一行，直接看到"到底发了什么"
            if time.ticks_diff(time.ticks_ms(), print_timer) > 1000:
                print_timer = time.ticks_ms()
                print("fps=%.1f  target=%s  send=(%d, %d, %d)" %
                      (fps, "YES" if best_pts is not None else "NO",
                       last_sent[0], last_sent[1], last_sent[2]))

            gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as e:
        print("Exception:", e)
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        if USE_LCD:
            Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        if uart is not None:
            uart.deinit()


if __name__ == "__main__":
    main()
