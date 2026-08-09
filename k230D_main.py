from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from machine import Pin, UART, FPIOA
import os
import gc
import time
import sys


# ---------------- DNK230D BOX hardware configuration ----------------
MODEL = "/sdcard/ball.kmodel"
DISPLAY_SIZE = [640, 480]       
RGB888P_SIZE = [320, 320]       
MODEL_INPUT_SIZE = [320, 320]   
LABELS = ["ball"]

UART2_TX_PIN = 44
UART2_RX_PIN = 45
UART_BAUDRATE = 115200


VELOCITY_ALPHA = 0.35
ACCELERATION_ALPHA = 0.20
POSITION_HALF_RANGE = 12.5
SCAN_END_RATIO = 0.40


def main():

    try:
        os.stat(MODEL)
    except OSError:
        raise OSError("K230 model not found: " + MODEL)

    fpioa = FPIOA()
    fpioa.set_function(34, FPIOA.GPIO34)
    fpioa.set_function(35, FPIOA.GPIO35)
    fpioa.set_function(40, FPIOA.UART1_TXD)
    fpioa.set_function(41, FPIOA.UART1_RXD)
    fpioa.set_function(UART2_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART2_RX_PIN, FPIOA.UART2_RXD)
    key0 = Pin(34, Pin.IN, pull=Pin.PULL_UP, drive=7)
    key1 = Pin(35, Pin.IN, pull=Pin.PULL_UP, drive=7)
    uart1 = UART(
        UART.UART1,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    uart2 = UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )

    pl = None
    yolo = None

    try:
        print("[1/4] Initializing DNK230D PipeLine at 30 FPS...")
        pl = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode="lcd",
            display_size=DISPLAY_SIZE,
        )

        pl.create(fps=30)
        print("[2/4] PipeLine ready")

        display = pl.get_display_size()
        display_width = int(display[0])
        display_height = int(display[1])
        center_x = display_width // 2
        center_y = display_height // 2
        scan_y = 0
        scan_height = int(display_height * SCAN_END_RATIO)

        print("LCD:", display_width, display_height)
        print("CENTER:", center_x, center_y)
        print("SCAN (top 40%):", scan_y, scan_height)
        print("UART2 PH2.0: TX=IO%d RX=IO%d %d baud" %
              (UART2_TX_PIN, UART2_RX_PIN, UART_BAUDRATE))

        print("[3/4] Loading model:", MODEL)
        yolo = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=MODEL,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=display,
            conf_thresh=0.35,
            nms_thresh=0.4,
            max_boxes_num=10,
            debug_mode=0,
        )
        yolo.config_preprocess()
        print("[4/4] Model ready; entering detection loop")

        tracking = False
        last_pos = 0.0
        last_time = 0
        velocity = 0.0
        acceleration = 0.0

        while True:
            os.exitpoint()
            img = pl.get_frame()
            res = yolo.run(img)
            msg = "NO_TARGET\r\n"

            has_detections = (
                res is not None
                and len(res) >= 3
                and len(res[0]) > 0
            )

            best = -1
            best_score = 0.0

            if has_detections:
                boxes = res[0]
                scores = res[2]

                for i in range(len(boxes)):
                    box = boxes[i]
                    box_center_y = float(box[1]) + float(box[3]) * 0.5
                    score = float(scores[i])
                    if box_center_y < scan_height and score > best_score:
                        best = i
                        best_score = score

                if best >= 0:
                    box = boxes[best]
                    box_center_x = float(box[0]) + float(box[2]) * 0.5
                    pos = ((box_center_x - center_x) / center_x
                           * POSITION_HALF_RANGE)
                    now = time.ticks_us()

                    if tracking:
                        dt_us = time.ticks_diff(now, last_time)
                        if 1000 < dt_us < 500000:
                            dt = dt_us / 1000000.0
                            new_velocity = (
                                VELOCITY_ALPHA * ((pos - last_pos) / dt)
                                + (1.0 - VELOCITY_ALPHA) * velocity
                            )
                            acceleration = (
                                ACCELERATION_ALPHA
                                * ((new_velocity - velocity) / dt)
                                + (1.0 - ACCELERATION_ALPHA) * acceleration
                            )
                            velocity = new_velocity
                        else:
                            velocity = 0.0
                            acceleration = 0.0
                    else:
                        tracking = True
                        velocity = 0.0
                        acceleration = 0.0

                    last_pos = pos
                    last_time = now
                    msg = "S%.1f,%.1f,%.1f\r\n" % (
                        pos,
                        velocity,
                        acceleration,
                    )
                else:
                    tracking = False
                    velocity = 0.0
                    acceleration = 0.0
            else:
                tracking = False
                velocity = 0.0
                acceleration = 0.0

            uart2.write(msg)
            print(msg.strip())

            pl.osd_img.clear()
            if has_detections:
                yolo.draw_result(res, pl.osd_img)

            pl.osd_img.draw_line(
                center_x, 0, center_x, display_height - 1,
                color=(255, 0, 0), thickness=2
            )
            pl.osd_img.draw_line(
                0, center_y, display_width - 1, center_y,
                color=(255, 0, 0), thickness=2
            )
            pl.osd_img.draw_rectangle(
                0, scan_y, display_width - 1, scan_height - 1,
                color=(0, 255, 0), thickness=2
            )

            pl.show_image()
            gc.collect()

    except KeyboardInterrupt:
        print("Stopped by user")
    except BaseException as exc:
        sys.print_exception(exc)
    finally:
        if yolo is not None:
            yolo.deinit()
        if pl is not None:
            pl.destroy()
        gc.collect()


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    main()
