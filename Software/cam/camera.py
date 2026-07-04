import csi, time
from machine import LED, UART
from math import sqrt


CALIBRATION = False # Gets gain and exposure and draws masking circle

FRAME_HEIGHT = 400
FRAME_WIDTH = 400
FRAME_CX = 210 # 210, 235
FRAME_CY = 201 # 201, 195
FRAME_R = 200 # 200, 193

GOAL_MIN_AREA = 200
GOAL_MIN_PIXELS = 100
BALL_MIN_AREA = 5 #25
BALL_MIN_PIXELS = 5 #15

BALL = 1 # blob.code(), binary
YELLOW = 2
BLUE = 4


# Ball, Yellow, Blue
THRESHOLDS = [(62, 80, 11, 62, 26, 56), (41, 61, 6, 36, 21, 41), (25, 38, -8, 12, -23, 1)]

led = LED("LED_GREEN")
led.on()

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize(csi.VGA)
cam.window((FRAME_WIDTH, FRAME_HEIGHT))

cam.snapshot(time=2000) # let auto‑exposure settle

if CALIBRATION:
    print("Calibration Mode")

else:
    cam.auto_gain(False, gain_db=23.26472)
    cam.auto_exposure(False, exposure_us=16648) # 120fps: 8248;   60fps: 16648
    cam.auto_whitebal(False)

clock = time.clock()
clock.reset()

uart = UART(3, 115200)
uart.init(115200)
led.off()


if CALIBRATION:
    while(True):
        clock.tick()
        img = cam.snapshot()
        img.mask_circle(FRAME_CX, FRAME_CY, FRAME_R)

        print("Gain:", cam.gain_db(), "Exposure:", cam.exposure_us())

else:
    while(True):
        clock.tick()
        img = cam.snapshot()

        blobs = img.find_blobs(THRESHOLDS, x_stride=5, y_stride=5, area_threshold=BALL_MIN_AREA, pixels_threshold=BALL_MIN_PIXELS, merge=False, margin=5)

        # Start bytes, ball, yellow, blue
        data = [255, 255, 500, 500, 500, 500, 500, 500]

        rawBlobs = [None, None, None]

        for blob in blobs:

            blobX = blob.cx() - FRAME_CX
            blobY = blob.cy() - FRAME_CY
            mag = sqrt(blobX * blobX + blobY * blobY)

            if mag > FRAME_R: # Makeshift masking circle
                continue

            if blob.code() == BALL:
                if rawBlobs[0] == None:
                    rawBlobs[0] = blob
                else:
                    if blob.area() > (rawBlobs[0]).area():
                        rawBlobs[0] = blob

            elif blob.code() == YELLOW:
                if rawBlobs[1] == None:
                    if blob.area() > GOAL_MIN_AREA and blob.pixels() > GOAL_MIN_PIXELS:
                        rawBlobs[1] = blob
                else:
                    if blob.area() > (rawBlobs[1]).area():
                        rawBlobs[1] = blob

            elif blob.code() == BLUE:
                if rawBlobs[2] == None:
                    if blob.area() > GOAL_MIN_AREA and blob.pixels() > GOAL_MIN_PIXELS:
                        rawBlobs[2] = blob
                else:
                    if blob.area() > (rawBlobs[2]).area():
                        rawBlobs[2] = blob


        if rawBlobs[0] != None:
            data[2] = (rawBlobs[0]).cx()
            data[3] = (rawBlobs[0]).cy()

        if rawBlobs[1] != None:
            data[4] = (rawBlobs[1]).cx()
            data[5] = (rawBlobs[1]).cy()

        if rawBlobs[2] != None:
            data[6] = (rawBlobs[2]).cx()
            data[7] = (rawBlobs[2]).cy()

        # img.draw_line(FRAME_CX - 5, FRAME_CY, FRAME_CX + 5, FRAME_CY) # horizontal
        # img.draw_line(FRAME_CX, FRAME_CY - 5, FRAME_CX, FRAME_CY + 5) # vertical

        # img.draw_line(FRAME_CX, FRAME_CY, data[2], data[3]); # draws line from center to ball
        # img.draw_line(FRAME_CX, FRAME_CY, data[4], data[5]); # draws line from center to ygoal
        # img.draw_line(FRAME_CX, FRAME_CY, data[6], data[7]); # draws line from center to bgoal

        uart.writechar(data[0])
        uart.writechar(data[1])
        uart.writechar(data[2] & 255)
        uart.writechar((data[2] >> 8) & 255)
        uart.writechar(data[3] & 255)
        uart.writechar((data[3] >> 8) & 255)
        uart.writechar(data[4] & 255)
        uart.writechar((data[4] >> 8) & 255)
        uart.writechar(data[5] & 255)
        uart.writechar((data[5] >> 8) & 255)
        uart.writechar(data[6] & 255)
        uart.writechar((data[6] >> 8) & 255)
        uart.writechar(data[7] & 255)
        uart.writechar((data[7] >> 8) & 255)
        # print(clock.fps())

        # print(data[2], data[2] & 255, (data[2] >> 8) & 255)
