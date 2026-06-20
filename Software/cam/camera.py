import csi, time
from machine import LED, UART

FRAME_HEIGHT = 200
FRAME_WIDTH = 200
FRAME_CX = 104
FRAME_CY = 100
FRAME_R = 93

GOAL_MIN_AREA = 20

LED = LED("LED_GREEN")
LED.on()

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize(csi.QVGA)

cam.window((FRAME_WIDTH, FRAME_HEIGHT))

cam.snapshot(time=2000) # let auto‑exposure settle

cam.framerate(120)
cam.auto_gain(False, gain_db=24.0824)
cam.auto_exposure(False, exposure_us=8000)
cam.auto_whitebal(False) #, rgb_gain_db=(4.972838, 0.0, 4.972838))

clock = time.clock()
clock.reset()

#byte type

DEBUG = True

# # Ball, Yellow, Blue
thresholds = [(43, 73, 24, 67, 31, 59), (30, 72, -8, 9, 27, 47), (31, 38, -9, 5, -30, -12)]

uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None) # timeout_char?

LED.off()

while(True):
    clock.tick()
    img = cam.snapshot()
    img.mask_circle(FRAME_CX, FRAME_CY, FRAME_R)

    # print("Gain:", cam.gain_db(), "Exposure:", cam.exposure_us(), "RGB Gain:", cam.rgb_gain_db())

    blobs = img.find_blobs(thresholds, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1, merge=False, margin=0)
    # goalBlobs = img.find_blobs([thresholds[1], thresholds[2]], x_stride=5, y_stride=5, area_threshold=30, pixels_threshold=30, merge=True, margin=5) # margin 5

    # Start bytes, ball, yellow, blue
    data = [255, 255, 120, 120, 120, 120, 120, 120]

    rawBlobs = [None, None, None]
    for blob in blobs:
        if blob.code() == 1:
            if rawBlobs[0] == None:
                rawBlobs[0] = blob
            else:
                if blob.area() > (rawBlobs[0]).area():
                    rawBlobs[0] = blob
        elif blob.code() == 2:
            if rawBlobs[1] == None:
                if blob.area() > GOAL_MIN_AREA:
                    rawBlobs[1] = blob
            else:
                if blob.area() > (rawBlobs[1]).area():
                    rawBlobs[1] = blob
        elif blob.code() == 4:
            if rawBlobs[2] == None:
                if blob.area() > GOAL_MIN_AREA:
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



    #     if largestBlob[0] == None:
    #         largestBlob[0] = blob # stores first blob as largestblob
    #     elif blob.area() > largestBlob[0].area():
    #         largestBlob[0] = blob # finds largest blob
    # if largestBlob[0] != None: # it is the largest blob
    #     data[2] = largestBlob[0].cx()
    #     data[3] = largestBlob[0].cy()

    # largestBlob = [None, None]
    # for blob in goalBlobs:
    #     if blob.code() == 1: # yellow
    #         if largestBlob[0] == None:
    #             largestBlob[0] = blob # stores first blob as largestblob
    #         elif blob.area() > largestBlob[0].area():
    #             largestBlob[0] = blob # finds largest blob
    #     else: # blob code == 2, blue
    #         if largestBlob[1] == None:
    #             largestBlob[1] = blob
    #         elif blob.area() > largestBlob[1].area():
    #             largestBlob[1] = blob # finds largest blob
    # if largestBlob[0] != None:
    #     data[4] = largestBlob[0].cx()
    #     data[5] = largestBlob[0].cy()
    # if largestBlob[1] != None:
    #     data[6] = largestBlob[1].cx()
    #     data[7] = largestBlob[1].cy()

    # if CROSSHAIR:
    #             x0   y0   x1   y1
    # img.draw_line(FRAME_CX - 5, FRAME_CY, FRAME_CX + 5, FRAME_CY) # horizontal
    # img.draw_line(FRAME_CX, FRAME_CY - 5, FRAME_CX, FRAME_CY + 5) # vertical
    # if DEBUG:
    #     #             x0   y0   x1   y1

    img.draw_line(FRAME_CX, FRAME_CY, data[2], data[3]); # draws line from center to ball
    img.draw_line(FRAME_CX, FRAME_CY, data[4], data[5]); # draws line from center to ygoal
    img.draw_line(FRAME_CX, FRAME_CY, data[6], data[7]); # draws line from center to bgoal

    # print(clock.fps())
    # print(data)
    for byte in data:
        uart.writechar(byte)
