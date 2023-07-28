# Untitled - By: bhabas - Fri Jul 28 2023

import sensor, image, pyb, os, time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.B128X128)
sensor.set_framebuffers(sensor.DOUBLE_BUFFER)

## ROTATE IMAGE
sensor.set_transpose(True)
sensor.set_vflip(False)
sensor.set_hmirror(True)

sensor.skip_frames(time = 1000)

def frame_cb():
    print("New frame available!")

clock = time.clock()

#extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)

print(sensor.get_framebuffers())

while(True):
    clock.tick()
    img = sensor.snapshot()
    sensor.set_frame_callback(frame_cb)

#    print(clock.fps())
