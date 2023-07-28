import sensor, image, pyb, os, time

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.B128X128) # or sensor.QQVGA (or others)
sensor.skip_frames(time = 1000) # Let new settings take affect.
clock = time.clock() # Tracks FPS.

print("Waiting here")
time.sleep(2)
print("now stuff")


buffer1 = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
buffer1.replace(sensor.snapshot())

sensor.flush()

while(True):
    clock.tick()
    img = sensor.snapshot()

    displacement = buffer1.difference(img)
    buffer1.replace(img)

    print(clock.fps())

