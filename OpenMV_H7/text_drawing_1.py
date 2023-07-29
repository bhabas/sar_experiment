# Text Drawing
#
# This example shows off drawing text on the OpenMV Cam.

import sensor, image, time, pyb

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # or GRAYSCALE...
sensor.set_framesize(sensor.QVGA) # or QQVGA...
sensor.set_framerate(10)
sensor.skip_frames(time = 2000)
clock = time.clock()

while(True):
    clock.tick()

    img = sensor.snapshot()


    x = img.width()//2
    y = img.height()//2

    # If the first argument is a scaler then this method expects
    # to see x, y, and text. Otherwise, it expects a (x,y,text) tuple.

    # Character and string rotation can be done at 0, 90, 180, 270, and etc. degrees.
    img.draw_string(x, y, f"Hello World! \t {clock.fps():.1f}", color = 255, scale = 2, mono_space = False,
                    char_rotation = 0, char_hmirror = False, char_vflip = False,
                    string_rotation = 0, string_hmirror = False, string_vflip = False)

#    print(clock.fps())
