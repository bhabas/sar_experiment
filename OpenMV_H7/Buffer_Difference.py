import sensor, image, pyb, os, time
sensor.reset() # Initialize the camera sensor.

## SET SIZE AND PIXEL FORMAT
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.B64X64)

## ROTATE IMAGE
sensor.set_transpose(True)
sensor.set_vflip(False)
sensor.set_hmirror(True)

## ALLOW SYSTEM TO UPDATE CAMERA SETTINGS
sensor.skip_frames(time = 1000) # Let new settings take affect.
clock = time.clock() # Tracks FPS.
##sensor.set_framerate(10)

## CAPTURE INITIAL IMAGE
img_prev = sensor.snapshot().copy()

t_prev = time.time_ns()
t_curr = time.time_ns()
t_delta = t_curr - t_prev

while True:

    clock.tick()


    ## CAPTURE CURRENT IMAGE
    img_curr = sensor.snapshot().copy()
    t_curr = time.time_ns()


    ## SUBTRACT CUR_IMG FROM PREV_IMG
    img_diff = img_curr.copy()  # Create a copy of current image
    img_diff.sub(img_prev)
    t_delta = t_curr - t_prev
    print(t_delta/1_000_000)
    print(clock.fps())
    print(clock.avg())


    ## SET IMAGE TO BE SEND TO IDE BUFFER
    img_diff.copy(copy_to_fb=True)

    # Now, we need to replace img_prev with img_current for the next iteration
    img_prev = img_curr
    t_prev = t_curr

#    time.sleep(0.1)


