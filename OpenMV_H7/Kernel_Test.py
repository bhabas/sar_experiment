import sensor, image, time

# Init the sensor.
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.QQVGA) # or whatever size
sensor.skip_frames(time = 1000)

#sensor.set_transpose(True)
#sensor.set_vflip(False)
#sensor.set_hmirror(True)

# This is a simple edge detection kernel.
edge_detection_kernel = [-1, -1, -1,
                         -1,  8, -1,
                         -1, -1, -1]

# This is the kernel size, it should be odd, like 1, 3, 5, 7, etc.
kernel_size = 1
clock = time.clock()

sensor.set_auto_exposure(True, exposure_us=20000) # make smaller to go faster

t_start = time.ticks_ms()
len_cor_val = 0.5

while(True):
    clock.tick()
    img = sensor.snapshot()

    if time.ticks_diff(time.ticks_ms(),t_start) > 1000:
        t_start = time.ticks_ms()
        len_cor_val += 0.25
        print(len_cor_val)


    img.lens_corr(len_cor_val)

    # Apply morph operation with the edge detection kernel.
#    img.morph(kernel_size, edge_detection_kernel)
    # Apply gaussian operation with the specified kernel size.
#    img.gaussian(kernel_size)
#    print(clock.fps())
    # Display the result.
    sensor.flush()
