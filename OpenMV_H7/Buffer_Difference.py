import sensor, image, pyb, os, time
from ulab import numpy as np
sensor.reset() # Initialize the camera sensor.

## SET SIZE AND PIXEL FORMAT
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.B64X64)

N_up = 64
N_vp = 64

## ROTATE IMAGE
sensor.set_transpose(True)
sensor.set_vflip(False)
sensor.set_hmirror(True)

## ALLOW SYSTEM TO UPDATE CAMERA SETTINGS
sensor.skip_frames(time = 1000) # Let new settings take affect.
clock = time.clock() # Tracks FPS.
sensor.set_auto_exposure(True, exposure_us=30_000) # make smaller to go faster

## CAPTURE INITIAL IMAGE
img_prev = sensor.snapshot().copy()

t_prev = time.time_ns()
t_curr = time.time_ns()
t_delta = t_curr - t_prev

K_up = [-1,  0,  1,
        -2,  0,  2,
        -1,  0,  1]

K_vp = [-1, -2, -1,
         0,  0,  0,
         1,  2,  1]
kernel_size = 1

while True:

    clock.tick()


    ## CAPTURE CURRENT IMAGE
    img_curr = sensor.snapshot()
    t_curr = time.time_ns()
    img_bytearray = img_curr.bytearray() # Convert image to bytestring

    img_G_tp = img_curr.copy()  # Create a copy of current image
    img_G_up = img_curr.copy()
    img_G_vp = img_curr.copy()
    img_G_rp = img_curr.copy()

    ## SUBTRACT CUR_IMG FROM PREV_IMG AND PERFORM GRAD CONVOLUTIONS
    img_G_tp.sub(img_prev)
    img_G_up.morph(kernel_size, K_up)
    img_G_vp.morph(kernel_size, K_up)


    G_tp = np.frombuffer(img_G_tp.bytearray(), dtype=np.uint8).reshape((N_up,N_vp))
    G_up = np.frombuffer(img_G_up.bytearray(), dtype=np.uint8).reshape((N_up,N_vp))
    G_vp = np.frombuffer(img_G_vp.bytearray(), dtype=np.uint8).reshape((N_up,N_vp))
    G_rp = np.zeros((N_up,N_vp),dtype=np.uint8)


    for u_p in range(0,N_up):
        for v_p in range(0,N_vp):
#            G_rp[u_p,v_p] = (2*u_p-64+1)*G_up[u_p,v_p] + (2*v_p-64+1)*G_vp[u_p,v_p]
            pass


#    for u_p in range(0,64):
#            for v_p in range(0,64):
#                pixel_val = (2*u_p - 64 + 1)*img_G_up.get_pixel(u_p,v_p) + (2*v_p - 64 + 1)*img_G_vp.get_pixel(u_p,v_p)
#                img_G_rp.set_pixel(u_p,v_p,pixel_val)

    t_delta = t_curr - t_prev
    print(f"t_delta: {t_delta/1_000_000:.3f} ms \t FPS_Avg: {clock.fps():.1f}")


    ## SET IMAGE TO BE SEND TO IDE BUFFER
    img_G_up.copy(copy_to_fb=True)

    # Now, we need to replace img_prev with img_current for the next iteration
    img_prev = img_curr
    t_prev = t_curr

#    time.sleep(0.1)


