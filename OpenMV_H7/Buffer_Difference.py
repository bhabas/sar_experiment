import sensor, image, pyb, os, time
from ulab import numpy as np
from ulab import scipy as spy
sensor.reset() # Initialize the camera sensor.

## SET SIZE AND PIXEL FORMAT
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.B64X64)

N_up = 64
N_vp = 64
IW = 1.152e-3
f = 0.66e-3

## ROTATE IMAGE
sensor.set_transpose(True)
sensor.set_vflip(False)
sensor.set_hmirror(True)

## ALLOW SYSTEM TO UPDATE CAMERA SETTINGS
sensor.skip_frames(time = 1000) # Let new settings take affect.
clock = time.clock() # Tracks FPS.
sensor.set_auto_exposure(True, exposure_us=10_000) # make smaller to go faster

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

u_p_dist = np.zeros((N_vp,N_up),dtype=np.int16)
v_p_dist = np.zeros((N_vp,N_up),dtype=np.int16)


for u_p in range(0,N_up):
    u_p_dist[:,u_p] = (2*u_p-N_up+1)

for v_p in range(0,N_vp):
    v_p_dist[v_p,:] = (2*v_p-N_vp+1)

while True:

    clock.tick()

    ## CAPTURE CURRENT IMAGE
    img_curr = sensor.snapshot()
    t_curr = time.time_ns()
    t_delta = t_curr - t_prev

    img_G_tp = img_curr.copy()  # Create a copy of current image
    img_G_up = img_curr.copy()
    img_G_vp = img_curr.copy()

    ## SUBTRACT CUR_IMG FROM PREV_IMG AND PERFORM GRAD CONVOLUTIONS
    img_G_tp.sub(img_prev)
    img_G_up.morph(kernel_size, K_up)
    img_G_vp.morph(kernel_size, K_up)

    ## CONVERT IMAGE BUFFERS TO ARRAYS
    img_bytearray = img_curr.bytearray() # Convert image to bytestring
    G_tp = np.frombuffer(img_G_tp.bytearray(), dtype=np.uint8).reshape((N_up,N_vp))
    G_up = np.frombuffer(img_G_up.bytearray(), dtype=np.uint8).reshape((N_up,N_vp))
    G_vp = np.frombuffer(img_G_vp.bytearray(), dtype=np.uint8).reshape((N_up,N_vp))
    G_rp = np.zeros((N_up,N_vp),dtype=np.uint8)
    G_rp = u_p_dist*G_up + v_p_dist*G_vp

    ## CALC DOT PRODUCTS
    G_vp_vp = np.sum(G_vp*G_vp)
    G_vp_up = np.sum(G_vp*G_up)
    G_vp_rp = np.sum(G_vp*G_rp)
    G_vp_tp = np.sum(G_vp*G_tp)

    G_up_up = np.sum(G_up*G_up)
    G_up_rp = np.sum(G_up*G_rp)
    G_up_tp = np.sum(G_up*G_tp)

    G_rp_rp = np.sum(G_rp*G_rp)
    G_rp_tp = np.sum(G_rp*G_tp)

    ## SOLVE LEAST SQUARES PROBLEM
    A = np.array([
        [G_vp_vp, -G_vp_up, -IW/(2*N_up*f)*G_vp_rp],
        [G_vp_up, -G_up_up, -IW/(2*N_up*f)*G_up_rp],
        [G_vp_rp, -G_up_rp, -IW/(2*N_up*f)*G_rp_rp]
    ])

    b = np.array([
        [G_vp_tp],
        [G_up_tp],
        [G_rp_tp]
    ])*(8*IW/(f*N_up*t_delta))

    A = np.array([[25, 15, -5], [15, 18,  0], [-5,  0, 11]])
    b = np.array([9,-3,7])


    ## SOLVE b VIA PSEUDO-INVERSE
    try:
        A = np.linalg.cholesky(A)
        x = spy.linalg.cho_solve(A, b)
#        print(x[:])
    except:
        print("Singular Matrix")

    print(f"t_delta: {t_delta/1_000_000:.3f} ms \t FPS_Avg: {clock.fps():.1f}")

    # Now, we need to replace img_prev with img_current for the next iteration
    img_prev = img_curr
    t_prev = t_curr

##    time.sleep(0.1)


