import sensor, image, pyb, os, time
from ulab import numpy as np
from ulab import scipy as spy
sensor.reset() # Initialize the camera sensor.

## SET SIZE AND PIXEL FORMAT
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.B64X64)

N_up = 10
N_vp = 10
IW = 0.00368
f = 0.00304

## ROTATE IMAGE
sensor.set_transpose(True)
sensor.set_vflip(False)
sensor.set_hmirror(True)

## ALLOW SYSTEM TO UPDATE CAMERA SETTINGS
sensor.skip_frames(time = 1000) # Let new settings take affect.
clock = time.clock() # Tracks FPS.
sensor.set_auto_exposure(True, exposure_us=30_000) # make smaller to go faster

### CAPTURE INITIAL IMAGE
#img_prev = sensor.snapshot().copy()

#t_prev = time.time_ns()
#t_curr = time.time_ns()
#t_delta = t_curr - t_prev

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

u_p_dist = np.zeros((N_vp,N_up),dtype=np.int16)
v_p_dist = np.zeros((N_vp,N_up),dtype=np.int16)

img_arr_cur = [
        5,8,4,6,1,8,7,0,7,0,
        0,0,0,5,0,0,4,2,4,2,
        2,4,8,8,3,1,0,1,0,1,
        0,6,0,8,2,9,8,5,8,5,
        7,1,9,6,1,5,5,3,5,3,
        8,2,0,3,1,3,8,1,8,1,
        3,0,8,8,0,7,6,1,6,1,
        8,0,8,1,9,9,3,5,3,5,
        0,6,0,8,2,9,8,5,8,5,
        7,1,9,6,1,5,5,3,5,3,
        ]
img_arr_cur = np.array(img_arr_cur).reshape((10,10))

img_arr_prev = [
        9,2,2,2,9,7,6,8,6,8,
        8,1,3,8,2,2,2,9,2,9,
        2,6,4,4,1,5,8,9,8,9,
        2,6,1,0,5,3,3,4,3,4,
        8,5,4,2,9,3,9,8,9,8,
        8,2,9,3,0,7,3,2,3,2,
        0,4,3,3,8,0,4,6,4,6,
        1,0,8,7,6,8,5,7,5,7,
        2,6,1,0,5,3,3,4,3,4,
        8,5,4,2,9,3,9,8,9,8,
        ]
img_arr_prev = np.array(img_arr_prev).reshape((10,10))

# Create a blank 10x10 image
img_prev = image.Image(10, 10, sensor.GRAYSCALE,copy_to_fb = False)
img_curr = image.Image(10, 10, sensor.GRAYSCALE,copy_to_fb = True)

## WRITE PREVIOUS IMAGE
for x in range(10):
    for y in range(10):
        val = int(img_arr_prev[x,y])
        img_prev.set_pixel(x, y, val)

## WRITE CURRENT IMAGE
for x in range(10):
    for y in range(10):
        val = int(img_arr_cur[x,y])
        img_curr.set_pixel(x, y, val)

img_arr_prev = np.array(img_arr_prev).reshape((10,10))



#while True:



#sensor.flush()  # Ensure framebuffer is clean

img_G_tp = img_curr.copy()  # Create a copy of current image
img_G_up = img_curr.copy()
img_G_vp = img_curr.copy()

#img_G_tp.sub(img_prev)

img_arr_prev = np.frombuffer(img_prev.bytearray(), dtype=np.int8).reshape((N_up,N_vp))
img_arr_cur = np.frombuffer(img_curr.bytearray(), dtype=np.int8).reshape((N_up,N_vp))



## WRITE CURRENT IMAGE
#for x in range(10):
#    for y in range(10):
#        print(img_G_tp.get_pixel(x, y),end=' ')
#    print()

print(img_arr_cur-img_arr_prev)

print("i")
#    t_delta = 0.01
#    time.sleep(0.1)

#    ## SUBTRACT CUR_IMG FROM PREV_IMG AND PERFORM GRAD CONVOLUTIONS
#    img_G_tp.sub(img_prev)
#    img_G_up.morph(kernel_size, K_up)
#    img_G_vp.morph(kernel_size, K_up)

### CONVERT IMAGE BUFFERS TO ARRAYS
#img_bytearray = img_curr.bytearray() # Convert image to bytestring
#G_tp = np.frombuffer(img_G_tp.bytearray(), dtype=np.uint8).reshape((N_up,N_vp))
##    G_up = np.frombuffer(img_G_up.bytearray(), dtype=np.uint8).reshape((N_up,N_vp))
##    G_vp = np.frombuffer(img_G_vp.bytearray(), dtype=np.uint8).reshape((N_up,N_vp))
##    G_rp = np.zeros((N_up,N_vp),dtype=np.uint8)
##    G_rp = u_p_dist*G_up + v_p_dist*G_vp

#print(G_tp)
