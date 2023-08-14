import sensor, image, pyb

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.B64X64)
sensor.skip_frames(time=2000)

import os
# List all files and directories in the root of the uSD card
entries = os.listdir('/Img_Dir')

for entry in entries:
    print(entry)

# Load the image from the SD card

i = 2
while True:

    if i > 191:
        i = 2
    img = image.Image(f"/Img_Dir/img_{i}.png", copy_to_fb = True)

    i += 1
