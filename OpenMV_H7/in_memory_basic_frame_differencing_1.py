import sensor, image, pyb, os, time

TRIGGER_THRESHOLD = 5

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.QVGA) # or sensor.QQVGA (or others)
sensor.skip_frames(time = 2000) # Let new settings take affect.
clock = time.clock() # Tracks FPS.


extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)

print("About to save background image...")
sensor.skip_frames(time = 2000) # Give the user time to get ready.
extra_fb.replace(sensor.snapshot())
print("Saved background image - Now frame differencing!")

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    # Replace the image with the "abs(NEW-OLD)" frame difference.
    img.difference(extra_fb)



    print(clock.fps()) # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.
