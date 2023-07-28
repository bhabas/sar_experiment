import sensor, image, time, pyb

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to Grayscale
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QQVGA (160x120)
sensor.skip_frames(time=2000)        # Wait for settings take effect.

# Use a clock object to track the FPS.
clock = time.clock()

buffer_size = 3
frame_buffer = [None]*buffer_size

while(True):
    clock.tick()                    # Update the FPS clock.
    frame_buffer.pop(0)             # Remove oldest image
    frame_buffer.append(sensor.snapshot().copy()) # Capture a new image and add to the buffer

    # Now you can access previous frames from the frame_buffer
    # For instance, to process the previous frame (frame_buffer[-2]):
    if frame_buffer[-2] is not None: # Check if it exists
        # frame_buffer[-2].find_edges(image.EDGE_CANNY)
        pass

    # Print the FPS.
    print(clock.fps())
