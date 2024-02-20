def compressXY(x, y):
    # Import necessary module
    import struct

    # Convert floats to ints offset by UINT16_MAX / 2.0
    # Note: Python doesn't have fixed-size integer types by default, so we'll handle overflow manually
    UINT16_MAX = 65535
    xnew = int(x * 1000.0 + 32767.0)
    ynew = int(y * 1000.0 + 32767.0)

    # Clip ranges of values
    xnew = min(max(xnew, 0), UINT16_MAX)
    ynew = min(max(ynew, 0), UINT16_MAX)

    # Combine xnew and ynew into a single 32-bit value
    # Use struct to ensure the result is treated as an unsigned 32-bit integer
    xy = (xnew << 16) | ynew

    # Ensuring xy is treated as uint32, though Python's int is unbounded
    # The following step ensures we're simulating a wrap-around effect as in fixed-size integers
    xy = xy & 0xFFFFFFFF

    # For strict uint32 behavior, you could use struct to pack and then unpack the integer
    # This step is not strictly necessary for most Python use cases, but shown here for completeness
    xy_packed = struct.pack('>I', xy)  # Pack as big-endian unsigned int
    xy_unpacked, = struct.unpack('>I', xy_packed)  # Unpack, ensuring uint32 behavior

    return xy_unpacked

def decompressXY(xy):
    # Extract the x and y components from the 32-bit integer
    xd = (xy >> 16) & 0xFFFF  # Shift right to get the x value and mask to get the lower 16 bits
    yd = xy & 0xFFFF  # Mask to get the lower 16 bits for y

    # Convert to floats and adjust
    x = (xd - 32767.0) * 1e-3
    y = (yd - 32767.0) * 1e-3

    return x, y


# Example usage
x = 1024/2*0.001
y = 1024/2*0.001
compressed_value = compressXY(x, y)
print(f"Compressed Value: {compressed_value}")

compressed_value = -2146664425
x, y = decompressXY(compressed_value)
print(f"Decompressed X: {x}, Y: {y}")


