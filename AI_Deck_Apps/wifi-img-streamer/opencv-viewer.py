import argparse
import time
import socket,os,struct, time
import numpy as np

# Args for setting IP/port of AI-deck. Default settings are for when
# AI-deck is in AP mode.
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
args = parser.parse_args()

deck_port = args.p
deck_ip = args.n

print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((deck_ip, deck_port))
print("Socket connected")

imgdata = None
data_buffer = bytearray()

def rx_bytes(size):
    data = bytearray()
    while len(data) < size:
        data.extend(client_socket.recv(size-len(data)))
    return data

import cv2

start = time.time()
count = 0

while(1):
    # First get the info
    packetInfoRaw = rx_bytes(4)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

    imgHeader = rx_bytes(length - 2)
    [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

    if magic == 0xBC:

        # Now we start rx the image, this will be split up in packages of some size
        imgStream = bytearray()

        while len(imgStream) < size:
            packetInfoRaw = rx_bytes(4)
            [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
            chunk = rx_bytes(length - 2)
            imgStream.extend(chunk)
        
        count = count + 1
        meanTimePerImage = (time.time()-start) / count
        print(f"Frame Rate: {1/meanTimePerImage:.2f}\t Image Time: {meanTimePerImage:.4f}s")


        bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
        bayer_img.shape = (122*2,162*2)
        # color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
        cv2.namedWindow('Raw',cv2.WINDOW_NORMAL)
        cv2.imshow('Raw', bayer_img)
        if args.save:
            cv2.imwrite(f"stream_out/raw/img_{count:06d}.png", bayer_img)
        cv2.waitKey(1)

