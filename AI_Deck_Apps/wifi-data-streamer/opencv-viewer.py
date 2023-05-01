#  Demo for showing streamed JPEG images from the AI-deck example.
#
#  By default this demo connects to the IP of the AI-deck example when in
#  Access point mode.
#
#  The demo works by opening a socket to the AI-deck, downloads a stream of
#  JPEG images and looks for start/end-of-frame for the streamed JPEG images.
#  Once an image has been fully downloaded it's rendered in the UI.
#
#  Note that the demo firmware is continously streaming JPEG files so a single
#  JPEG image is taken from the stream using the JPEG start-of-frame (0xFF 0xD8)
#  and the end-of-frame (0xFF 0xD9).

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


## INITIALIZE TCP CONNECTION (RECEIVE AND ACKNOWLEDGE EACH DATA PACKET)
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

    ## RECIEVE AND PARSE DATA PACKET HEADER
    packetInfoRaw = rx_bytes(4)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

    ## RECIEVE REMAINDER OF DATA PACKET
    imgHeader = rx_bytes(length - 2)
    [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

    if magic == 0xBC: ## VALIDATE LOCATION IS SYNCED BETWEEN PACKETS AND IMAGE DATA

        ## START RECIEVING IMAGE WHICH IS SPLIT IN MULTIPLE PACKETS
        imgStream = bytearray()

        while len(imgStream) < size:
            ## PROCESS CPX HEADER
            packetInfoRaw = rx_bytes(4)
            [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)

            ## PROCESS DATA
            chunk = rx_bytes(length - 2)
            imgStream.extend(chunk)
        
        count = count + 1
        meanTimePerImage = (time.time()-start) / count
        print(f"Frame Rate: {1/meanTimePerImage:.2f}\t Image Time: {meanTimePerImage:.4f}s")


        bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
        bayer_img.shape = (50,50)
        cv2.imshow('Raw', bayer_img)
        if args.save:
            cv2.imwrite(f"stream_out/raw/img_{count:06d}.png", bayer_img)
        cv2.waitKey(1)
