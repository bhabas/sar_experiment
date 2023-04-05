"""
Simple example that connects to the first Crazyflie found, Sends and
receive CRTP packets

The protocol is:
 - 3 floats are send, x, y and z
 - The Crazyflie sends back the sum as one float
"""

import time
from threading import Thread

import struct
import cflib
from cflib.crazyflie import Crazyflie

class CRTP_Test:

    def __init__(self, link_uri):

        self._cf = Crazyflie()
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.appchannel.packet_received.add_callback(self._crtp_packet_received)

        self._cf.open_link(link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._test_crtp).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)


    def _test_crtp(self):

        for i in range(10):
            (x,y,z) = (i,i+1,i+2)
            data = struct.pack("<fff",x,y,z)
            self._cf.appchannel.send_packet(data)
            print(f"Sent x: {x} y:{y} z:{z}")
            time.sleep(0.5)

        self._cf.close_link()

    def _crtp_packet_received(self,data):
        (sum, ) = struct.unpack("<f",data)
        print(f"Received sum: {sum}")


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    uri = "radio://0/80/2M/E7E7E7E701"
    CRTP_Test(uri)