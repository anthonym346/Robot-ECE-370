import socket
import ctypes
from ctypes import *

class Packet(Structure):
	_pack_ = 0
	_fields_ = [("odo", c_double*3),
                ("imu", c_double*6),
                ("heading", c_double)]

def Rec():
 
	UDP_IP = "" 
	UDP_PORT = 4242
	
	sock = socket.socket(socket.AF_INET, # Internet
						 socket.SOCK_DGRAM) # UDP
        sock.bind((UDP_IP, UDP_PORT))

	return sock

def main():
	i = 0
        sock = Rec()
	print "start"
        while 1:
		try:
			i=i+1
			# get keyboard input, waits until enter pressed
			sock.settimeout(30.0)
			data, addr = sock.recvfrom(sizeof(Packet))
			out = Packet.from_buffer_copy(data)
			print "received message: Odo(%d,%d,%d), Imu(%d,%d,%d,%d,%d,%d), Heading(%d)" % (out.odo[0],out.odo[1],out.odo[2], out.imu[0],out.imu[1],out.imu[2],out.imu[3],out.imu[4],out.imu[5],out.heading)

		except socket.timeout:
			socket.close()

if __name__ == '__main__':
    main()
