import socket
import readchar
import ctypes
from pynput.keyboard import Key, Listener
from ctypes import *

class Commands(Structure):
	_pack_ = 0
	_fields_ = [("Mode", c_int),
                ("dir", c_int)]

def on_press(key):
	if key == '\x1b':
		print "Stop Listening"
		quit()

	if key == b'\x1b':
		print "Stop"
		quit()

	if key == 'z':
		print "Stop"
		quit()

	if(key == '\x1b[C'):
		Send(2,1)
	elif(key=='\x1b[D'):
		Send(2,3)
	elif(key=='\x1b[A'):
		Send(1,0)
	elif(key=='\x1b[B'):
		Send(1,2)
	elif(key==' '):
		Send(0,0)
	elif(key=='w'):
		Send(3,0)
	elif(key=='d'):
		Send(3,1)
	elif(key=='s'):
		Send(3,2)
	elif(key=='a'):
		Send(3,3)
	return True


def Send(a,b): #send string through udp
	UDP_IP = "192.168.43.153" #Replace with feather AP IP
	UDP_PORT = 5005
	#MESSAGE = "(10,0,1)" #(vel_d,theta_d,mode) #mode: 0=reset,1=normal

	print "UDP target IP:", UDP_IP
	print "UDP target port:", UDP_PORT
	print "message: (%d,%d)" % (a,b)

	Com = Commands(a,b)

	sock = socket.socket(socket.AF_INET, # Internet
						 socket.SOCK_DGRAM) # UDP
	sock.sendto(Com, (UDP_IP, UDP_PORT))

def main():
	i = 0
	#Connect2Port()
        while 1:
		i=i+1
		key = readchar.readkey()
		on_press(key)

if __name__ == '__main__':
    main()
