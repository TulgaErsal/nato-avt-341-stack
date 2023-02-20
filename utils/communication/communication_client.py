#
#file communication_client.py
#
# Simple python client 
# 
# author Daniel Carruth
#
# contact dwc2@cavs.msstate.edu
#
# date 2/19/2023
#
import sys
import socket
import select

def communication_client():
	if(len(sys.argv) < 3):
		print 'Usage: python communication_client.py hostname port'
		sys.exit()
	
	host = sys.argv[1]
	port = int(sys.argv[2])

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.settimeout(2)

	# connect to remote host
	try:
		s.connect((host, port))
	except:
		print 'Communication Client:: unable to connect to %s:%s', host, str(port)
		sys.exit()
	
	print 'Communication Client:: connected to remote host (%s:%s). Ready to send', host, str(port)
	
	quit = False
	while(not quit):
		socket_list = [sys.stdin, s]

		# get the list of sockets that are readable
		ready_to_read, ready_to_write, in_error = select.select(socket_list, [], [])

		for sock in ready_to_read:
			if sock == s:
				# incoming message from server, s
				data = sock.recv(4096)
				if not data:
					print 'Communication Client: disconnected from server'
					sys.exit()
				else:
					print 'Communication Client received: ' + str(data)
			else:
				# user entered a message
				msg = sys.stdin.readline().strip()
				s.send(msg)

if __name__ == "__main__":
	sys.exit(communication_client())