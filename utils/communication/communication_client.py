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
import json
import os
import time

def communication_client():
	if(len(sys.argv) < 3):
		print 'Usage: python communication_client.py hostname port <message_file>'
		sys.exit()
	
	host = sys.argv[1]
	port = int(sys.argv[2])
	data = 0
	if(len(sys.argv) == 4):
		if(os.path.isfile(sys.argv[3])):
			with open(sys.argv[3]) as infile:
				data = json.load(infile)
			print 'Loaded ' + str(len(data['messages'])) + ' messages.'
		else:
			print "File " + sys.argv[3] + " not found."

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.settimeout(2)

	# connect to remote host
	try:
		s.connect((host, port))
	except:
		print 'Communication Client:: unable to connect to %s:%s', host, str(port)
		sys.exit()
	
	print 'Communication Client:: connected to remote host (' + host + ':' + str(port) + '). Ready to send'
	
	quit = False
	start_time = time.time()
	msg_index = 0
	while(not quit):
		socket_list = [sys.stdin, s]
		dt = time.time() - start_time
		
		# get the list of sockets that are readable
		ready_to_read, ready_to_write, in_error = select.select(socket_list, [], [], 0.1)

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
				# check if user entered a message
				msg = sys.stdin.readline().strip()
				print 'User entered message "' + msg + '" is being sent to server.'
				if(len(msg) != 0):
					s.send(msg)
		
		# check if there are stored messages in data
		if(data):
			if(msg_index < len(data['messages'])):
				message = data['messages'][msg_index]
				# check timing on the message
				if(dt > float(message['delay'])):
					s.send(message['message'])
					msg_index = msg_index + 1
					start_time = time.time()
					#print 'Sent ' + message['message'] + '. New index: ' + str(msg_index) + ' ' + str(dt) + ' ' + data['messages'][msg_index]['delay']
					print 'Sent ' + message['message']
				#else:
					#print 'Waiting until ' + str(dt) + ' exceeds ' + str(message['delay'])				
			else:
				print 'No messages (' + str(len(data['messages'])) + ')'
				exit(1)

if __name__ == "__main__":
	sys.exit(communication_client())