#
#file communication_server.py
#
# Simple python echo server 
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

HOST = ''
SOCKET_LIST = []
RECV_BUFFER = 4096
PORT = 9000

def communication_server():
	# set up socket
	server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	server_socket.bind((HOST, PORT))
	server_socket.listen(10)

	# add server_socket to list of connections
	SOCKET_LIST.append(server_socket)

	print "Communication Server started on port " + str(PORT)

	quit = False
	while(not quit):
		# get the list of connections to be read
		ready_to_read, ready_to_write, in_error = select.select(SOCKET_LIST, [], [], 0)

		for sock in ready_to_read:
			# new connection
			if sock == server_socket:
				sockfd, addr = server_socket.accept()
				SOCKET_LIST.append(sockfd)
				print "Communication Server: Client (%s %s) connected" % addr
				
			# message received
			else: 
				# process data from client
				try:
					# receiving data 
					data = sock.recv(RECV_BUFFER)
					if data:
						# received something
						#message = '[' + str(sock.getpeername()) + '] ' + data
						message = data
						broadcast(server_socket, sock, message)
						print "Communication Server broadcasting: " + str(message)
					else:
						# broken socket
						if sock in SOCKET_LIST:
							SOCKET_LIST.remove(sock)
							print "Communication Server kicking " + str(sock.getpeername())
				# exception
				except:
					print "Communication Server: Client (%s, %s) is offline\n" % addr
					continue
	print "Communication Server shutting down."				
	server_socket.close()

# broadcast chat messages to all connected clients
def broadcast (server_socket, sock, message):
    for socket in SOCKET_LIST:
        # send the message only to peer
        if socket != server_socket and socket != sock :
            try :
                socket.send(message)
            except :
                # broken socket connection
                socket.close()
                # broken socket, remove it
                if socket in SOCKET_LIST:
                    SOCKET_LIST.remove(socket)
 
if __name__ == "__main__":
    sys.exit(communication_server()) 
