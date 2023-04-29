#include "avt_341/communication/tcp_socket_proxy.h"

namespace avt_341 {
  namespace communication {

#ifdef __linux__

TcpSocketClient::TcpSocketClient(const std::string & ip_address, int port)
  : ip_address_(ip_address), port_(port) {

}


bool TcpSocketClient::connect(){
    std::cout << "Connecting to host:" << hostname.c_str() << "port: " << port << std::endl;
    // Create the socket
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 9)
        std::cout << "Error opening socket" << std::endl
    server = gethostbyname(hostname.c_str());
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
    serv_addr.sin_port = htons(port);

    // connect to the server
    return connect(sockfd_,(struct sockaddr *)&serv_addr, sizeof(serv_addr)) >= 0;
}

int TcpSocketClient::write(const char *buffer, const int size){
  return write(sockfd_, buffer, size);
}

int TcpSocketClient::read_available(char *buffer, const int size){
  FD_ZERO(&read_fds_);
  FD_SET(sockfd_, &read_fds_);
  timeout_.tv_sec = 0;
  timeout_.tv_usec = 0;

  // check for message from the server
  int ready = select(sockfd_ + 1, &read_fds_, NULL, NULL, &timeout_);
  if(ready > 0 && FD_ISSET(sockfd_, &read_fds_)) {
      // read the socket
      return read(sockfd_, buffer, size);
  }
  return -1;
}

#else

    TcpSocketClient::TcpSocketClient(const std::string & ip_address, int port)
        : ip_address_(ip_address), port_(port), io_service_(), socket_(io_service_) {

    }

    bool TcpSocketClient::connect(){
      try{
        std::cout << "TcpSocketClient::connect() attempting to connect to " << ip_address_ << ":" << port_ << std::endl;
        socket_.connect(boost::asio::ip::tcp::endpoint( boost::asio::ip::address::from_string(ip_address_ == "localhost" ? "127.0.0.1" : ip_address_), port_ ));
      }
      catch (std::exception& e){
        std::cout << "TcpSocketClient::connect() failed: " << e.what() << std::endl;
        return false;
      }
      std::cout << "TcpSocketClient::connect() connection success: " << ip_address_ << ":" << port_ << std::endl;
      return true;
    }

    int TcpSocketClient::write(const char *buffer, const int size){
      boost::system::error_code error;
      boost::asio::write(socket_, boost::asio::buffer(std::string(buffer)), error);
      if(!error) {
        return 0;
      }
      std::cout << "write error: " << error.message() << std::endl;
      return -1;
    }

    int TcpSocketClient::read_available(char *buffer, const int size) {
      boost::system::error_code error;
      const auto n_available = socket_.available();
      if(n_available > 0) {
        boost::asio::streambuf receive_buffer;
        const auto read_size = std::min(size, static_cast<int>(n_available));
        boost::asio::read(socket_, receive_buffer, boost::asio::transfer_exactly(read_size), error);
        const char* data = boost::asio::buffer_cast<const char*>(receive_buffer.data());
        const auto n_data = receive_buffer.size();
        std::copy(data, data + n_data, buffer);
        return static_cast<int>(n_data);
      }
      return -1;
    }

    void TcpSocketClient::read_handler(boost::system::error_code ec)
    {
      if (!ec)
      {
        std::vector<char> buf(socket_.available());
        socket_.read_some(boost::asio::buffer(buf));
      }
    }

#endif

  }
}