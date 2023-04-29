#ifndef AVT_341_TCP_SOCKET_PROXY_H
#define AVT_341_TCP_SOCKET_PROXY_H

#include <iostream>

namespace avt_341 {
  namespace communication {

    class TcpSocketClientBase{
      public:
        virtual bool connect() = 0;
        virtual int read_available(char *buffer, const int size) = 0;
        virtual int write(const char *buffer, const int size) = 0;
    };

    class NullTcpSocketClient : public TcpSocketClientBase{
      bool connect() override { return true; }
      int read_available(char *buffer, const int size) override { return 0; }
      int write(const char *buffer, const int size) override { return 0; }
    };

  }
}


#ifdef __linux__

#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>

#else

#include <SDKDDKVer.h>
#include <boost/asio.hpp>

#endif

namespace avt_341 {
  namespace communication {

    class TcpSocketClient : public TcpSocketClientBase {

    public:
      TcpSocketClient(const std::string & ip_address, int port);

      bool connect() override;
      int read_available(char *buffer, const int size) override;
      int write(const char *buffer, const int size) override;

    private:
      std::string ip_address_;
      int port_;

#ifdef __linux__
      int sockfd_;
      fd_set read_fds_;
      struct timeval timeout_;
      struct sockaddr_in serv_addr_;
      struct hostent *server_;
#else
      void read_handler(boost::system::error_code ec);
      boost::asio::io_service io_service_;
      boost::asio::ip::tcp::socket socket_;
      std::vector<std::vector<char>> read_buffers_;
#endif
    };

  }
}



#endif //AVT_341_TCP_SOCKET_PROXY_H
