#ifndef AVT_341_SOCKET_MESSAGE_FRAMER_H
#define AVT_341_SOCKET_MESSAGE_FRAMER_H

#include <string>
#include <vector>

namespace avt_341_nav {
namespace communication {

/**
 * @brief Splits the socket byte stream into serialized messages. Messages are terminated by '\n'.
 * Until the first '\n' is seen on a connection, each read is taken as one whole message, so peers that
 * send unterminated messages (older comm nodes) keep working.
 */
class SocketMessageFramer {
  public:
    static constexpr char kDelimiter = '\n';

    /** @brief Add received bytes and return the messages they complete. */
    std::vector<std::string> Feed(const std::string & data);

    /** @brief Message as it is written to the socket. */
    static std::string Frame(const std::string & message) { return message + kDelimiter; }

  private:
    std::string pending_;
    bool delimited_ = false;
};

}  // namespace communication
}  // namespace avt_341_nav

#endif  // AVT_341_SOCKET_MESSAGE_FRAMER_H
