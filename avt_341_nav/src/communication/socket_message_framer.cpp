#include "avt_341_nav/communication/socket_message_framer.h"

namespace avt_341_nav {
namespace communication {

std::vector<std::string> SocketMessageFramer::Feed(const std::string & data) {
    std::vector<std::string> messages;
    if(!delimited_ && data.find(kDelimiter) == std::string::npos) {
        if(!data.empty()) {
            messages.push_back(data);
        }
        return messages;
    }
    delimited_ = true;

    pending_ += data;
    size_t start = 0;
    size_t end;
    while((end = pending_.find(kDelimiter, start)) != std::string::npos) {
        if(end > start) {
            messages.push_back(pending_.substr(start, end - start));
        }
        start = end + 1;
    }
    pending_.erase(0, start);
    return messages;
}

}  // namespace communication
}  // namespace avt_341_nav
