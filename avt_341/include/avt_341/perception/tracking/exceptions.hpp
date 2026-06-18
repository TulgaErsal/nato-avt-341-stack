#include <stdexcept>

namespace avt_341 {
namespace perception {

class TransformException : public std::runtime_error {
  public:
    TransformException(char const* const message)
        : std::runtime_error(message) {}

    const char* what() const noexcept { return message_.c_str(); }

  private:
    std::string message_;
};


class ClusteringException : public std::runtime_error {
  public:
    ClusteringException(char const* const message)
        : std::runtime_error(message) {}

    const char* what() const noexcept { return message_.c_str(); }

  private:
    std::string message_;
};

class PCAException : public std::runtime_error {
  public:
    PCAException(char const* const message) : std::runtime_error(message) {}

    const char* what() const noexcept { return message_.c_str(); }

  private:
    std::string message_;
};

} // namespace perception
} // namespace avt_341