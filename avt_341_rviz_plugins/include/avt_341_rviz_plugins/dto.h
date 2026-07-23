#ifndef DTO_H
#define DTO_H

namespace avt_341 {
namespace rviz_plugins {

// Message severity used to pick the icon (and future styling) of a MessageLabel.
enum class MessageType
{
    Info,
    Success,
    Warning,
    Error
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // DTO_H
