#include <avt_341_rviz_plugins/primitives/message_label.h>

#include <QGridLayout>

#include <avt_341_rviz_plugins/primitives/icon_utils.h>

namespace
{

// Maps a message type to its icon file in resources/icons.
QString iconFileForType( avt_341::rviz_plugins::MessageType type )
{
    using avt_341::rviz_plugins::MessageType;
    switch ( type )
    {
        case MessageType::Success:
            return "msg_success.svg";
        case MessageType::Warning:
            return "msg_warn.svg";
        case MessageType::Error:
            return "msg_error.svg";
        case MessageType::Info:
        default:
            return "msg_info.svg";
    }
}

}  // namespace

namespace avt_341::rviz_plugins
{

MessageLabel::MessageLabel( MessageType type, const QString& text, QWidget* parent )
    : QWidget( parent )
{
    // Icon column: the icon for the message type, pinned to a fixed, tight size
    // (scaled to the display's DPI) so column 0 never reserves more width than
    // the icon itself.
    const int icon_size = scaledSize( 16, this );
    icon_label_ = new QLabel();
    icon_label_->setPixmap( renderSvg( iconFileForType( type ), 16, this ) );
    icon_label_->setFixedSize( icon_size, icon_size );

    // Text column: a single line sized to its content. A word-wrapping label
    // reports a narrow heuristic size hint, which makes a centered (content-
    // sized) message wrap prematurely, so it is left on one line here.
    text_label_ = new QLabel( text );

    // Two-column grid: icon hugging column 0, text in column 1, both centered
    // vertically.
    QGridLayout* layout = new QGridLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->setHorizontalSpacing( 6 );
    layout->addWidget( icon_label_, 0, 0, Qt::AlignVCenter );
    layout->addWidget( text_label_, 0, 1, Qt::AlignVCenter );
    setLayout( layout );
}

}
