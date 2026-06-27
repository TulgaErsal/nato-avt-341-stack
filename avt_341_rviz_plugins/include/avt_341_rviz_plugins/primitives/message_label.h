#ifndef MESSAGE_LABEL_H
#define MESSAGE_LABEL_H

#ifndef Q_MOC_RUN
#include <QLabel>
#include <QString>
#include <QWidget>
#endif

#include <avt_341_rviz_plugins/dto.h>

namespace avt_341 {
namespace rviz_plugins {

/// A re-usable two-column message banner: a type-dependent icon in a tight
/// first column and word-wrapping text that fills the remaining width. The
/// icon stays vertically centered when the text wraps onto multiple lines.
class MessageLabel: public QWidget
{

Q_OBJECT
public:
    MessageLabel( MessageType type, const QString& text, QWidget* parent = nullptr );

protected:
    // QT Widgets
    QLabel* icon_label_;
    QLabel* text_label_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MESSAGE_LABEL_H
