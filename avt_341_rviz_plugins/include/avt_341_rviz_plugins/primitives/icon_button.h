#ifndef ICON_BUTTON_H
#define ICON_BUTTON_H

#ifndef Q_MOC_RUN
#include <QIcon>
#include <QPushButton>
#include <QString>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// A compact, icon-only push button with a tooltip. Re-usable for toolbars
/// and vertical button strips.
class IconButton: public QPushButton
{

Q_OBJECT
public:
    IconButton( const QIcon& icon, const QString& tooltip, QWidget* parent = nullptr );

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // ICON_BUTTON_H
