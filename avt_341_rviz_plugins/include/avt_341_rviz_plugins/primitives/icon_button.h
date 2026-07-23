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
    /// Loads resources/icons/<svg_file_name> and renders it crisply at the
    /// current display DPI. Preferred constructor for the package's SVG icons.
    IconButton( const QString& svg_file_name, const QString& tooltip,
                QWidget* parent = nullptr );

    /// Pre-built icon variant. DPI-scales the button geometry; the icon itself
    /// must already be sized appropriately by the caller.
    IconButton( const QIcon& icon, const QString& tooltip, QWidget* parent = nullptr );

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // ICON_BUTTON_H
