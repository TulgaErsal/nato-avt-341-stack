#ifndef ICON_HEADER_H
#define ICON_HEADER_H

#ifndef Q_MOC_RUN
#include <QLabel>
#include <QPixmap>
#include <QString>
#include <QWidget>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// A small header primitive showing an icon next to an emphasized title label
/// on a single horizontal line. Re-usable as a section header across the UI.
class IconHeader: public QWidget
{

Q_OBJECT
public:
    IconHeader( const QPixmap& icon, const QString& title,
                int icon_size = 32, QWidget* parent = nullptr );

protected:
    // QT Widgets
    QLabel* icon_label_;
    QLabel* title_label_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // ICON_HEADER_H
