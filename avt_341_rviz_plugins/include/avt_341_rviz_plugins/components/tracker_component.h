#ifndef TRACKER_COMPONENT_H
#define TRACKER_COMPONENT_H

#ifndef Q_MOC_RUN
#include <QLabel>
#include <QString>
#include <QWidget>
#endif

namespace avt_341 {
namespace rviz_plugins {

class TrackerComponent: public QWidget
{

Q_OBJECT
public:
    TrackerComponent( const QString& vehicle_id, QWidget* parent = nullptr );

protected:
    QString vehicle_id_;

    // QT Widgets
    QLabel* placeholder_label_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // TRACKER_COMPONENT_H
