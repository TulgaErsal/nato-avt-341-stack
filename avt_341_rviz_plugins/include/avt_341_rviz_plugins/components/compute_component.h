#ifndef COMPUTE_COMPONENT_H
#define COMPUTE_COMPONENT_H

#ifndef Q_MOC_RUN
#include <QLabel>
#include <QString>
#include <QWidget>
#endif

namespace avt_341 {
namespace rviz_plugins {

class ComputeComponent: public QWidget
{

Q_OBJECT
public:
    ComputeComponent( const QString& vehicle_id, QWidget* parent = nullptr );

protected:
    QString vehicle_id_;

    // QT Widgets
    QLabel* placeholder_label_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // COMPUTE_COMPONENT_H
