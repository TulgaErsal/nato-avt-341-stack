#ifndef SETUP_COMPONENT_H
#define SETUP_COMPONENT_H

#ifndef Q_MOC_RUN
#include <QStringList>
#include <QWidget>
#endif

namespace avt_341 {
namespace rviz_plugins {

class IconHeader;
class EntityListComponent;

class SetupComponent: public QWidget
{

Q_OBJECT
public:
    SetupComponent( QWidget* parent = nullptr );

Q_SIGNALS:
    // Emitted whenever the managed vehicle list changes (add or remove).
    void vehiclesChanged( const QStringList& vehicles );

protected:
    // QT Widgets
    IconHeader* header_;
    EntityListComponent* vehicles_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // SETUP_COMPONENT_H
