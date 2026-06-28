#ifndef SETUP_COMPONENT_H
#define SETUP_COMPONENT_H

#ifndef Q_MOC_RUN
#include <QColor>
#include <QString>
#include <QStringList>
#include <QWidget>

#include <avt_341_rviz_plugins/components/topic_config.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

class IconHeader;
class VehicleTableComponent;
class TopicConfigComponent;

class SetupComponent: public QWidget
{

Q_OBJECT
public:
    SetupComponent( QWidget* parent = nullptr );

    // The current topic configuration edited in the Topics section.
    const TopicConfig& topicConfig() const;

    // The current vehicle ids, in display order.
    QStringList vehicles() const;

    // Restore the vehicle list / topic configuration (e.g. from saved state).
    void setVehicles( const QStringList& vehicles );
    void setTopicConfig( const TopicConfig& config );

    // Mirror a vehicle's live Nav State / Compute status into the vehicle table.
    void setVehicleNavState( const QString& vehicle_id, const QString& text,
                             const QColor& color );
    void setVehicleComputeHealth( const QString& vehicle_id, bool healthy );

Q_SIGNALS:
    // Emitted whenever the managed vehicle list changes (add or remove).
    void vehiclesChanged( const QStringList& vehicles );

    // Emitted when a configurable topic changes; group identifies which tab's
    // components consume it.
    void topicConfigChanged( avt_341::rviz_plugins::TopicGroup group );

protected:
    // QT Widgets
    IconHeader* header_;
    VehicleTableComponent* vehicles_;
    TopicConfigComponent* topics_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // SETUP_COMPONENT_H
