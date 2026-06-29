#ifndef VEHICLE_PALETTE_H
#define VEHICLE_PALETTE_H

#ifndef Q_MOC_RUN
#include <QColor>
#include <QPixmap>
#include <QString>
#include <QVector>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// A named entry in the vehicle label-color palette.
struct VehicleColor
{
    QString name;
    QColor color;
};

/// The fixed palette of distinguishable vehicle label colors. A vehicle's index
/// selects its default color via modulo, so the list never runs out of colors.
/// The first three entries are red, blue and green; ten colors total.
const QVector<VehicleColor>& vehiclePalette();

/// The default color for the vehicle at `index`, looked up as `index` modulo the
/// palette size (handles negative indices).
QColor vehicleColorForIndex( int index );

/// Renders a small filled, rounded color square with a subtle border. Used to
/// tag a vehicle with its label color in the Setup table and accordion headers.
QPixmap makeColorSwatch( const QColor& color, int size = 12 );

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // VEHICLE_PALETTE_H
