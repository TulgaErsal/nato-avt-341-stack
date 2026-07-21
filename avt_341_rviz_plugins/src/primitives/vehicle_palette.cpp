#include <avt_341_rviz_plugins/primitives/vehicle_palette.h>

#include <QPainter>
#include <QPen>
#include <QRectF>

namespace avt_341::rviz_plugins
{

const QVector<VehicleColor>& vehiclePalette()
{
    // First three are red / blue / green as required; the remaining seven are
    // chosen to stay visually distinct from one another and from the first three.
    static const QVector<VehicleColor> palette = {
        { "Red",     QColor( 0xE5, 0x39, 0x35 ) },
        { "Blue",    QColor( 0x1E, 0x88, 0xE5 ) },
        { "Green",   QColor( 0x43, 0xA0, 0x47 ) },
        { "Orange",  QColor( 0xFB, 0x8C, 0x00 ) },
        { "Purple",  QColor( 0x8E, 0x24, 0xAA ) },
        { "Teal",    QColor( 0x00, 0x89, 0x7B ) },
        { "Magenta", QColor( 0xD8, 0x1B, 0x60 ) },
        { "Yellow",  QColor( 0xF9, 0xA8, 0x25 ) },
        { "Brown",   QColor( 0x6D, 0x4C, 0x41 ) },
        { "Cyan",    QColor( 0x00, 0xAC, 0xC1 ) },
    };
    return palette;
}

QColor vehicleColorForIndex( int index )
{
    const QVector<VehicleColor>& palette = vehiclePalette();
    const int n = palette.size();
    // Wrap with a positive modulo so negative indices stay in range.
    const int i = ( ( index % n ) + n ) % n;
    return palette.at( i ).color;
}

QPixmap makeColorSwatch( const QColor& color, int size )
{
    QPixmap pixmap( size, size );
    pixmap.fill( Qt::transparent );

    QPainter painter( &pixmap );
    painter.setRenderHint( QPainter::Antialiasing, true );
    painter.setPen( QPen( color.darker( 135 ), 1 ) );
    painter.setBrush( color );
    // Inset by half a pixel so the 1px border isn't clipped at the edges.
    painter.drawRoundedRect( QRectF( 0.5, 0.5, size - 1.0, size - 1.0 ), 2.0, 2.0 );
    return pixmap;
}

}
