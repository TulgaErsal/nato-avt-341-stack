#ifndef STATUS_STYLE_H
#define STATUS_STYLE_H

#ifndef Q_MOC_RUN
#include <QColor>
#include <QString>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// Shared status palette: a single definition of the colors used as status
/// badges / alert highlights across the components and field primitives.
namespace status_colors {
const QColor kGreen( 40, 167, 69 );    // ok / active / full tracking
const QColor kOrange( 230, 126, 34 );  // warning / startup / partial
const QColor kRed( 220, 53, 69 );      // error / lost / below threshold
const QColor kGray( 108, 117, 125 );   // inactive / idle / unknown / none
}  // namespace status_colors

/// Stylesheet for a status "badge" label: the given background color, white
/// text, small padding and rounded corners. \p h_padding is the horizontal
/// padding in pixels (most callers use 8; the matrix cells use 6).
inline QString statusBadgeStyleSheet( const QColor& color, int h_padding = 8 )
{
    return QString( "background-color: %1; color: white; padding: 2px %2px; "
                    "border-radius: 2px;" )
        .arg( color.name() )
        .arg( h_padding );
}

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // STATUS_STYLE_H
