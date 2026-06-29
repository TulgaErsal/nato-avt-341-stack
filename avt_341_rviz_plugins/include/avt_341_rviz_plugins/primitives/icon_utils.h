#ifndef ICON_UTILS_H
#define ICON_UTILS_H

#ifndef Q_MOC_RUN
#include <QIcon>
#include <QPixmap>
#include <QString>
#include <QtGlobal>
#endif

class QWidget;

namespace avt_341 {
namespace rviz_plugins {

/// DPI-aware icon helpers.
///
/// The panel's pixel sizes were tuned on a 96-DPI display. These helpers scale
/// those base sizes to the current monitor so icons keep the same *relative*
/// screen size at any DPI, and rasterize SVGs crisply at the target resolution.
/// `ref` is the widget the icon lives on (used to read that widget's screen);
/// when null the primary screen is used.

/// UI scale factor relative to the 96-DPI baseline (1.0 at 96 DPI).
qreal uiScale( const QWidget* ref = nullptr );

/// `base_px` scaled to the current display and rounded to whole logical pixels.
/// Feed this to setFixedSize()/setIconSize().
int scaledSize( int base_px, const QWidget* ref = nullptr );

/// Renders resources/icons/<file_name> crisply at the DPI-scaled size. The
/// returned pixmap carries the screen's devicePixelRatio so it occupies
/// scaledSize(base_px) logical pixels while staying sharp.
QPixmap renderSvg( const QString& file_name, int base_px, const QWidget* ref = nullptr );

/// DPI-aware rescale of an already-rasterized pixmap (e.g. the PNG logo, which
/// QSvgRenderer cannot handle). Same sizing contract as renderSvg().
QPixmap scalePixmap( const QPixmap& src, int base_px, const QWidget* ref = nullptr );

/// QIcon wrapper around renderSvg() for QAbstractButton icons.
QIcon iconFromSvg( const QString& file_name, int base_px, const QWidget* ref = nullptr );

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // ICON_UTILS_H
