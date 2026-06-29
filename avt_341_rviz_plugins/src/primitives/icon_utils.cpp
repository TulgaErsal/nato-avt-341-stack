#include <avt_341_rviz_plugins/primitives/icon_utils.h>

#include <algorithm>
#include <cmath>

#include <QGuiApplication>
#include <QImage>
#include <QPainter>
#include <QScreen>
#include <QSvgRenderer>
#include <QWidget>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace avt_341::rviz_plugins
{

namespace
{

// The DPI at which the panel's pixel sizes were originally tuned.
constexpr qreal kBaselineDpi = 96.0;

// Logical DPI of the display backing `ref`, or the primary screen when `ref` is
// null or not yet attached to a screen.
qreal logicalDpi( const QWidget* ref )
{
    if ( ref != nullptr )
    {
        return ref->logicalDpiX();
    }
    if ( QScreen* screen = QGuiApplication::primaryScreen() )
    {
        return screen->logicalDotsPerInchX();
    }
    return kBaselineDpi;
}

// Device pixel ratio of the display backing `ref` (the factor Qt itself applies
// when AA_EnableHighDpiScaling is active), or the primary screen's otherwise.
qreal devicePixelRatio( const QWidget* ref )
{
    if ( ref != nullptr )
    {
        return ref->devicePixelRatioF();
    }
    if ( QScreen* screen = QGuiApplication::primaryScreen() )
    {
        return screen->devicePixelRatio();
    }
    return 1.0;
}

// Absolute path to resources/icons/<file_name> in the installed share dir. The
// share directory is resolved once and cached for the process lifetime.
QString iconPath( const QString& file_name )
{
    static const QString icons_dir =
        QString::fromStdString(
            ament_index_cpp::get_package_share_directory( "avt_341_rviz_plugins" ) ) +
        "/resources/icons/";
    return icons_dir + file_name;
}

// Physical (device) pixel side length for a base size on `ref`'s screen.
int physicalSize( int base_px, const QWidget* ref )
{
    const int logical = scaledSize( base_px, ref );
    return std::max( 1, static_cast<int>( std::lround( logical * devicePixelRatio( ref ) ) ) );
}

}  // namespace

qreal uiScale( const QWidget* ref )
{
    return logicalDpi( ref ) / kBaselineDpi;
}

int scaledSize( int base_px, const QWidget* ref )
{
    return static_cast<int>( std::lround( base_px * uiScale( ref ) ) );
}

QPixmap renderSvg( const QString& file_name, int base_px, const QWidget* ref )
{
    const int physical = physicalSize( base_px, ref );

    QImage image( physical, physical, QImage::Format_ARGB32_Premultiplied );
    image.fill( Qt::transparent );

    QSvgRenderer renderer( iconPath( file_name ) );
    if ( renderer.isValid() )
    {
        QPainter painter( &image );
        renderer.render( &painter );
    }

    QPixmap pixmap = QPixmap::fromImage( image );
    pixmap.setDevicePixelRatio( devicePixelRatio( ref ) );
    return pixmap;
}

QPixmap scalePixmap( const QPixmap& src, int base_px, const QWidget* ref )
{
    const int physical = physicalSize( base_px, ref );

    QPixmap pixmap =
        src.scaled( physical, physical, Qt::KeepAspectRatio, Qt::SmoothTransformation );
    pixmap.setDevicePixelRatio( devicePixelRatio( ref ) );
    return pixmap;
}

QIcon iconFromSvg( const QString& file_name, int base_px, const QWidget* ref )
{
    return QIcon( renderSvg( file_name, base_px, ref ) );
}

}  // namespace avt_341::rviz_plugins
