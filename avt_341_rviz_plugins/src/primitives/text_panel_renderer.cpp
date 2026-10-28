#include <avt_341_rviz_plugins/primitives/text_panel_renderer.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <QFont>
#include <QFontMetricsF>
#include <QPainter>
#include <QRectF>
#include <QString>

namespace avt_341::rviz_plugins
{

namespace
{
/// Largest texture edge requested from the GPU.
constexpr double kMaxExtent = 4096.0;
/// Vertical gap between lines.
constexpr double kLineGap = 4.0;
constexpr int kTextFlags = Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap;

QFont makeFont( const TextPanelStyle& style, bool bold )
{
    QFont font;
    font.setPixelSize( std::max( 1, style.font_size ) );
    font.setBold( bold );
    font.setStyleStrategy( QFont::PreferAntialias );
    return font;
}
} // namespace

QImage RenderTextPanel( const TextPanelStyle& style, const QStringList& lines_in,
                        double pixel_ratio )
{
    QStringList lines;
    for ( const QString& line : lines_in )
    {
        if ( !line.trimmed().isEmpty() )
        {
            lines.append( line );
        }
    }
    if ( lines.isEmpty() )
    {
        return QImage();
    }

    const double padding = std::max( 0, style.padding );
    const double max_width =
        std::max( 2.0 * padding + 1.0, static_cast<double>( style.max_width ) );
    const double wrap_width = max_width - 2.0 * padding;

    // Lay the lines out (wrapped to the panel width) to size the panel.
    std::vector<QRectF> boxes;
    double content_width = 0.0;
    double top = padding;
    for ( int i = 0; i < lines.size(); ++i )
    {
        const QFontMetricsF metrics( makeFont( style, style.first_line_bold && i == 0 ) );
        const QRectF bounds = metrics.boundingRect(
            QRectF( 0.0, 0.0, wrap_width, kMaxExtent ), kTextFlags, lines[i] );
        boxes.emplace_back( padding, top, wrap_width, bounds.height() );
        content_width = std::max( content_width, std::min( wrap_width, bounds.width() ) );
        top += bounds.height() + kLineGap;
    }
    const double width = std::min( kMaxExtent, std::ceil( content_width + 2.0 * padding ) );
    const double height = std::min( kMaxExtent, std::ceil( top - kLineGap + padding ) );

    const double ratio = pixel_ratio > 0.0 ? pixel_ratio : 1.0;
    QImage image( QSize( static_cast<int>( std::ceil( width * ratio ) ),
                         static_cast<int>( std::ceil( height * ratio ) ) ),
                  QImage::Format_ARGB32_Premultiplied );
    image.setDevicePixelRatio( ratio );
    image.fill( Qt::transparent );

    // Everything below is in logical pixels; the painter scales by the ratio.
    QPainter painter( &image );
    painter.setRenderHint( QPainter::Antialiasing );
    painter.setRenderHint( QPainter::TextAntialiasing );

    QColor background = style.background_color;
    background.setAlphaF( std::clamp( style.background_alpha, 0.0f, 1.0f ) );
    painter.setPen( Qt::NoPen );
    painter.setBrush( background );
    const double radius = std::max( 0, style.corner_radius );
    painter.drawRoundedRect( QRectF( 0.0, 0.0, width, height ), radius, radius );

    painter.setPen( style.text_color );
    for ( int i = 0; i < lines.size(); ++i )
    {
        painter.setFont( makeFont( style, style.first_line_bold && i == 0 ) );
        // Same wrap width as measured, so the lines break identically.
        painter.drawText( boxes[i], kTextFlags, lines[i] );
    }
    return image;
}

} // namespace avt_341::rviz_plugins
