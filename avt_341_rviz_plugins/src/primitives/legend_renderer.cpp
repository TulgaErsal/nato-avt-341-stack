#include <avt_341_rviz_plugins/primitives/legend_renderer.h>

#include <algorithm>
#include <cmath>

#include <QBrush>
#include <QFont>
#include <QFontMetricsF>
#include <QLinearGradient>
#include <QPainter>
#include <QPen>
#include <QRectF>
#include <QSize>

namespace avt_341::rviz_plugins
{

namespace
{
/// Largest legend side (logical px) that is rasterized; guards against absurd
/// property values producing a texture the GPU refuses.
constexpr double kMaxExtent = 4096.0;

/// A column of entries, filled top to bottom.
struct Column
{
    std::vector<const LegendItem*> items;
    int rows = 0;              ///< Rows used by the entries.
    double label_width = 0.0;  ///< Widest label in the column.
    double x = 0.0;            ///< Left edge relative to the content origin.
};

int rowSpan( const LegendItem& item )
{
    return item.type == LegendItemType::Gradient ? std::max( 1, item.rows ) : 1;
}

/// A single-row gradient has room for one label: both ends joined.
QString singleRowGradientLabel( const LegendItem& item )
{
    if ( item.top_label.isEmpty() || item.bottom_label.isEmpty() )
    {
        return item.top_label + item.bottom_label;
    }
    // An en dash, spelled out: MSVC mis-encodes "–" inside QStringLiteral.
    return item.top_label + ' ' + QChar( 0x2013 ) + ' ' + item.bottom_label;
}

double labelWidth( const LegendItem& item, const QFontMetricsF& metrics )
{
    if ( item.type != LegendItemType::Gradient )
    {
        return metrics.horizontalAdvance( item.label );
    }
    if ( rowSpan( item ) == 1 )
    {
        return metrics.horizontalAdvance( singleRowGradientLabel( item ) );
    }
    return std::max( metrics.horizontalAdvance( item.top_label ),
                     metrics.horizontalAdvance( item.bottom_label ) );
}

QFont makeFont( const LegendStyle& style, bool bold )
{
    QFont font( style.font_family );
    font.setPixelSize( std::max( 1, style.font_size ) );
    font.setBold( bold );
    font.setStyleStrategy( QFont::PreferAntialias );
    return font;
}

void drawLabel( QPainter& painter, const QRectF& row, const QString& text, const LegendStyle& style )
{
    if ( text.isEmpty() )
    {
        return;
    }
    painter.setPen( style.text_color );
    painter.drawText( row, Qt::AlignLeft | Qt::AlignVCenter, text );
}

/// Fill `rect` and stroke the outline just inside its edge, so a marker covers
/// exactly `rect` whatever the outline width. A marker too thin to hold its
/// outline is drawn solid in the outline color.
void paintMarker( QPainter& painter, const QRectF& rect, const QBrush& fill, const LegendStyle& style )
{
    const double outline = std::max( 0, style.outline_width );

    painter.setPen( Qt::NoPen );
    if ( outline > 0.0 && ( rect.width() <= 2.0 * outline || rect.height() <= 2.0 * outline ) )
    {
        painter.setBrush( style.outline_color );
        painter.drawRect( rect );
        return;
    }

    painter.setBrush( fill );
    painter.drawRect( rect );

    if ( outline > 0.0 )
    {
        QPen pen( style.outline_color, outline );
        pen.setJoinStyle( Qt::MiterJoin );
        painter.setPen( pen );
        painter.setBrush( Qt::NoBrush );
        const double half = 0.5 * outline;
        painter.drawRect( rect.adjusted( half, half, -half, -half ) );
    }
}

/// Stops spread evenly from the top of `bar` to its bottom.
QBrush gradientBrush( const QRectF& bar, const std::vector<QColor>& stops )
{
    if ( stops.empty() )
    {
        return Qt::NoBrush;
    }
    if ( stops.size() == 1 )
    {
        return QBrush( stops.front() );
    }
    QLinearGradient gradient( bar.topLeft(), bar.bottomLeft() );
    const double last = static_cast<double>( stops.size() - 1 );
    for ( std::size_t i = 0; i < stops.size(); ++i )
    {
        gradient.setColorAt( static_cast<double>( i ) / last, stops[i] );
    }
    return QBrush( gradient );
}
} // namespace

QImage RenderLegend( const LegendStyle& style, const std::vector<LegendItem>& items,
                     double pixel_ratio )
{
    const QFont font = makeFont( style, style.bold );
    const QFont title_font = makeFont( style, true );
    const QFontMetricsF metrics( font );
    const QFontMetricsF title_metrics( title_font );

    const double marker = std::max( 1, style.marker_size );
    const double row_height = std::max( std::ceil( metrics.height() ), marker );
    const double row_spacing = std::max( 0, style.row_spacing );
    const double column_spacing = std::max( 0, style.column_spacing );
    const double label_gap = std::max( 0, style.label_gap );
    const int max_rows = std::max( 1, style.max_rows );

    // Flow the entries into columns of at most max_rows rows.
    std::vector<Column> columns;
    for ( const LegendItem& item : items )
    {
        const int span = rowSpan( item );
        if ( columns.empty() ||
             ( columns.back().rows > 0 && columns.back().rows + span > max_rows ) )
        {
            columns.emplace_back();
        }
        Column& column = columns.back();
        column.items.push_back( &item );
        column.rows += span;
        column.label_width = std::max( column.label_width, labelWidth( item, metrics ) );
    }

    const bool has_title = !style.title.isEmpty();
    if ( columns.empty() && !has_title )
    {
        return QImage();
    }

    // Columns side by side, each as wide as its widest label.
    double content_width = 0.0;
    int content_rows = 0;
    for ( Column& column : columns )
    {
        column.x = &column == &columns.front() ? 0.0 : content_width + column_spacing;
        const double label_width = std::ceil( column.label_width );
        content_width = column.x + marker + ( label_width > 0.0 ? label_gap + label_width : 0.0 );
        content_rows = std::max( content_rows, column.rows );
    }
    const double content_height = content_rows > 0
        ? content_rows * row_height + ( content_rows - 1 ) * row_spacing
        : 0.0;

    const double title_height = has_title ? std::ceil( title_metrics.height() ) : 0.0;
    const double title_width = has_title ? std::ceil( title_metrics.horizontalAdvance( style.title ) ) : 0.0;
    const double title_gap = has_title && content_rows > 0 ? row_spacing : 0.0;

    const double border = std::max( 0, style.border_width );
    const double frame = border + std::max( 0, style.padding );
    const double inner_width = std::max( content_width, title_width );
    const double width = std::min( kMaxExtent, std::ceil( inner_width ) + 2.0 * frame );
    const double height = std::min(
        kMaxExtent, std::ceil( title_height + title_gap + content_height ) + 2.0 * frame );

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

    // Box: the border is stroked inside the image so it is never clipped.
    QColor background = style.background_color;
    background.setAlphaF( std::clamp( style.background_alpha, 0.0f, 1.0f ) );
    painter.setBrush( background );
    if ( border > 0.0 )
    {
        QPen pen( style.border_color, border );
        pen.setJoinStyle( Qt::MiterJoin );
        painter.setPen( pen );
    }
    else
    {
        painter.setPen( Qt::NoPen );
    }
    const double half_border = 0.5 * border;
    const QRectF box = QRectF( 0.0, 0.0, width, height )
        .adjusted( half_border, half_border, -half_border, -half_border );
    const double radius = std::max( 0, style.corner_radius );
    if ( radius > 0.0 )
    {
        painter.drawRoundedRect( box, radius, radius );
    }
    else
    {
        painter.drawRect( box );
    }

    double top = frame;
    if ( has_title )
    {
        painter.setFont( title_font );
        drawLabel( painter, QRectF( frame, top, inner_width, title_height ), style.title, style );
        top += title_height + title_gap;
    }

    painter.setFont( font );
    for ( const Column& column : columns )
    {
        const double marker_x = frame + column.x;
        const double label_x = marker_x + marker + label_gap;
        const double label_width = std::ceil( column.label_width );

        double y = top;
        for ( const LegendItem* item : column.items )
        {
            const int span = rowSpan( *item );
            const double item_height = span * row_height + ( span - 1 ) * row_spacing;
            const QRectF first_row( label_x, y, label_width, row_height );

            switch ( item->type )
            {
            case LegendItemType::Line:
            {
                const double thickness = std::clamp<double>( item->thickness, 1.0, row_height );
                const QRectF bar( marker_x, y + std::round( 0.5 * ( row_height - thickness ) ),
                                  marker, thickness );
                paintMarker( painter, bar, QBrush( item->color ), style );
                drawLabel( painter, first_row, item->label, style );
                break;
            }
            case LegendItemType::Cell:
            {
                const QRectF cell( marker_x, y + std::round( 0.5 * ( row_height - marker ) ),
                                   marker, marker );
                paintMarker( painter, cell, QBrush( item->color ), style );
                drawLabel( painter, first_row, item->label, style );
                break;
            }
            case LegendItemType::Gradient:
            {
                const QRectF bar( marker_x, y, marker, item_height );
                paintMarker( painter, bar, gradientBrush( bar, item->stops ), style );
                if ( span == 1 )
                {
                    drawLabel( painter, first_row, singleRowGradientLabel( *item ), style );
                }
                else
                {
                    const QRectF last_row( label_x, y + item_height - row_height, label_width, row_height );
                    drawLabel( painter, first_row, item->top_label, style );
                    drawLabel( painter, last_row, item->bottom_label, style );
                }
                break;
            }
            }

            y += item_height + row_spacing;
        }
    }

    return image;
}

} // namespace avt_341::rviz_plugins
