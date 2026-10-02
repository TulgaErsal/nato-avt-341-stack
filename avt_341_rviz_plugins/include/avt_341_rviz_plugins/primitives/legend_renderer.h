#ifndef LEGEND_RENDERER_H
#define LEGEND_RENDERER_H

#include <vector>

#include <QColor>
#include <QImage>
#include <QString>

namespace avt_341 {
namespace rviz_plugins {

/// The kinds of legend entry. The values are the Legend display's "Type" enum
/// option ints, so they are persisted indirectly and must not be renumbered.
enum class LegendItemType
{
    Line = 0,     ///< A slim horizontal bar, e.g. a path.
    Gradient = 1, ///< A vertical color ramp spanning several rows.
    Cell = 2      ///< A filled square, e.g. an obstacle cell.
};

/// One legend entry. Which fields are used depends on `type`.
struct LegendItem
{
    LegendItemType type = LegendItemType::Line;

    QString label;            ///< Line / Cell: text right of the marker.
    QColor color;             ///< Line / Cell: marker fill.
    int thickness = 6;        ///< Line: bar height (px), capped at the row height.

    QString top_label;        ///< Gradient: text beside the bar's first row.
    QString bottom_label;     ///< Gradient: text beside the bar's last row.
    int rows = 4;             ///< Gradient: number of legend rows the bar spans.
    std::vector<QColor> stops;///< Gradient: colors from top to bottom, evenly spaced.
};

/// Appearance and layout of the legend box. Sizes are logical pixels.
struct LegendStyle
{
    QString title;            ///< Optional bold heading; empty for none.

    QColor background_color{ 255, 255, 255 };
    float background_alpha = 0.85f;
    QColor border_color{ 64, 64, 64 };
    int border_width = 1;     ///< 0 for no border.
    int corner_radius = 4;

    int max_rows = 10;        ///< Rows per column before entries wrap to a new column.
    int padding = 10;         ///< Space between the border and the content.
    int row_spacing = 6;
    int column_spacing = 20;

    QString font_family;
    int font_size = 14;       ///< Pixel size.
    bool bold = false;
    QColor text_color{ 0, 0, 0 };

    int marker_size = 24;     ///< Marker column width and cell side; sets the minimum row height.
    int label_gap = 10;       ///< Space between a marker and its label.
    QColor outline_color{ 0, 0, 0 };
    int outline_width = 1;    ///< Marker outline, drawn inside the marker; 0 for none.
};

/// Lay out and paint a legend box into a transparent, premultiplied-ARGB image.
///
/// Entries flow top to bottom in rows of uniform height - the taller of the text
/// line and the marker size. Line and Cell entries take one row; a Gradient bar
/// spans its `rows` rows (row spacing included), with the top label beside its
/// first row and the bottom label beside its last. When the next entry would push
/// a column past `max_rows`, it starts a new column to the right (an entry taller
/// than `max_rows` gets a column of its own). Each column is as wide as its
/// widest label, so the box is as compact as the entries allow.
///
/// The image is rendered at `pixel_ratio` device pixels per logical pixel (and
/// tagged with that devicePixelRatio), so text stays crisp on scaled displays.
/// Returns a null image when there is neither a title nor any entry.
QImage RenderLegend( const LegendStyle& style, const std::vector<LegendItem>& items,
                     double pixel_ratio );

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // LEGEND_RENDERER_H
