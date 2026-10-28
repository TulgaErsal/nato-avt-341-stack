#ifndef TEXT_PANEL_RENDERER_H
#define TEXT_PANEL_RENDERER_H

#include <QColor>
#include <QImage>
#include <QStringList>

namespace avt_341 {
namespace rviz_plugins {

/// Appearance of a text panel. Sizes are logical pixels.
struct TextPanelStyle
{
    QColor background_color{ 30, 30, 30 };
    float background_alpha = 0.8f;
    QColor text_color{ 255, 255, 255 };
    int font_size = 14;           ///< Pixel size.
    bool first_line_bold = true;
    int padding = 8;              ///< Space between the panel edge and the text.
    int corner_radius = 4;
    int max_width = 320;          ///< Panel width cap; longer lines word-wrap.
};

/// Paint `lines` (one paragraph each, top to bottom) on a rounded, translucent
/// panel into a transparent, premultiplied-ARGB image. Lines wider than
/// `max_width` wrap; empty lines are skipped. The image is rendered at
/// `pixel_ratio` device pixels per logical pixel (and tagged with that
/// devicePixelRatio) so text stays crisp on scaled displays. Returns a null
/// image when every line is empty.
QImage RenderTextPanel( const TextPanelStyle& style, const QStringList& lines,
                        double pixel_ratio );

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // TEXT_PANEL_RENDERER_H
