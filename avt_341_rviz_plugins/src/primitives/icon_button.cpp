#include <avt_341_rviz_plugins/primitives/icon_button.h>

#include <QSize>

#include <avt_341_rviz_plugins/primitives/icon_utils.h>

namespace avt_341::rviz_plugins
{

namespace
{

// Base (96-DPI) sizes: the glyph and the square button framing it.
constexpr int kIconBase = 16;
constexpr int kButtonBase = 24;

// Size the button and its icon relative to the display so they keep the same
// on-screen proportions at any DPI.
void applyGeometry( IconButton* button )
{
    const int icon = scaledSize( kIconBase, button );
    const int side = scaledSize( kButtonBase, button );
    button->setIconSize( QSize( icon, icon ) );
    button->setFixedSize( side, side );
}

}  // namespace

IconButton::IconButton( const QString& svg_file_name, const QString& tooltip, QWidget* parent )
    : QPushButton( parent )
{
    setIcon( iconFromSvg( svg_file_name, kIconBase, this ) );
    setToolTip( tooltip );
    applyGeometry( this );
}

IconButton::IconButton( const QIcon& icon, const QString& tooltip, QWidget* parent )
    : QPushButton( parent )
{
    setIcon( icon );
    setToolTip( tooltip );
    applyGeometry( this );
}

}
