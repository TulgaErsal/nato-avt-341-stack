#include <avt_341_rviz_plugins/primitives/icon_button.h>

#include <QSize>

namespace avt_341::rviz_plugins
{

IconButton::IconButton( const QIcon& icon, const QString& tooltip, QWidget* parent )
    : QPushButton( parent )
{
    setIcon( icon );
    setIconSize( QSize( 18, 18 ) );
    setToolTip( tooltip );
    setFixedSize( 28, 28 );
}

}
