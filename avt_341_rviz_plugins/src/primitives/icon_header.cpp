#include <avt_341_rviz_plugins/primitives/icon_header.h>

#include <QFont>
#include <QHBoxLayout>

#include <avt_341_rviz_plugins/primitives/icon_utils.h>

namespace avt_341::rviz_plugins
{

IconHeader::IconHeader( const QPixmap& icon, const QString& title,
                        int icon_size, QWidget* parent )
    : QWidget( parent )
{
    // Icon, scaled to a square (preserving aspect ratio) at the display's DPI.
    icon_label_ = new QLabel();
    icon_label_->setPixmap( scalePixmap( icon, icon_size, this ) );

    // Title text, emphasized relative to the default font
    title_label_ = new QLabel( title );
    QFont title_font = title_label_->font();
    title_font.setPointSize( title_font.pointSize() + 4 );
    title_font.setBold( true );
    title_label_->setFont( title_font );

    // Icon and title on a single line, jointly centered in the available width
    // (equal stretches on both sides).
    QHBoxLayout* layout = new QHBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->addStretch();
    layout->addWidget( icon_label_ );
    layout->addWidget( title_label_ );
    layout->addStretch();
    setLayout( layout );
}

}
