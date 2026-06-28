#include <avt_341_rviz_plugins/primitives/icon_header.h>

#include <QFont>
#include <QHBoxLayout>

namespace avt_341::rviz_plugins
{

IconHeader::IconHeader( const QPixmap& icon, const QString& title,
                        int icon_size, QWidget* parent )
    : QWidget( parent )
{
    // Icon, scaled to a square while preserving aspect ratio
    icon_label_ = new QLabel();
    icon_label_->setPixmap(
        icon.scaled( icon_size, icon_size, Qt::KeepAspectRatio, Qt::SmoothTransformation ) );

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
