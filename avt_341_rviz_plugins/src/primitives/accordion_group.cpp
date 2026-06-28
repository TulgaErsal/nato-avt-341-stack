#include <avt_341_rviz_plugins/primitives/accordion_group.h>

#include <QEvent>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPalette>
#include <QPixmap>
#include <QVBoxLayout>

#include <rviz_common/load_resource.hpp>

namespace
{

// A 1px, theme-aware separator drawn under the header row.
QWidget* makeUnderline()
{
    QWidget* line = new QWidget;
    line->setFixedHeight( 1 );
    line->setAutoFillBackground( true );
    QPalette palette = line->palette();
    palette.setColor( QPalette::Window, palette.color( QPalette::Mid ) );
    line->setPalette( palette );
    return line;
}

}  // namespace

namespace avt_341::rviz_plugins
{

AccordionGroup::AccordionGroup( const QString& title, QWidget* parent )
    : QWidget( parent )
{
    // Header text: left-aligned, slightly enlarged and bold so it reads as a
    // section heading.
    title_label_ = new QLabel( title );
    QFont header_font = title_label_->font();
    if ( header_font.pointSizeF() > 0.0 )
    {
        header_font.setPointSizeF( header_font.pointSizeF() * 1.15 );
    }
    else if ( header_font.pixelSize() > 0 )
    {
        header_font.setPixelSize( static_cast<int>( header_font.pixelSize() * 1.15 ) );
    }
    header_font.setBold( true );
    title_label_->setFont( header_font );

    caret_label_ = new QLabel;

    // Load the caret icons once and cache them, pre-scaled, for each state.
    const QString icon_url = "package://avt_341_rviz_plugins/resources/icons/";
    const int caret_size = 16;
    caret_expanded_pixmap_ = rviz_common::loadPixmap( icon_url + "caret_expanded.svg" )
        .scaled( caret_size, caret_size, Qt::KeepAspectRatio, Qt::SmoothTransformation );
    caret_collapsed_pixmap_ = rviz_common::loadPixmap( icon_url + "caret_collapsed.svg" )
        .scaled( caret_size, caret_size, Qt::KeepAspectRatio, Qt::SmoothTransformation );
    caret_label_->setFixedSize( caret_size, caret_size );

    // The labels are transparent to mouse events so a click anywhere on the
    // header row reaches header_ (and its event filter) rather than a child.
    title_label_->setAttribute( Qt::WA_TransparentForMouseEvents, true );
    caret_label_->setAttribute( Qt::WA_TransparentForMouseEvents, true );

    // Header row: title on the left, caret pushed to the right edge. No
    // horizontal margins so the title aligns flush with the content below.
    QHBoxLayout* header_layout = new QHBoxLayout;
    header_layout->setContentsMargins( 0, 4, 0, 4 );
    header_layout->addWidget( title_label_ );
    header_layout->addStretch();
    header_layout->addWidget( caret_label_ );

    header_ = new QWidget;
    header_->setLayout( header_layout );
    header_->setCursor( Qt::PointingHandCursor );
    header_->installEventFilter( this );

    // Collapsible content area: zero horizontal margins so embedded controls add
    // no extra left/right padding; a small top gap separates it from the line.
    content_container_ = new QWidget;
    content_layout_ = new QVBoxLayout( content_container_ );
    content_layout_->setContentsMargins( 0, 6, 0, 0 );
    content_layout_->setSpacing( 0 );

    // No outer margins: the accordion adds no padding around its header/content.
    QVBoxLayout* main_layout = new QVBoxLayout( this );
    main_layout->setContentsMargins( 0, 0, 0, 0 );
    main_layout->setSpacing( 0 );
    main_layout->addWidget( header_ );
    main_layout->addWidget( makeUnderline() );
    main_layout->addWidget( content_container_ );

    updateCaret();
}

void AccordionGroup::setContentWidget( QWidget* content )
{
    if ( content_widget_ != nullptr )
    {
        content_layout_->removeWidget( content_widget_ );
        content_widget_->deleteLater();
    }

    content_widget_ = content;
    if ( content_widget_ != nullptr )
    {
        content_layout_->addWidget( content_widget_ );
    }
}

void AccordionGroup::setExpanded( bool expanded )
{
    if ( expanded_ == expanded )
    {
        return;
    }
    expanded_ = expanded;
    content_container_->setVisible( expanded_ );
    updateCaret();
}

void AccordionGroup::toggle()
{
    setExpanded( !expanded_ );
}

bool AccordionGroup::eventFilter( QObject* watched, QEvent* event )
{
    if ( watched == header_ && event->type() == QEvent::MouseButtonPress )
    {
        const QMouseEvent* mouse_event = static_cast<QMouseEvent*>( event );
        if ( mouse_event->button() == Qt::LeftButton )
        {
            toggle();
            return true;
        }
    }
    return QWidget::eventFilter( watched, event );
}

void AccordionGroup::updateCaret()
{
    caret_label_->setPixmap( expanded_ ? caret_expanded_pixmap_ : caret_collapsed_pixmap_ );
}

}
