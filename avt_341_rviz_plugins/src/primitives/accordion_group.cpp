#include <avt_341_rviz_plugins/primitives/accordion_group.h>

#include <algorithm>

#include <QEvent>
#include <QFont>
#include <QFontMetrics>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPalette>
#include <QPixmap>
#include <QVBoxLayout>

#include <avt_341_rviz_plugins/primitives/icon_utils.h>
#include <avt_341_rviz_plugins/primitives/status_style.h>
#include <avt_341_rviz_plugins/primitives/vehicle_palette.h>

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

// `font` with its size scaled by `factor`; handles both point- and pixel-sized
// fonts, and leaves a font with neither set alone.
QFont scaledFont( QFont font, qreal factor )
{
    if ( font.pointSizeF() > 0.0 )
    {
        font.setPointSizeF( font.pointSizeF() * factor );
    }
    else if ( font.pixelSize() > 0 )
    {
        font.setPixelSize(
            std::max( 1, static_cast<int>( font.pixelSize() * factor ) ) );
    }
    return font;
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
    QFont header_font = scaledFont( title_label_->font(), 1.15 );
    header_font.setBold( true );
    title_label_->setFont( header_font );

    // Optional color square shown to the left of the title; hidden until a valid
    // color is supplied via setSwatchColor().
    swatch_label_ = new QLabel;
    const int swatch_size = scaledSize( 12, this );
    swatch_label_->setFixedSize( swatch_size, swatch_size );
    swatch_label_->setVisible( false );

    // Optional status pill shown at the right end of the header row; hidden
    // until text is supplied via setBadge(). Slightly smaller and bold so it
    // reads as a tag rather than part of the title. Its height is fixed (and
    // its text centered in it) because the pill's corner radius is derived from
    // that height.
    badge_label_ = new QLabel;
    QFont badge_font = scaledFont( badge_label_->font(), 0.85 );
    badge_font.setBold( true );
    badge_label_->setFont( badge_font );
    badge_height_ = QFontMetrics( badge_font ).height() + scaledSize( 4, this );
    badge_label_->setFixedHeight( badge_height_ );
    badge_label_->setAlignment( Qt::AlignCenter );
    badge_label_->setVisible( false );

    caret_label_ = new QLabel;

    // Render the caret icons once and cache them, sized to the display's DPI,
    // for each state.
    const int caret_base = 16;
    caret_expanded_pixmap_ = renderSvg( "caret_expanded.svg", caret_base, this );
    caret_collapsed_pixmap_ = renderSvg( "caret_collapsed.svg", caret_base, this );
    const int caret_size = scaledSize( caret_base, this );
    caret_label_->setFixedSize( caret_size, caret_size );

    // The labels are transparent to mouse events so a click anywhere on the
    // header row reaches header_ (and its event filter) rather than a child.
    swatch_label_->setAttribute( Qt::WA_TransparentForMouseEvents, true );
    title_label_->setAttribute( Qt::WA_TransparentForMouseEvents, true );
    badge_label_->setAttribute( Qt::WA_TransparentForMouseEvents, true );
    caret_label_->setAttribute( Qt::WA_TransparentForMouseEvents, true );

    // Header row: title on the left, badge and caret pushed to the right edge.
    // No horizontal margins so the title aligns flush with the content below.
    QHBoxLayout* header_layout = new QHBoxLayout;
    header_layout->setContentsMargins( 0, 4, 0, 4 );
    header_layout->setSpacing( 6 );
    header_layout->addWidget( swatch_label_ );
    header_layout->addWidget( title_label_ );
    header_layout->addStretch();
    header_layout->addWidget( badge_label_ );
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

void AccordionGroup::setSwatchColor( const QColor& color )
{
    if ( color.isValid() )
    {
        swatch_label_->setPixmap( makeColorSwatch( color, swatch_label_->width() ) );
        swatch_label_->setVisible( true );
    }
    else
    {
        swatch_label_->clear();
        swatch_label_->setVisible( false );
    }
}

void AccordionGroup::setBadge( const QString& text, const QColor& color )
{
    if ( text.isEmpty() || !color.isValid() )
    {
        clearBadge();
        return;
    }

    badge_label_->setText( text );
    badge_label_->setStyleSheet( statusPillStyleSheet( color, badge_height_ ) );
    badge_label_->setVisible( true );
}

void AccordionGroup::clearBadge()
{
    badge_label_->clear();
    badge_label_->setStyleSheet( QString() );
    badge_label_->setVisible( false );
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
