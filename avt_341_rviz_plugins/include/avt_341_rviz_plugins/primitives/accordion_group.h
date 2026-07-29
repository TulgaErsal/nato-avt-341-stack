#ifndef ACCORDION_GROUP_H
#define ACCORDION_GROUP_H

#ifndef Q_MOC_RUN
#include <QColor>
#include <QPixmap>
#include <QString>
#include <QWidget>
#endif

class QLabel;
class QVBoxLayout;

namespace avt_341 {
namespace rviz_plugins {

/// A lightweight collapsible container ("accordion"). The header is a left-
/// aligned, slightly enlarged title with an optional right-aligned status badge
/// and a caret indicating the expanded / collapsed state, an underline beneath
/// it, and the content below.
///
/// Unlike a QGroupBox it draws no surrounding box and adds no left/right padding,
/// so embedded controls sit flush with the header. Clicking anywhere on the
/// header row toggles the group.
class AccordionGroup: public QWidget
{

Q_OBJECT
public:
    explicit AccordionGroup( const QString& title, QWidget* parent = nullptr );

    // Places `content` in the collapsible area, replacing (and deleting) any
    // previously set content. The accordion reparents and owns `content`.
    void setContentWidget( QWidget* content );

    void setExpanded( bool expanded );
    bool isExpanded() const { return expanded_; }

    // Shows a small filled color square to the left of the title (e.g. to tag a
    // group with its vehicle's label color). Passing an invalid QColor hides the
    // square; it is hidden by default.
    void setSwatchColor( const QColor& color );

    // Shows a rounded "pill" badge with `text` on `color`, right-aligned in the
    // header row just left of the caret (e.g. to tag a group with a short status
    // such as "active"). An empty `text` hides the badge, as does an invalid
    // `color`; the badge is hidden by default.
    void setBadge( const QString& text, const QColor& color );

    // Hides the badge, if one is shown.
    void clearBadge();

public Q_SLOTS:
    void toggle();

protected:
    // Toggles the group when the header row is clicked.
    bool eventFilter( QObject* watched, QEvent* event ) override;

private:
    // Refreshes the caret glyph for the current expanded / collapsed state.
    void updateCaret();

    // QT Widgets
    QWidget* header_;
    QLabel* swatch_label_;
    QLabel* title_label_;
    QLabel* badge_label_;
    QLabel* caret_label_;
    QWidget* content_container_;
    QVBoxLayout* content_layout_;
    QWidget* content_widget_ = nullptr;

    // Pre-scaled caret pixmaps, loaded once for each state.
    QPixmap caret_expanded_pixmap_;
    QPixmap caret_collapsed_pixmap_;

    // The badge's fixed height, which also sets its corner radius.
    int badge_height_ = 0;

    bool expanded_ = true;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // ACCORDION_GROUP_H
