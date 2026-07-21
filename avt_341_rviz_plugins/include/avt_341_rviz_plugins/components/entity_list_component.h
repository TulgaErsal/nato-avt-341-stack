#ifndef ENTITY_LIST_COMPONENT_H
#define ENTITY_LIST_COMPONENT_H

#ifndef Q_MOC_RUN
#include <QString>
#include <QStringList>
#include <QWidget>
#endif

class QListView;
class QStackedLayout;
class QStringListModel;

namespace avt_341 {
namespace rviz_plugins {

class IconButton;

/// A titled group box that manages a user-editable list of named entities.
/// A vertical strip of icon-only buttons (add, delete, edit, move up, move
/// down) sits beside a list view that fills the remaining width. Re-usable for
/// any list of named things (vehicles, objectives, etc.).
class EntityListComponent: public QWidget
{

Q_OBJECT
public:
    /// \param title     Text shown on the surrounding group box.
    /// \param item_noun Singular noun used in the add/edit popups (e.g. "Vehicle").
    EntityListComponent( const QString& title, const QString& item_noun,
                         QWidget* parent = nullptr );

    /// Current entries, in display order.
    QStringList items() const;

Q_SIGNALS:
    /// Emitted whenever the list changes (add, delete, rename or reorder).
    void itemsChanged( const QStringList& items );

protected Q_SLOTS:
    void onAdd();
    void onDelete();
    void onEdit();
    void onMoveUp();
    void onMoveDown();
    void updateEmptyMessage();

protected:
    // Returns true if name already exists (case-insensitive), ignoring ignore_row.
    bool isDuplicate( const QString& name, int ignore_row = -1 ) const;

    QString item_noun_;

    // QT Widgets
    IconButton* add_button_;
    IconButton* delete_button_;
    IconButton* edit_button_;
    IconButton* up_button_;
    IconButton* down_button_;
    QListView* list_view_;
    QStringListModel* model_;
    QStackedLayout* list_stack_;
    QWidget* empty_overlay_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // ENTITY_LIST_COMPONENT_H
