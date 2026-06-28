#ifndef VEHICLE_TABLE_COMPONENT_H
#define VEHICLE_TABLE_COMPONENT_H

#ifndef Q_MOC_RUN
#include <QColor>
#include <QHash>
#include <QString>
#include <QStringList>
#include <QWidget>
#endif

class QLabel;
class QStackedLayout;
class QTableWidget;

namespace avt_341 {
namespace rviz_plugins {

class IconButton;

/// Titled group box managing the per-vehicle status table shown in the Setup
/// tab. A vertical strip of icon-only buttons (add, delete, edit, move up, move
/// down) sits beside a table whose rows are vehicles and whose columns are
/// "Vehicle Id", "Nav State" and "Compute".
///
/// The component owns only the vehicle list and its presentation; the live
/// status values are pushed in from outside via setVehicleNavState() and
/// setVehicleComputeHealth() (the panel forwards them from each vehicle's
/// Nav State and Compute components). Statuses are cached per vehicle so they
/// survive table rebuilds (add / delete / rename / reorder).
class VehicleTableComponent: public QWidget
{

Q_OBJECT
public:
    VehicleTableComponent( QWidget* parent = nullptr );

    /// Current vehicle ids, in display (row) order.
    QStringList items() const;

    /// Replace the vehicle list (e.g. when restoring saved panel state); renders
    /// the table and emits itemsChanged().
    void setItems( const QStringList& vehicle_ids );

    /// Set the Nav State cell for a vehicle: text on a colored background,
    /// matching the Nav State field in NavStateComponent. No-op if the vehicle is
    /// not in the table.
    void setVehicleNavState( const QString& vehicle_id, const QString& text,
                             const QColor& color );

    /// Set the Compute cell for a vehicle: a success icon when healthy, an error
    /// icon when any monitored topic is below threshold. No-op if not present.
    void setVehicleComputeHealth( const QString& vehicle_id, bool healthy );

Q_SIGNALS:
    /// Emitted whenever the vehicle list changes (add, delete, rename, reorder).
    void itemsChanged( const QStringList& vehicles );

protected Q_SLOTS:
    void onAdd();
    void onDelete();
    void onEdit();
    void onMoveUp();
    void onMoveDown();

private:
    // Per-vehicle status, cached so the cells survive table rebuilds and a row
    // added later can show the last known status.
    struct VehicleStatus
    {
        QString nav_text = "None";
        QColor nav_color { 108, 117, 125 };   // gray, matches the idle/none default
        bool compute_healthy = true;
    };

    // Rebuilds every table row from vehicle_ids_ + status_ and toggles the
    // empty-state overlay.
    void renderTable();

    // Writes one row's three cells from the id and its cached status.
    void writeRow( int row, const QString& vehicle_id );

    // Row index of a vehicle id, or -1 if absent.
    int rowOf( const QString& vehicle_id ) const;

    // True if vehicle_id already exists (case-insensitive), ignoring ignore_row.
    bool isDuplicate( const QString& vehicle_id, int ignore_row = -1 ) const;

    QStringList vehicle_ids_;
    QHash<QString, VehicleStatus> status_;

    // QT Widgets
    IconButton* add_button_;
    IconButton* delete_button_;
    IconButton* edit_button_;
    IconButton* up_button_;
    IconButton* down_button_;
    QTableWidget* table_;
    QStackedLayout* table_stack_;
    QWidget* empty_overlay_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // VEHICLE_TABLE_COMPONENT_H
