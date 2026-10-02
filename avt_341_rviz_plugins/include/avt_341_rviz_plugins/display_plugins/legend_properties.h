#ifndef LEGEND_PROPERTIES_H
#define LEGEND_PROPERTIES_H

#ifndef Q_MOC_RUN
#include <vector>

#include <rviz_common/config.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/property.hpp>

#include <avt_341_rviz_plugins/primitives/legend_renderer.h>
#endif

namespace rviz_common {
namespace properties {
class ColorProperty;
class EnumProperty;
class IntProperty;
class StringProperty;
} // namespace properties
} // namespace rviz_common

namespace avt_341 {
namespace rviz_plugins {

/// One legend entry in the property tree: a checkbox row (unchecked hides the
/// entry from the legend) named after the entry's label(s), holding its fields.
/// Only the current Type's fields are shown; the others keep their values (and
/// are saved), so switching the type back and forth loses nothing.
///
/// Gradient stop colors are child properties created on demand from "Stop
/// Count". The base Property::load only fills children that already exist, so
/// load() sizes the stop list from the config before the children are loaded.
/// It is only called by LegendItemListProperty::load, with the subtree detached
/// from the property tree model (see there).
class LegendItemProperty : public rviz_common::properties::BoolProperty
{
    Q_OBJECT

public:
    /// \param receiver      Object whose \p changed_slot fires on any edit.
    /// \param changed_slot  A SLOT(...) string; must outlive this property
    ///                      (SLOT() literals do).
    LegendItemProperty( const LegendItem& defaults, rviz_common::properties::Property* parent,
                        QObject* receiver, const char* changed_slot );

    /// Snapshot the current field values.
    LegendItem toItem() const;

    void load( const rviz_common::Config& config ) override;

private Q_SLOTS:
    /// Name the row after the label (Line / Cell) or both end labels (Gradient).
    void updateName();
    /// Show only the fields used by the current Type.
    void updateTypeFields();
    void updateStopCount();

private:
    LegendItemType type() const;
    void addStop( const QColor& color );
    void resizeStops( int count );

    QObject* receiver_;
    const char* changed_slot_;

    rviz_common::properties::EnumProperty*   type_;
    rviz_common::properties::StringProperty* label_;
    rviz_common::properties::ColorProperty*  color_;
    rviz_common::properties::IntProperty*    thickness_;
    rviz_common::properties::StringProperty* top_label_;
    rviz_common::properties::StringProperty* bottom_label_;
    rviz_common::properties::IntProperty*    rows_;
    rviz_common::properties::IntProperty*    stop_count_;
    std::vector<rviz_common::properties::ColorProperty*> stops_; ///< The last children.
};

/// The legend's ordered entries. "Count" grows the list (appending default
/// entries) or shrinks it (dropping entries from the end).
///
/// Saved as an ordered list ("Count" plus "Entries") rather than the default
/// name-keyed map: entry rows are named after their labels, which are neither
/// stable nor unique, and the order of the entries is the legend's order.
/// load() rebuilds the entries detached from the property tree model, because
/// RViz loads a display while the model is mid-insertion of it.
class LegendItemListProperty : public rviz_common::properties::Property
{
    Q_OBJECT

public:
    /// \param defaults      Entries a newly added display starts with.
    /// \param receiver      Object whose \p changed_slot fires on any edit,
    ///                      including entries being added or removed.
    LegendItemListProperty( const QString& name, const std::vector<LegendItem>& defaults,
                            rviz_common::properties::Property* parent,
                            QObject* receiver, const char* changed_slot );

    /// The checked entries, in order.
    std::vector<LegendItem> visibleItems() const;

    void load( const rviz_common::Config& config ) override;
    void save( rviz_common::Config config ) const override;

private Q_SLOTS:
    void updateCount();

private:
    void resize( int count );

    QObject* receiver_;
    const char* changed_slot_;

    rviz_common::properties::IntProperty* count_;
    std::vector<LegendItemProperty*> items_; ///< Children 1..n (after count_).
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // LEGEND_PROPERTIES_H
