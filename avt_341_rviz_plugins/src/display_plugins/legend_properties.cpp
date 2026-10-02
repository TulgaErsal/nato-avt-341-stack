#include <avt_341_rviz_plugins/display_plugins/legend_properties.h>

#include <algorithm>
#include <initializer_list>

#include <QColor>
#include <QStringList>

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace avt_341::rviz_plugins
{

using rviz_common::Config;
using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::EnumProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::Property;
using rviz_common::properties::StringProperty;

namespace
{
constexpr int kMaxStops = 16;
constexpr int kMaxItems = 64;

QString typeName( LegendItemType type )
{
    switch ( type )
    {
    case LegendItemType::Gradient: return "Gradient";
    case LegendItemType::Cell:     return "Cell";
    case LegendItemType::Line:     break;
    }
    return "Line";
}

/// Green -> yellow -> red: the ramp an entry gets when none is given, so switching
/// a Line or Cell entry to Gradient shows something sensible.
std::vector<QColor> defaultStops()
{
    return { QColor( 0, 166, 81 ), QColor( 255, 204, 0 ), QColor( 239, 65, 54 ) };
}

/// Remove `count` children of `parent` from `start` on, deleting them on the next
/// event-loop pass rather than now. On every model change RViz queues a deferred
/// call per hidden property that dereferences the raw property (rviz_common
/// 14.1.x Property::setModel), so deleting a hidden property before that call has
/// run - e.g. while a config is loading - is a use-after-free. Taking a child out
/// first clears its model, so the pending call skips it, and deleteLater() runs
/// after it.
void removeChildrenLater( Property* parent, int start, int count )
{
    for ( int i = start + count - 1; i >= start; --i )
    {
        if ( Property* child = parent->takeChildAt( i ) )
        {
            child->deleteLater();
        }
    }
}

/// Let `property` be hidden in the tree. rviz_common 14.1.x's tree view applies a
/// hidden state only to properties with a QObject parent
/// (PropertyTreeWidget::propertyHiddenChanged tests QObject::parent(), which RViz
/// properties never set; later releases dropped the test), so give it its tree
/// parent as QObject parent too. Ownership is unaffected: ~Property deletes its
/// children before ~QObject runs, and a child's ~QObject unlinks it.
void makeHideable( Property* property, Property* owner )
{
    property->QObject::setParent( owner );
}

/// The entry appended when "Count" grows: a line in a cycling palette color.
LegendItem defaultItem( std::size_t index )
{
    static const QColor kPalette[] = {
        QColor( 31, 119, 180 ), QColor( 255, 127, 14 ), QColor( 44, 160, 44 ),
        QColor( 214, 39, 40 ),  QColor( 148, 103, 189 ), QColor( 140, 86, 75 ) };

    LegendItem item;
    item.type = LegendItemType::Line;
    item.label = QString( "Item %1" ).arg( index + 1 );
    item.color = kPalette[index % ( sizeof( kPalette ) / sizeof( kPalette[0] ) )];
    item.top_label = "High";
    item.bottom_label = "Low";
    return item;
}
} // namespace

// ---------------------------------------------------------------- LegendItemProperty

LegendItemProperty::LegendItemProperty( const LegendItem& defaults, Property* parent,
                                        QObject* receiver, const char* changed_slot )
    : BoolProperty( QString(), true,
                    "Show this entry in the legend. Expand the row to edit the entry; the "
                    "row is named after its label.",
                    parent, changed_slot, receiver ),
      receiver_( receiver ),
      changed_slot_( changed_slot )
{
    type_ = new EnumProperty(
        "Type", typeName( defaults.type ),
        "Line: a slim horizontal bar (e.g. a path). Gradient: a vertical color ramp "
        "spanning several rows, labelled at both ends. Cell: a filled square (e.g. "
        "obstacles).",
        this, changed_slot, receiver );
    type_->addOption( "Line", static_cast<int>( LegendItemType::Line ) );
    type_->addOption( "Gradient", static_cast<int>( LegendItemType::Gradient ) );
    type_->addOption( "Cell", static_cast<int>( LegendItemType::Cell ) );

    label_ = new StringProperty(
        "Label", defaults.label, "Text shown right of the marker.",
        this, changed_slot, receiver );
    color_ = new ColorProperty(
        "Color", defaults.color, "Fill color of the marker.",
        this, changed_slot, receiver );
    thickness_ = new IntProperty(
        "Thickness", defaults.thickness,
        "Height of the line bar (px); capped at the row height. Its width is the "
        "legend's marker size.",
        this, changed_slot, receiver );
    thickness_->setMin( 1 );

    top_label_ = new StringProperty(
        "Top Label", defaults.top_label, "Text beside the top row of the gradient bar.",
        this, changed_slot, receiver );
    bottom_label_ = new StringProperty(
        "Bottom Label", defaults.bottom_label,
        "Text beside the bottom row of the gradient bar.",
        this, changed_slot, receiver );
    rows_ = new IntProperty(
        "Rows", defaults.rows,
        "Height of the gradient bar, in legend rows (row spacing included). Every "
        "other entry takes one row.",
        this, changed_slot, receiver );
    rows_->setMin( 1 );

    const std::vector<QColor> stops = defaults.stops.empty() ? defaultStops() : defaults.stops;
    stop_count_ = new IntProperty(
        "Stop Count", static_cast<int>( stops.size() ),
        "Number of gradient colors, spread evenly from the top of the bar (Stop 1) "
        "to the bottom.",
        this, changed_slot, receiver );
    stop_count_->setMin( 1 );
    stop_count_->setMax( kMaxStops );
    for ( const QColor& color : stops )
    {
        addStop( color );
    }
    for ( Property* field : std::initializer_list<Property*>{
              label_, color_, thickness_, top_label_, bottom_label_, rows_, stop_count_ } )
    {
        makeHideable( field, this );
    }

    connect( type_, &Property::changed, this, &LegendItemProperty::updateTypeFields );
    connect( type_, &Property::changed, this, &LegendItemProperty::updateName );
    connect( label_, &Property::changed, this, &LegendItemProperty::updateName );
    connect( top_label_, &Property::changed, this, &LegendItemProperty::updateName );
    connect( bottom_label_, &Property::changed, this, &LegendItemProperty::updateName );
    connect( stop_count_, &Property::changed, this, &LegendItemProperty::updateStopCount );

    updateTypeFields();
    updateName();
}

LegendItem LegendItemProperty::toItem() const
{
    LegendItem item;
    item.type = type();
    item.label = label_->getString();
    item.color = color_->getColor();
    item.thickness = thickness_->getInt();
    item.top_label = top_label_->getString();
    item.bottom_label = bottom_label_->getString();
    item.rows = rows_->getInt();
    item.stops.reserve( stops_.size() );
    for ( const ColorProperty* stop : stops_ )
    {
        item.stops.push_back( stop->getColor() );
    }
    return item;
}

void LegendItemProperty::load( const Config& config )
{
    int stop_count = 0;
    if ( config.mapGetInt( "Stop Count", &stop_count ) )
    {
        resizeStops( stop_count );
    }
    BoolProperty::load( config );
}

void LegendItemProperty::updateName()
{
    QString name;
    if ( type() == LegendItemType::Gradient )
    {
        QStringList ends;
        if ( !top_label_->getString().isEmpty() )
        {
            ends << top_label_->getString();
        }
        if ( !bottom_label_->getString().isEmpty() )
        {
            ends << bottom_label_->getString();
        }
        // Not "/": RViz saves a tree row's expanded state as a "/"-separated path.
        name = ends.join( QString( " " ) + QChar( 0x2013 ) + " " );
    }
    else
    {
        name = label_->getString();
    }
    setName( name.isEmpty() ? QString( "(unlabelled %1)" ).arg( type_->getString().toLower() ) : name );
}

void LegendItemProperty::updateTypeFields()
{
    const LegendItemType item_type = type();
    const bool gradient = item_type == LegendItemType::Gradient;

    label_->setHidden( gradient );
    color_->setHidden( gradient );
    thickness_->setHidden( item_type != LegendItemType::Line );
    top_label_->setHidden( !gradient );
    bottom_label_->setHidden( !gradient );
    rows_->setHidden( !gradient );
    stop_count_->setHidden( !gradient );
    for ( ColorProperty* stop : stops_ )
    {
        stop->setHidden( !gradient );
    }
}

void LegendItemProperty::updateStopCount()
{
    resizeStops( stop_count_->getInt() );
}

LegendItemType LegendItemProperty::type() const
{
    return static_cast<LegendItemType>( type_->getOptionInt() );
}

void LegendItemProperty::addStop( const QColor& color )
{
    ColorProperty* stop = new ColorProperty(
        QString( "Stop %1" ).arg( stops_.size() + 1 ), color,
        "Gradient color. Stops are spread evenly from the top of the bar to the bottom.",
        this, changed_slot_, receiver_ );
    makeHideable( stop, this );
    stop->setHidden( type() != LegendItemType::Gradient );
    stops_.push_back( stop );
}

void LegendItemProperty::resizeStops( int count )
{
    count = std::clamp( count, 1, kMaxStops );
    const int current = static_cast<int>( stops_.size() );
    if ( count < current )
    {
        // The stops are the last children.
        removeChildrenLater( this, numChildren() - ( current - count ), current - count );
        stops_.resize( count );
    }
    while ( static_cast<int>( stops_.size() ) < count )
    {
        // A new stop repeats the current bottom color, so the ramp is unchanged
        // until it is edited.
        addStop( stops_.empty() ? QColor( 255, 255, 255 ) : stops_.back()->getColor() );
    }
    if ( stop_count_->getInt() != count )
    {
        stop_count_->setInt( count );
    }
}

// ------------------------------------------------------------ LegendItemListProperty

LegendItemListProperty::LegendItemListProperty( const QString& name,
                                                const std::vector<LegendItem>& defaults,
                                                Property* parent, QObject* receiver,
                                                const char* changed_slot )
    : Property( name, QVariant(),
                "The legend entries, listed in drawing order (top to bottom, then left "
                "to right once a column is full).",
                parent ),
      receiver_( receiver ),
      changed_slot_( changed_slot )
{
    count_ = new IntProperty(
        "Count", static_cast<int>( defaults.size() ),
        "Number of entries. Raising it appends new entries; lowering it removes "
        "entries from the end.",
        this, changed_slot, receiver );
    count_->setMin( 0 );
    count_->setMax( kMaxItems );

    for ( const LegendItem& item : defaults )
    {
        items_.push_back( new LegendItemProperty( item, this, receiver_, changed_slot_ ) );
    }

    connect( count_, &Property::changed, this, &LegendItemListProperty::updateCount );
}

std::vector<LegendItem> LegendItemListProperty::visibleItems() const
{
    std::vector<LegendItem> items;
    for ( const LegendItemProperty* item : items_ )
    {
        if ( item->getBool() )
        {
            items.push_back( item->toItem() );
        }
    }
    return items;
}

void LegendItemListProperty::load( const Config& config )
{
    int count = 0;
    if ( !config.mapGetInt( "Count", &count ) )
    {
        return; // nothing saved (e.g. a hand-written config): keep the defaults
    }
    // Loading adds and removes rows (entries, gradient stops), which must not be
    // announced to the property tree model here: RViz's DisplayGroup::load loads
    // each display between one beginInsert/endInsert covering all of them, and a
    // row change nested inside it, under rows the view hasn't been told about
    // yet, is invalid. A display is only ever loaded right after it is created
    // (config load, Duplicate), before its subtree has been shown, so rebuild the
    // subtree detached from the model; reattaching re-announces its hidden rows.
    // (Removed rows are deleted later, see removeChildrenLater.)
    rviz_common::properties::PropertyTreeModel* model = getModel();
    setModel( nullptr );

    resize( count );
    const Config entries = config.mapGetChild( "Entries" );
    for ( std::size_t i = 0; i < items_.size(); ++i )
    {
        items_[i]->load( entries.listChildAt( static_cast<int>( i ) ) );
    }

    setModel( model );
}

void LegendItemListProperty::save( Config config ) const
{
    config.mapSetValue( "Count", static_cast<int>( items_.size() ) );
    if ( items_.empty() )
    {
        return;
    }
    Config entries = config.mapMakeChild( "Entries" );
    for ( const LegendItemProperty* item : items_ )
    {
        item->save( entries.listAppendNew() );
    }
}

void LegendItemListProperty::updateCount()
{
    resize( count_->getInt() );
}

void LegendItemListProperty::resize( int count )
{
    count = std::clamp( count, 0, kMaxItems );
    const int current = static_cast<int>( items_.size() );
    if ( count < current )
    {
        // Entry i is child i + 1, after the Count property.
        removeChildrenLater( this, count + 1, current - count );
        items_.resize( count );
    }
    while ( static_cast<int>( items_.size() ) < count )
    {
        items_.push_back( new LegendItemProperty(
            defaultItem( items_.size() ), this, receiver_, changed_slot_ ) );
    }
    if ( count_->getInt() != count )
    {
        count_->setInt( count );
    }
}

} // namespace avt_341::rviz_plugins
