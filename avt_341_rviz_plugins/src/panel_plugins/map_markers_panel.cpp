#include <avt_341_rviz_plugins/panel_plugins/map_markers_panel.h>

#include <cmath>

#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/msg/quaternion.hpp>

namespace
{

// Default topic; matches the mock publisher (publish_mock_ui_data.py).
constexpr const char* kDefaultTopic = "/avt_341/map_markers_";

// Key under which the configured topic is saved in the RViz config.
constexpr const char* kTopicConfigKey = "Topic";

// Table column layout.
enum Column
{
    kMarkerIdColumn = 0,
    kXColumn = 1,
    kYColumn = 2,
    kYawColumn = 3,
    kColumnCount = 4
};

// Extracts the yaw (rotation about z) from a quaternion, in radians.
double quaternionToYaw( const geometry_msgs::msg::Quaternion& q )
{
    const double siny_cosp = 2.0 * ( q.w * q.z + q.x * q.y );
    const double cosy_cosp = 1.0 - 2.0 * ( q.y * q.y + q.z * q.z );
    return std::atan2( siny_cosp, cosy_cosp );
}

// A centered, read-only numeric cell.
QTableWidgetItem* numberItem( double value, int decimals )
{
    QTableWidgetItem* item = new QTableWidgetItem( QString::number( value, 'f', decimals ) );
    item->setTextAlignment( Qt::AlignCenter );
    return item;
}

}  // namespace

namespace avt_341::rviz_plugins
{

MapMarkersPanel::MapMarkersPanel( QWidget* parent )
    : rviz_common::Panel( parent )
{
    // Topic selector: an editable field plus an Apply button. The topic is also
    // re-subscribed on Enter for convenience.
    topic_entry_ = new QLineEdit( kDefaultTopic );
    apply_button_ = new QPushButton( "Apply" );

    QHBoxLayout* topic_layout = new QHBoxLayout;
    topic_layout->addWidget( new QLabel( "Topic:" ) );
    topic_layout->addWidget( topic_entry_, 1 );
    topic_layout->addWidget( apply_button_ );

    // Marker table: one row per marker, read-only and non-interactive.
    table_ = new QTableWidget( 0, kColumnCount );
    table_->setHorizontalHeaderLabels( { "Marker ID", "X (m)", "Y (m)", "Yaw (rad)" } );
    table_->setEditTriggers( QAbstractItemView::NoEditTriggers );
    table_->setSelectionMode( QAbstractItemView::NoSelection );
    table_->setFocusPolicy( Qt::NoFocus );
    table_->verticalHeader()->setVisible( false );
    table_->horizontalHeader()->setSectionResizeMode( QHeaderView::Stretch );

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( topic_layout );
    layout->addWidget( table_ );
    setLayout( layout );

    connect( apply_button_, SIGNAL( clicked() ), this, SLOT( updateTopic() ) );
    connect( topic_entry_, SIGNAL( returnPressed() ), this, SLOT( updateTopic() ) );
}

void MapMarkersPanel::onInitialize()
{
    node_ = std::make_shared<rclcpp::Node>( "map_markers_panel_node" );

    // Pump the node from the Qt event loop. Spinning on the UI thread keeps the
    // subscription callback on the same thread as the table it updates, so no
    // extra synchronization is needed.
    executor_.add_node( node_ );
    spin_timer_ = new QTimer( this );
    connect( spin_timer_, &QTimer::timeout, this, [this]() { executor_.spin_some(); } );
    spin_timer_->start( 10 );

    // Subscribe to the topic currently in the entry (a default, or whatever
    // load() restored before initialization).
    subscribe( topic_entry_->text() );
}

void MapMarkersPanel::updateTopic()
{
    subscribe( topic_entry_->text() );
}

void MapMarkersPanel::subscribe( const QString& topic )
{
    // The node may not exist yet if this runs during load(); onInitialize() will
    // subscribe to the restored topic once it does.
    if ( !node_ || topic.isEmpty() )
    {
        return;
    }

    // Latched (transient-local) QoS so a marker list published once, before the
    // panel started, is still delivered. Note this requires the publisher to be
    // transient-local too; a volatile publisher will be QoS-incompatible.
    const rclcpp::QoS qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().transient_local();

    subscription_ = node_->create_subscription<avt_341_msgs::msg::MapMarkerList>(
        topic.toStdString(), qos,
        [this]( const avt_341_msgs::msg::MapMarkerList::ConstSharedPtr msg )
        {
            updateTable( *msg );
        } );
}

void MapMarkersPanel::updateTable( const avt_341_msgs::msg::MapMarkerList& msg )
{
    table_->setRowCount( static_cast<int>( msg.markers.size() ) );

    for ( int row = 0; row < static_cast<int>( msg.markers.size() ); ++row )
    {
        const avt_341_msgs::msg::MapMarker& marker = msg.markers[row];

        table_->setItem( row, kMarkerIdColumn,
                         new QTableWidgetItem( QString::fromStdString( marker.marker_id ) ) );
        table_->setItem( row, kXColumn, numberItem( marker.pose.position.x, 2 ) );
        table_->setItem( row, kYColumn, numberItem( marker.pose.position.y, 2 ) );
        table_->setItem( row, kYawColumn, numberItem( quaternionToYaw( marker.pose.orientation ), 3 ) );
    }
}

void MapMarkersPanel::save( rviz_common::Config config ) const
{
    rviz_common::Panel::save( config );
    config.mapSetValue( kTopicConfigKey, topic_entry_->text() );
}

void MapMarkersPanel::load( const rviz_common::Config& config )
{
    rviz_common::Panel::load( config );

    QString topic;
    if ( config.mapGetString( kTopicConfigKey, &topic ) )
    {
        topic_entry_->setText( topic );
        subscribe( topic );
    }
}

}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(avt_341::rviz_plugins::MapMarkersPanel, rviz_common::Panel )
