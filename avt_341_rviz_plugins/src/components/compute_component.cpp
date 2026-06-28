#include <avt_341_rviz_plugins/components/compute_component.h>

#include <algorithm>
#include <exception>
#include <map>
#include <string>
#include <vector>

#include <QAbstractScrollArea>
#include <QBrush>
#include <QColor>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

namespace
{

// Window over which messages are counted to derive a rate. One second gives
// ~1 Hz display resolution, which is plenty for this check.
constexpr int kMeasurementWindowMs = 1000;

// Status table column layout.
enum Column
{
    kTopicColumn = 0,
    kRateColumn = 1,
    kExpectedColumn = 2,
    kColumnCount = 3
};

// Shown in the "Rate" column before a topic has a publisher to measure.
constexpr const char* kUnknownRate = "-";

// Colors used to flag a topic running below threshold.
const QColor kAlertBackground( 220, 53, 69 );
const QColor kAlertForeground( Qt::white );

QString formatHz( double hz )
{
    return QString::number( hz, 'f', 1 ) + " Hz";
}

}  // namespace

namespace avt_341::rviz_plugins
{

// Shared (per-process) configuration. The defaults match what the component
// previously hard-coded. Edited via the Configure popup and applied to every
// live instance, so the configuration is global rather than per-vehicle.
std::vector<ComputeComponent::MonitoredTopicSpec> ComputeComponent::s_monitored_topics_ = {
    { "avt_341/odometry", 30.0 }
};
double ComputeComponent::s_threshold_fraction_ = 0.80;
std::vector<ComputeComponent*> ComputeComponent::s_instances_ = {};

ComputeComponent::ComputeComponent( const QString& vehicle_id,
                                    rclcpp::Node::SharedPtr node, QWidget* parent )
    : QWidget( parent ), vehicle_id_( vehicle_id ), node_( std::move( node ) )
{
    s_instances_.push_back( this );

    buildUi();

    // Build the monitored topics from the shared config and subscribe to any
    // whose publisher is already up; the rest are picked up on later ticks.
    applyConfig();

    // Recompute and repaint the rates once per measurement window.
    measurement_timer_ = new QTimer( this );
    connect( measurement_timer_, SIGNAL( timeout() ), this, SLOT( updateRates() ) );
    measurement_timer_->start( kMeasurementWindowMs );
}

ComputeComponent::~ComputeComponent()
{
    s_instances_.erase(
        std::remove( s_instances_.begin(), s_instances_.end(), this ),
        s_instances_.end() );
}

void ComputeComponent::buildUi()
{
    table_ = new QTableWidget( 0, kColumnCount );
    table_->setHorizontalHeaderLabels( { "Topic", "Rate", "Expected" } );

    // A read-only status grid: no editing, selection, focus or row numbers, and
    // a header that stretches the topic column to fill the remaining width.
    table_->setEditTriggers( QAbstractItemView::NoEditTriggers );
    table_->setSelectionMode( QAbstractItemView::NoSelection );
    table_->setFocusPolicy( Qt::NoFocus );
    table_->verticalHeader()->setVisible( false );
    table_->horizontalHeader()->setSectionResizeMode( kTopicColumn, QHeaderView::Stretch );
    table_->horizontalHeader()->setSectionResizeMode( kRateColumn, QHeaderView::ResizeToContents );
    table_->horizontalHeader()->setSectionResizeMode( kExpectedColumn, QHeaderView::ResizeToContents );

    // Keep the table tight to its rows; it already sits inside a scroll area.
    table_->setSizeAdjustPolicy( QAbstractScrollArea::AdjustToContents );

    // Bottom-of-control button that opens the shared-config popup.
    configure_button_ = new QPushButton( "Configure Monitored Topics" );
    connect( configure_button_, SIGNAL( clicked() ), this, SLOT( onConfigureTopics() ) );

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->addWidget( table_ );
    layout->addWidget( configure_button_ );
    setLayout( layout );
}

void ComputeComponent::populateTable()
{
    table_->setRowCount( static_cast<int>( topics_.size() ) );

    for ( int row = 0; row < static_cast<int>( topics_.size() ); ++row )
    {
        const MonitoredTopic& topic = *topics_[row];

        table_->setItem( row, kTopicColumn, new QTableWidgetItem( topic.label ) );

        // Rate is unknown until the topic has a publisher and a window elapses.
        QTableWidgetItem* rate_item = new QTableWidgetItem( kUnknownRate );
        rate_item->setTextAlignment( Qt::AlignCenter );
        table_->setItem( row, kRateColumn, rate_item );

        QTableWidgetItem* expected_item = new QTableWidgetItem( formatHz( topic.expected_hz ) );
        expected_item->setTextAlignment( Qt::AlignCenter );
        table_->setItem( row, kExpectedColumn, expected_item );
    }
}

void ComputeComponent::applyConfig()
{
    // Rebuild the monitored topics from the shared config. Clearing the vector
    // destroys the previous subscriptions.
    topics_.clear();
    for ( const MonitoredTopicSpec& spec : s_monitored_topics_ )
    {
        auto topic = std::make_unique<MonitoredTopic>();
        topic->label = spec.suffix;
        topic->topic = ( "/" + vehicle_id_ + "/" + spec.suffix ).toStdString();
        topic->expected_hz = spec.expected_hz;
        topics_.push_back( std::move( topic ) );
    }

    populateTable();
    discoverAndSubscribe();
}

void ComputeComponent::applyGlobalConfig( const std::vector<MonitoredTopicSpec>& specs,
                                          double threshold_fraction )
{
    s_monitored_topics_ = specs;
    s_threshold_fraction_ = threshold_fraction;

    // Re-apply to every live compute component so the change affects all vehicles.
    for ( ComputeComponent* instance : s_instances_ )
    {
        instance->applyConfig();
    }
}

void ComputeComponent::onConfigureTopics()
{
    QDialog dialog( this );
    dialog.setWindowTitle( "Configure Monitored Topics" );
    dialog.resize( 440, 320 );

    // Editable topic suffix + expected rate table, seeded from the shared config.
    QTableWidget* edit_table = new QTableWidget( 0, 2, &dialog );
    edit_table->setHorizontalHeaderLabels( { "Topic", "Expected Hz" } );
    edit_table->horizontalHeader()->setSectionResizeMode( 0, QHeaderView::Stretch );
    edit_table->horizontalHeader()->setSectionResizeMode( 1, QHeaderView::ResizeToContents );
    edit_table->verticalHeader()->setVisible( false );
    for ( const MonitoredTopicSpec& spec : s_monitored_topics_ )
    {
        const int row = edit_table->rowCount();
        edit_table->insertRow( row );
        edit_table->setItem( row, 0, new QTableWidgetItem( spec.suffix ) );
        edit_table->setItem(
            row, 1, new QTableWidgetItem( QString::number( spec.expected_hz, 'g' ) ) );
    }

    // Add / remove row controls.
    QPushButton* add_button = new QPushButton( "Add", &dialog );
    QPushButton* remove_button = new QPushButton( "Remove", &dialog );
    connect( add_button, &QPushButton::clicked, edit_table, [edit_table]()
    {
        const int row = edit_table->rowCount();
        edit_table->insertRow( row );
        edit_table->setItem( row, 0, new QTableWidgetItem( "avt_341/" ) );
        edit_table->setItem( row, 1, new QTableWidgetItem( "30" ) );
        edit_table->editItem( edit_table->item( row, 0 ) );
    } );
    connect( remove_button, &QPushButton::clicked, edit_table, [edit_table]()
    {
        const int row = edit_table->currentRow();
        if ( row >= 0 )
        {
            edit_table->removeRow( row );
        }
    } );

    QHBoxLayout* button_row = new QHBoxLayout;
    button_row->addWidget( add_button );
    button_row->addWidget( remove_button );
    button_row->addStretch();

    // Global alert threshold (a percentage of the expected rate) for all topics.
    QDoubleSpinBox* threshold_spin = new QDoubleSpinBox( &dialog );
    threshold_spin->setRange( 0.0, 100.0 );
    threshold_spin->setDecimals( 0 );
    threshold_spin->setSuffix( " %" );
    threshold_spin->setValue( s_threshold_fraction_ * 100.0 );
    threshold_spin->setToolTip(
        "A topic is flagged when its measured rate drops below this percentage "
        "of its expected rate." );

    QFormLayout* threshold_form = new QFormLayout;
    threshold_form->addRow( "Alert threshold:", threshold_spin );

    QDialogButtonBox* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog );
    connect( buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept );
    connect( buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject );

    QVBoxLayout* layout = new QVBoxLayout( &dialog );
    layout->addWidget(
        new QLabel( "Monitored topics (subscribed at /<vehicle>/<topic>):" ) );
    layout->addWidget( edit_table );
    layout->addLayout( button_row );
    layout->addLayout( threshold_form );
    layout->addWidget( buttons );

    if ( dialog.exec() != QDialog::Accepted )
    {
        return;
    }

    // Read the edited configuration back, skipping rows with a blank topic.
    std::vector<MonitoredTopicSpec> specs;
    for ( int row = 0; row < edit_table->rowCount(); ++row )
    {
        const QTableWidgetItem* topic_item = edit_table->item( row, 0 );
        const QTableWidgetItem* hz_item = edit_table->item( row, 1 );
        const QString suffix = topic_item ? topic_item->text().trimmed() : QString();
        if ( suffix.isEmpty() )
        {
            continue;
        }
        const double hz = hz_item ? hz_item->text().toDouble() : 0.0;
        specs.push_back( { suffix, hz } );
    }

    applyGlobalConfig( specs, threshold_spin->value() / 100.0 );
}

void ComputeComponent::discoverAndSubscribe()
{
    // Without a node we still render the table; rates simply stay unknown. This
    // can happen if the component is built before the panel's node exists.
    if ( !node_ )
    {
        return;
    }

    // Best-effort, shallow-history QoS: we only count arrivals, and a best-effort
    // subscription stays compatible with both reliable and best-effort publishers
    // (e.g. high-rate odometry streams).
    const rclcpp::QoS qos = rclcpp::SensorDataQoS();

    // The graph snapshot is fetched lazily, so once every topic is subscribed
    // (the steady state) this method does no work.
    std::map<std::string, std::vector<std::string>> graph;
    bool graph_loaded = false;

    for ( std::unique_ptr<MonitoredTopic>& entry : topics_ )
    {
        // Stable across vector growth: the MonitoredTopic is owned by unique_ptr,
        // so this pointer remains valid for the subscription callback's lifetime.
        MonitoredTopic* topic = entry.get();
        if ( topic->subscription || topic->subscribe_failed )
        {
            continue;
        }

        if ( !graph_loaded )
        {
            graph = node_->get_topic_names_and_types();
            graph_loaded = true;
        }

        // No publisher has advertised the topic yet; try again next tick.
        const auto it = graph.find( topic->topic );
        if ( it == graph.end() || it->second.empty() )
        {
            continue;
        }

        // Subscribe generically using the discovered type. Messages arrive as raw
        // serialized buffers and are never deserialized; we count arrivals only.
        const std::string& type = it->second.front();
        try
        {
            topic->subscription = node_->create_generic_subscription(
                topic->topic, type, qos,
                [topic]( std::shared_ptr<rclcpp::SerializedMessage> )
                {
                    topic->count.fetch_add( 1, std::memory_order_relaxed );
                } );
        }
        catch ( const std::exception& e )
        {
            // A type whose typesupport cannot be loaded will never succeed, so
            // stop retrying it and surface the reason once.
            topic->subscribe_failed = true;
            RCLCPP_WARN( node_->get_logger(),
                         "ComputeComponent: cannot subscribe to '%s' [%s]: %s",
                         topic->topic.c_str(), type.c_str(), e.what() );
        }
    }
}

void ComputeComponent::updateRates()
{
    bool any_alert = false;
    for ( int row = 0; row < static_cast<int>( topics_.size() ); ++row )
    {
        MonitoredTopic& topic = *topics_[row];

        if ( !topic.subscription )
        {
            // No publisher discovered yet: show unknown, leave the row uncolored.
            table_->item( row, kRateColumn )->setText( kUnknownRate );
            setRowAlert( row, false );
            continue;
        }

        // Consume the window's count atomically so concurrent callbacks (should
        // spinning ever move off the UI thread) are never double-counted.
        const std::uint64_t count = topic.count.exchange( 0, std::memory_order_relaxed );
        const double hz = count * 1000.0 / kMeasurementWindowMs;

        const bool alert = hz < topic.expected_hz * s_threshold_fraction_;
        table_->item( row, kRateColumn )->setText( formatHz( hz ) );
        setRowAlert( row, alert );
        any_alert = any_alert || alert;
    }

    // Report overall health (healthy unless some topic is below threshold) to the
    // Setup table, but only on a real transition.
    const bool healthy = !any_alert;
    if ( !health_known_ || healthy != healthy_ )
    {
        healthy_ = healthy;
        health_known_ = true;
        Q_EMIT healthChanged( healthy );
    }

    // Pick up topics whose publisher has appeared since the last tick. Done after
    // measuring so a just-created subscription is first measured next window,
    // rather than reading an empty count and flashing red for one tick.
    discoverAndSubscribe();
}

void ComputeComponent::setRowAlert( int row, bool alert )
{
    // Paint (or clear) every cell in the row so the whole row reads as healthy
    // or alerting. Default-constructed brushes restore the theme's colors.
    const QBrush background = alert ? QBrush( kAlertBackground ) : QBrush();
    const QBrush foreground = alert ? QBrush( kAlertForeground ) : QBrush();

    for ( int column = 0; column < kColumnCount; ++column )
    {
        QTableWidgetItem* item = table_->item( row, column );
        item->setBackground( background );
        item->setForeground( foreground );
    }
}

}
