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
#include <QFont>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

#include <avt_341_rviz_plugins/components/topic_config.h>
#include <avt_341_rviz_plugins/primitives/status_style.h>

namespace
{

// Window over which messages are counted to derive a rate. One second gives
// ~1 Hz display resolution, which is plenty for this check.
constexpr int kMeasurementWindowMs = 1000;

// Status table column layout.
enum TopicColumn
{
    kTopicColumn = 0,
    kRateColumn = 1,
    kExpectedColumn = 2,
    kColumnCount = 3
};

// Code-section tree column layout.
enum CodeSectionColumn
{
    kSectionColumn = 0,
    kMeanColumn = 1,
    kStdDevColumn = 2,
    kWindowColumn = 3,
    kWarningColumn = 4,
    kCodeSectionColumnCount = 5
};

// Shown in the "Rate" column before a topic has a publisher to measure.
constexpr const char* kUnknownRate = "-";

// Foreground used to flag a topic running below threshold; the background comes
// from the shared status palette (status_colors::kRed).
const QColor kAlertForeground( Qt::white );

QString formatHz( double hz )
{
    return QString::number( hz, 'f', 1 ) + " Hz";
}

QString compactNumber( double value )
{
    QString text = QString::number( value, 'f', 2 );
    while ( text.endsWith( '0' ) )
    {
        text.chop( 1 );
    }
    if ( text.endsWith( '.' ) )
    {
        text.chop( 1 );
    }
    return text;
}

QString formatDuration( double seconds )
{
    if ( seconds < 0.0001 )
    {
        return "< 0.1 ms";
    }
    if ( seconds < 1.0 )
    {
        return compactNumber( seconds * 1000.0 ) + " ms";
    }
    return compactNumber( seconds ) + " s";
}

QString formatWindow( std::int32_t window_num_samples, float window_time )
{
    if ( window_num_samples > 0 )
    {
        return QString::number( window_num_samples ) + " samples";
    }
    if ( window_time > 0.0f )
    {
        return QString::number( window_time, 'f', 2 ) + " s";
    }
    return "All samples";
}

std::string normalizeSectionId( const std::string& section_id )
{
    std::string normalized;
    std::size_t start = 0;
    while ( start < section_id.size() )
    {
        const std::size_t slash = section_id.find( '/', start );
        const std::size_t length =
            slash == std::string::npos ? std::string::npos : slash - start;
        const std::string segment = section_id.substr( start, length );
        if ( !segment.empty() )
        {
            if ( !normalized.empty() )
            {
                normalized += '/';
            }
            normalized += segment;
        }

        if ( slash == std::string::npos )
        {
            break;
        }
        start = slash + 1;
    }
    return normalized;
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

    // Every node for this vehicle publishes timing summaries to the same topic.
    // The callback merges each message into the tree instead of treating it as
    // a complete snapshot of the topic.
    if ( node_ )
    {
        const std::string topic =
            makeTopicPath( vehicle_id_, "avt_341/compute_times" );
        compute_times_subscription_ =
            node_->create_subscription<avt_341_msgs::msg::ComputeTimeArray>(
                topic, rclcpp::QoS( 10 ),
                [this]( avt_341_msgs::msg::ComputeTimeArray::ConstSharedPtr msg )
                {
                    updateCodeSections( *msg );
                } );
    }
}

ComputeComponent::~ComputeComponent()
{
    s_instances_.erase(
        std::remove( s_instances_.begin(), s_instances_.end(), this ),
        s_instances_.end() );
}

void ComputeComponent::buildUi()
{
    topics_table_ = new QTableWidget( 0, kColumnCount );
    topics_table_->setHorizontalHeaderLabels( { "Topic", "Rate", "Expected" } );

    // A read-only status grid: no editing, selection, focus or row numbers, and
    // a header that stretches the topic column to fill the remaining width.
    topics_table_->setEditTriggers( QAbstractItemView::NoEditTriggers );
    topics_table_->setSelectionMode( QAbstractItemView::NoSelection );
    topics_table_->setFocusPolicy( Qt::NoFocus );
    topics_table_->verticalHeader()->setVisible( false );
    topics_table_->horizontalHeader()->setSectionResizeMode(
        kTopicColumn, QHeaderView::Stretch );
    topics_table_->horizontalHeader()->setSectionResizeMode(
        kRateColumn, QHeaderView::ResizeToContents );
    topics_table_->horizontalHeader()->setSectionResizeMode(
        kExpectedColumn, QHeaderView::ResizeToContents );

    // Keep the table tight to its rows; it already sits inside a scroll area.
    topics_table_->setSizeAdjustPolicy( QAbstractScrollArea::AdjustToContents );

    // Bottom-of-control button that opens the shared-config popup.
    configure_button_ = new QPushButton( "Configure Monitored Topics" );
    connect( configure_button_, SIGNAL( clicked() ), this, SLOT( onConfigureTopics() ) );

    QWidget* topics_page = new QWidget;
    QVBoxLayout* topics_layout = new QVBoxLayout( topics_page );
    topics_layout->setContentsMargins( 0, 0, 0, 0 );
    topics_layout->addWidget( topics_table_ );
    topics_layout->addWidget( configure_button_ );

    // A QTreeWidget gives the code-section view the same tabular presentation
    // as the topic-rate table while reconstructing section_id paths as an
    // expandable hierarchy.
    code_sections_tree_ = new QTreeWidget;
    code_sections_tree_->setColumnCount( kCodeSectionColumnCount );
    code_sections_tree_->setHeaderLabels(
        { "Section", "Mean", "Std Dev", "Window", "Warning" } );
    code_sections_tree_->setEditTriggers( QAbstractItemView::NoEditTriggers );
    code_sections_tree_->setSelectionMode( QAbstractItemView::NoSelection );
    code_sections_tree_->setFocusPolicy( Qt::NoFocus );
    code_sections_tree_->setRootIsDecorated( true );
    code_sections_tree_->setUniformRowHeights( true );
    code_sections_tree_->setSortingEnabled( true );
    code_sections_tree_->header()->setSectionResizeMode(
        kSectionColumn, QHeaderView::Stretch );
    for ( int column = kMeanColumn; column < kCodeSectionColumnCount; ++column )
    {
        code_sections_tree_->header()->setSectionResizeMode(
            column, QHeaderView::ResizeToContents );
    }
    code_sections_tree_->setSizeAdjustPolicy(
        QAbstractScrollArea::AdjustToContents );

    QWidget* code_sections_page = new QWidget;
    QVBoxLayout* code_sections_layout = new QVBoxLayout( code_sections_page );
    code_sections_layout->setContentsMargins( 0, 0, 0, 0 );
    code_sections_layout->addWidget( code_sections_tree_ );

    sub_tabs_ = new QTabWidget;
    sub_tabs_->setTabPosition( QTabWidget::North );
    sub_tabs_->addTab( topics_page, "Topics" );
    sub_tabs_->addTab( code_sections_page, "Sections" );

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->addWidget( sub_tabs_ );
    setLayout( layout );
}

void ComputeComponent::populateTable()
{
    topics_table_->setRowCount( static_cast<int>( topics_.size() ) );

    for ( int row = 0; row < static_cast<int>( topics_.size() ); ++row )
    {
        const MonitoredTopic& topic = *topics_[row];

        topics_table_->setItem(
            row, kTopicColumn, new QTableWidgetItem( topic.label ) );

        // Rate is unknown until the topic has a publisher and a window elapses.
        QTableWidgetItem* rate_item = new QTableWidgetItem( kUnknownRate );
        rate_item->setTextAlignment( Qt::AlignCenter );
        topics_table_->setItem( row, kRateColumn, rate_item );

        QTableWidgetItem* expected_item = new QTableWidgetItem( formatHz( topic.expected_hz ) );
        expected_item->setTextAlignment( Qt::AlignCenter );
        topics_table_->setItem( row, kExpectedColumn, expected_item );
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
        topic->topic = makeTopicPath( vehicle_id_, spec.suffix );
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
            topics_table_->item( row, kRateColumn )->setText( kUnknownRate );
            setTopicRowAlert( row, false );
            continue;
        }

        // Consume the window's count atomically so concurrent callbacks (should
        // spinning ever move off the UI thread) are never double-counted.
        const std::uint64_t count = topic.count.exchange( 0, std::memory_order_relaxed );
        const double hz = count * 1000.0 / kMeasurementWindowMs;

        const bool alert = hz < topic.expected_hz * s_threshold_fraction_;
        topics_table_->item( row, kRateColumn )->setText( formatHz( hz ) );
        setTopicRowAlert( row, alert );
        any_alert = any_alert || alert;
    }

    topic_alert_ = any_alert;
    updateOverallHealth();

    // Pick up topics whose publisher has appeared since the last tick. Done after
    // measuring so a just-created subscription is first measured next window,
    // rather than reading an empty count and flashing red for one tick.
    discoverAndSubscribe();
}

void ComputeComponent::setTopicRowAlert( int row, bool alert )
{
    // Paint (or clear) every cell in the row so the whole row reads as healthy
    // or alerting. Default-constructed brushes restore the theme's colors.
    const QBrush background = alert ? QBrush( status_colors::kRed ) : QBrush();
    const QBrush foreground = alert ? QBrush( kAlertForeground ) : QBrush();

    for ( int column = 0; column < kColumnCount; ++column )
    {
        QTableWidgetItem* item = topics_table_->item( row, column );
        item->setBackground( background );
        item->setForeground( foreground );
    }
}

void ComputeComponent::updateCodeSections(
    const avt_341_msgs::msg::ComputeTimeArray& msg )
{
    // The tag identifies the publishing node. Keep an explicit bucket for an
    // empty optional tag so its section ids remain separate and visible too.
    const std::string source = msg.tag.empty() ? "<untagged>" : msg.tag;

    for ( const avt_341_msgs::msg::ComputeTime& timing : msg.compute_times )
    {
        const std::string section_id = normalizeSectionId( timing.section_id );
        if ( section_id.empty() )
        {
            continue;
        }

        CodeSection section;
        section.time = timing.time;
        section.time_std = timing.time_std;
        section.window_num_samples = timing.window_num_samples;
        section.window_time = timing.window_time;
        section.warning_threshold = timing.warning_threshold;
        section.auto_parent_stats = timing.auto_parent_stats;

        // insert_or_assign is the dictionary replacement: only ids present in
        // this message change; every previously seen id remains in the map.
        code_sections_[source].insert_or_assign( section_id, section );
        updateCodeSectionItem(
            ensureCodeSectionItem( source, section_id ), section );
    }

    code_section_alert_ = false;
    for ( const auto& [source_name, sections] : code_sections_ )
    {
        (void)source_name;
        for ( const auto& [section_id, section] : sections )
        {
            (void)section_id;
            if ( section.warning_threshold > 0.0f &&
                 section.time > section.warning_threshold )
            {
                code_section_alert_ = true;
                break;
            }
        }
        if ( code_section_alert_ )
        {
            break;
        }
    }
    updateOverallHealth();
}

QTreeWidgetItem* ComputeComponent::ensureCodeSectionItem(
    const std::string& source, const std::string& section_id )
{
    QTreeWidgetItem* source_item = nullptr;
    const auto source_it = code_source_items_.find( source );
    if ( source_it == code_source_items_.end() )
    {
        source_item = new QTreeWidgetItem( code_sections_tree_ );
        source_item->setText( kSectionColumn, QString::fromStdString( source ) );
        source_item->setFirstColumnSpanned( true );
        source_item->setExpanded( true );
        QFont source_font = source_item->font( kSectionColumn );
        source_font.setBold( true );
        source_item->setFont( kSectionColumn, source_font );
        code_source_items_.emplace( source, source_item );
    }
    else
    {
        source_item = source_it->second;
    }

    QTreeWidgetItem* parent = source_item;
    std::string path;
    std::size_t start = 0;
    while ( start < section_id.size() )
    {
        const std::size_t slash = section_id.find( '/', start );
        const std::size_t length =
            slash == std::string::npos ? std::string::npos : slash - start;
        const std::string segment = section_id.substr( start, length );
        if ( !segment.empty() )
        {
            if ( !path.empty() )
            {
                path += '/';
            }
            path += segment;

            const auto key = std::make_pair( source, path );
            const auto item_it = code_section_items_.find( key );
            if ( item_it == code_section_items_.end() )
            {
                QTreeWidgetItem* item = new QTreeWidgetItem( parent );
                item->setText( kSectionColumn,
                               QString::fromStdString( segment ) );
                for ( int column = kMeanColumn;
                      column < kCodeSectionColumnCount; ++column )
                {
                    item->setText( column, kUnknownRate );
                    item->setTextAlignment( column, Qt::AlignCenter );
                }
                item->setExpanded( true );
                code_section_items_.emplace( key, item );
                parent = item;
            }
            else
            {
                parent = item_it->second;
            }
        }

        if ( slash == std::string::npos )
        {
            break;
        }
        start = slash + 1;
    }

    return parent;
}

void ComputeComponent::updateCodeSectionItem(
    QTreeWidgetItem* item, const CodeSection& section )
{
    item->setText( kMeanColumn, formatDuration( section.time ) );
    item->setText( kStdDevColumn, formatDuration( section.time_std ) );
    item->setText( kWindowColumn,
                   formatWindow( section.window_num_samples,
                                 section.window_time ) );
    item->setText(
        kWarningColumn,
        section.warning_threshold > 0.0f
            ? formatDuration( section.warning_threshold )
            : QString( kUnknownRate ) );

    QString tooltip;
    if ( section.auto_parent_stats )
    {
        tooltip = "Statistics automatically calculated from child sections.";
    }
    for ( int column = 0; column < kCodeSectionColumnCount; ++column )
    {
        item->setToolTip( column, tooltip );
    }

    const bool alert = section.warning_threshold > 0.0f &&
                       section.time > section.warning_threshold;
    const QBrush background = alert ? QBrush( status_colors::kRed ) : QBrush();
    const QBrush foreground = alert ? QBrush( kAlertForeground ) : QBrush();
    for ( int column = 0; column < kCodeSectionColumnCount; ++column )
    {
        item->setBackground( column, background );
        item->setForeground( column, foreground );
    }
}

void ComputeComponent::updateOverallHealth()
{
    const bool healthy = !topic_alert_ && !code_section_alert_;
    if ( !health_known_ || healthy != healthy_ )
    {
        healthy_ = healthy;
        health_known_ = true;
        Q_EMIT healthChanged( healthy );
    }
}

}
