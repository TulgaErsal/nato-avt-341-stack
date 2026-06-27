#include <avt_341_rviz_plugins/components/nav_state_component.h>

#include <QVBoxLayout>

namespace avt_341::rviz_plugins
{

NavStateComponent::NavStateComponent( const QString& vehicle_id, QWidget* parent )
    : QWidget( parent ), vehicle_id_( vehicle_id )
{
    // Create widgets
    placeholder_label_ = new QLabel( vehicle_id_ + " Nav State" );

    // Layout widgets
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget( placeholder_label_, 0, Qt::AlignCenter );
    setLayout( layout );
}

}
