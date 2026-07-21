#include <avt_341_rviz_plugins/primitives/nav_goal_visual.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreMath.h>
#include <OgrePass.h>
#include <OgreRenderOperation.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

namespace avt_341::rviz_plugins
{

namespace
{
// Monotonic counter so every visual gets globally-unique Ogre resource names
// (Ogre keys ManualObjects and materials by name across the whole scene).
std::atomic<std::uint64_t> g_instance_counter{ 0 };
}

NavGoalVisual::NavGoalVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
    : scene_manager_( scene_manager )
{
    // Everything is built in this node's local frame; the display drives the node
    // to the goal pose, so local +X is the goal heading and the local X-Y plane is
    // the goal's ground plane.
    frame_node_ = parent_node->createChildSceneNode();

    arrow_ = std::make_unique<rviz_rendering::Arrow>( scene_manager_, frame_node_ );
    circle_ = std::make_unique<rviz_rendering::BillboardLine>( scene_manager_, frame_node_ );

    const std::string id = std::to_string( g_instance_counter++ );

    // A self-illuminated, double-sided, alpha-blended material for the wedge. The
    // colour (including alpha) is supplied per-vertex, so one material serves any
    // fill colour the user picks.
    fill_material_ = Ogre::MaterialManager::getSingleton().create(
        "avt_341_nav_goal_fill_" + id,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::Technique* technique = fill_material_->getNumTechniques() > 0
        ? fill_material_->getTechnique( 0 )
        : fill_material_->createTechnique();
    Ogre::Pass* pass = technique->getNumPasses() > 0
        ? technique->getPass( 0 )
        : technique->createPass();
    fill_material_->setReceiveShadows( false );
    pass->setLightingEnabled( false );
    pass->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    pass->setDepthWriteEnabled( false );
    pass->setCullingMode( Ogre::CULL_NONE );
    pass->setVertexColourTracking( Ogre::TVC_DIFFUSE );

    fill_ = scene_manager_->createManualObject( "avt_341_nav_goal_fill_obj_" + id );
    fill_->setDynamic( true );
    frame_node_->attachObject( fill_ );
}

NavGoalVisual::~NavGoalVisual()
{
    // Drop the rviz_rendering objects (they detach from frame_node_) before the
    // node itself, then release the Ogre-owned manual object and material.
    arrow_.reset();
    circle_.reset();

    if ( fill_ != nullptr )
    {
        frame_node_->detachObject( fill_ );
        scene_manager_->destroyManualObject( fill_ );
        fill_ = nullptr;
    }
    if ( fill_material_ )
    {
        Ogre::MaterialManager::getSingleton().remove( fill_material_->getName() );
        fill_material_.reset();
    }
    scene_manager_->destroySceneNode( frame_node_ );
}

void NavGoalVisual::setPose( const Ogre::Vector3& position, const Ogre::Quaternion& orientation )
{
    frame_node_->setPosition( position );
    frame_node_->setOrientation( orientation );
}

void NavGoalVisual::setThresholds( double dist_threshold, double yaw_threshold )
{
    dist_threshold_ = dist_threshold;
    yaw_threshold_ = yaw_threshold;
}

void NavGoalVisual::setStyle( const NavGoalStyle& style )
{
    style_ = style;
    rebuild();
}

void NavGoalVisual::rebuild()
{
    // Arrow: identical look to the PoseStamped "Arrow" shape. Its identity
    // direction is -Z, so rotate it onto the pose's +X (forward) axis.
    arrow_->set( style_.shaft_length, style_.shaft_radius, style_.head_length, style_.head_radius );
    arrow_->setColor( style_.arrow_color );
    arrow_->setPosition( Ogre::Vector3::ZERO );
    arrow_->setOrientation( Ogre::Quaternion( Ogre::Degree( -90.0f ), Ogre::Vector3::UNIT_Y ) );

    // Resolve the thresholds against built-in fallbacks for messages that leave
    // them unspecified (value < 0): a 1 m circle, and a yaw above pi so the yaw
    // check is "disabled" and the whole circle reads as viable (filled green).
    constexpr double kDefaultDistThreshold = 1.0;             // meters
    const double kDefaultYawThreshold = 2.0 * Ogre::Math::PI; // radians, > pi

    const double radius = dist_threshold_ >= 0.0 ? dist_threshold_ : kDefaultDistThreshold;

    double yaw = yaw_threshold_ >= 0.0 ? yaw_threshold_ : kDefaultYawThreshold;
    const double half_angle = yaw >= Ogre::Math::PI ? Ogre::Math::PI : std::max( 0.0, yaw );

    rebuildCircle( radius );
    rebuildFill( radius, half_angle );
}

void NavGoalVisual::rebuildCircle( double radius )
{
    const int segments = std::max( 3, style_.segments );

    circle_->clear();
    circle_->setMaxPointsPerLine( static_cast<std::uint32_t>( segments + 1 ) );  // +1 closes the loop
    circle_->setNumLines( 1 );
    circle_->setLineWidth( std::max( 0.0f, style_.circle_thickness ) );
    circle_->setColor( style_.circle_color.r, style_.circle_color.g,
                       style_.circle_color.b, style_.circle_color.a );

    if ( radius <= 0.0 )
    {
        return;
    }

    for ( int i = 0; i <= segments; ++i )
    {
        const double a = 2.0 * Ogre::Math::PI * static_cast<double>( i ) / segments;
        circle_->addPoint( Ogre::Vector3( static_cast<float>( radius * std::cos( a ) ),
                                          static_cast<float>( radius * std::sin( a ) ),
                                          0.0f ) );
    }
}

void NavGoalVisual::rebuildFill( double radius, double half_angle )
{
    fill_->clear();

    if ( radius <= 0.0 || half_angle <= 0.0 )
    {
        return;   // nothing viable to fill
    }

    // The wedge is centred on the local +X (forward) axis and spans
    // [-half_angle, +half_angle], built as a triangle fan from the centre. Scale
    // the tessellation to the wedge so a narrow slice still looks smooth while a
    // full circle uses the whole segment budget.
    const double start = -half_angle;
    const double span = 2.0 * half_angle;
    const int segments = std::max( 3, style_.segments );
    const int wedge_segments = std::max( 1,
        static_cast<int>( std::ceil( segments * span / ( 2.0 * Ogre::Math::PI ) ) ) );

    const Ogre::ColourValue& c = style_.fill_color;
    fill_->begin( fill_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST );
    for ( int i = 0; i < wedge_segments; ++i )
    {
        const double a0 = start + span * static_cast<double>( i ) / wedge_segments;
        const double a1 = start + span * static_cast<double>( i + 1 ) / wedge_segments;

        fill_->position( 0.0f, 0.0f, 0.0f );
        fill_->colour( c );
        fill_->position( static_cast<float>( radius * std::cos( a0 ) ),
                         static_cast<float>( radius * std::sin( a0 ) ), 0.0f );
        fill_->colour( c );
        fill_->position( static_cast<float>( radius * std::cos( a1 ) ),
                         static_cast<float>( radius * std::sin( a1 ) ), 0.0f );
        fill_->colour( c );
    }
    fill_->end();
}

}
