#include <avt_341_rviz_plugins/primitives/map_marker_visual.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreMath.h>
#include <OgrePass.h>
#include <OgreRenderOperation.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include <avt_341_rviz_plugins/primitives/flat_text.h>

namespace avt_341::rviz_plugins
{

namespace
{
std::atomic<std::uint64_t> g_instance_counter{ 0 };

// Geometry proportions, expressed as fractions of the ring radius.
constexpr float kArrowGap = 0.05f;        // gap between ring edge and arrow base
constexpr float kArrowLength = 0.35f;     // radial length of the arrow
constexpr float kArrowHalfWidth = 0.20f;  // half the arrow base width

// Auto-sized text cap heights, as fractions of the ring radius.
constexpr float kSingleLineHeight = 1.0f;
constexpr float kTopLineHeight = 0.42f;     // small line above (e.g. "MP")
constexpr float kBottomLineHeight = 0.9f;   // large line below (e.g. "E")

// Small vertical lifts (meters) above the marker plane to avoid z-fighting with
// the ground and to keep the text rendering on top of the ring.
constexpr float kShapeLift = 0.03f;
constexpr float kTextLift = 0.06f;
}

MapMarkerVisual::MapMarkerVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
    : scene_manager_( scene_manager )
{
    // Everything is built in this node's local frame; the display drives it to
    // the marker pose, so local +X is the heading and the local X-Y plane is the
    // marker's ground plane.
    frame_node_ = parent_node->createChildSceneNode();

    const std::string id = std::to_string( g_instance_counter++ );

    // Unlit, double-sided, alpha-blended material; colour (incl. alpha) is set
    // per-vertex so one material serves any marker colour.
    shape_material_ = Ogre::MaterialManager::getSingleton().create(
        "avt_341_map_marker_shape_" + id,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::Technique* technique = shape_material_->getNumTechniques() > 0
        ? shape_material_->getTechnique( 0 )
        : shape_material_->createTechnique();
    Ogre::Pass* pass = technique->getNumPasses() > 0
        ? technique->getPass( 0 )
        : technique->createPass();
    shape_material_->setReceiveShadows( false );
    pass->setLightingEnabled( false );
    pass->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    pass->setDepthWriteEnabled( false );
    pass->setCullingMode( Ogre::CULL_NONE );
    pass->setVertexColourTracking( Ogre::TVC_DIFFUSE );

    shape_ = scene_manager_->createManualObject( "avt_341_map_marker_shape_obj_" + id );
    shape_->setDynamic( true );
    frame_node_->attachObject( shape_ );

    text_ = std::make_unique<FlatText>( scene_manager_, frame_node_ );
    // Map the text's local up (+Y) onto the marker heading (+X): a -90 deg turn
    // about +Z. The text then reads "up" toward the heading and rotates with the
    // marker. Lift it slightly so it sits above the ring.
    text_->node()->setOrientation( Ogre::Quaternion( Ogre::Degree( -90.0f ), Ogre::Vector3::UNIT_Z ) );
    text_->node()->setPosition( 0.0f, 0.0f, kTextLift );
}

MapMarkerVisual::~MapMarkerVisual()
{
    text_.reset();   // detaches from frame_node_

    if ( shape_ != nullptr )
    {
        frame_node_->detachObject( shape_ );
        scene_manager_->destroyManualObject( shape_ );
        shape_ = nullptr;
    }
    if ( shape_material_ )
    {
        Ogre::MaterialManager::getSingleton().remove( shape_material_->getName() );
        shape_material_.reset();
    }
    scene_manager_->destroySceneNode( frame_node_ );
}

void MapMarkerVisual::setPose( const Ogre::Vector3& position, const Ogre::Quaternion& orientation )
{
    frame_node_->setPosition( position );
    frame_node_->setOrientation( orientation );
}

void MapMarkerVisual::setMarkerId( const std::string& marker_id )
{
    marker_id_ = marker_id;
}

void MapMarkerVisual::setStyle( const MapMarkerStyle& style )
{
    style_ = style;
    rebuild();
}

void MapMarkerVisual::rebuild()
{
    rebuildRingAndArrow();
    rebuildText();
}

void MapMarkerVisual::rebuildRingAndArrow()
{
    shape_->clear();

    const float radius = std::max( 0.0f, style_.radius );
    if ( radius <= 0.0f )
    {
        return;
    }
    const float thickness = std::max( 0.0f, style_.circle_thickness );
    const float inner = std::max( 0.0f, radius - 0.5f * thickness );
    const float outer = radius + 0.5f * thickness;
    const int segments = std::max( 8, style_.segments );
    const Ogre::ColourValue& c = style_.color;

    const auto vertex = [&]( float x, float y ) {
        shape_->position( x, y, kShapeLift );
        shape_->colour( c );
    };

    shape_->begin( shape_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST );

    // Ring: an annulus made of quads between the inner and outer radii.
    for ( int i = 0; i < segments; ++i )
    {
        const double a0 = 2.0 * Ogre::Math::PI * static_cast<double>( i ) / segments;
        const double a1 = 2.0 * Ogre::Math::PI * static_cast<double>( i + 1 ) / segments;
        const float c0 = static_cast<float>( std::cos( a0 ) );
        const float s0 = static_cast<float>( std::sin( a0 ) );
        const float c1 = static_cast<float>( std::cos( a1 ) );
        const float s1 = static_cast<float>( std::sin( a1 ) );

        vertex( outer * c0, outer * s0 );
        vertex( inner * c0, inner * s0 );
        vertex( outer * c1, outer * s1 );

        vertex( outer * c1, outer * s1 );
        vertex( inner * c0, inner * s0 );
        vertex( inner * c1, inner * s1 );
    }

    // Heading arrow: a triangle just outside the ring, apex pointing along +X.
    const float base_x = outer + kArrowGap * radius;
    const float apex_x = base_x + kArrowLength * radius;
    const float half_w = kArrowHalfWidth * radius;
    vertex( apex_x, 0.0f );
    vertex( base_x, half_w );
    vertex( base_x, -half_w );

    shape_->end();
}

void MapMarkerVisual::rebuildText()
{
    const float radius = std::max( 0.0f, style_.radius );
    const float scale = std::max( 0.0f, style_.text_scale );

    std::vector<FlatTextLine> lines;
    if ( !marker_id_.empty() && radius > 0.0f && scale > 0.0f )
    {
        const std::size_t split = marker_id_.find( '_' );
        if ( split == std::string::npos )
        {
            lines.push_back( { marker_id_, kSingleLineHeight * radius * scale } );
        }
        else
        {
            // Everything before the first underscore goes small on top; the
            // remainder goes large below.
            lines.push_back( { marker_id_.substr( 0, split ), kTopLineHeight * radius * scale } );
            lines.push_back( { marker_id_.substr( split + 1 ), kBottomLineHeight * radius * scale } );
        }
    }

    text_->setLines( lines, style_.color );
}

}
