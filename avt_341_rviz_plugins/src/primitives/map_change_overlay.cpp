#include <avt_341_rviz_plugins/primitives/map_change_overlay.h>

#include <atomic>
#include <string>
#include <vector>

#include <OgreException.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgrePixelFormat.h>
#include <OgreRenderOperation.h>
#include <OgreRenderQueue.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>

namespace avt_341::rviz_plugins
{

namespace
{
std::atomic<std::uint64_t> g_instance_counter{ 0 };

/// Swatches render at the default priority (100) with a -16 depth push-back.
constexpr Ogre::ushort kRenderPriority = 101;
constexpr float kDepthBias = -8.0f;
} // namespace

MapChangeOverlay::MapChangeOverlay( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
    : scene_manager_( scene_manager )
{
    node_ = parent_node->createChildSceneNode();

    const std::string id = std::to_string( g_instance_counter++ );
    material_ = Ogre::MaterialManager::getSingleton().create(
        "avt_341_map_change_overlay_mat_" + id,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );

    Ogre::Technique* technique = material_->getNumTechniques() > 0
        ? material_->getTechnique( 0 )
        : material_->createTechnique();
    Ogre::Pass* pass = technique->getNumPasses() > 0
        ? technique->getPass( 0 )
        : technique->createPass();

    material_->setReceiveShadows( false );
    pass->setLightingEnabled( false );
    pass->setCullingMode( Ogre::CULL_NONE );
    pass->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    pass->setDepthWriteEnabled( false );
    pass->setDepthBias( kDepthBias, 0.0f );
}

MapChangeOverlay::~MapChangeOverlay()
{
    clear();
    if ( material_ )
    {
        Ogre::MaterialManager::getSingleton().remove( material_->getName() );
        material_.reset();
    }
    scene_manager_->destroySceneNode( node_ );
}

void MapChangeOverlay::clear()
{
    if ( quad_ != nullptr )
    {
        node_->detachObject( quad_ );
        scene_manager_->destroyManualObject( quad_ );
        quad_ = nullptr;
    }
    if ( texture_ )
    {
        Ogre::TextureManager::getSingleton().remove( texture_ );
        texture_.reset();
    }
    tex_unit_ = nullptr;
}

bool MapChangeOverlay::resize( std::uint32_t width, std::uint32_t height, float resolution )
{
    clear();

    if ( width == 0 || height == 0 || resolution <= 0.0f )
    {
        return false;
    }

    const std::string id = std::to_string( g_instance_counter++ );
    try
    {
        texture_ = Ogre::TextureManager::getSingleton().createManual(
            "avt_341_map_change_overlay_tex_" + id,
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_BYTE_RGBA,
            Ogre::TU_DYNAMIC_WRITE_ONLY );

        const std::vector<std::uint8_t> transparent(
            static_cast<std::size_t>( width ) * height * 4u, 0 );
        texture_->getBuffer()->blitFromMemory(
            Ogre::PixelBox( width, height, 1, Ogre::PF_BYTE_RGBA,
                            const_cast<std::uint8_t*>( transparent.data() ) ) );
    }
    catch ( const Ogre::Exception& )
    {
        if ( texture_ )
        {
            Ogre::TextureManager::getSingleton().remove( texture_ );
            texture_.reset();
        }
        return false;
    }

    Ogre::Pass* pass = material_->getTechnique( 0 )->getPass( 0 );
    pass->removeAllTextureUnitStates();
    tex_unit_ = pass->createTextureUnitState();
    tex_unit_->setTexture( texture_ );
    tex_unit_->setTextureAddressingMode( Ogre::TextureUnitState::TAM_CLAMP );
    tex_unit_->setTextureFiltering( Ogre::TFO_NONE );
    tex_unit_->setColourOperation( Ogre::LBO_REPLACE );
    setAlpha( alpha_ );

    const float w = static_cast<float>( width ) * resolution;
    const float h = static_cast<float>( height ) * resolution;

    quad_ = scene_manager_->createManualObject( "avt_341_map_change_overlay_obj_" + id );
    quad_->begin( material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST );

    quad_->position( 0.0f, 0.0f, 0.0f ); quad_->textureCoord( 0.0f, 0.0f );
    quad_->position( w, h, 0.0f );       quad_->textureCoord( 1.0f, 1.0f );
    quad_->position( 0.0f, h, 0.0f );    quad_->textureCoord( 0.0f, 1.0f );

    quad_->position( 0.0f, 0.0f, 0.0f ); quad_->textureCoord( 0.0f, 0.0f );
    quad_->position( w, 0.0f, 0.0f );    quad_->textureCoord( 1.0f, 0.0f );
    quad_->position( w, h, 0.0f );       quad_->textureCoord( 1.0f, 1.0f );

    quad_->end();
    node_->attachObject( quad_ );

    applyRenderQueue();
    quad_->setVisible( visible_ );
    return true;
}

void MapChangeOverlay::upload( const Ogre::Box& box, const std::uint8_t* rgba )
{
    if ( !texture_ || box.getWidth() == 0 || box.getHeight() == 0 )
    {
        return;
    }
    texture_->getBuffer()->blitFromMemory(
        Ogre::PixelBox( box.getWidth(), box.getHeight(), 1, Ogre::PF_BYTE_RGBA,
                        const_cast<std::uint8_t*>( rgba ) ),
        box );
}

void MapChangeOverlay::setAlpha( float alpha )
{
    alpha_ = alpha;
    if ( tex_unit_ != nullptr )
    {
        tex_unit_->setAlphaOperation(
            Ogre::LBX_MODULATE, Ogre::LBS_TEXTURE, Ogre::LBS_MANUAL, 1.0f, alpha_ );
    }
}

void MapChangeOverlay::setDrawUnder( bool under )
{
    draw_under_ = under;
    applyRenderQueue();
}

void MapChangeOverlay::setVisible( bool visible )
{
    visible_ = visible;
    if ( quad_ != nullptr )
    {
        quad_->setVisible( visible_ );
    }
}

void MapChangeOverlay::applyRenderQueue()
{
    if ( quad_ != nullptr )
    {
        quad_->setRenderQueueGroupAndPriority(
            draw_under_ ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN, kRenderPriority );
    }
}

} // namespace avt_341::rviz_plugins
