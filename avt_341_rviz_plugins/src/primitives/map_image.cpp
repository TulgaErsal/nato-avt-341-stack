#include <avt_341_rviz_plugins/primitives/map_image.h>

#include <atomic>
#include <cstdint>
#include <string>

#include <QImage>

#include <OgreDataStream.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgrePixelFormat.h>
#include <OgreRenderOperation.h>
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
// Monotonic counter so every texture / manual object / material gets a globally
// unique Ogre resource name (Ogre keys these by name across the whole scene).
std::atomic<std::uint64_t> g_instance_counter{ 0 };
} // namespace

MapImage::MapImage( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
    : scene_manager_( scene_manager )
{
    node_ = parent_node->createChildSceneNode();

    // One self-illuminated, double-sided, alpha-blended material. The texture unit
    // (and its texture) is (re)created in setImage(); the pass state below is fixed.
    const std::string id = std::to_string( g_instance_counter++ );
    material_ = Ogre::MaterialManager::getSingleton().create(
        "avt_341_map_image_mat_" + id,
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
}

MapImage::~MapImage()
{
    clearGeometry();
    if ( material_ )
    {
        Ogre::MaterialManager::getSingleton().remove( material_->getName() );
        material_.reset();
    }
    scene_manager_->destroySceneNode( node_ );
}

void MapImage::clearGeometry()
{
    if ( quad_ != nullptr )
    {
        node_->detachObject( quad_ );
        scene_manager_->destroyManualObject( quad_ );
        quad_ = nullptr;
    }
    if ( texture_ )
    {
        // Remove by pointer, not by name: the name-based overload defaults to the
        // "General" group and asserts if the resource isn't found there.
        Ogre::TextureManager::getSingleton().remove( texture_ );
        texture_.reset();
    }
    tex_unit_ = nullptr;
}

void MapImage::setImage( const QImage& image, float resolution )
{
    clearGeometry();

    if ( image.isNull() || image.width() <= 0 || image.height() <= 0 || resolution <= 0.0f )
    {
        return;
    }

    // QImage rows are top-to-bottom; Format_RGBA8888 is byte-ordered R,G,B,A which
    // matches Ogre::PF_BYTE_RGBA. For this format bytesPerLine == width * 4, so the
    // buffer is tightly packed and can be handed to Ogre directly.
    const QImage rgba = image.convertToFormat( QImage::Format_RGBA8888 );
    const auto width = static_cast<std::uint32_t>( rgba.width() );
    const auto height = static_cast<std::uint32_t>( rgba.height() );

    const std::string id = std::to_string( g_instance_counter++ );

    // Wrap the pixel buffer for upload. loadRawData() reads it synchronously (copies
    // into the GPU texture) while `rgba` is still alive, so the stream must NOT own or
    // free the QImage's buffer: freeOnClose = false, readOnly = true.
    const std::size_t byte_count = static_cast<std::size_t>( width ) * height * 4u;
    Ogre::DataStreamPtr stream(
        new Ogre::MemoryDataStream(
            const_cast<uchar*>( rgba.constBits() ), byte_count, false, true ) );
    texture_ = Ogre::TextureManager::getSingleton().loadRawData(
        "avt_341_map_image_tex_" + id,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, stream,
        static_cast<std::uint16_t>( width ), static_cast<std::uint16_t>( height ),
        Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_2D, 0 );

    // Point a fresh texture unit at the new texture: RGB straight from the texture,
    // alpha = texture-alpha * global-alpha (LBS_MANUAL), so opacity is adjustable live.
    // Bind the TexturePtr directly rather than by name so Ogre never does a name/group
    // lookup (which would try, and fail, to load it as a file).
    Ogre::Pass* pass = material_->getTechnique( 0 )->getPass( 0 );
    pass->removeAllTextureUnitStates();
    tex_unit_ = pass->createTextureUnitState();
    tex_unit_->setTexture( texture_ );
    tex_unit_->setTextureAddressingMode( Ogre::TextureUnitState::TAM_CLAMP );
    tex_unit_->setColourOperation( Ogre::LBO_REPLACE );
    setAlpha( alpha_ );
    setInterpolation( linear_ );

    // Build the quad in local meters: lower-left corner at the node origin, extending
    // +X (width) and +Y (height). Texture v is flipped so image row 0 (top) maps to
    // the +Y edge, i.e. the image is shown upright.
    const float w = static_cast<float>( width ) * resolution;
    const float h = static_cast<float>( height ) * resolution;

    quad_ = scene_manager_->createManualObject( "avt_341_map_image_obj_" + id );
    quad_->begin( material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST );

    // Triangle 1: LL, LR, UR
    quad_->position( 0.0f, 0.0f, 0.0f ); quad_->textureCoord( 0.0f, 1.0f );
    quad_->position( w, 0.0f, 0.0f );    quad_->textureCoord( 1.0f, 1.0f );
    quad_->position( w, h, 0.0f );       quad_->textureCoord( 1.0f, 0.0f );
    // Triangle 2: LL, UR, UL
    quad_->position( 0.0f, 0.0f, 0.0f ); quad_->textureCoord( 0.0f, 1.0f );
    quad_->position( w, h, 0.0f );       quad_->textureCoord( 1.0f, 0.0f );
    quad_->position( 0.0f, h, 0.0f );    quad_->textureCoord( 0.0f, 0.0f );

    quad_->end();
    node_->attachObject( quad_ );

    setDrawUnder( draw_under_ );
}

void MapImage::setAlpha( float alpha )
{
    alpha_ = alpha;
    if ( tex_unit_ != nullptr )
    {
        tex_unit_->setAlphaOperation(
            Ogre::LBX_MODULATE, Ogre::LBS_TEXTURE, Ogre::LBS_MANUAL, 1.0f, alpha_ );
    }
}

void MapImage::setDrawUnder( bool under )
{
    draw_under_ = under;
    if ( quad_ != nullptr )
    {
        quad_->setRenderQueueGroup(
            under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN );
    }
}

void MapImage::setInterpolation( bool linear )
{
    linear_ = linear;
    if ( tex_unit_ != nullptr )
    {
        tex_unit_->setTextureFiltering( linear ? Ogre::TFO_BILINEAR : Ogre::TFO_NONE );
    }
}

} // namespace avt_341::rviz_plugins
