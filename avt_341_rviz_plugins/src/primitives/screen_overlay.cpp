#include <avt_341_rviz_plugins/primitives/screen_overlay.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>

#include <OgreException.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgrePixelFormat.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>
#include <OgreViewport.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgreOverlaySystem.h>
#include <Overlay/OgrePanelOverlayElement.h>

namespace avt_341::rviz_plugins
{

namespace
{
std::atomic<std::uint64_t> g_instance_counter{ 0 };

/// Overlays draw in ascending z-order (0-650); stay above typical HUD text.
constexpr Ogre::ushort kZOrder = 500;

bool isRight( ScreenCorner corner )
{
    return corner == ScreenCorner::TopRight || corner == ScreenCorner::BottomRight;
}

bool isBottom( ScreenCorner corner )
{
    return corner == ScreenCorner::BottomLeft || corner == ScreenCorner::BottomRight;
}

/// Clamp a box's position along one axis to [0, extent - size]. When the box is
/// larger than the extent it is pinned to the anchor side instead.
double clampAxis( double position, double size, double extent, bool far_anchor )
{
    const double max_position = extent - size;
    if ( max_position < 0.0 )
    {
        return far_anchor ? max_position : 0.0;
    }
    return std::clamp( position, 0.0, max_position );
}
} // namespace

QPointF AnchoredTopLeft( ScreenCorner corner, const QPointF& offset,
                         const QSizeF& size, const QSizeF& viewport )
{
    const bool right = isRight( corner );
    const bool bottom = isBottom( corner );
    const double x = right ? viewport.width() - size.width() - offset.x() : offset.x();
    const double y = bottom ? viewport.height() - size.height() - offset.y() : offset.y();
    return QPointF( clampAxis( x, size.width(), viewport.width(), right ),
                    clampAxis( y, size.height(), viewport.height(), bottom ) );
}

QPointF AnchorOffset( ScreenCorner corner, const QPointF& top_left,
                      const QSizeF& size, const QSizeF& viewport )
{
    return QPointF(
        isRight( corner ) ? viewport.width() - size.width() - top_left.x() : top_left.x(),
        isBottom( corner ) ? viewport.height() - size.height() - top_left.y() : top_left.y() );
}

ScreenOverlay::ScreenOverlay( Ogre::SceneManager* scene_manager,
                              rviz_rendering::RenderWindow* render_window )
    : render_window_( render_window )
{
    // The overlay system queues overlays for rendering from a render queue
    // listener on the scene manager. rviz_rendering's prepareOverlays() adds it
    // unconditionally, and RViz may already have done so for the main scene; a
    // second registration would draw every overlay twice (doubling translucent
    // backgrounds). Remove-then-add leaves exactly one.
    if ( Ogre::OverlaySystem* overlay_system = Ogre::OverlaySystem::getSingletonPtr() )
    {
        scene_manager->removeRenderQueueListener( overlay_system );
        scene_manager->addRenderQueueListener( overlay_system );
    }

    const std::string id = std::to_string( g_instance_counter++ );

    material_ = Ogre::MaterialManager::getSingleton().create(
        "avt_341_screen_overlay_mat_" + id,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::Technique* technique = material_->getNumTechniques() > 0
        ? material_->getTechnique( 0 )
        : material_->createTechnique();
    Ogre::Pass* pass = technique->getNumPasses() > 0
        ? technique->getPass( 0 )
        : technique->createPass();
    material_->setReceiveShadows( false );
    pass->setLightingEnabled( false );
    pass->setDepthCheckEnabled( false );
    pass->setDepthWriteEnabled( false );
    pass->setCullingMode( Ogre::CULL_NONE );
    // QImage's ARGB32_Premultiplied: color is already scaled by alpha.
    pass->setSceneBlending( Ogre::SBF_ONE, Ogre::SBF_ONE_MINUS_SOURCE_ALPHA );
    tex_unit_ = pass->createTextureUnitState();
    tex_unit_->setTextureAddressingMode( Ogre::TextureUnitState::TAM_CLAMP );
    // The texture maps 1:1 onto screen pixels; any filtering would only blur.
    tex_unit_->setTextureFiltering( Ogre::TFO_NONE );
    tex_unit_->setColourOperationEx( Ogre::LBX_SOURCE1, Ogre::LBS_TEXTURE, Ogre::LBS_CURRENT );
    tex_unit_->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_TEXTURE, Ogre::LBS_CURRENT );

    Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();
    overlay_ = overlay_manager.create( "avt_341_screen_overlay_" + id );
    panel_ = static_cast<Ogre::PanelOverlayElement*>(
        overlay_manager.createOverlayElement( "Panel", "avt_341_screen_overlay_panel_" + id ) );
    panel_->setMetricsMode( Ogre::GMM_PIXELS );
    overlay_->add2D( panel_ );
    overlay_->setZOrder( kZOrder );
    overlay_->hide();

    rviz_rendering::RenderWindowOgreAdapter::addListener( render_window, this );
}

ScreenOverlay::~ScreenOverlay()
{
    // The render window (and with it its Ogre target) may already be gone at
    // shutdown, in which case there is no listener left to remove.
    if ( render_window_ )
    {
        rviz_rendering::RenderWindowOgreAdapter::removeListener( render_window_, this );
    }

    Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();
    overlay_->remove2D( panel_ );
    overlay_manager.destroyOverlayElement( panel_ );
    overlay_manager.destroy( overlay_ );

    Ogre::MaterialManager::getSingleton().remove( material_->getName() );
    material_.reset();
    if ( texture_ )
    {
        Ogre::TextureManager::getSingleton().remove( texture_ );
        texture_.reset();
    }
}

bool ScreenOverlay::setImage( const QImage& image )
{
    has_image_ = false;
    size_ = QSizeF();
    if ( image.isNull() )
    {
        return true;
    }

    // ARGB32 is stored B, G, R, A per pixel on little-endian hosts, which is
    // Ogre's PF_BYTE_BGRA; scanlines are 4-byte aligned, so rows are packed.
    const QImage bgra = image.convertToFormat( QImage::Format_ARGB32_Premultiplied );
    const auto width = static_cast<std::uint32_t>( bgra.width() );
    const auto height = static_cast<std::uint32_t>( bgra.height() );

    try
    {
        if ( !texture_ || texture_->getWidth() != width || texture_->getHeight() != height )
        {
            if ( texture_ )
            {
                Ogre::TextureManager::getSingleton().remove( texture_ );
                texture_.reset();
            }
            texture_ = Ogre::TextureManager::getSingleton().createManual(
                "avt_341_screen_overlay_tex_" + std::to_string( g_instance_counter++ ),
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_BYTE_BGRA,
                Ogre::TU_DYNAMIC_WRITE_ONLY );
            tex_unit_->setTexture( texture_ );
            panel_->setMaterial( material_ );
        }
        texture_->getBuffer()->blitFromMemory(
            Ogre::PixelBox( width, height, 1, Ogre::PF_BYTE_BGRA,
                            const_cast<uchar*>( bgra.constBits() ) ) );
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

    has_image_ = true;
    image_pixels_ = bgra.size();
    image_ratio_ = image.devicePixelRatio() > 0.0 ? image.devicePixelRatio() : 1.0;
    size_ = QSizeF( image_pixels_.width() / image_ratio_, image_pixels_.height() / image_ratio_ );
    return true;
}

void ScreenOverlay::setPlacement( ScreenCorner corner, const QPointF& offset )
{
    corner_ = corner;
    offset_ = offset;
}

void ScreenOverlay::setVisible( bool visible )
{
    visible_ = visible;
}

double ScreenOverlay::pixelRatio() const
{
    return render_window_ ? render_window_->devicePixelRatio() : 1.0;
}

QSizeF ScreenOverlay::viewportSize() const
{
    return render_window_ ? QSizeF( render_window_->width(), render_window_->height() ) : QSizeF();
}

QRectF ScreenOverlay::rect() const
{
    if ( !has_image_ )
    {
        return QRectF();
    }
    return QRectF( AnchoredTopLeft( corner_, offset_, size_, viewportSize() ), size_ );
}

void ScreenOverlay::preViewportUpdate( const Ogre::RenderTargetViewportEvent& evt )
{
    const QSizeF logical = viewportSize();
    if ( !visible_ || !has_image_ || evt.source == nullptr || logical.isEmpty() )
    {
        return;
    }

    // Device pixels per logical pixel, measured on the viewport being drawn.
    // Ogre's pixel metrics are viewport (device) pixels: RViz leaves the overlay
    // manager's HiDPI ratio at 1 (and Humble's Ogre 1.12.1 has no accessor for it).
    const double scale_x = evt.source->getActualWidth() / logical.width();
    const double scale_y = evt.source->getActualHeight() / logical.height();

    // Whole device pixels keep the texture's texels on screen pixels (crisp).
    const QPointF top_left = AnchoredTopLeft( corner_, offset_, size_, logical );
    panel_->setPosition( static_cast<Ogre::Real>( std::round( top_left.x() * scale_x ) ),
                         static_cast<Ogre::Real>( std::round( top_left.y() * scale_y ) ) );
    panel_->setDimensions(
        static_cast<Ogre::Real>( image_pixels_.width() * scale_x / image_ratio_ ),
        static_cast<Ogre::Real>( image_pixels_.height() * scale_y / image_ratio_ ) );
    overlay_->show();
}

void ScreenOverlay::postViewportUpdate( const Ogre::RenderTargetViewportEvent& /*evt*/ )
{
    overlay_->hide();
}

} // namespace avt_341::rviz_plugins
