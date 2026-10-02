#ifndef SCREEN_OVERLAY_H
#define SCREEN_OVERLAY_H

#include <QImage>
#include <QPointer>
#include <QPointF>
#include <QRectF>
#include <QSize>
#include <QSizeF>

#include <OgrePrerequisites.h>
#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreTexture.h>

#include <rviz_rendering/render_window.hpp>

namespace Ogre
{
class Overlay;
class PanelOverlayElement;
class SceneManager;
class TextureUnitState;
}

namespace avt_341 {
namespace rviz_plugins {

/// Viewport corner a ScreenOverlay is anchored to. The values are the Legend
/// display's "Anchor" enum option ints and must not be renumbered.
enum class ScreenCorner
{
    TopLeft = 0,
    TopRight = 1,
    BottomLeft = 2,
    BottomRight = 3
};

/// Top-left of a `size` box placed `offset` inward from `corner` of a
/// `viewport`-sized area: offset.x is the distance from the corner's vertical
/// edge to the box's nearest edge, offset.y likewise from its horizontal edge.
/// The result is clamped to keep the box on screen; a box larger than the
/// viewport keeps its anchor-side edges visible.
QPointF AnchoredTopLeft( ScreenCorner corner, const QPointF& offset,
                         const QSizeF& size, const QSizeF& viewport );

/// Inverse of AnchoredTopLeft (without clamping): the offset from `corner` that
/// puts the box's top-left at `top_left`.
QPointF AnchorOffset( ScreenCorner corner, const QPointF& top_left,
                      const QSizeF& size, const QSizeF& viewport );

/// A QImage drawn as a screen-space overlay on one RViz render window, at a fixed
/// pixel offset from a viewport corner. The image is unaffected by the camera -
/// zooming, orbiting and panning neither move nor scale it.
///
/// Rendering is an Ogre overlay panel textured with the image. Overlays render on
/// every viewport of the scene manager that has overlays enabled, which would
/// include other views of the main scene (e.g. a Camera display's panel), so the
/// overlay is shown only for the duration of its own render window's viewport
/// update. That update is also where it is placed: the viewport's actual pixel
/// size at render time maps the logical (Qt) placement onto device pixels, so a
/// resized window keeps the overlay at its anchor without a frame of lag.
///
/// Placement and hit-testing use the render window's logical pixels (the units of
/// its QMouseEvents). The image may be rendered at a higher device pixel ratio;
/// its logical size is its pixel size divided by its devicePixelRatio.
class ScreenOverlay : private Ogre::RenderTargetListener
{
public:
    ScreenOverlay( Ogre::SceneManager* scene_manager, rviz_rendering::RenderWindow* render_window );
    ~ScreenOverlay() override;

    ScreenOverlay( const ScreenOverlay& ) = delete;
    ScreenOverlay& operator=( const ScreenOverlay& ) = delete;

    /// Replace the drawn image; a null image hides the overlay. Returns false
    /// (and hides the overlay) when the GPU refuses a texture that size.
    bool setImage( const QImage& image );

    /// Anchor corner and inward offset (logical px) of the overlay.
    void setPlacement( ScreenCorner corner, const QPointF& offset );

    void setVisible( bool visible );

    /// Device pixels per logical pixel of the render window: the ratio at which
    /// to render images for crisp output.
    double pixelRatio() const;

    /// Logical size of the render window (zero if it is gone).
    QSizeF viewportSize() const;

    /// Logical size of the current image (empty when there is none).
    QSizeF size() const { return size_; }

    /// Where the overlay currently is in render-window logical pixels (empty
    /// when it has no image).
    QRectF rect() const;

private:
    void preViewportUpdate( const Ogre::RenderTargetViewportEvent& evt ) override;
    void postViewportUpdate( const Ogre::RenderTargetViewportEvent& evt ) override;

    QPointer<rviz_rendering::RenderWindow> render_window_;

    Ogre::Overlay* overlay_ = nullptr;
    Ogre::PanelOverlayElement* panel_ = nullptr;
    Ogre::MaterialPtr material_;
    Ogre::TextureUnitState* tex_unit_ = nullptr;
    Ogre::TexturePtr texture_;

    bool has_image_ = false;
    QSize image_pixels_;        ///< Image size in device pixels (= texture size).
    double image_ratio_ = 1.0;  ///< Device pixels per logical pixel of the image.
    QSizeF size_;               ///< Image size in logical pixels.

    ScreenCorner corner_ = ScreenCorner::TopLeft;
    QPointF offset_;
    bool visible_ = false;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // SCREEN_OVERLAY_H
