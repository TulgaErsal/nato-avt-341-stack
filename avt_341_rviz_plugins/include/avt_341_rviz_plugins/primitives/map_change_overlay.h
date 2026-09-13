#ifndef MAP_CHANGE_OVERLAY_H
#define MAP_CHANGE_OVERLAY_H

#include <cstdint>

#include <OgreCommon.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace Ogre
{
class ManualObject;
class SceneManager;
class SceneNode;
class TextureUnitState;
}

namespace avt_341 {
namespace rviz_plugins {

/// A translucent quad covering an occupancy grid, with one RGBA texel per cell,
/// drawn just above the Map display's swatches to flash recently changed cells.
/// The quad's lower-left corner sits at the node origin and it extends along +X
/// (columns) and +Y (rows), matching the swatch layout, so the owner only has to
/// hang it off the map display's scene node.
///
/// Texel RGB is the highlight color and texel alpha the per-cell intensity; both
/// are written by the owner through upload(), which updates an arbitrary
/// sub-rectangle of the texture in place. A global alpha and the draw-behind
/// toggle mirror the map's appearance controls. The quad renders in the same
/// render queue group as the swatches at a higher priority, and with a smaller
/// depth push-back, so it lands above the map but below geometry at z = 0.
class MapChangeOverlay
{
public:
    MapChangeOverlay( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
    ~MapChangeOverlay();

    MapChangeOverlay( const MapChangeOverlay& ) = delete;
    MapChangeOverlay& operator=( const MapChangeOverlay& ) = delete;

    /// (Re)create the texture and quad for a width x height cell grid at
    /// `resolution` meters per cell, initially fully transparent. Returns false
    /// (and leaves the overlay invalid) when the GPU refuses a texture that size.
    bool resize( std::uint32_t width, std::uint32_t height, float resolution );

    /// Release the texture and quad; valid() becomes false.
    void clear();

    /// Copy tightly packed RGBA bytes (box width * height * 4) into `box`, given
    /// in cell coordinates with exclusive right/bottom edges.
    void upload( const Ogre::Box& box, const std::uint8_t* rgba );

    /// Global opacity multiplier in [0, 1] applied on top of the texel alpha.
    void setAlpha( float alpha );

    /// Follow the map's Draw Behind setting (render queue group).
    void setDrawUnder( bool under );

    void setVisible( bool visible );

    bool valid() const { return quad_ != nullptr; }

private:
    void applyRenderQueue();

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* node_;
    Ogre::ManualObject* quad_ = nullptr;
    Ogre::TexturePtr texture_;
    Ogre::MaterialPtr material_;
    Ogre::TextureUnitState* tex_unit_ = nullptr;

    float alpha_ = 1.0f;
    bool draw_under_ = false;
    bool visible_ = false;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_CHANGE_OVERLAY_H
