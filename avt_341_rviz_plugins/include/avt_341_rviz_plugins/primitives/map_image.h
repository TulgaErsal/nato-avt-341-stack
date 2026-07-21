#ifndef MAP_IMAGE_H
#define MAP_IMAGE_H

#include <OgreMaterial.h>
#include <OgreTexture.h>

class QImage;

namespace Ogre
{
class ManualObject;
class SceneManager;
class SceneNode;
class TextureUnitState;
}

namespace avt_341 {
namespace rviz_plugins {

/// A flat, textured rectangle laid in its node's local X-Y plane (z = 0), used to
/// show a static map image as a full-colour ground decal. The rectangle's lower-left
/// corner sits at the node origin and it extends along +X (image columns) and +Y
/// (image rows), so the owning display only has to drive the node to the map origin
/// pose to place it in the world.
///
/// The image is uploaded as an RGBA Ogre texture and sampled straight into the quad's
/// RGB (no lighting, no palette), so colours are shown as-is. A global alpha and a
/// draw-behind (render-queue) toggle mirror the built-in Map display's appearance
/// controls. Ogre resources are uniquely named and released in the destructor,
/// following the same convention as the other avt_341 primitives.
class MapImage
{
public:
    MapImage( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
    ~MapImage();

    MapImage( const MapImage& ) = delete;
    MapImage& operator=( const MapImage& ) = delete;

    /// Rebuild the texture and quad from the image, scaled at `resolution` meters per
    /// pixel. A null/empty image clears the geometry (see valid()).
    void setImage( const QImage& image, float resolution );

    /// Global opacity multiplier in [0, 1]. Cheap: updates the texture-unit alpha op.
    void setAlpha( float alpha );

    /// When true, draw in a low render queue so other displays render on top.
    void setDrawUnder( bool under );

    /// Texture filtering: true = bilinear (smooth), false = nearest (crisp pixels).
    void setInterpolation( bool linear );

    /// The node the quad hangs off; position/orient it to place the image.
    Ogre::SceneNode* node() { return node_; }

    /// True once a valid image has been uploaded and the quad built.
    bool valid() const { return quad_ != nullptr; }

private:
    void clearGeometry();

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* node_;
    Ogre::ManualObject* quad_ = nullptr;
    Ogre::TexturePtr texture_;
    Ogre::MaterialPtr material_;
    Ogre::TextureUnitState* tex_unit_ = nullptr;

    float alpha_ = 1.0f;
    bool draw_under_ = true;
    bool linear_ = true;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_IMAGE_H
