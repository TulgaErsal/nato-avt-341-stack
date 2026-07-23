#ifndef FLAT_TEXT_H
#define FLAT_TEXT_H

#include <string>
#include <vector>

#include <OgreColourValue.h>
#include <OgreMaterial.h>

namespace Ogre
{
class Font;
class ManualObject;
class SceneManager;
class SceneNode;
}

namespace avt_341 {
namespace rviz_plugins {

/// One line of text plus the world-space cap height (meters) to render it at.
struct FlatTextLine
{
    std::string text;
    float height;
};

/// Renders one or more horizontally-centred lines of text as flat, textured
/// geometry in its node's local X-Y plane: text advances along +X, grows upward
/// along +Y and faces +Z. Lines stack top-to-bottom and the whole block is
/// centred on the node origin.
///
/// Unlike rviz_rendering::MovableText (a camera-facing billboard) this does NOT
/// turn to face the camera, so the owner can lay it flat on the ground and
/// rotate it to a heading — which is exactly what a map marker needs. The glyph
/// quads are textured from the Ogre font atlas; a dedicated material samples the
/// glyph coverage as alpha and takes its RGB from the per-vertex colour, so the
/// text colour is whatever the caller passes regardless of the font texture's
/// own RGB.
class FlatText
{
public:
    FlatText( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
              const std::string& font_name = "Liberation Sans" );
    ~FlatText();

    FlatText( const FlatText& ) = delete;
    FlatText& operator=( const FlatText& ) = delete;

    /// Replace the rendered lines. Empty input clears the text.
    void setLines( const std::vector<FlatTextLine>& lines, const Ogre::ColourValue& color );

    /// The node the geometry hangs off; orient/position it to place the text.
    Ogre::SceneNode* node() { return node_; }

private:
    void ensureMaterial( Ogre::Font* font );

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* node_;
    Ogre::ManualObject* object_ = nullptr;
    Ogre::MaterialPtr material_;
    std::string font_name_;
    bool material_ready_ = false;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // FLAT_TEXT_H
