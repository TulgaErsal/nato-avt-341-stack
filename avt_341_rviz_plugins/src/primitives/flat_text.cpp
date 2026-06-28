#include <avt_341_rviz_plugins/primitives/flat_text.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <vector>

#include <OgreBlendMode.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreRenderOperation.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureUnitState.h>

#include <Overlay/OgreFont.h>
#include <Overlay/OgreFontManager.h>

namespace avt_341::rviz_plugins
{

namespace
{
// Monotonic counter for globally-unique Ogre resource names.
std::atomic<std::uint64_t> g_counter{ 0 };

// Width of a glyph the font can't measure (e.g. space), as a fraction of height.
constexpr float kFallbackGlyphRatio = 0.3f;

// Vertical gap between stacked lines, as a fraction of the taller line's height.
constexpr float kLineGapRatio = 0.12f;
}

FlatText::FlatText( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                    const std::string& font_name )
    : scene_manager_( scene_manager ), font_name_( font_name )
{
    node_ = parent_node->createChildSceneNode();
    object_ = scene_manager_->createManualObject(
        "avt_341_flat_text_" + std::to_string( g_counter++ ) );
    object_->setDynamic( true );
    node_->attachObject( object_ );
}

FlatText::~FlatText()
{
    if ( object_ != nullptr )
    {
        node_->detachObject( object_ );
        scene_manager_->destroyManualObject( object_ );
        object_ = nullptr;
    }
    if ( material_ )
    {
        Ogre::MaterialManager::getSingleton().remove( material_->getName() );
        material_.reset();
    }
    scene_manager_->destroySceneNode( node_ );
}

void FlatText::ensureMaterial( Ogre::Font* font )
{
    if ( material_ready_ )
    {
        return;
    }

    // Clone the font's own material rather than building a fresh one that
    // references the glyph atlas by name. The atlas is a manually-created texture
    // living in the font's resource group; a new material in the "General" group
    // cannot resolve it, so Ogre falls back to loading it from disk and throws
    // FileNotFound ("Cannot locate resource Liberation SansTexture ..."). Cloning
    // keeps the texture binding (and the font's correct colour/alpha setup) and we
    // only override the render states we care about. This mirrors how
    // rviz_rendering::MovableText prepares its material.
    Ogre::MaterialPtr font_material = font->getMaterial();
    font_material->load();
    material_ = font_material->clone(
        "avt_341_flat_text_mat_" + std::to_string( g_counter++ ) );

    material_->setReceiveShadows( false );
    Ogre::Pass* pass = material_->getTechnique( 0 )->getPass( 0 );
    pass->setLightingEnabled( false );
    pass->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    pass->setDepthWriteEnabled( false );
    pass->setCullingMode( Ogre::CULL_NONE );
    // Tint the glyphs with the per-vertex colour the caller emits; the font's
    // texture unit already supplies the glyph coverage as alpha.
    pass->setVertexColourTracking( Ogre::TVC_DIFFUSE );

    material_ready_ = true;
}

void FlatText::setLines( const std::vector<FlatTextLine>& lines, const Ogre::ColourValue& color )
{
    object_->clear();

    Ogre::ResourcePtr resource = Ogre::FontManager::getSingleton().getByName(
        font_name_, Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME );
    Ogre::Font* font = static_cast<Ogre::Font*>( resource.get() );
    if ( font == nullptr )
    {
        return;   // font not registered; render nothing
    }
    font->load();

    ensureMaterial( font );
    if ( !material_ )
    {
        return;
    }

    const auto glyph_width = [font]( char ch, float height ) {
        float ratio = font->getGlyphAspectRatio( static_cast<unsigned char>( ch ) );
        if ( ratio <= 0.0f )
        {
            ratio = kFallbackGlyphRatio;
        }
        return ratio * height;
    };

    // Measure each line so the block (and each line) can be centred.
    std::vector<float> line_widths( lines.size(), 0.0f );
    float block_height = 0.0f;
    float max_height = 0.0f;
    int visible_lines = 0;
    for ( std::size_t i = 0; i < lines.size(); ++i )
    {
        if ( lines[i].text.empty() || lines[i].height <= 0.0f )
        {
            continue;
        }
        float width = 0.0f;
        for ( char ch : lines[i].text )
        {
            width += glyph_width( ch, lines[i].height );
        }
        line_widths[i] = width;
        block_height += lines[i].height;
        max_height = std::max( max_height, lines[i].height );
        ++visible_lines;
    }
    if ( visible_lines == 0 )
    {
        return;
    }
    const float line_gap = kLineGapRatio * max_height;
    block_height += line_gap * static_cast<float>( visible_lines - 1 );

    object_->begin( material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST );
    float y_top = 0.5f * block_height;   // top edge of the block (text-space +Y up)
    for ( std::size_t i = 0; i < lines.size(); ++i )
    {
        if ( line_widths[i] <= 0.0f )
        {
            continue;
        }
        const float h = lines[i].height;
        const float y_bottom = y_top - h;
        float pen_x = -0.5f * line_widths[i];

        for ( char ch : lines[i].text )
        {
            const Ogre::Font::CodePoint cp = static_cast<unsigned char>( ch );
            const Ogre::Font::UVRect uv = font->getGlyphTexCoords( cp );
            const float width = glyph_width( ch, h );
            const bool drawable = uv.right > uv.left && uv.bottom > uv.top;
            if ( drawable )
            {
                const float xl = pen_x;
                const float xr = pen_x + width;

                const auto vertex = [&]( float x, float y, float u, float v ) {
                    object_->position( x, y, 0.0f );
                    object_->colour( color );
                    object_->textureCoord( u, v );
                };

                // Two triangles; culling is disabled so winding is irrelevant.
                vertex( xl, y_top, uv.left, uv.top );
                vertex( xl, y_bottom, uv.left, uv.bottom );
                vertex( xr, y_top, uv.right, uv.top );

                vertex( xr, y_top, uv.right, uv.top );
                vertex( xl, y_bottom, uv.left, uv.bottom );
                vertex( xr, y_bottom, uv.right, uv.bottom );
            }
            pen_x += width;
        }
        y_top = y_bottom - line_gap;
    }
    object_->end();
}

}
