#ifndef MAP_MARKER_VISUAL_H
#define MAP_MARKER_VISUAL_H

#include <memory>
#include <string>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>

namespace Ogre
{
class ManualObject;
class SceneManager;
class SceneNode;
}

namespace avt_341 {
namespace rviz_plugins {

class FlatText;

/// Resolved appearance for a MapMarker visual. The displays own the RViz
/// properties and pack their current values into one of these; the visual is
/// pure rendering. Shared verbatim by the single-marker and list displays so
/// both look identical. The member defaults double as the property defaults.
struct MapMarkerStyle
{
    Ogre::ColourValue color { 1.0f, 1.0f, 1.0f, 1.0f };   // ring, arrow and text
    float radius = 1.0f;             // meters
    float circle_thickness = 0.15f;  // meters (ring band width)
    float text_scale = 1.0f;         // multiplier on the auto-sized text
    int segments = 64;               // ring tessellation
};

/// Renders one avt_341_msgs/MapMarker as a flat ground decal, similar to the
/// in-sim compass marker: a thick ring, a triangular pointer at the ring's edge
/// in the heading direction, and the marker id laid out in the centre.
///
/// The id is split on its first underscore: with no underscore it is drawn as a
/// single large line; otherwise the part before the underscore is drawn small
/// above the (larger) part after it (e.g. "MP_E" -> small "MP" over large "E").
/// The text is rotated with the marker so it always reads "up" toward the
/// heading.
///
/// The class is display-agnostic so the MapMarker and MapMarkerList displays can
/// both reuse it (a list simply owns many).
class MapMarkerVisual
{
public:
    MapMarkerVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
    ~MapMarkerVisual();

    MapMarkerVisual( const MapMarkerVisual& ) = delete;
    MapMarkerVisual& operator=( const MapMarkerVisual& ) = delete;

    /// Pose of the marker, already transformed into the fixed frame.
    void setPose( const Ogre::Vector3& position, const Ogre::Quaternion& orientation );

    /// The marker id whose text is drawn in the centre. Stored only; geometry is
    /// (re)built by setStyle().
    void setMarkerId( const std::string& marker_id );

    /// Apply the appearance and rebuild the geometry.
    void setStyle( const MapMarkerStyle& style );

private:
    void rebuild();
    void rebuildRingAndArrow();
    void rebuildText();

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* frame_node_;   ///< Carries the marker pose; everything hangs off it.

    Ogre::ManualObject* shape_ = nullptr;   ///< Ring + heading arrow.
    Ogre::MaterialPtr shape_material_;
    std::unique_ptr<FlatText> text_;

    MapMarkerStyle style_;
    std::string marker_id_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_MARKER_VISUAL_H
