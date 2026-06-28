#ifndef NAV_GOAL_VISUAL_H
#define NAV_GOAL_VISUAL_H

#include <memory>

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

namespace rviz_rendering
{
class Arrow;
class BillboardLine;
}

namespace avt_341 {
namespace rviz_plugins {

/// Resolved appearance for a NavGoal visual.
///
/// The displays own the RViz property tree and pack the current values into one
/// of these; the visual itself is pure rendering and never touches a property.
/// Sharing the struct keeps the single-goal and sequence displays pixel-for-
/// pixel identical. The member defaults below double as the "sensible defaults"
/// for the properties.
struct NavGoalStyle
{
    // Pose arrow — mirrors the geometry_msgs/PoseStamped "Arrow" display.
    Ogre::ColourValue arrow_color { 1.0f, 0.1f, 0.0f, 1.0f };
    float shaft_length = 1.0f;
    float shaft_radius = 0.05f;
    float head_length  = 0.3f;
    float head_radius  = 0.1f;

    // Outline of the distance-threshold circle.
    Ogre::ColourValue circle_color { 0.0f, 0.667f, 1.0f, 1.0f };
    float circle_thickness = 0.05f;

    // Filled "viable" region — the yaw-threshold wedge. Alpha lives in the colour.
    Ogre::ColourValue fill_color { 0.0f, 0.8f, 0.0f, 0.3f };

    // Fallbacks used when a message asks for the "stack default" (value < 0).
    float default_dist_threshold = 1.0f;   // meters
    float default_yaw_threshold  = 0.5f;   // radians (half-wedge angle)

    // Tessellation of the circle / wedge.
    int segments = 64;
};

/// Renders one avt_341_msgs/NavGoal into the scene.
///
/// The goal pose is drawn as a PoseStamped-style arrow. Around it, on the pose's
/// ground plane, a circle marks the distance threshold and its interior is
/// filled over the angular range permitted by the yaw threshold — i.e. the
/// region of poses in which the goal counts as reached. A yaw threshold greater
/// than pi is treated as "disabled" and fills the whole circle.
///
/// The class is deliberately display-agnostic so the NavGoal and NavGoalSequence
/// displays can both reuse it (a sequence simply owns many).
class NavGoalVisual
{
public:
    NavGoalVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
    ~NavGoalVisual();

    NavGoalVisual( const NavGoalVisual& ) = delete;
    NavGoalVisual& operator=( const NavGoalVisual& ) = delete;

    /// Pose of the goal, already transformed into the fixed frame.
    void setPose( const Ogre::Vector3& position, const Ogre::Quaternion& orientation );

    /// Raw thresholds straight from the message: a negative value means "use the
    /// configured default" and a yaw above pi means "disabled". Stored only; the
    /// geometry is (re)built by setStyle().
    void setThresholds( double dist_threshold, double yaw_threshold );

    /// Apply the appearance + default fallbacks and rebuild the geometry.
    void setStyle( const NavGoalStyle& style );

private:
    void rebuild();
    void rebuildCircle( double radius );
    void rebuildFill( double radius, double half_angle );

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* frame_node_;   ///< Carries the goal pose; everything hangs off it.

    std::unique_ptr<rviz_rendering::Arrow> arrow_;
    std::unique_ptr<rviz_rendering::BillboardLine> circle_;
    Ogre::ManualObject* fill_ = nullptr;
    Ogre::MaterialPtr fill_material_;

    NavGoalStyle style_;
    double dist_threshold_ = -1.0;   ///< Raw message value (negative = default).
    double yaw_threshold_  = -1.0;   ///< Raw message value (negative = default).
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // NAV_GOAL_VISUAL_H
