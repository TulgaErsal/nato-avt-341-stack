#include <avt_341_rviz_plugins/display_plugins/static_map_image_display.h>

#include <cmath>
#include <string>

#include <QColor>
#include <QUrl>
#include <QVariant>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <geometry_msgs/msg/pose.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/file_picker_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>

namespace avt_341::rviz_plugins
{

using rviz_common::properties::BoolProperty;
using rviz_common::properties::EnumProperty;
using rviz_common::properties::FilePickerProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::TfFrameProperty;

namespace
{
// Resolve a user-entered image location to an absolute filesystem path. Accepts a
// plain path (e.g. from the file browser), a file:// URL, or a ROS resource URL
// "package://<pkg>/<relative/path>" (resolved via the package's share directory, the
// same scheme rviz_common::loadPixmap uses for icons). Returns an empty string when a
// package:// URL is malformed or its package cannot be found.
QString resolveImagePath( const QString& raw )
{
    const QString path = raw.trimmed();

    const QString kPackage = "package://";
    if ( path.startsWith( kPackage ) )
    {
        const QString rest = path.mid( kPackage.length() );
        const int slash = rest.indexOf( '/' );
        if ( slash <= 0 )
        {
            return QString(); // missing package name or relative path
        }
        const std::string pkg = rest.left( slash ).toStdString();
        const QString rel = rest.mid( slash + 1 );
        try
        {
            const std::string share = ament_index_cpp::get_package_share_directory( pkg );
            return QString::fromStdString( share ) + "/" + rel;
        }
        catch ( const std::exception& )
        {
            return QString(); // package not found
        }
    }

    if ( path.startsWith( "file:" ) )
    {
        return QUrl( path ).toLocalFile(); // handles file:///C:/... etc.
    }

    return path; // plain filesystem path
}
} // namespace

StaticMapImageDisplay::StaticMapImageDisplay()
{
    image_file_ = new FilePickerProperty(
        "Image File", "",
        "Location of the map image (.png/.pgm). Accepts a filesystem path, a file:// "
        "URL, or a package://<pkg>/<path> resource URL. Shown in full colour; width and "
        "height are taken from the image.",
        this, SLOT( reloadImage() ), this );

    resolution_ = new FloatProperty(
        "Resolution", 0.05f, "Map resolution in meters per pixel.",
        this, SLOT( updateGeometry() ), this );
    resolution_->setMin( 0.0f );

    origin_x_ = new FloatProperty(
        "Origin X", 0.0f, "X of the lower-left image corner in the map frame (m).",
        this, SLOT( updateGeometry() ), this );
    origin_y_ = new FloatProperty(
        "Origin Y", 0.0f, "Y of the lower-left image corner in the map frame (m).",
        this, SLOT( updateGeometry() ), this );
    origin_yaw_ = new FloatProperty(
        "Origin Yaw", 0.0f, "Rotation of the image about +Z, in radians.",
        this, SLOT( updateGeometry() ), this );

    frame_property_ = new TfFrameProperty(
        "Frame", "map", "TF frame the origin is expressed in.",
        this, nullptr, false, SLOT( updateGeometry() ), this );

    alpha_ = new FloatProperty(
        "Alpha", 1.0f, "Opacity of the image (1 = opaque).",
        this, SLOT( updateAppearance() ), this );
    alpha_->setMin( 0.0f );
    alpha_->setMax( 1.0f );

    draw_under_ = new BoolProperty(
        "Draw Behind", true,
        "Draw the image beneath everything else so other displays render on top.",
        this, SLOT( updateAppearance() ), this );

    interpolation_ = new EnumProperty(
        "Interpolation", "Linear",
        "Texture filtering: Linear is smooth, Nearest shows crisp pixels.",
        this, SLOT( updateAppearance() ), this );
    interpolation_->addOption( "Linear", 1 );
    interpolation_->addOption( "Nearest", 0 );

    width_ = new IntProperty( "Width", 0, "Image width in pixels.", this );
    width_->setReadOnly( true );
    height_ = new IntProperty( "Height", 0, "Image height in pixels.", this );
    height_->setReadOnly( true );
}

StaticMapImageDisplay::~StaticMapImageDisplay() = default;

void StaticMapImageDisplay::onInitialize()
{
    Display::onInitialize();
    frame_property_->setFrameManager( context_->getFrameManager() );
    map_image_ = std::make_unique<MapImage>( scene_manager_, scene_node_ );
}

void StaticMapImageDisplay::onEnable()
{
    reloadImage();
}

void StaticMapImageDisplay::onDisable()
{
    scene_node_->setVisible( false );
}

void StaticMapImageDisplay::reset()
{
    Display::reset();
    reloadImage();
}

void StaticMapImageDisplay::update( float /*wall_dt*/, float /*ros_dt*/ )
{
    if ( !isEnabled() )
    {
        return;
    }
    updateTransform();
}

void StaticMapImageDisplay::reloadImage()
{
    if ( !map_image_ )
    {
        return; // not initialised yet; onEnable() will load
    }
    decodeImage();
    rebuild();
}

void StaticMapImageDisplay::updateGeometry()
{
    if ( !map_image_ )
    {
        return;
    }
    rebuild();
}

void StaticMapImageDisplay::updateAppearance()
{
    if ( !map_image_ )
    {
        return;
    }
    map_image_->setAlpha( alpha_->getFloat() );
    map_image_->setDrawUnder( draw_under_->getBool() );
    map_image_->setInterpolation( interpolation_->getOptionInt() != 0 );
    context_->queueRender();
}

bool StaticMapImageDisplay::decodeImage()
{
    const QString raw = image_file_->getString();
    if ( raw.trimmed().isEmpty() )
    {
        image_ = QImage();
        width_->setInt( 0 );
        height_->setInt( 0 );
        setStatus( StatusProperty::Warn, "Image", "No image file set" );
        return false;
    }

    const QString path = resolveImagePath( raw );
    if ( path.isEmpty() )
    {
        image_ = QImage();
        width_->setInt( 0 );
        height_->setInt( 0 );
        setStatus(
            StatusProperty::Error, "Image",
            "Could not resolve path (malformed URL or unknown package): " + raw );
        return false;
    }

    QImage img;
    if ( !img.load( path ) )
    {
        image_ = QImage();
        width_->setInt( 0 );
        height_->setInt( 0 );
        setStatus( StatusProperty::Error, "Image", "Failed to load image: " + path );
        return false;
    }

    image_ = img;
    width_->setInt( img.width() );
    height_->setInt( img.height() );
    setStatus(
        StatusProperty::Ok, "Image",
        QString( "Loaded %1 (%2 x %3)" ).arg( path ).arg( img.width() ).arg( img.height() ) );
    return true;
}

void StaticMapImageDisplay::rebuild()
{
    // (Re)build the quad from the cached image; an empty image clears it.
    map_image_->setImage( image_, resolution_->getFloat() );
    updateAppearance();
    updateTransform();
    context_->queueRender();
}

void StaticMapImageDisplay::updateTransform()
{
    if ( !map_image_ || !map_image_->valid() )
    {
        scene_node_->setVisible( false );
        return;
    }

    // Origin pose (x, y, 0; yaw about +Z) expressed in the chosen frame.
    geometry_msgs::msg::Pose pose;
    pose.position.x = origin_x_->getFloat();
    pose.position.y = origin_y_->getFloat();
    pose.position.z = 0.0;
    const double half_yaw = 0.5 * static_cast<double>( origin_yaw_->getFloat() );
    pose.orientation.z = std::sin( half_yaw );
    pose.orientation.w = std::cos( half_yaw );

    const std::string frame = frame_property_->getFrameStd();

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    auto* fm = context_->getFrameManager();
    // Prefer the latest transform (now); fall back to time 0, mirroring MapDisplay.
    if ( fm->transform( frame, context_->getClock()->now(), pose, position, orientation ) ||
         fm->transform(
             frame, rclcpp::Time( 0, 0, context_->getClock()->get_clock_type() ),
             pose, position, orientation ) )
    {
        scene_node_->setPosition( position );
        scene_node_->setOrientation( orientation );
        scene_node_->setVisible( true );
        setTransformOk();
    }
    else
    {
        scene_node_->setVisible( false );
        setMissingTransformToFixedFrame( frame );
    }
}

} // namespace avt_341::rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::StaticMapImageDisplay, rviz_common::Display )
