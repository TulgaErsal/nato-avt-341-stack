#ifndef VISUAL_DISPLAY_BASE_H
#define VISUAL_DISPLAY_BASE_H

#ifndef Q_MOC_RUN
#include <cstddef>
#include <memory>
#include <vector>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/status_property.hpp>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// Base for a display that renders a single VisualT from a message exposing a
/// std_msgs/Header `header` and a geometry `pose`. It holds the shared flow:
/// validate -> transform into the fixed frame -> create/update the visual ->
/// apply style -> queue a render.
///
/// The concrete display supplies the per-type pieces through the hooks and keeps
/// the Q_OBJECT, the `updateStyle()` slot and the properties member (a class
/// template cannot carry Q_OBJECT, and the slot must resolve against the derived
/// metaobject).
template <typename MessageT, typename VisualT>
class SingleVisualDisplay : public rviz_common::MessageFilterDisplay<MessageT>
{
protected:
    // Per-type hooks implemented by the concrete display.
    virtual bool validate( const MessageT& msg ) const = 0;
    virtual void setDomainFields( VisualT& visual, const MessageT& msg ) = 0;
    virtual void applyStyle( VisualT& visual ) = 0;
    virtual const char* invalidText() const = 0;

    void onInitialize() override
    {
        rviz_common::MessageFilterDisplay<MessageT>::onInitialize();
    }

    void reset() override
    {
        rviz_common::MessageFilterDisplay<MessageT>::reset();
        visual_.reset();
    }

    // Re-apply the current style to the existing visual; the concrete display's
    // updateStyle() slot calls this when a property changes.
    void applyStyleAndRender()
    {
        if ( visual_ )
        {
            applyStyle( *visual_ );
        }
        this->context_->queueRender();
    }

    void processMessage( typename MessageT::ConstSharedPtr msg ) override
    {
        if ( !validate( *msg ) )
        {
            this->setStatus( rviz_common::properties::StatusProperty::Error, "Topic",
                             invalidText() );
            return;
        }

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if ( !this->context_->getFrameManager()->transform(
                 msg->header, msg->pose, position, orientation ) )
        {
            this->setMissingTransformToFixedFrame( msg->header.frame_id );
            return;
        }
        this->setTransformOk();

        if ( !visual_ )
        {
            visual_ = std::make_unique<VisualT>( this->scene_manager_, this->scene_node_ );
        }
        visual_->setPose( position, orientation );
        setDomainFields( *visual_, *msg );
        applyStyle( *visual_ );

        this->context_->queueRender();
    }

    std::unique_ptr<VisualT> visual_;
};

/// Base for a display that renders one VisualT per element of an array carried by
/// MessageT, where each ItemT element exposes its own `header` and `pose`. Same
/// shared flow as SingleVisualDisplay, applied per item over a pooled set of
/// visuals (indices line up across messages; a shorter list drops the trailing
/// visuals). The Q_OBJECT, slot and properties stay in the concrete display.
template <typename MessageT, typename ItemT, typename VisualT>
class VisualArrayDisplay : public rviz_common::MessageFilterDisplay<MessageT>
{
protected:
    virtual const std::vector<ItemT>& items( const MessageT& msg ) const = 0;
    virtual bool validate( const ItemT& item ) const = 0;
    virtual void setDomainFields( VisualT& visual, const ItemT& item ) = 0;
    virtual void applyStyle( VisualT& visual ) = 0;
    virtual const char* invalidText() const = 0;

    void onInitialize() override
    {
        rviz_common::MessageFilterDisplay<MessageT>::onInitialize();
    }

    void reset() override
    {
        rviz_common::MessageFilterDisplay<MessageT>::reset();
        visuals_.clear();
    }

    void applyStyleAndRender()
    {
        for ( const std::unique_ptr<VisualT>& visual : visuals_ )
        {
            if ( visual )
            {
                applyStyle( *visual );
            }
        }
        this->context_->queueRender();
    }

    void processMessage( typename MessageT::ConstSharedPtr msg ) override
    {
        const std::vector<ItemT>& list = items( *msg );

        // Keep the pool sized to the item count so indices line up across
        // messages (a shorter list drops the trailing visuals).
        if ( visuals_.size() > list.size() )
        {
            visuals_.resize( list.size() );
        }

        bool transform_failed = false;
        for ( std::size_t i = 0; i < list.size(); ++i )
        {
            const ItemT& item = list[i];

            if ( !validate( item ) )
            {
                this->setStatus( rviz_common::properties::StatusProperty::Error, "Topic",
                                 invalidText() );
                continue;
            }

            // Each item carries its own header/frame, so transform individually
            // rather than assuming the message's frame.
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            if ( !this->context_->getFrameManager()->transform(
                     item.header, item.pose, position, orientation ) )
            {
                transform_failed = true;
                continue;
            }

            if ( i >= visuals_.size() )
            {
                visuals_.push_back(
                    std::make_unique<VisualT>( this->scene_manager_, this->scene_node_ ) );
            }
            else if ( !visuals_[i] )
            {
                visuals_[i] =
                    std::make_unique<VisualT>( this->scene_manager_, this->scene_node_ );
            }

            visuals_[i]->setPose( position, orientation );
            setDomainFields( *visuals_[i], item );
            applyStyle( *visuals_[i] );
        }

        if ( transform_failed )
        {
            this->setMissingTransformToFixedFrame( msg->header.frame_id );
        }
        else
        {
            this->setTransformOk();
        }

        this->context_->queueRender();
    }

    std::vector<std::unique_ptr<VisualT>> visuals_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // VISUAL_DISPLAY_BASE_H
