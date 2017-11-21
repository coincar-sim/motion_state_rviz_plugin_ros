#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

#include "motionstate_display.hpp"
#include "motionstate_visual.hpp"

namespace motion_state_rviz_plugin_ros {

MotionStateDisplay::MotionStateDisplay() {

    /**
     * Coloring
     */
    propObjectColor_ =
        new rviz::ColorProperty("Object Color", QColor(204, 51, 204), "Color of the Object.", this, SLOT(update()));
    propObjectAlpha_ = new rviz::FloatProperty(
        "Object Alpha", 1.0, "0.0 is fully transparent, 1.0 is fully opaque.", this, SLOT(update()));
    propObjectAlpha_->setMin(0.0);
    propObjectAlpha_->setMax(1.0);

    /**
     * Arrow
     */
    propArrowShow_ =
        new rviz::BoolProperty("Arrows", true, "Whether to show velocity arrows or not.", this, SLOT(update()));
    propArrowVMin_ = new rviz::FloatProperty("Minimum velocity",
                                             0,
                                             "The minimum velocity in m/s at which a velocity arrow is still shown",
                                             propArrowShow_,
                                             SLOT(update()),
                                             this);
    propArrowVMax =
        new rviz::FloatProperty("Maximum velocity",
                                10,
                                "The velocity in m/s at which the arrow has maximum length and is clipped afterwards.",
                                propArrowShow_,
                                SLOT(update()),
                                this);
    propArrowLength_ = new rviz::FloatProperty(
        "Length", 3, "The maximum arrow length in meters.", propArrowShow_, SLOT(update()), this);

    /**
     * Text
     */
    propTextShow_ =
        new rviz::BoolProperty("Text", true, "Whether to show textual information for objects.", this, SLOT(update()));
    propTextSize_ = new rviz::FloatProperty("Size", 0.4, "The text size.", propTextShow_, SLOT(update()), this);
    propTextShowDebug_ = new rviz::BoolProperty(
        "Debug info", false, "Whether to show debug info for objects.", propTextShow_, SLOT(update()), this);

    updateParameters();
}

void MotionStateDisplay::updateParameters() {
    paramsVisual_.arrowShow = propArrowShow_->getBool();
    paramsVisual_.arrowVMin = propArrowVMin_->getFloat();
    paramsVisual_.arrowVMax = propArrowVMax->getFloat();
    paramsVisual_.arrowLength = propArrowLength_->getFloat();

    paramsVisual_.textShow = propTextShow_->getBool();
    paramsVisual_.textContent = topic_property_->getTopicStd();
    paramsVisual_.textFontSize = propTextSize_->getFloat();
    paramsVisual_.textVMin = propArrowVMin_->getFloat();
    paramsVisual_.textShowDebug = propTextShowDebug_->getBool();

    paramsVisual_.color = propObjectColor_->getOgreColor();
    paramsVisual_.color.a = propObjectAlpha_->getFloat();
}

// This is our callback to handle an incoming message.
void MotionStateDisplay::processMessage(const automated_driving_msgs::MotionState::ConstPtr& msg) {
    msg_last_ = msg;
    bool transformOk = true;

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if (msg == nullptr || messages_received_ == 0) {
        setStatusStd(rviz::StatusProperty::Warn, "Topic", "No message received");
        visual_->setVisible(false);
        return;
    }

    if ((msg->header.stamp - ros::Time::now()).toSec() > 1.0) {
        setStatusStd(rviz::StatusProperty::Warn, "Topic", "Message delay > 1 second");
    }

    // Transform the received pose into the fixed_frame
    if (!context_->getFrameManager()->transform(
            msg->header.frame_id, msg->header.stamp, msg->pose.pose, position, orientation)) {
        transformOk = false;
    }

    if (!transformOk) {
        visual_->setVisible(false);
        std::string errorMsg = "Error transforming from frame '" + msg->header.frame_id + "' to fixed frame";
        setStatusStd(rviz::StatusProperty::Error, "Frame", errorMsg);
        return;
    }

    setStatusStd(rviz::StatusProperty::Ok, "Frame", "Found transform '" + msg->header.frame_id + "' -> fixed frame");

    // Set or update the contents of the visual.
    visual_->setVisible(true);

    visual_->setFramePosition(position);
    visual_->setFrameOrientation(orientation);

    visual_->setParams(paramsVisual_);
    visual_->updateColor();
    visual_->makeArrow(*msg);
    visual_->makeText(*msg);
}


} // namespace motion_state_rviz_plugin_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(motion_state_rviz_plugin_ros::MotionStateDisplay, rviz::Display)
