#pragma once

#include <memory>
#include <automated_driving_msgs/MotionState.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

#include "motionstate_visual.hpp"

namespace motion_state_rviz_plugin_ros {

class MotionStateDisplay : public rviz::MessageFilterDisplay<automated_driving_msgs::MotionState> {
    Q_OBJECT
public:
    MotionStateDisplay();
    inline virtual ~MotionStateDisplay() {
    }

protected:
    // Overrides of protected virtual functions from Display.
    virtual void onInitialize() override {
        MFDClass::onInitialize();
        updateParameters();
        visual_.reset(new MotionStateVisual(context_->getSceneManager(), scene_node_, paramsVisual_));
        visual_->setVisible(false);
        processMessage(msg_last_);
    }

    inline virtual void reset() override {
        MFDClass::reset();
        updateParameters();
        visual_.reset(new MotionStateVisual(context_->getSceneManager(), scene_node_, paramsVisual_));
        processMessage(msg_last_);
    }

    inline virtual void onEnable() override {
        MFDClass::onEnable();
        processMessage(msg_last_);
    }

    inline virtual void updateTopic() override {
        MFDClass::updateTopic();
        processMessage(msg_last_);
    }

    // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
    inline void update() {
        updateParameters();
        processMessage(msg_last_);
    }

private:
    // Function to handle an incoming ROS message.
    void processMessage(const automated_driving_msgs::MotionState::ConstPtr& msg) override;
    void updateParameters();


    automated_driving_msgs::MotionState::ConstPtr msg_last_ = nullptr;

    // The visual
    std::shared_ptr<MotionStateVisual> visual_;
    MotionStateVisual::Parameters paramsVisual_;

    // User-editable property variables.
    rviz::ColorProperty* propObjectColor_;
    rviz::FloatProperty *propObjectAlpha_, *propArrowVMin_, *propArrowVMax, *propArrowLength_, *propTextSize_;
    rviz::BoolProperty *propArrowShow_, *propTextShow_, *propTextShowDebug_;
};

} // namespace motion_state_rviz_plugin_ros
