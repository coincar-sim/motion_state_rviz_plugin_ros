/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <memory>
#include <automated_driving_msgs/MotionState.h>
// Ogre still uses the deprecated volatile keyword on bionic
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
#pragma GCC diagnostic ignored "-Wregister"
#include <rviz/message_filter_display.h>
#pragma GCC diagnostic pop
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
