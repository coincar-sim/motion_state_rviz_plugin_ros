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

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include <rviz/msg_conversions.h>
#include <util_rviz/util_rviz.hpp>
#include <util_rviz/util_rvizshapes.hpp>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/movable_text.h>

#include "motionstate_visual.hpp"

namespace motion_state_rviz_plugin_ros {

MotionStateVisual::MotionStateVisual(Ogre::SceneManager* scene_manager,
                                     Ogre::SceneNode* parent_node,
                                     const Parameters& p) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    params_ = p;

    // car and arrow
    car_.reset(new rviz::SimpleCar(scene_manager_, frame_node_));
    velocity_arrow_.reset(new rviz::Arrow(scene_manager_, frame_node_));
    Ogre::Vector3 arrowPosition(0, 0, 0.4);
    velocity_arrow_->setPosition(arrowPosition);
    updateColor();

    // text
    movable_text_.reset(new rviz::MovableText(params_.textContent));
    movable_text_->setTextAlignment(rviz::MovableText::HorizontalAlignment::H_CENTER,
                                    rviz::MovableText::VerticalAlignment::V_ABOVE);
    movable_text_->showOnTop(true);
    frame_node_->attachObject(movable_text_.get());

    setVisible(false);
}

MotionStateVisual::~MotionStateVisual() {
    scene_manager_->destroySceneNode(frame_node_);
}

// Position and orientation are passed through to the SceneNode.
void MotionStateVisual::setFramePosition(const Ogre::Vector3& position) {
    util_rviz::setPositionSafely(frame_node_, position);
}

void MotionStateVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
    util_rviz::setOrientationSafely(frame_node_, orientation);
}

void MotionStateVisual::setVisible(const bool visible) {
    car_->visible(visible);
    velocity_arrow_->getSceneNode()->setVisible(visible && params_.arrowShow);
    movable_text_->setVisible(visible && params_.textShow);
}

void MotionStateVisual::updateColor() {
    car_->setColorPartly(params_.color);
    velocity_arrow_->setColor(params_.color);
}

void MotionStateVisual::setParams(const Parameters& p) {
    params_ = p;
}

void MotionStateVisual::makeArrow(const automated_driving_msgs::MotionState& ms) {

    if (!util_automated_driving_msgs::checks::twistValid(ms)) {
        return;
    }

    const Ogre::Vector3 v{rviz::vector3MsgToOgre(ms.twist.twist.linear)};
    if (v.isNaN()) {
        return;
    }

    velocity_arrow_->setColor(params_.color);
    velocity_arrow_->setDirection(v);

    const double velocity{v.length()};
    double scale;

    if (velocity > params_.arrowVMax)
        scale = params_.arrowLength;
    else if (velocity < params_.arrowVMin)
        scale = 0.0;
    else
        scale = params_.arrowLength * (velocity - params_.arrowVMin) / (params_.arrowVMax - params_.arrowVMin);

    velocity_arrow_->setScale(Ogre::Vector3(scale, 1, 1));
}

void MotionStateVisual::makeText(const automated_driving_msgs::MotionState& ms) {

    if (!params_.textShow) {
        movable_text_->setVisible(false);
    } else {
        movable_text_->setVisible(true);
        std::ostringstream os;
        os << params_.textContent;

        auto const& v = ms.twist.twist.linear;
        const double velocity{sqrt(v.x * v.x + v.y * v.y + v.z * v.z)};
        if (velocity > params_.textVMin || params_.textShowDebug)
            os << "\n"
               << std::to_string(static_cast<int>(3.6 * velocity)) << " km/h"
               << "\n";

        if (params_.textShowDebug) {
            /**
             * Additional debug info here
             */
            auto const& cp = ms.pose.covariance;
            const double cov{cp[0] * cp[0] + cp[7] * cp[7] + cp[14] * cp[14]};
            os << "\nCOV " << cov;
        }

        movable_text_->setCaption(os.str());
        movable_text_->setCharacterHeight(params_.textFontSize);
    }
}

} // namespace motion_state_rviz_plugin_ros
