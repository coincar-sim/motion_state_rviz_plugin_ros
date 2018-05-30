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
#include <util_automated_driving_msgs/util_automated_driving_msgs.hpp>

#ifndef Q_MOC_RUN
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <util_rviz/util_rviz.hpp>
#include <util_rviz/util_rvizshapes.hpp>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/movable_text.h>
#endif


namespace motion_state_rviz_plugin_ros {

class MotionStateVisual {
public:
    struct Parameters {
        bool arrowShow{true};
        float arrowVMin{1};
        float arrowVMax{10};
        float arrowLength{1};

        bool textShow{true};
        std::string textContent = "";
        float textFontSize{1};
        float textVMin{1};
        bool textShowDebug{false};

        Ogre::ColourValue color{Ogre::ColourValue::Black};
    };

    MotionStateVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, const Parameters& p);
    virtual ~MotionStateVisual();

    // Configure the visual to show the data in the message.
    void setVisible(const bool visible);

    // Set the pose of the coordinate frame the message refers to.
    void setFramePosition(const Ogre::Vector3& position);
    void setFrameOrientation(const Ogre::Quaternion& orientation);

    void setParams(const Parameters& p);
    void makeArrow(const automated_driving_msgs::MotionState& ms);
    void makeText(const automated_driving_msgs::MotionState& ms);
    void updateColor();

private:
    // objects implementing the actual arrow shape
    boost::shared_ptr<rviz::Arrow> velocity_arrow_;
    // objects implementing the Car
    boost::shared_ptr<rviz::SimpleCar> car_;
    // object implementing the text
    boost::shared_ptr<rviz::MovableText> movable_text_;

    // A SceneNode whose pose is set to match the coordinate frame of
    // the MotionState message header.
    Ogre::SceneNode* frame_node_;

    // The SceneManager, kept here only so the destructor can ask it to
    // destroy the ``frame_node_``.
    Ogre::SceneManager* scene_manager_;

    Parameters params_;
};

} // namespace motion_state_rviz_plugin_ros
