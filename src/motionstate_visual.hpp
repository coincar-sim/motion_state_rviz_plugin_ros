#pragma once

#include <memory>
#include <automated_driving_msgs/MotionState.h>
#include <simulation_utils/util_perception.hpp>

#ifndef Q_MOC_RUN
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <simulation_utils/util_rviz.hpp>
#include <simulation_utils/util_rvizshapes.hpp>
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
