/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Francesco Rovida
 *	Robotics, Vision and Machine Intelligence Laboratory
 *  Aalborg University, Denmark
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Aalborg Universitet nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef LIB_TOOLS_H
#define LIB_TOOLS_H

#include<string>
#include "tf/tf.h"

//Forward declarations
namespace skiros_wm
{
    class Element;
    class WorldModelInterfaceS;
}

namespace skiros_lib_tools
{

/*!
 * \brief Wait until the object FrameId is published, returns the updated object
 * \param wm
 * \param e
 */
void waitHasFrameId(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element & e);
/*!
 * \brief Updates an Element entry in the world model
 * \param wm Pointer to the WorldModelInterface
 * \param e Element to update
 * \param parent_id ID (in the world model) of the desired parent of the updated Element
 */
void updateOrAdd(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element & e, int parent_id = 0);

/*!
 * \brief Adds an Element into the world model
 * \param wm Pointer to the WorldModelInterface
 * \param e Element to add
 * \param parent_id ID (in the world model) of the desired parent of the added Element
 * \param label Label to associate with the Element upon insertion
 */
void addObject(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element & e,  int parent_id = 0 , std::string label = "");

/*!
 * \brief Get the FrameId of the parent element in the world model
 * \param wm Pointer to the WorldModelInterface
 * \param e Element to get the parent of
 * \return FrameId of the parent as a std::string
 */ 
std::string getFirstFrameId(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element e);

/*! \brief Get the pose of an element transformed into a specific frame
 * \param wm Pointer to the interface to the world model
 * \param e The element to transform the pose of
 * \param target_frame The frame to transform the element pose into
 * \param[out] tranformed_pose The transformed stamped TF pose of the element
 */
void getElementPoseWrtFrame(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element & e, std::string target_frame, tf::Stamped<tf::Pose> & transformed_pose);

/*! \brief Transform a pose to another frame
 * \param wm Pointer to the interface to the world model
 * \param initial_pose The pose to transform
 * \param target_frame The desired new frame to transform the pose into
 * \param[out] tranformed_pose The transformed pose
 */
void getPoseWrtFrame(skiros_wm::WorldModelInterfaceS & wm, tf::Stamped<tf::Pose> & initial_pose, std::string target_frame, tf::Stamped<tf::Pose> & transformed_pose);

/*! \brief Extract only the pose from a stamped pose
 * \param stamped_pose The stamped pose to extract the pose information from
 * \return The pose component of the input stamped pose
 */
tf::Pose getPoseFromPoseStamped(tf::Stamped<tf::Pose> & stamped_pose);

/*!
 * \brief Get observation pose and add it to the world model
 * \param wm Pointer to the interface to the world model
 * \param object The object to get the observation pose for
 * \param container The container the object is in
 * \param gripper The Gripper used for the observation pose
 * \param camera The Camera that should observe the object
 * \return The observation pose (as a GraspingPose) that was added to the world model
 */
skiros_wm::Element addDefaultObservationPose(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element &object, skiros_wm::Element container, skiros_wm::Element gripper, skiros_wm::Element camera);

/*!
 * \brief Get observation pose of camera wrt object
 * 
 * \param wm Pointer to the interface to the world model
 * \param object_pose The stamped pose of the object wrt the robot base frame (root of the move group)
 * \param gripper The Gripper used for the observation pose
 * \param camera The Camera that should observe the object
 * \param obs_dist Distance the camera should be from the object
 * \return The stamped pose of the gripper to achieve the desired obervation pose
 */
tf::Stamped<tf::Pose> getObservationPose(skiros_wm::WorldModelInterfaceS & wm, tf::Stamped<tf::Pose> object_pose, skiros_wm::Element gripper, skiros_wm::Element camera, float obs_dist=0.6);
}

#endif // LIB_TOOLS_H
