#include "skiros_lib_tools/tools.h"
#include "skiros_world_model/world_model_interface.h"
#include "skiros_config/declared_uri.h"
#include "skiros_common/logger_sys.h"
#include "skiros_world_model/reasoners_loading_func.h"


using namespace skiros_wm;
using namespace skiros_config::owl;

namespace skiros_lib_tools
{
    void waitHasFrameId(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element & e)
    {
        while(!e.hasProperty(data::FrameId))
            e = wm.getElement(e.id());
    }

    void updateOrAdd(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element & e, int parent_id)
    {
        if(parent_id == 0) parent_id = wm.getRobotLocation().id();
        //if(label=="" && e.label()=="") e.label() = "Not specified";
        //Looks for possible matchs in the parent branch
        IdLkhoodListType matches = wm.identify(e, parent_id);
        //If a match is found with a confidence >0.6 is updated. If not I add a new element
        if (matches.size() <= 0)
        {
            FINFO("Added object " << e.type() << " in World model." );
            wm.addElement(e, parent_id, relation::Str[relation::contain]);
            return;
        }
        int bestMatchId = matches[0].first;
        double bestMatchProb = matches[0].second;
        if(bestMatchProb>0.4)
        {
            e.id() = bestMatchId;
            wm.updateElement(e);
            FINFO("Updated object " << e.type() << " in World model. Confidence: " << bestMatchProb);
        }
        else
        {
            FINFO("Added object " << e.type() << " in World model." );
            wm.addElement(e, parent_id, relation::Str[relation::contain]);
        }
    }

    void addObject(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element & e,  int parent_id, std::string label)
    {
        if(parent_id == 0) parent_id = wm.getRobotLocation().id();
        if(label=="" && e.label()=="") e.label() = "Not specified";
        wm.addElement(e, parent_id, relation::Str[relation::contain]);
        FINFO("Added object " << e.type() << " to World model.");
    }

    std::string getFirstFrameId(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element e)
    {
        skiros_wm::Element parent = e;
        while(!parent.hasProperty(data::Str[data::FrameId]) && parent.id() > 0) parent = wm.getParentElement(parent);
        return parent.properties(data::Str[data::FrameId]).getValue<std::string>();
    }

    void getPoseWrtFrame(skiros_wm::WorldModelInterfaceS & wm, tf::Stamped<tf::Pose> & initial_pose, std::string target_frame, tf::Stamped<tf::Pose> & transformed_pose)
    {
        ros::Time time = ros::Time::now()+ros::Duration(0.5);
        initial_pose.stamp_ = time;
        wm.waitForTransform(target_frame, initial_pose.frame_id_, time, ros::Duration(3.0));
        wm.transformPose(target_frame, initial_pose, transformed_pose);
    }

    void getElementPoseWrtFrame(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element & e, std::string target_frame, tf::Stamped<tf::Pose> & transformed_pose)
    {
        if(e.id()<0)
        {
            throw std::invalid_argument("[getElementPoseWrtFrame] The element must be present in world model to use this function. ");
        }
        skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner("AauSpatialReasoner");
        tf::Pose pose = reasoner->getData<tf::Pose>(e, "Pose");
        tf::Stamped<tf::Pose> stamped_pose;
        stamped_pose.setData(pose);
        while(!e.hasProperty(data::Str[data::FrameId]))
            e = wm.getElement(e.id());
        stamped_pose.frame_id_ = getFirstFrameId(wm, wm.getParentElement(e));
        getPoseWrtFrame(wm, stamped_pose, target_frame, transformed_pose);
    }

    tf::Pose getPoseFromPoseStamped(tf::Stamped<tf::Pose> & stamped_pose)
    {
        return tf::Pose(stamped_pose.getRotation(),stamped_pose.getOrigin());
    }
    
    skiros_wm::Element addDefaultObservationPose(skiros_wm::WorldModelInterfaceS & wm, skiros_wm::Element & object, skiros_wm::Element container, skiros_wm::Element gripper, skiros_wm::Element camera)
    {
        std::string robot_frame = wm.getRobot().properties(data::Str[data::FrameId]).getValue<std::string>();
        tf::Stamped<tf::Pose> obj_pose, obs_pose;
        getElementPoseWrtFrame(wm, object, robot_frame , obj_pose); // pose of the object in the robot frame
        //FINFO(object_.printState("",false));
        obs_pose = getObservationPose(wm, obj_pose, gripper, camera); // observation pose of the gripper in robot frame
        //FINFO("Got observation pose");
        std::string base_frame = getFirstFrameId(wm, container);
        //FINFO(obs_pose.frame_id_);
        getPoseWrtFrame(wm, obs_pose, base_frame, obs_pose); // observation pose in robot frame
        skiros_wm::Element grasping_pose = skiros_wm::Element(concept::Str[concept::ObservationPose]);
        skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner("AauSpatialReasoner");
        reasoner->storeData(grasping_pose, getPoseFromPoseStamped(obs_pose), "Pose");
        wm.addElement(grasping_pose, container.id(), relation::Str[relation::hasA]); // add grasping pose to WM
        return grasping_pose;
    }
    
    tf::Stamped<tf::Pose> getObservationPose(skiros_wm::WorldModelInterfaceS & wm, tf::Stamped<tf::Pose> object_pose, skiros_wm::Element gripper, skiros_wm::Element camera, float obs_dist)
    {
        // get the frame names of the robot, gripper and camera
        skiros_wm::Element robot = wm.getRobot();
        std::string robot_frame = robot.properties(data::Str[data::FrameId]).getValue<std::string>();
        std::string gripper_frame = gripper.properties(data::Str[data::FrameId]).getValue<std::string>();
        std::string camera_frame = camera.properties(data::Str[data::DriverAddress]).getValue<std::string>(); // "/<camera>/depth_registered/points"
        camera_frame = camera_frame.substr(1); // "<camera>/depth_registered/points"
        camera_frame = camera_frame.substr(0, camera_frame.find("/")); // "<camera>"
        camera_frame += "_depth_optical_frame"; // "<camera>_depth_optical_frame" 
        //std::cout << "camera_frame: " << camera_frame << std::endl;
        // construct the desired camera pose wrt the object
        tf::Stamped<tf::Pose> oriented_object;
        if(object_pose.frame_id_==robot_frame)
            oriented_object = object_pose;
        else {
            wm.waitForTransform(robot_frame,object_pose.frame_id_,ros::Time(0),ros::Duration(1.5)); 	
            wm.transformPose(robot_frame,object_pose,oriented_object); // object pose in robot frame
            //std::cout << "transformed object pose" << std::endl;
        }
        oriented_object.setRotation(tf::createIdentityQuaternion()); // pose without rotation (aligned with the robot frame)
        tf::Point obs_position = oriented_object.getOrigin(); // extract the position of the object
        obs_position += tf::Point(0.0,0.0,obs_dist); // position translated in z direction (upwards in the robot frame)
        tf::Stamped<tf::Pose> observation_pose;
        observation_pose = oriented_object; // copy 
        observation_pose.setOrigin(obs_position); // Update with correct position for the observation
        observation_pose *= tf::Transform(tf::createQuaternionFromRPY(M_PI,0.0,0.0)); // rotate so that camera_frame z axis is pointing towards robot negative z axis
        //std::cout << "calculated observation pose" << std::endl;
        // get transform from camera frame to gripper frame
        tf::StampedTransform cam_transf;
        wm.waitForTransform(camera_frame,gripper_frame,ros::Time(0),ros::Duration(1.5)); 	
        wm.lookupTransform(camera_frame,gripper_frame,ros::Time(0),cam_transf); // transformation from camera frame to gripper frame 
        //std::cout << "got cam->gripper transform" << std::endl;
        observation_pose *= cam_transf; // final observation pose of the gripper_frame in robot_frame, so that camera_frame is looking directly at the object.
        // finally fill in the header
        observation_pose.frame_id_ = robot_frame;
        observation_pose.stamp_ = ros::Time::now();
        // and done!
        FINFO("Observation pose found");
        return observation_pose;
    }


}
