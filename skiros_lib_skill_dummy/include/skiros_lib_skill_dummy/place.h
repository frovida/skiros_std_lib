//---------- Mandatory skill's include ----------
#include <pluginlib/class_list_macros.h>//Plugin export library
#include <ros/ros.h>
#include "skiros_skill/skill_base.h"    //Base template for all skills
#include "skiros_common/logger_sys.h"   //Skiros logging system
#include "skiros_config/param_types.h"  //Default parameters
#include "skiros_config/declared_uri.h" //Default world model URIs (Unified Resource Identifier) -> generated by skiros_world_model/generate_uri
//---------- Mandatory skill's include end ----------

#include "skiros_world_model/reasoners_loading_func.h"

namespace skiros_skill
{
/*!
 * \brief The base class for all place skills. Defines common use variables
 *
 * Params: \n
 *  Arm - Arm to use \n
 *  Gripper - Gripper to use \n
 *  CameraUp - Upper workspace camera \n
 *  placingLocation - Destination for placing. A placing pose must be pre-teached for this location type. \n
 *
 */
class Place : public skiros_skill::SkillBase
{

public:

    /*!
      Here should be defined:
        -name, description, version
        -skill's parameters
      */
    Place();

    ~Place() {}

    /*!
      Here should be defined:
        -pre-/post- conditions
        -persistents ROS listeners/advertiser
      */
    bool onInit();

    //! Execute a sensing routine before the pre-condition check
    virtual int preSense();

    //! Skill's main execution routine
    virtual int execute()
    {
        return 1;
    }


protected:
    //Skiros elements
    skiros_wm::Element placing_pose_;
    skiros_wm::Element placing_location_;
    skiros_wm::Element gripper_;
    skiros_wm::Element in_hand_object_;
    skiros_wm::Element camera_;
    skiros_wm::Element arm_;

}; // class

} // skiros_skill
