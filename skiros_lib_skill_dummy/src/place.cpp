#include <skiros_lib_skill_dummy/place.h>

#include "skiros_world_model/reasoners_loading_func.h"

namespace skiros_skill
{
using namespace skiros_config::owl;
using namespace skiros_wm;


Place::Place()
{
    ////////////////////////////////////////////////////////
    // Specify the skill description constants
    ////////////////////////////////////////////////////////
    this->setSkillType("Place");
    this->setDescription("Place the object from the selected gripper to the selected location.");
    this->setVersion("0.0.6");

    ////////////////////////////////////////////////////////
    // Define the parameters for the skill
    ////////////////////////////////////////////////////////
    getParamHandle()->addParamWithDefaultValue("PlacingCell",
                                              skiros_wm::Element(concept::Str[concept::Cell]),
                                              "Destination cell");
    ////////////////////////////////////////////////////////
    // Required Hardware
    ////////////////////////////////////////////////////////
    getParamHandle()->addParamWithDefaultValue("Arm",
                                              skiros_wm::Element(concept::Str[concept::Arm]),
                                              "Arm to use",
                                              skiros_common::hardware);
    ////////////////////////////////////////////////////////
    // Planning and optional parameters
    ////////////////////////////////////////////////////////
    getParamHandle()->addParamWithDefaultValue("PlacingKit",
                                              skiros_wm::Element(concept::Str[concept::Kit]),
                                              "Destination kit",
                                              skiros_common::planning);

    getParamHandle()->addParamWithDefaultValue("Robot",
                                              skiros_wm::Element(concept::Str[concept::Agent]),
                                              "",
                                              skiros_common::planning);

    getParamHandle()->addParamWithDefaultValue("Gripper",
                                              skiros_wm::Element(concept::Str[concept::Gripper]),
                                              "Gripper mounted on the arm",
                                              skiros_common::planning);

    getParamHandle()->addParamWithDefaultValue("ObjectInHand",
                                              skiros_wm::Element(concept::Str[concept::Manipulatable]),
                                              "Object in the gripper",
                                              skiros_common::planning);

}

bool Place::onInit()
{
    ////////////////////////////////////////////////////////
    // Define the pre conditions
    ////////////////////////////////////////////////////////

    addPrecondition(newCondition("Holding", true, "Gripper", "ObjectInHand"));
    addPrecondition(newCondition("Carrying", true, "Robot", "PlacingKit"));
    addPrecondition(newCondition("FitsIn", true, "PlacingCell", "ObjectInHand"));
    addPrecondition(newCondition("LocationEmpty", true, "PlacingCell"));
    addPrecondition(newCondition("CellInKit", true, "PlacingKit", "PlacingCell"));
    //addPrecondition(newCondition("ObjectAtLocation", true, "PlacingCell", "PlacingKit"));

    ////////////////////////////////////////////////////////
    // Define the post conditions
    ////////////////////////////////////////////////////////

    addPostcondition("NotHoldingObject", newCondition("Holding", false,"Gripper", "ObjectInHand"));//Redundant, needed for planning
    addPostcondition("CellEmpty", newCondition("LocationEmpty", false,  "PlacingCell"));
    addPostcondition("GripperEmpty", newCondition("EmptyHanded", true, "Gripper"));
    addPostcondition("ObjectInCell", newCondition("ObjectInCell", true, "PlacingCell", "ObjectInHand"));
    addPostcondition("ObjectInKit", newCondition("InKit", true, "ObjectInHand", "PlacingKit"));//Redundant, needed for planning

    ////////////////////////////////////////////////////////
    // Advertise ROS communications
    ////////////////////////////////////////////////////////
    return true;
}   

int Place::preSense()
{
    std::vector<skiros_wm::Element> v;
    ////////////////////////////////////////////////////////
    // Get parameters
    ////////////////////////////////////////////////////////
    arm_ = getParamHandle()->getParamValue<skiros_wm::Element>("Arm");
    placing_location_ = getParamHandle()->getParamValue<skiros_wm::Element>("PlacingCell");

    /// Robot
    getParamHandle()->specify("Robot", getWorldHandle()->getRobot());
    ///Gripper
    v = getWorldHandle()->getChildElements(arm_.id(), "", concept::Str[concept::Gripper]);
    if(v.size()<=0)
    {
        setProgress(-1, "No gripper found for the arm specified. ");
        return -1;
    }
    gripper_ = v[0];
    getParamHandle()->specify("Gripper", gripper_);

    ///Camera arm
    v = getWorldHandle()->getChildElements(arm_.id(), "", concept::Str[concept::Camera]);
    if(v.size()<=0)
    {
        setProgress(-1, "No camera found on the arm specified. ");
        return -1;
    }
    camera_ = v[0];

    ///Optional placing camera (if it is not found the arm camera is used)
    v = getWorldHandle()->getRobotHardware();
    for(Element e : v)
    {
        if(e.label()=="top_front_camera") //Temporary hardcoded solution until we think something better
            camera_ = e;
    }

    /// Get in-hand object
    v = this->getWorldHandle()->getChildElements(gripper_, relation::Str[relation::contain]);
    if(v.size())
    {
        in_hand_object_ = v[0];
        getParamHandle()->specify("ObjectInHand", in_hand_object_);
    }

    /// Get kit
    getParamHandle()->specify("PlacingKit", getWorldHandle()->getParentElement(placing_location_)); //Note, if the parent is not a kit, the pre-condition will fail
    return 1;
}

/*!
 * \brief A fake place. Place the object from the selected gripper to the selected location.
 *
 * Teach required: Yes, see placing_pose_learn module. \n
 * Pre-condition: the gripper contains an Element that can be placed at the destination \n
 * Post-condition: the element is placed at the destination using a pre-teached placing pose (no real movement is done) \n
 * Failures: \n
 *  Never fail \n
 *
 */
class PlaceFake : public Place
{

public:

    /*!
      Here should be defined:
        -name, description, version
        -skill's parameters
      */
    PlaceFake()
    {
        /// Specify the skill description constants
        this->setDescription("A fake place. Transfer the object from the selected gripper to the selected location.");
        this->setVersion("0.0.1");
    }

    ~PlaceFake() {}


    //! Fake grasping routine
    int execute()
    {
        setAllPostConditions();
        return 1;
    }
};

} // namespace
//Export
PLUGINLIB_EXPORT_CLASS(skiros_skill::PlaceFake, skiros_skill::SkillBase)
