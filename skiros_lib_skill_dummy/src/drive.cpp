#include <skiros_lib_skill_dummy/drive.h>

namespace skiros_skill
{

using namespace skiros_config::owl;
using namespace skiros_wm;

Drive::Drive()
{
    ////////////////////////////////////////////////////////
    // Specify the skill description constants
    ////////////////////////////////////////////////////////
    this->setSkillType("Drive");
    this->setDescription("Move the robot close to a selected container");
    this->setVersion("0.0.2");

    ////////////////////////////////////////////////////////
    // Define the parameters for the skill
    ////////////////////////////////////////////////////////
    // 1. parameter is the object to grasp
    getParamHandle()->addParamWithDefaultValue("TargetLocation",skiros_wm::Element(concept::Str[concept::Location]), "Target object to drive to", skiros_common::online);
    getParamHandle()->addParamWithDefaultValue("Robot",skiros_wm::Element(concept::Str[concept::Agent]), "Robot to move", skiros_common::hardware);
    getParamHandle()->addParamWithDefaultValue("Gripper",skiros_wm::Element(concept::Str[concept::Gripper]), "Robot to move", skiros_common::offline);
    ////////////////////////////////////////////////////////
    // Define planning parameters
    ////////////////////////////////////////////////////////
    //getParamHandle()->addParamWithDefaultValue("StartLocation",skiros_wm::Element(concept::Str[concept::Location]), "Starting location", skiros_common::planning);

}

bool Drive::onInit()
{
    ////////////////////////////////////////////////////////
    // Define the pre conditions
    ////////////////////////////////////////////////////////
    //addPrecondition(newCondition("RobotAtLocation", true, "Robot", "StartLocation"));//Redundant, for planning
    addPrecondition(newCondition("EmptyHanded", true, "Gripper"));
    ////////////////////////////////////////////////////////
    // Define the post conditions
    ////////////////////////////////////////////////////////
    //addPostcondition("NotAtStart", newCondition("RobotAtLocation", false, "Robot",  "StartLocation"));
    addPostcondition("AtTarget", newCondition("RobotAtLocation", true, "Robot", "TargetLocation"));
    return true;
}

int Drive::preSense()
{
    ////////////////////////////////////////////////////////
    // Get parameters
    ////////////////////////////////////////////////////////
    mobile_base_ = getParamHandle()->getParamValue<skiros_wm::Element>("Robot");
    target_object_ = getParamHandle()->getParamValue<skiros_wm::Element>("TargetLocation");
    std::vector<skiros_wm::Element> v = getWorldHandle()->getChildElements(mobile_base_, relation::Str[relation::robotAt]);
    if(v.size()==0) //FIXME: this should never happen
    {
        setProgress("[ERROR] Initial location not found!");
        skiros_wm::Element fake(concept::Str[concept::Location]);
        fake.label() = getWorldHandle()->getRobot().properties(relation::Str[relation::hasStartLocation]).getValue<std::string>();
        v = getWorldHandle()->resolveElement(fake);
        initial_location_ = v[0];
    }
    else initial_location_ = v[0];
    target_location_ = target_object_;
    std::stringstream ss;
    ss <<  "Navigating to " << target_location_.label() << " type " << target_location_.type() << " id " << target_location_.id();
    setProgress(ss.str());
    return 1;
}

int Drive::execute()
{
    return 1;
}


/*!
 * \brief A fake drive. Move a 'MobileBase' element to a selected location
 *
 * Teach required: \n
 * Pre-condition:  \n
 * Post-condition: \n
 * Failures: \n
 *  Never fail. \n
 *
 */
class DriveFake : public Drive
{
public:
    DriveFake()
    {
        // Specify the skill description constants
        this->setDescription("Fake drive. Move the robot close to a selected container");
        this->setVersion("0.0.1");
    }
    ~DriveFake() {}

    int execute()
    {
        getWorldHandle()->setRelation(mobile_base_.id(), relation::Str[relation::robotAt], initial_location_.id(), false);
        setAllPostConditions();
        return 1;
    }

}; // class

} // namespace 

//Export
PLUGINLIB_EXPORT_CLASS(skiros_skill::DriveFake, skiros_skill::SkillBase)
