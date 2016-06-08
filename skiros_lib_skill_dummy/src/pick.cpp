#include <skiros_lib_skill_dummy/pick.h>

namespace skiros_skill
{
using namespace skiros_config::owl;
using namespace skiros_wm;

Pick::Pick()
{
    ////////////////////////////////////////////////////////
    // Specify the skill description constants
    ////////////////////////////////////////////////////////
    this->setSkillType("Pick");
    this->setVersion("0.0.5");

    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    getParamHandle()->addParamWithDefaultValue("Container",
                                              skiros_wm::Element(concept::Str[concept::Location]),
                                              "Container to get the object from");

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
    getParamHandle()->addParamWithDefaultValue("Object",
                                              skiros_wm::Element(concept::Str[concept::Manipulatable]),
                                              "The object to grasp",
                                              skiros_common::optional);

    getParamHandle()->addParamWithDefaultValue("Robot",
                                              skiros_wm::Element(concept::Str[concept::Agent]),
                                              "",
                                              skiros_common::planning);

    getParamHandle()->addParamWithDefaultValue("Gripper",
                                              skiros_wm::Element(concept::Str[concept::Gripper]),
                                              "Gripper mounted on the arm",
                                              skiros_common::planning);

    /*getParamHandle()->addParamWithDefaultValue("ArmCamera",
                                              skiros_wm::Element(concept::Str[concept::Camera]),
                                              "Camera mounted on the arm",
                                              skiros_common::planning);*/
}

bool Pick::onInit()
{
    reasoner_ = skiros_wm::getDiscreteReasoner("AauSpatialReasoner");
    ////////////////////////////////////////////////////////
    // Define the pre conditions
    ////////////////////////////////////////////////////////
    addPrecondition(newCondition("EmptyHanded", true, "Gripper"));
    addPrecondition(newCondition("RobotAtLocation", true, "Robot", "Container"));
    addPrecondition(newCondition("ObjectAtLocation", true, "Container", "Object"));
    ////////////////////////////////////////////////////////
    // Define the post conditions
    ////////////////////////////////////////////////////////
    addPostcondition("GripperEmpty", newCondition("EmptyHanded", false, "Gripper"));
    addPostcondition("HoldingObject", newCondition("Holding", true, "Gripper", "Object"));
    return true;
}

int Pick::preSense()
{
    std::vector<skiros_wm::Element> v;
    ////////////////////////////////////////////////////////
    // Get parameters
    ////////////////////////////////////////////////////////
    /// Arm
    arm_ = getParamHandle()->getParamValue<skiros_wm::Element>("Arm");

    /// Robot
    auto robot = getWorldHandle()->getRobot();
    getParamHandle()->specify("Robot", robot);

    /// Gripper
    v = getWorldHandle()->getChildElements(arm_.id(), "", concept::Str[concept::Gripper]);
    if(v.size()<=0)
    {
        setProgress(-1, "No gripper found for the arm specified. ");
        return -1;
    }
    gripper_ = v[0];
    getParamHandle()->specify("Gripper", gripper_);

    ///Get camera arm
    v = getWorldHandle()->getChildElements(arm_.id(), "", concept::Str[concept::Camera]);
    if(v.size()<=0)
    {
        setProgress(-1, "No camera found on the arm specified. ");
        return -1;
    }
    camera_arm_ = v[0];
    camera_up_ = v[0];

    ///Get camera to locate (optional)
    v = getWorldHandle()->getRobotHardware();

    ///Get container
    container_ = this->getParamHandle()->getParamValue<skiros_wm::Element>("Container");
    container_ = getWorldHandle()->getElement(container_.id());//TODO: this update should be automated. TOFIX

    std::string container_frame = container_.properties("FrameId").getValue<std::string>();
    std::set<std::string> rel = robot.getRelationsWrt(container_, "AauSpatialReasoner");

    if (rel.find("left")!=rel.end())
    {
        setProgress("Box on the left - using left camera");
        for(Element e : v)
        {
            if(e.label()=="top_left_camera")
                camera_up_ = e;
        }
    }
    else if(rel.find("right")!=rel.end())
    {
        setProgress("Box on the right - using right camera");
        for(Element e : v)
        {
            if(e.label()=="top_right_camera")
                camera_up_ = e;
        }
    }
    else
    {
        //If container or robot has no pose... hardcoded solution
        setProgress("Can't estimate the box pose - using right camera");
        for(Element e : v)
        {
            if(e.label()=="top_right_camera")
                camera_up_ = e;
        }
    }

    /// locate object
    objObject = getParamHandle()->getParamValue<Element>("Object");
    if(objObject.id()<0)
    {
        skiros::Module locate(getModulesHandler(), "locate_fake", objObject.type());
        locate.setParam("Camera", camera_up_);
        locate.setParam("Container", container_);
        locate.exe();
        locate.waitResult();
        v = getWorldHandle()->getChildElements(container_,"", objObject.type());
    }
    if(v.size())
    {
        objObject = v[0];
        getParamHandle()->specify("Object", objObject);
    }
    return 1;
}

/*!
 * \brief A fake pick. Transfer a 'Product' element from the container to the gripper
 *
 * Teach required: No \n
 * Pre-condition: container has a Element subtype of 'Product' as child \n
 * Post-condition: a random element in the container is transfered in the gripper. \n
 * Failures: \n
 *  Never fail. \n
 *
 */
class PickFake : public Pick
{
public:
    PickFake() : Pick()
    {
        /// Specify the skill description constants
        this->setSkillType("Pick");
        this->setDescription("A fake pick. Set all post-conditions to the desired value");
        this->setVersion("0.0.1");
    }
    ~PickFake(){}

    //Simulated grasping routine
    int execute()
    {
        setAllPostConditions();
        return 1;
    }

};


} // namespace 

//Export
PLUGINLIB_EXPORT_CLASS(skiros_skill::PickFake, skiros_skill::SkillBase)
