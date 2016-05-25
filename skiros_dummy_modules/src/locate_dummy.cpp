#include <skiros_dummy_modules/locate.h>

#include <boost/any.hpp>
#include "skiros_lib_tools/tools.h"

using namespace skiros_config::owl;
using namespace skiros_wm;
using namespace skiros_lib_tools;

namespace skiros
{

namespace module
{

Locate::Locate()
{
    /// Specify the skill description constants
    this->setModuleType("LocateModule");
    this->setVersion("0.0.3");

    ////////////////////////////////////////////////////////
    // Define the parameters for the skill
    ////////////////////////////////////////////////////////

    getParamHandle()->addParamWithDefaultValue("Camera",
                                              skiros_wm::Element(concept::Str[concept::Camera]),
                                              "Camera to use",
                                              skiros_common::hardware);
    getParamHandle()->addParamWithDefaultValue("Container",
                                              skiros_wm::Element(concept::Str[concept::Container]),
                                              "Container where the detected object are located",
                                              skiros_common::optional);
}

bool Locate::onInit()
{
    reasoner_ = skiros_wm::getDiscreteReasoner("AauSpatialReasoner");
    return true;
}

//! Skill's main execution routine
int Locate::execute()
{
    return 1;
}

/*!
 * \brief Add fake objects to the world model.
 *
 * Teach required: No \n
 * Pre-condition: None \n
 * Post-condition: the container pose is updated and 2 objects are added as container childs \n
 * Params: \n
 *  Camera - Camera to use \n
 *  Container - Container where the detected object are located. The 'partReference' property is required. \n
 * Failures: \n
 *  Never fail. \n
 *
 */
class LocateFake : public Locate
{
public:
    LocateFake() : Locate()
    {
        /// Specify the skill description constants
        this->setDescription("Add fake objects to the world model. ");
        this->setVersion("0.0.5");
    }
    ~LocateFake() {}


    //! \brief Simulated locate routine
    int execute()
    {
        container_ = this->getParamHandle()->getParamValue<skiros_wm::Element>("Container");
        camera_ = this->getParamHandle()->getParamValue<skiros_wm::Element>("Camera");
        if(!container_.hasProperty(data::Str[data::FrameId]) || !ros::param::has("robot_description")) return fixed_position();
        else return camera_fov();
    }
    
    //! \brief This version adds an object in the middle of the camera FOV, and at the same height and orientation as the robot base
    int camera_fov()
    {
        setProgress("Initializing.");


        /** Execute: Move arm to segmentation pose if camera is on the wrist **/
        skiros_wm::Element parent = getWorldHandle()->getParentElement(camera_);
        if(parent.type()==concept::Str[concept::Arm])
        {
            this->setProgress("Move to segmentation pose.");
            skiros::Module arm_motion(getModulesHandler(),"arm_motion", this->moduleType());
            arm_motion.setParam("StartingJointState", "current");
            arm_motion.setParam("DestinationJointState", "segmentation_pose");
            arm_motion.exe();
            if(!arm_motion.waitResult()) return -1;
        }

        //get tf robot -> camera
        // camera frame name
        std::string camera_frame = camera_.properties(data::Str[data::DriverAddress]).getValue<std::string>(); // "/<camera>/depth_registered/points"
        camera_frame = camera_frame.substr(1); // "<camera>/depth_registered/points"
        camera_frame = camera_frame.substr(0, camera_frame.find("/")); // "<camera>"
        camera_frame += "_depth_optical_frame"; // "<camera>_depth_optical_frame" 
        //std::cout << camera_frame << std::endl;
        // robot frame name
        skiros_wm::Element robot_ = this->getWorldHandle()->getRobot();
        std::string robot_frame = container_.properties(data::Str[data::FrameId]).getValue<std::string>();
        //std::cout << robot_frame << std::endl;
        // get tf
        tf::StampedTransform rob_cam;
        this->getWorldHandle()->waitForTransform(robot_frame,camera_frame,ros::Time(0),ros::Duration(1.5)); 
        this->getWorldHandle()->lookupTransform(robot_frame,camera_frame,ros::Time(0),rob_cam);
        //compute intersection point
        // unit vector and origin of the camera z axis
        tf::Vector3 cam_z = rob_cam.getBasis().getColumn(2);
        tf::Vector3 cam_origin = rob_cam.getOrigin();
        // normal of the robot XY plane (we're assuming this is parallel to the ground plane)
        tf::Vector3 rob_z(0.0,0.0,1.0);
        tf::Vector3 rob_origin(0,0,0);
        // intersection (wikipedia reference, deal with it) https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
        if (rob_z.dot(cam_z) == 0.0 || (rob_origin - cam_origin).dot(rob_z) == 0.0)
        {
            setProgress("Camera and robot z axes are parallel or identical, swtiching to fixed location method");
            return fixed_position();
        }
        // intersection point in camera frame
        double cam_dist = (rob_origin - cam_origin).dot(rob_z) / rob_z.dot(cam_z);
        //std::cout << cam_dist << std::endl;
        tf::Vector3 intersection_point = cam_dist * cam_z + cam_origin;
        // intersection pose in robot frame (the rotation doesn't matter right now, is overwritten later)
        tf::Transform intersection_robot_frame = tf::Transform(tf::Quaternion(0,0,0,1),intersection_point);// * rob_cam;
        // set the same orientation as the robot frame (no rotation, since it's in the robot frame)
        intersection_robot_frame.setRotation(tf::Quaternion(0,0,0,1));
        // generate object at intersection point
        skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner("AauSpatialReasoner");
        // get first parent of the container, where we want to add the object pose
        std::string container_parent_frame = skiros_lib_tools::getFirstFrameId(*getWorldHandle(), container_);
        //std::cout << container_parent_frame << std::endl;
        // if it's not the robot frame, transform the intersection pose to the parent frame
        tf::Transform obj_tf;
        if (container_parent_frame != robot_frame)
        {
            tf::StampedTransform rob_container;
            this->getWorldHandle()->waitForTransform(robot_frame,container_parent_frame,ros::Time(0),ros::Duration(1.5)); 
            this->getWorldHandle()->lookupTransform(robot_frame,container_parent_frame,ros::Time(0),rob_container);
            obj_tf = rob_container.inverseTimes(intersection_robot_frame);
        }
        else
        {
            obj_tf = intersection_robot_frame;
        }
        // get all the objects associated with the container
        std::vector<std::string> objects = container_.properties(data::Str[data::partReference]).getValues<std::string>();
        //printf("%s",objects);
        // add each object 
        for (int n=0;n<objects.size();n++)
        {
          std::cout << objects[n] << std::endl;
          // generate object on the surface
          skiros_wm::Element object_element = this->getWorldHandle()->getDefaultElement(objects[n]);
          // input data
          reasoner->storeData(object_element, obj_tf.getOrigin(), "Position");
          reasoner->storeData(object_element, obj_tf.getRotation(), "Orientation");
          object_element.addPropertyString("hasRegisteredShot","==FAKE==");
          // add to world model
          updateOrAdd(*getWorldHandle(), object_element, container_.id());
          
        }

        return 1;
    } 
    
    //! \brief This version fakes the locate and adds an object at a fixed location (the previous default behavior)
    int fixed_position()
    {
        setProgress("Initializing.");
        container_ = getParamHandle()->getParamValue<skiros_wm::Element>("Container");
        camera_ = getParamHandle()->getParamValue<skiros_wm::Element>("Camera");
        std::string ObjType = container_.properties(data::Str[data::partReference]).getValue<std::string>();
        skiros_wm::Element skiros_surface;
        skiros_surface.type() = concept::Str[concept::Surface];
        //Fake data input
        skiros_surface.storeData(tf::Vector3(0.5, 0.5, 0.5), data::Size, "AauSpatialReasoner");
        skiros_surface.storeData(tf::Vector3(0.5, 0.0, 0.0), data::Position);
        tf::Quaternion q;
        q.setRPY(0.0,0.0,0.0);
        skiros_surface.storeData(q, data::Orientation);
        skiros_surface.storeData(std::string("map"), data::BaseFrameId);
        std::set<std::string> relations = skiros_surface.getRelationsWrt(container_);
        for(auto s: relations)
            FINFO("Relation: " << s);
        //skiros_surface.properties(data::Str[data::PublishTf]).setValue(false);
        //Fake objects creation
        std::vector<skiros_wm::Element> objects;
        for(int i=0;i<2;i++)
        {
          skiros_wm::Element e;
          if(i<2)
          {
              e = this->getWorldHandle()->getDefaultElement(ObjType);
          }
          else e = this->getWorldHandle()->getDefaultElement(individual::Str[individual::large_box]);
          tf::Vector3 pos;
          pos.setValue(0.0,0.8,0.5);
          e.storeData(pos, "Position", "AauSpatialReasoner");
          tf::Quaternion q;
          q.setRPY(0.0*i,0.0*i,0.5*i);
          e.storeData(q, "Orientation");
          objects.push_back(e);
        }
        ////////////////////////////////////////////////////////
        // Update/Add surface to world model
        ////////////////////////////////////////////////////////
        skiros_surface = container_;
        // Check each found cluster with object database and world model
        for (size_t i=0; i<objects.size(); ++i)
        {
          skiros_wm::Element skiros_object = objects[i];
          skiros_object.addPropertyString("hasRegisteredShot","");
          updateOrAdd(*getWorldHandle(), skiros_object, skiros_surface.id());
        }
        return 1;
    }

};

}
} // namespace

//Export
PLUGINLIB_EXPORT_CLASS(skiros::module::LocateFake, skiros::ModuleBase)
