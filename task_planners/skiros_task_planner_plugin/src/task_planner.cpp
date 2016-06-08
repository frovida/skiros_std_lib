#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include "boost/foreach.hpp"
#include "boost/lexical_cast.hpp"
#include "skiros_task/external_interface.h"
#include "skiros_config/declared_uri.h"
#include "skiros_world_model/reasoners_loading_func.h"
#include <skiros_task/planner_base.h>
#include "skiros_world_model/plugin_loading_func.h"
#include "skiros_common/utility.h"
#include "skiros_world_model/utility.h"
#include "skiros_msgs/TmGoal.h"
#include <skiros_task/planner_interface.h>
    
using namespace skiros_config::owl;
using namespace skiros_wm;
using namespace skiros::planner;
using namespace std;
using namespace skiros_wm::pddl;

namespace skiros_task
{

    typedef boost::shared_ptr<PlannerBase> PlannerInterfacePtrType;
    /*!
     * \brief The TaskPlanner class allows to give a goal state to the task manager and let it plan the skills sequence to achieve it
     *
     * Ontology    ->   Domain              ->
     * World model ->   Present state       ->  Planner      -> Skill sequence generation
     * ?           ->   Goal state          ->
     * ?           ->   Trigger             ->
     *
     */
    class TaskPlanner : public skiros_task::ExternalInterface
    {
        //!< A ROS service for testing
        ros::ServiceServer test_;
        //!< Planner ptr
        PlannerInterfacePtrType planner_ptr_;
        PlannerInterface pddl_;
    public:
        TaskPlanner() {}
        ~TaskPlanner() {}

        void execute()
        {
            /// Advertise a test ROS service
            test_ = getNodeHandle()->advertiseService("task_plan", &TaskPlanner::goalService, this);
        }

        ///The test ROS service
        bool goalService(skiros_msgs::TmGoal::Request  &req,
                 skiros_msgs::TmGoal::Response &res)
        {
            ros::Time start_time = ros::Time::now();
            pddl_.initWorldInterface(getWorldHandle());
            ///Get domain
            pddl_.initDomain();
            FINFO("Domain time:" << (ros::Time::now()-start_time).toSec());
            
            //Need to set the goal before can make initial state.
            pddl_.setGoal(skiros_wm::msgs2elements(req.conditions), req.pddl_goals);

            ///Get initial state
            pddl_.initProblem();
            FINFO("Initial state time:" << (ros::Time::now()-start_time).toSec());

            ///Print domain.pddl and p01.pddl
            pddl_.outputPDDL(); //TODO: Here should return strings with domain and p01. Should be optional (save=true) to save the files

            ///Call planner
            std::vector<Element> plan = pddl_.callPlanner(); //TODO: The planner call is wrapped deep into this function and should be extracted to allow to use other planners
            if(plan.size()<=0)
            {
                res.ok = false;
                return true;
            }
            res.ok = true;
            FINFO("Plan time:" << (ros::Time::now()-start_time).toSec());

            getTaskManager()->startModifingTask();
            clearTask();
            for(Element s : plan)
            {
                FINFO("Adding: " << s.printState("", false));
                insertSkillInSequence(getSkillMgr(s), s.label(), s.properties());
            }
            getTaskManager()->endModifingTask();

            FINFO("Task time:" << (ros::Time::now()-start_time).toSec());
            return true;
        }


    protected:
        //------- Helper functions to manipulate the skill sequence
        std::string getSkillMgr(Element skill)
        {
            if(!skill.hasProperty("Robot"))
            {
                FERROR("Can't find the 'Robot' property in the skill. Can't tell to which robot assign skill: " << skill.printState());
            }
            else
                return skill.properties("Robot").getValue<Element>().properties("SkillMgr").getValue<std::string>();
            return "";
        }

        void clearTask()
        {
            getTaskManager()->clearSkillSequence("planner");
        }

        void executeSequence()
        {
            getTaskManager()->exeSkillSequence();
        }

        void removeSkillInSequence(int index=-1)
        {
            getTaskManager()->removeSkillInSequence("planner", index);
        }

        void insertSkillInSequence(std::string robot_name, std::string skill_name, skiros_common::ParamMap params, int index=-1)
        {
            SkillHolder skill;
            skill.manager = robot_name;
            skill.name = skill_name;
            skill.params = params;
            getTaskManager()->insertSkillInSequence("planner", skill, index);
        }

    };
}

//Export
PLUGINLIB_EXPORT_CLASS(skiros_task::TaskPlanner, skiros_task::ExternalInterface)
