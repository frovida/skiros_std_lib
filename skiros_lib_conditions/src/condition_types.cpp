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

#include <sstream>
#include "skiros_world_model/condition.h"
#include "skiros_common/param_handler.h"
#include "skiros_world_model/world_element.h"
#include "skiros_config/declared_uri.h"
#include <pluginlib/class_list_macros.h>//Plugin export library

using namespace skiros_config::owl;

namespace skiros
{
namespace condition
{
    class ObjectAtLocation : public ConditionRelation
    {
        public:

            ObjectAtLocation(){}

            void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                      boost::shared_ptr<skiros_common::ParamHandler> ph,
                      bool desired_state,
                      std::string location_key,
                      std::string object_key)
            {
                ConditionRelation::init(wm,ph, desired_state, location_key, object_key);
                setAllowAbstractTypes();
                setType(skiros_config::owl::concept::Str[skiros_config::owl::concept::ObjectAtLocation]);
                setRelationType(skiros_config::owl::relation::Str[skiros_config::owl::relation::contain]);
                setDescription("["+getType()+"]Is object '" + object_key +"' at location '" + location_key +"'?");
            }

            ~ObjectAtLocation(){}

    };

    class ObjectInCell : public ConditionRelation
    {
        public:

            ObjectInCell(){}

            void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                      boost::shared_ptr<skiros_common::ParamHandler> ph,
                      bool desired_state,
                      std::string object_key,
                      std::string cell_key)
            {
                ConditionRelation::init(wm,ph, desired_state, object_key, cell_key);
                setAllowAbstractTypes();
                setType(skiros_config::owl::concept::Str[skiros_config::owl::concept::ObjectInCell]);
                setRelationType(skiros_config::owl::relation::Str[skiros_config::owl::relation::contain]);
                setDescription("["+getType()+"]Is object '" + object_key +"' in cell '" + cell_key +"'?");
            }

            ~ObjectInCell(){}

    };

    class CellInKit : public ConditionRelation
    {
        public:

            CellInKit(){}

            void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                      boost::shared_ptr<skiros_common::ParamHandler> ph,
                      bool desired_state,
                      std::string cell_key,
                      std::string kit_key)
            {
                ConditionRelation::init(wm,ph, desired_state, cell_key, kit_key);
                setAllowAbstractTypes();
                setType(skiros_config::owl::concept::Str[skiros_config::owl::concept::CellInKit]);
                setRelationType(skiros_config::owl::relation::Str[skiros_config::owl::relation::spatiallyRelated]);
                setDescription("["+getType()+"]Is cell '" + cell_key +"' in kit '" + kit_key +"'?");
            }

            ~CellInKit(){}

    };

    class RobotAtLocation : public ConditionRelation
    {
        public:

            RobotAtLocation(){}

            void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                      boost::shared_ptr<skiros_common::ParamHandler> ph,
                      bool desired_state,
                      std::string robot_key,
                      std::string location_key)
            {
                ConditionRelation::init(wm,ph, desired_state, robot_key, location_key);
                setType("RobotAtLocation");
                setRelationType(skiros_config::owl::relation::Str[skiros_config::owl::relation::robotAt]);
                setDescription("["+getType()+"]Is robot '" + robot_key +"' at location '" + location_key +"'?");
            }

            ~RobotAtLocation(){}


            bool evaluate()
            {
              updateDescription();
              //Extract the ID values from the parameter handler
              skiros_wm::Element subject = getSubject();
              if(subject.id()<0) return false;
              skiros_wm::Element object = getObject();
              if(object.id()<0) return false;
              std::stringstream ss;
              ss << "SELECT ?y WHERE {{stmn:"<< subject.toUrl() << " stmn:robotAt ?y.} UNION{ ?z stmn:hasA stmn:" << subject.toUrl() << ".  ?z stmn:robotAt ?y. } }";
              ss.str(wm_->queryOntology(ss.str()));
              std::string object_url;
              ss >> object_url;
              //If the query return a match means that the object is contained into location
              if(object_url==getObject().toUrl()) return getDesiredState();
              else return !getDesiredState();
            }

    };

    class Holding : public ConditionRelation
    {
        public:

            Holding(){}

            void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                      boost::shared_ptr<skiros_common::ParamHandler> ph,
                      bool desired_state,
                      std::string gripper_key,
                      std::string object_key)
            {
                ConditionRelation::init(wm,ph, desired_state, gripper_key, object_key);
                setAllowAbstractTypes(true);
                setType("Holding");
                setRelationType(skiros_config::owl::relation::Str[skiros_config::owl::relation::contain]);
                setDescription("["+getType()+"]Is the gripper '" + gripper_key +"' holding '" + object_key +"'?");
            }

            ~Holding(){}

    };

    class Carrying : public ConditionRelation
    {
        public:

            Carrying()
            {
                setRelationType(skiros_config::owl::relation::Str[skiros_config::owl::relation::spatiallyRelated]);
            }

            void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                      boost::shared_ptr<skiros_common::ParamHandler> ph,
                      bool desired_state,
                      std::string subject_key,
                      std::string object_key)
            {
                ConditionRelation::init(wm,ph, desired_state, subject_key, object_key);
                setType("Carrying");
                setDescription("["+getType()+"]Is the robot '" + subject_key +"' carrying '" + object_key +"'?");
            }

            ~Carrying(){}

    };

    class InKit : public ConditionRelation
    {
    public:

        InKit()
        {
            setRelationType(skiros_config::owl::relation::Str[skiros_config::owl::relation::isInKit]);
        }

        void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                  boost::shared_ptr<skiros_common::ParamHandler> ph,
                  bool desired_state,
                  std::string subject_key,
                  std::string object_key)
        {
            ConditionRelation::init(wm,ph, desired_state, subject_key, object_key);
            setAllowAbstractTypes();
            setType("InKit");
            setDescription("["+getType()+"]Is the part '" + object_key +"' inside the kit '" + subject_key +"'?");
        }

        ~InKit(){}

    };

    class EmptyHanded :  public ConditionProperty<std::string>
    {
    public:

        EmptyHanded()
        {
            setType("EmptyHanded");
        }

        void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                  boost::shared_ptr<skiros_common::ParamHandler> ph,
                  bool desired_state,
                  std::string gripper_key,
                  std::string object_key)
        {
            ConditionProperty<std::string>::init(wm,ph, desired_state, gripper_key, object_key);
            setType("EmptyHanded");
            setPropertyType(skiros_config::owl::data::Str[skiros_config::owl::data::ContainerState]);
            setPropertyValue("Empty");
            setDescription("["+getType()+"]Is gripper '" + gripper_key +"' empty?");
        }

        bool evaluate()
        {
            auto childs = wm_->getChildElements(getSubject());
            if(childs.size()!=0 ^ getDesiredState())
                ConditionProperty::set();
            return ConditionProperty::evaluate();
        }

        ~EmptyHanded(){}

    };

    class LocationEmpty : public ConditionProperty<std::string>
	{
		public:

        LocationEmpty()
        {
            setPropertyType(skiros_config::owl::data::Str[skiros_config::owl::data::ContainerState]);
            setPropertyValue("Empty");
        }

        void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                  boost::shared_ptr<skiros_common::ParamHandler> ph,
                  bool desired_state,
                  std::string location_key,
                  std::string object_key)
        {
            ConditionProperty<std::string>::init(wm,ph, desired_state, location_key, object_key);
            setType("LocationEmpty");
            setDescription("["+getType()+"]Is location '" + location_key +"' empty?");
        }

        ~LocationEmpty(){}


       virtual bool evaluate()
        {
            std::vector<skiros_wm::Element> v = wm_->getChildElements(getSubject(), relation::sceneProperty, concept::PhysicalObject);
            if(v.size()==0 && getDesiredState())ConditionProperty<std::string>::set();
            else if (v.size()!=0 && !getDesiredState())ConditionProperty<std::string>::set();
            return ConditionProperty<std::string>::evaluate();
        }
	};

    class FitsIn : public ConditionProperty<std::string>
    {
    public:

        FitsIn()
        {
            setPropertyType(skiros_config::owl::data::Str[skiros_config::owl::data::partReference]);
        }

        void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                  boost::shared_ptr<skiros_common::ParamHandler> ph,
                  bool desired_state,
                  std::string container,
                  std::string object)
        {
            ConditionProperty<std::string>::init(wm, ph, desired_state, container, object);
            setType("FitsIn");
            setPropertyValue();
        }

        ~FitsIn(){}

        bool evaluate()
        {
            setPropertyValue();
            std::string to_query;
            skiros_wm::Element temp = wm_->getDefaultElement(property_value_);
            to_query = "SELECT ?x WHERE { ?x rdf:type stmn:PlacingPose. ?x stmn:AssociatedClass '"+temp.type()+"'^^xsd:string. ?x stmn:AssociatedClass '"+getSubject().type()+"'^^xsd:string. }";
            //FINFO(to_query);
            std::stringstream ss(wm_->queryOntology(to_query));
            //FINFO(ss.str());
            std::string placing_pose;
            ss >> placing_pose;
            if(!ss.eof() && placing_pose!="")
                if(getDesiredState()) ConditionProperty::set();
            else
                if(!getDesiredState()) ConditionProperty::set();
            return ConditionProperty::evaluate();
        }

        bool set()
        {
            setPropertyValue();
            return ConditionProperty::set();
        }

    private:

        void setPropertyValue()
        {
            skiros_wm::Element e = ph_->getParamValue<skiros_wm::Element>(object_key_);
            property_value_ = e.label();
            setDescription("["+getType()+"]Does '" + property_value_ +"' fit in '"+ getSubject().printState("", false) +"'?");
        }
    };

}
}

//Export
PLUGINLIB_EXPORT_CLASS(skiros::condition::ObjectAtLocation, skiros::condition::ConditionBase)
PLUGINLIB_EXPORT_CLASS(skiros::condition::ObjectInCell, skiros::condition::ConditionBase)
PLUGINLIB_EXPORT_CLASS(skiros::condition::CellInKit, skiros::condition::ConditionBase)
PLUGINLIB_EXPORT_CLASS(skiros::condition::RobotAtLocation, skiros::condition::ConditionBase)
PLUGINLIB_EXPORT_CLASS(skiros::condition::Holding, skiros::condition::ConditionBase)
PLUGINLIB_EXPORT_CLASS(skiros::condition::Carrying, skiros::condition::ConditionBase)
PLUGINLIB_EXPORT_CLASS(skiros::condition::InKit, skiros::condition::ConditionBase)
PLUGINLIB_EXPORT_CLASS(skiros::condition::EmptyHanded, skiros::condition::ConditionBase)
PLUGINLIB_EXPORT_CLASS(skiros::condition::LocationEmpty, skiros::condition::ConditionBase)
PLUGINLIB_EXPORT_CLASS(skiros::condition::FitsIn, skiros::condition::ConditionBase)
