#include <string>
#include <ostream>
#include <iomanip>
#include <vector>
#include <math.h>
#include <numeric>

#include <ros/ros.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "skiros_config/param_types.h"
#include "aau_spatial_reasoner/aau_spatial_reasoner.h"
#include "skiros_common/logger_sys.h"
#include <pluginlib/class_list_macros.h>
#include "skiros_config/declared_uri.h"

//For publishTf
#include <tf/transform_broadcaster.h>
//For publish Markers
#include <visualization_msgs/Marker.h>

namespace skiros_reasoner
{
    const std::string THIS_REASONER_NAME = "AauSpatialReasoner";

    using SPair = std::pair<std::string, std::string>;
    using namespace skiros_config;
    using namespace skiros_wm;
    using namespace skiros_config::owl;
    using namespace std;

    AauSpatialReasoner::AauSpatialReasoner()
    {
        setName(THIS_REASONER_NAME);
        setType(data::Str[data::SpatialReasoner]);
    }
    AauSpatialReasoner::~AauSpatialReasoner(){}

    void AauSpatialReasoner::onAddProperties(skiros_wm::Element& e)
    {
      skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();
      //Add properties
      if(!e.hasProperty(data::Size)) e.addProperty(param_types.getDefault(data::Size));
      if(!e.hasProperty(data::Position)) e.addProperty(param_types.getDefault(data::Position));
      if(!e.hasProperty(data::Orientation)) e.addProperty(param_types.getDefault(data::Orientation));
      if(!e.hasProperty(data::PublishTf)) e.addProperty(data::PublishTf, true);
    }

    bool AauSpatialReasoner::storeData(skiros_wm::Element  & e, boost::any any, std::string  set_code)
    {
      addProperties(e);
      if (any.type() == typeid(skiros_wm::ReasonerDataMap)) //The property list read from OWL
      {
          //The property list read from OWL is converted in a runtime-suitable format
          skiros_wm::ReasonerDataMap map = boost::any_cast<skiros_wm::ReasonerDataMap>(any);
          if(map.find("SizeX")!=map.end())
          {
              //Size
              try
              {
                  std::vector<double> v;
                  v.push_back(boost::lexical_cast<double>(map.find("SizeX")->second.first));
                  v.push_back(boost::lexical_cast<double>(map.find("SizeY")->second.first));
                  v.push_back(boost::lexical_cast<double>(map.find("SizeZ")->second.first));
                  e.properties(data::Size).setAllValues(v);
                  e.removeProperty("SizeX");
                  e.removeProperty("SizeY");
                  e.removeProperty("SizeZ");
              }
              catch(boost::bad_lexical_cast err){}
          }
          if(map.find("OrientationW")!=map.end())
          {
              //Orientation
              try
              {
                  std::vector<double> v;
                  v.push_back(boost::lexical_cast<double>(map.find("OrientationX")->second.first));
                  v.push_back(boost::lexical_cast<double>(map.find("OrientationY")->second.first));
                  v.push_back(boost::lexical_cast<double>(map.find("OrientationZ")->second.first));
                  v.push_back(boost::lexical_cast<double>(map.find("OrientationW")->second.first));
                  e.properties(data::Orientation).setAllValues(v);
                  e.removeProperty("OrientationW");
                  e.removeProperty("OrientationX");
                  e.removeProperty("OrientationY");
                  e.removeProperty("OrientationZ");
              }
              catch(boost::bad_lexical_cast err){}
          }
          if(map.find("PositionX")!=map.end())
          {
              //Position
              try
              {
                  std::vector<double> v;
                  v.push_back(boost::lexical_cast<double>(map.find("PositionX")->second.first));
                  v.push_back(boost::lexical_cast<double>(map.find("PositionY")->second.first));
                  v.push_back(boost::lexical_cast<double>(map.find("PositionZ")->second.first));
                  e.properties(data::Position).setAllValues(v);
                  e.removeProperty("PositionX");
                  e.removeProperty("PositionY");
                  e.removeProperty("PositionZ");
              }
              catch(boost::bad_lexical_cast err){}
          }
          return true;
      }
      else if(set_code=="Orientation" && any.type() == typeid(tf::Quaternion))
      {
          tf::Quaternion q = boost::any_cast<tf::Quaternion>(any);
          std::vector<double> v;
          v.push_back(q.getX());
          v.push_back(q.getY());
          v.push_back(q.getZ());
          v.push_back(q.getW());
          e.properties(data::Orientation).setAllValues(v);
          return true;
      }
      else if(set_code=="Size" && any.type() == typeid(tf::Vector3))
      {
          tf::Vector3 q = boost::any_cast<tf::Vector3>(any);
          std::vector<double> v;
          v.push_back(q.getX());
          v.push_back(q.getY());
          v.push_back(q.getZ());
          e.properties(data::Size).setAllValues(v);
          return true;
      }
      else if(set_code=="Position" && any.type() == typeid(tf::Vector3))
      {
          tf::Vector3 q = boost::any_cast<tf::Vector3>(any);
          std::vector<double> v;
          v.push_back(q.getX());
          v.push_back(q.getY());
          v.push_back(q.getZ());
          e.properties(data::Position).setAllValues(v);
          return true;
      }
      else if(set_code=="Pose" && any.type() == typeid(tf::Pose))
      {
          tf::Pose p = boost::any_cast<tf::Pose>(any);
          std::vector<double> v;
          v.push_back(p.getOrigin().getX());
          v.push_back(p.getOrigin().getY());
          v.push_back(p.getOrigin().getZ());
          e.properties(data::Position).setAllValues(v);
          v.clear();
          v.push_back(p.getRotation().getX());
          v.push_back(p.getRotation().getY());
          v.push_back(p.getRotation().getZ());
          v.push_back(p.getRotation().getW());
          e.properties(data::Orientation).setAllValues(v);
          return true;
      }
      else if(set_code=="PoseMsg" && any.type() == typeid(geometry_msgs::Pose))
      {
          geometry_msgs::Pose pm = boost::any_cast<geometry_msgs::Pose>(any);
          tf::Pose p;
          tf::poseMsgToTF(pm, p);
          std::vector<double> v;
          v.push_back(p.getOrigin().getX());
          v.push_back(p.getOrigin().getY());
          v.push_back(p.getOrigin().getZ());
          e.properties(data::Position).setAllValues(v);
          v.clear();
          v.push_back(p.getRotation().getX());
          v.push_back(p.getRotation().getY());
          v.push_back(p.getRotation().getZ());
          v.push_back(p.getRotation().getW());
          e.properties(data::Orientation).setAllValues(v);
          return true;
      }
      else if(set_code=="Transform" && any.type() == typeid(tf::Transform))
      {
          tf::Transform p = boost::any_cast<tf::Transform>(any);
          std::vector<double> v;
          v.push_back(p.getOrigin().getX());
          v.push_back(p.getOrigin().getY());
          v.push_back(p.getOrigin().getZ());
          e.properties(data::Position).setAllValues(v);
          v.clear();
          v.push_back(p.getRotation().getX());
          v.push_back(p.getRotation().getY());
          v.push_back(p.getRotation().getZ());
          v.push_back(p.getRotation().getW());
          e.properties(data::Orientation).setAllValues(v);
          return true;
      }
      else if(set_code=="BaseFrame" && any.type() == typeid(std::string))
      {
          e.addPropertyString(data::BaseFrameId, boost::any_cast<std::string>(any));
      }
      else
      {
        FERROR("[AauSpatialReasoner] Cannot convert " << any.type().name() << " to element data: Type unknown");
        return false;
      }
      return true;
    }

    boost::any AauSpatialReasoner::getData(skiros_wm::Element & e, std::string  get_code)
    {
        if(get_code=="Position" && e.properties(data::Position).isSpecified())
        {
            std::vector<double> v = e.properties(data::Position).getValues<double>();
            return boost::any(tf::Vector3(v[0], v[1], v[2]));
        }
        else if(get_code=="Orientation" && e.properties(data::Orientation).isSpecified())
        {
            std::vector<double> v = e.properties(data::Orientation).getValues<double>();
            return boost::any(tf::Quaternion(v[0], v[1], v[2], v[3]));
        }
        else if(get_code=="Size" && e.properties(data::Size).isSpecified())
        {
            std::vector<double> v = e.properties(data::Size).getValues<double>();
            return boost::any(tf::Vector3(v[0], v[1], v[2]));
        }
        else if(get_code=="Transform" &&
                e.properties(data::Position).isSpecified() &&
                e.properties(data::Orientation).isSpecified())
        {
            tf::Transform transform;
            std::vector<double> v = e.properties(data::Position).getValues<double>();
            transform.setOrigin(tf::Vector3(v[0], v[1], v[2]));
            v = e.properties(data::Orientation).getValues<double>();
            transform.setRotation(tf::Quaternion(v[0], v[1], v[2], v[3]));
            return boost::any(transform);
        }
        else if(get_code=="Pose" &&
                e.properties(data::Position).isSpecified() &&
                e.properties(data::Orientation).isSpecified())
        {
            tf::Pose pose;
            std::vector<double> v = e.properties(data::Position).getValues<double>();
            pose.setOrigin(tf::Vector3(v[0], v[1], v[2]));
            v = e.properties(data::Orientation).getValues<double>();
            pose.setRotation(tf::Quaternion(v[0], v[1], v[2], v[3]));
            return boost::any(pose);
        }
        else if(get_code=="PoseMsg" &&
                e.properties(data::Position).isSpecified() &&
                e.properties(data::Orientation).isSpecified())
        {
            tf::Pose pose;geometry_msgs::Pose msg;
            std::vector<double> v = e.properties(data::Position).getValues<double>();
            pose.setOrigin(tf::Vector3(v[0], v[1], v[2]));
            v = e.properties(data::Orientation).getValues<double>();
            pose.setRotation(tf::Quaternion(v[0], v[1], v[2], v[3]));
            tf::poseTFToMsg(pose, msg);
            return boost::any(msg);
        }
        else if(get_code=="FPFHFeatures" &&
                e.properties("FPFHFeatures").isSpecified())
        {
            //std::string filename = e.properties("FPFHFeatures").getValue<std::string>();
            //TODO: get from file name
            //return boost::any(whatever);
        }
        else throw std::invalid_argument("[AauSpatialReasoner::getData] Impossible to get specified data.");
    }

    ReasonerDataMap AauSpatialReasoner::extractOwlData(skiros_wm::Element & e)
    {
        //This cleans the object of all params that cannot be converted to OWL params directly (mainly the vectors), and return them in the ReasonerDataMap
        skiros_wm::ReasonerDataMap to_ret;
        skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();
        if(e.hasProperty(data::Position) && e.properties(data::Position).isSpecified())
        {
            std::vector<std::string> v;
            e.properties(data::Position).getValuesStr(v);
            to_ret.insert(ReasonerDataPair("PositionX", SPair(v[0], XSD_DOUBLE)));
            to_ret.insert(ReasonerDataPair("PositionY", SPair(v[1], XSD_DOUBLE)));
            to_ret.insert(ReasonerDataPair("PositionZ", SPair(v[2], XSD_DOUBLE)));
        }
        if(e.hasProperty(data::Orientation) &&  e.properties(data::Orientation).isSpecified())
        {
            std::vector<std::string> v;
            e.properties(data::Orientation).getValuesStr(v);
            to_ret.insert(skiros_wm::ReasonerDataPair("OrientationX", std::pair<std::string, std::string>(v[0], XSD_DOUBLE)));
            to_ret.insert(skiros_wm::ReasonerDataPair("OrientationY", std::pair<std::string, std::string>(v[1], XSD_DOUBLE)));
            to_ret.insert(skiros_wm::ReasonerDataPair("OrientationZ", std::pair<std::string, std::string>(v[2], XSD_DOUBLE)));
            to_ret.insert(skiros_wm::ReasonerDataPair("OrientationW", std::pair<std::string, std::string>(v[3], XSD_DOUBLE)));
        }
        if(e.hasProperty(data::Size) &&  e.properties(data::Size).isSpecified())
        {
            std::vector<std::string> v;
            e.properties(data::Size).getValuesStr(v);
            to_ret.insert(skiros_wm::ReasonerDataPair("SizeX", std::pair<std::string, std::string>(v[0], XSD_DOUBLE)));
            to_ret.insert(skiros_wm::ReasonerDataPair("SizeY", std::pair<std::string, std::string>(v[1], XSD_DOUBLE)));
            to_ret.insert(skiros_wm::ReasonerDataPair("SizeZ", std::pair<std::string, std::string>(v[2], XSD_DOUBLE)));
        }
        e.removeProperty(data::Position);
        e.removeProperty(data::Orientation);
        e.removeProperty(data::Size);
        e.removeProperty(data::FrameId);
        return to_ret;
    }

    float AauSpatialReasoner::computeCorrelationCoefficient(std::vector<float> &v1, std::vector<float> &v2)
    {
      assert(v1.size() == v2.size());

      // Pearson's correlation coefficient

      float m1 = std::accumulate(v1.begin(), v1.end(), 0)/(float)v1.size();
      float m2 = std::accumulate(v2.begin(), v2.end(), 0)/(float)v2.size();

      float cov = 0;
      float var1 = 0;
      float var2 = 0;
      for (int dim=0; dim<v1.size(); ++dim)
      {
        float diff1 = (v1[dim]-m1);
        float diff2 = (v2[dim]-m2);
        cov += diff1 * diff2;
        var1 += diff1 * diff1;
        var2 += diff2 * diff2;
      }

      return ((var1 == 0) || (var2 == 0)) ? 0 : (cov / (sqrt(var1) * sqrt(var2)));
    }

    float AauSpatialReasoner::computeSimilarityPosition(skiros_wm::Element &lhs, skiros_wm::Element &rhs)
    {
      if (rhs.properties(data::Position).state() != skiros_common::specified ||
          lhs.properties(data::Position).state() != skiros_common::specified)
        return 1.0;


      // similarity based on object size - higher tolerance for larger objects
      tf::Vector3 rhsSize = boost::any_cast<tf::Vector3>(getData(rhs, "Size"));
      tf::Vector3 rhsPos = boost::any_cast<tf::Vector3>(getData(rhs, "Position"));
      tf::Vector3 lhsPos = boost::any_cast<tf::Vector3>(getData(lhs, "Position"));

      float sizeParam = rhsSize.x() * rhsSize.y() * rhsSize.z() * 10;
      if (sizeParam < std::numeric_limits<float>::epsilon()) sizeParam = 1;

      return exp(-tf::tfDistance(lhsPos, rhsPos)/sizeParam);
    }

    float AauSpatialReasoner::computeSimilaritySize(skiros_wm::Element &lhs, skiros_wm::Element &rhs)
    {
      if (rhs.properties(data::Size).state() != skiros_common::specified ||
          lhs.properties(data::Size).state() != skiros_common::specified)
        return 0.0;

      // similarity based on object size - higher tolerance for larger object
      tf::Vector3 rhsSize = boost::any_cast<tf::Vector3>(getData(rhs, "Size"));
      tf::Vector3 lhsSize = boost::any_cast<tf::Vector3>(getData(lhs, "Size"));
      float sizeParam = rhsSize.x() * rhsSize.y() * rhsSize.z();

      float dist = exp(-tf::tfDistance(lhsSize, rhsSize)/(sizeParam*20.0));

      // similarity based on Pearson's correlation coefficient
      tf::Vector3FloatData rv1;
      lhsSize.serializeFloat(rv1);
      std::vector<float> v1(rv1.m_floats, rv1.m_floats+3);

      tf::Vector3FloatData rv2;
      rhsSize.serializeFloat(rv2);
      std::vector<float> v2(rv2.m_floats, rv2.m_floats+3);

      return computeCorrelationCoefficient(v1, v2) * dist;
    }

    float AauSpatialReasoner::computeSimilarityOrientation(skiros_wm::Element &lhs, skiros_wm::Element &rhs)
    {
      if (rhs.properties(data::Orientation).state() != skiros_common::specified ||
          lhs.properties(data::Orientation).state() != skiros_common::specified)
        return 1.0;

      tf::Quaternion rhsOri = boost::any_cast<tf::Quaternion>(getData(rhs, "Orientation"));
      tf::Quaternion lhsOri = boost::any_cast<tf::Quaternion>(getData(lhs, "Orientation"));

     return 1.0 - tf::angleShortestPath(lhsOri, rhsOri)/M_PI;
    }

    float AauSpatialReasoner::computeIstanceSimilarity(skiros_wm::Element &lhs, skiros_wm::Element &rhs)
    {
        try
        {
            if(lhs.properties(data::SpatialReasoner).getValue<std::string>() != THIS_REASONER_NAME ||
                    rhs.properties(data::SpatialReasoner).getValue<std::string>() != THIS_REASONER_NAME)
            {
                throw std::invalid_argument("The spatial reasoner specified for arguments doesn't match.");
            }
        }
        catch(std::invalid_argument err)
        {
            throw err;
        }
      //float w_pos = params_[w_pos].getValue<float>(),  w_size = params_[w_size].getValue<float>(),  w_ori = params_[w_ori].getValue<float>();
      float w_pos = 2, w_size = 0.5, w_ori = 0.1;
      float w_total = w_pos + w_size + w_ori;
      return w_pos/w_total * computeSimilarityPosition(lhs, rhs) +
             w_size/w_total * computeSimilaritySize(lhs, rhs) +
             w_ori/w_total * computeSimilarityOrientation(lhs, rhs);
    }

    float AauSpatialReasoner::computeClassSimilarity(skiros_wm::Element &lhs, skiros_wm::Element &rhs)
    {
        try
        {
            if(lhs.properties(data::SpatialReasoner).getValue<std::string>() != THIS_REASONER_NAME ||
                    rhs.properties(data::SpatialReasoner).getValue<std::string>() != THIS_REASONER_NAME)
            {
                throw std::invalid_argument("The spatial reasoner specified for arguments doesn't match.");
            }
        }
        catch(std::invalid_argument err)
        {
            throw err;
        }
        return computeSimilaritySize(lhs, rhs);
    }

    RelationsMap AauSpatialReasoner::computeRelations(skiros_wm::Element &subject, skiros_wm::Element &object)
    {
      RelationsMap to_ret;
      std::string subject_frame, object_frame;
      tf::StampedTransform sub2obj;
      ros::Time now = ros::Time::now();
      subject_frame = subject.toUrl();
      object_frame = object.toUrl();
      if(!subject.hasProperty(data::FrameId) || !object.hasProperty(data::FrameId))
      {
          if(subject.hasProperty(data::BaseFrameId) && !subject.hasProperty(data::FrameId))
              getTfListener()->setTransform(tf::StampedTransform(DiscreteReasoner::getData<tf::Transform>(subject, "Transform"),
                                                        now,
                                                        subject.properties(data::BaseFrameId).getValue<std::string>(),
                                                        subject_frame));
          if(object.hasProperty(data::BaseFrameId) && !object.hasProperty(data::FrameId))
              getTfListener()->setTransform(tf::StampedTransform(DiscreteReasoner::getData<tf::Transform>(object, "Transform"),
                                                        now,
                                                        object.properties(data::BaseFrameId).getValue<std::string>(),
                                                        object_frame));
      }
      if(waitAndLookupFirstTransform(subject_frame, object_frame, sub2obj, now))
      {
          //Front-back
          if (sub2obj.getOrigin().getX() < 0)
              to_ret.insert(RelationsPair("front", SPair(to_string(abs(sub2obj.getOrigin().getX())), "")));
          else
              to_ret.insert(RelationsPair("back", SPair(to_string(abs(sub2obj.getOrigin().getX())), "")));
          //Left-right
          if (sub2obj.getOrigin().getY() < 0)
              to_ret.insert(RelationsPair("left", SPair(to_string(abs(sub2obj.getOrigin().getY())), "")));
          else
              to_ret.insert(RelationsPair("right", SPair(to_string(abs(sub2obj.getOrigin().getY())), "")));
          //Over-under
          if (sub2obj.getOrigin().getZ() < 0)
              to_ret.insert(RelationsPair("over", SPair(to_string(abs(sub2obj.getOrigin().getZ())), "")));
          else
              to_ret.insert(RelationsPair("under", SPair(to_string(abs(sub2obj.getOrigin().getZ())), "")));
      }
      else FERROR("[AauSpatialReasoner] No tf found between " << subject_frame << " and " << object_frame);
      return to_ret;
    }

    bool AauSpatialReasoner::waitAndLookupFirstTransform(std::string target_frame, std::string  source_frame, tf::StampedTransform & transform, ros::Time start, ros::Duration d, ros::Duration interval)
    {
        if(target_frame=="" || source_frame=="")
            return false;
        ros::Time start_time = start;
        ros::Time now = start;
        while(!getTfListener()->waitForTransform(target_frame, source_frame, now, interval))
        {
            now = ros::Time::now();
            if((now-start_time)>d)
                return false;
        }
        getTfListener()->lookupTransform(target_frame, source_frame, now, transform);
        return true;
    }

    //------------ WORLD MODEL THREAD AND UTILS ----------------

    std::list<TfPubData> AauSpatialReasoner::getTfListRecursive(skiros_wm::Element root, std::string frame_id, skiros_wm::owl::WorldModel * model)
    {
        std::list<TfPubData> to_ret;
        skiros_wm::Element child;
        TfPubData temp;
        //If the root has a frameId it override the parent's one
        if(root.hasProperty("FrameId")) temp.parent_frame = root.properties("FrameId").getValue<std::string>();
        else temp.parent_frame = frame_id;
        //Get all root childs
        std::list<ChildsPair> childs = model->getChilds(root, "spatiallyRelated");
        for(ChildsPair pair : childs)
        {
            bool update = false;
            child = pair.second;
            try
            {
                if(child.properties(data::SpatialReasoner).find<std::string>("AauSpatialReasoner")>=0 && child.properties(data::Str[data::PublishTf]).getValue<bool>())
                {
                    temp.tf = DiscreteReasoner::getData<tf::Transform>(child, "Transform");
                    try
                    {
                        temp.size = DiscreteReasoner::getData<tf::Vector3>(child, data::Size);
                    }
                    catch(std::invalid_argument e){}

                    if(!child.hasProperty(data::Str[data::FrameId]))
                    {
                        child.addPropertyString(data::Str[data::FrameId], child.toUrl());
                        update = true;
                    }
                    if(!child.hasProperty(data::Str[data::BaseFrameId]))
                    {
                        child.addPropertyString(data::Str[data::BaseFrameId], temp.parent_frame);
                        update = true;
                    }
                    else
                    {
                        if(child.properties(data::Str[data::BaseFrameId]).getValue<std::string>()!=temp.parent_frame)
                        {
                            child.properties(data::Str[data::BaseFrameId]).setValue(temp.parent_frame);
                            //UNSAFE! TODO: fix this
                            //tf::Transform parent_tf = getData<tf::Transform>(root, "Transform");
                            //temp.tf = temp.tf.inverseTimes(parent_tf);
                            //storeData(child, temp.tf, "Transform");
                            update = true;
                        }
                    }
                    if(update)
                        model->updateElement(child);
                    temp.child_frame = child.properties(data::Str[data::FrameId]).getValue<std::string>();
                    to_ret.push_back(temp);
                    //FINFO("publishing: " << temp.parent_frame << "contain" << pair.second.label());
                }
            }
            catch(std::invalid_argument e){/*FERROR(e.what());I just skip the element in case a property is missing*/}
            std::list<TfPubData> temp1 = getTfListRecursive(child, temp.parent_frame, model);
            to_ret.insert(to_ret.end(), temp1.begin(), temp1.end());
        }
        return to_ret;
    }

    //Publish object marker
    void AauSpatialReasoner::publishObjectMarker(ros::Publisher & marker_pub, TfPubData & obj)
    {
        static int id_counter = 0;
        if(!obj.size.x() && !obj.size.y() && !obj.size.z())return;
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(2.0);
        marker.type = visualization_msgs::Marker::CUBE;

        marker.pose.position.x = obj.tf.getOrigin().getX();
        marker.pose.position.y = obj.tf.getOrigin().getY();
        marker.pose.position.z = obj.tf.getOrigin().getZ();

        marker.pose.orientation.w = obj.tf.getRotation().getW();
        marker.pose.orientation.x = obj.tf.getRotation().getX();
        marker.pose.orientation.y = obj.tf.getRotation().getY();
        marker.pose.orientation.z = obj.tf.getRotation().getZ();

        marker.scale.x = obj.size.getX();
        marker.scale.y = obj.size.getY();
        marker.scale.z = obj.size.getZ();

        marker.color.r = 0.0;//rand()%300/300.0;
        marker.color.g = 0.0;//rand()%300/300.0;
        marker.color.b = 1.0;//rand()%300/300.0;
        marker.color.a = 0.5;

        marker.header.frame_id = obj.parent_frame;
        marker.ns = "skiros_wm";
        if(obj.marker_id!=0) marker.id = obj.marker_id;
        else
        {
            marker.id = id_counter++;
            obj.marker_id = marker.id;
        }
        marker_pub.publish(marker);
    }

    // Publish the tf_list on tf
    void AauSpatialReasoner::execute(skiros_wm::owl::WorldModel * wm)
    {
        std::list<TfPubData> tf_list;
        std::string frame_id("");
        skiros_wm::Element root = wm->getElement(0);
        if(root.hasProperty("FrameId"))frame_id = root.properties("FrameId").getValue<std::string>();
        else
        {
            FERROR("[publishTf] Can't publish on tf without a scene frame. Not tf will be published");
            return;
        }
        tf::TransformBroadcaster tf_pub;
        //! Publisher for markers
        ros::NodeHandle nh;
        ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(nh.resolveName("skiros_markers_out"), 10);
        ros::Rate loop_rate(10);
        auto lock = wm->startChange();
        lock->unlock();
        //Start loop
        int counter = 0;
        while(ros::ok())
        {
            if(wm->hasChanged())
            {
                tf_list.clear();
                lock->lock();
                tf_list = getTfListRecursive(root, frame_id, wm);
                lock->unlock();
            }
            for(TfPubData & d : tf_list)
            {
                //FINFO(d.child_frame  << ","  << d.tf.getOrigin().getX() << "," << d.tf.getOrigin().getY() << "," << d.tf.getOrigin().getZ()<< ","  << d.tf.getRotation().getW()<< "," << d.tf.getRotation().getX()<< "," << d.tf.getRotation().getY()<< "," << d.tf.getRotation().getZ());
                tf_pub.sendTransform(tf::StampedTransform(d.tf, ros::Time::now(), d.parent_frame, d.child_frame));
                if(counter==10)
                    publishObjectMarker(marker_pub, d);
            }
            if(counter++>10)counter=0;
            loop_rate.sleep();
        }
        //std::cout << "[publishTf] Exit" << std::endl;
    }

} //namespace tabletop_object_detector

//Export
PLUGINLIB_EXPORT_CLASS(skiros_reasoner::AauSpatialReasoner, skiros_wm::DiscreteReasoner)
