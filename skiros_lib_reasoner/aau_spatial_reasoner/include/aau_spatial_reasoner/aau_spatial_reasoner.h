#include "skiros_world_model/discrete_reasoner.h"
#include <tf/transform_listener.h>


namespace skiros_reasoner
{


//!< A static tf listener. Allow  multiple instances to interact with TF with a unique listener.
static boost::shared_ptr<tf::TransformListener> tf_listener;

boost::shared_ptr<tf::TransformListener> getTfListener()
{
    if(tf_listener==NULL)
    {
        tf_listener.reset(new tf::TransformListener());
        ros::Duration(0.2).sleep(); //Bufferize some transforms
    }
    return tf_listener;
}

//For tf publish
struct TfPubData
{
    TfPubData()
    {
        size.setX(0); size.setY(0); size.setZ(0); marker_id = 0;
    }

    tf::Vector3 size;
    tf::Transform tf;
    std::string parent_frame;
    std::string child_frame;
    int marker_id;
};

/*!
   * \brief  This spatial reasoner is used for Pose and Size information in 3D space.
   *
   * The pose is stored as Position (double[3] = tf::Vector3) and orientation (double[4] = tf::Quaternion).
   * The size is stored as a BoundingBox (double[3] = tf::Vector3)
   *
   * The class similarity is calculated based only on the BoundingBox.
   * The instance similarity is calculated based on all the data available.
   *
   */
class AauSpatialReasoner : public skiros_wm::DiscreteReasoner
{
public:
    AauSpatialReasoner();
    ~AauSpatialReasoner();

    void execute(skiros_wm::owl::WorldModel * wm) override;

    skiros_wm::RelationsMap computeRelations(skiros_wm::Element &subject, skiros_wm::Element &object) override;

    virtual float computeSimilarityPosition(skiros_wm::Element &lhs, skiros_wm::Element &rhs);
    virtual float computeSimilaritySize( skiros_wm::Element &lhs,  skiros_wm::Element &rhs);
    virtual float computeSimilarityOrientation(skiros_wm::Element &lhs, skiros_wm::Element &rhs);

    //Virtual functions for the world model
    virtual float computeIstanceSimilarity(skiros_wm::Element &lhs, skiros_wm::Element &rhs) override;
    virtual float computeClassSimilarity(skiros_wm::Element &lhs, skiros_wm::Element &rhs) override;
    /*!
         * \brief See description in the base class skiros_wm::DiscreteReasoner
         * \param any, valid inputs are tf::Vector3, tf::Quaternion, tf::Pose, geometry_msgs::Pose
         * \param set_code valid codes are 'Orientation', 'Size', 'Position', 'Pose', 'PoseMsg'
         * \return
         */
    virtual bool storeData(skiros_wm::Element  & e, boost::any any, std::string  set_code="") override;
    /*!
         * \brief See description in the base class skiros_wm::DiscreteReasoner
         * \param get_code valid codes are 'Orientation', 'Size', 'Position', 'Pose', 'PoseMsg'
         * \return available return are tf::Vector3, tf::Quaternion, tf::Pose, geometry_msgs::Pose
         */
    virtual boost::any getData(skiros_wm::Element & e, std::string  get_code="") override;
    virtual void onAddProperties(skiros_wm::Element& e) override;
    skiros_wm::ReasonerDataMap extractOwlData(skiros_wm::Element & e) override;
private:
    void publishObjectMarker(ros::Publisher & marker_pub, TfPubData & obj);
    std::list<TfPubData> getTfListRecursive(skiros_wm::Element root, std::string frame_id, skiros_wm::owl::WorldModel * model);
    bool waitAndLookupFirstTransform(std::string target_frame,
                                     std::string  source_frame,
                                     tf::StampedTransform & transform,
                                     ros::Time start=ros::Time::now(),
                                     ros::Duration d = ros::Duration(4.0),
                                     ros::Duration interval = ros::Duration(1.0));
    float computeCorrelationCoefficient(std::vector<float> &v1, std::vector<float> &v2);
};
}
