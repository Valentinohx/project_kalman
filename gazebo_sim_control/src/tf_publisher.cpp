#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace gazebo
{


/// Utility function to copy a pose from Gazebo to a ROS message.
inline void pose2transform(const ignition::math::Pose3d& pose, geometry_msgs::Transform& transform)
{
  transform.translation.x = pose.Pos().X();
  transform.translation.y = pose.Pos().Y();
  transform.translation.z = pose.Pos().Z();
  transform.rotation.w = pose.Rot().W();
  transform.rotation.x = pose.Rot().X();
  transform.rotation.y = pose.Rot().Y();
  transform.rotation.z = pose.Rot().Z();
}



/// Plugin that publishes frame transformations over /tf.
/** Attach this plugin to a Gazebo model in order to publish in the /tf topic
  * information relative to the links of the model.
  *
  * The following parameters customize the behavior of the plugin:
  *  - publishRate: frequency (in Hz) for publishing.
  *  - worldName: by default, this plugin uses the name of the world as the base
  *    frame of most of the transformations. If this is not ok, you can
  *    'override' this by explicitly giving a base frame name. This simply means
  *    that whenever world's name should be used, it is replaced by the name
  *    given by the user.
  *  - tfPrefix: by default, link names are published 'as they are'. For some
  *    applications this might not be good, and therefore a prefix can be added
  *    to links names. As an example, if a link is named 'the_link' and the
  *    prefix is 'the_prefix', the resulting frame id will be
  *    'the_prefix/the_link'. Note that the prefix is not added to the world
  *    link.
  * All parameters can be passed as elements of the main plugin tag. Example:
  *
  *        <?xml version="1.0" ?>
  *        <sdf version="1.4">
  *          <world name="my_wonderful_world">
  *            <include>
  *              <uri>model://ground_plane</uri>
  *            </include>
  *            <include>
  *              <uri>model://sun</uri>
  *            </include>
  *
  *            <model name="rover">
  *              <include>
  *                <uri>model://pioneer2dx</uri>
  *                <pose>0.2 0 0.2 0 0 0</pose>
  *              </include>
  *
  *              <plugin name="gzsim_control_tfpub" filename="libgzsim_control_tfpub.so">
  *                <publishRate>50</publishRate>
  *                <worldName>world</worldName>
  *                <tfPrefix>foo</tfPrefix>
  *              </plugin>
  *            </model>
  *          </world>
  *        </sdf>
  *
  */
class TFPublisher : public ModelPlugin
{
public:
  /// Constructor.
  TFPublisher() : ModelPlugin() { }

  /// Initializes the plugin.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    // this plugin needs ROS, so check if it is running in a valid node
    if(!ros::isInitialized()) {
      ROS_ERROR_NAMED("tf_publisher", "TFPublisher: ros::init not called, "
        "cannot create connection to master.");
      return;
    }

    model_ = model;

    // get the rate at which transformations should be published
    double pub_rate = 10;
    if(sdf->HasElement("publishRate")) {
      double rate = sdf->GetElement("publishRate")->Get<double>();
      if(rate <= 0)
        ROS_WARN_NAMED("tf_publisher", "TFPublisher: 'publishRate' must be positive (got %f)", rate);
      else
        pub_rate = rate;
    }
    else
      ROS_INFO_NAMED("tf_publisher", "TFPublisher: 'publishRate' not found, using default %f", pub_rate);
    last_ = ros::Time(0);
    pub_period_ = ros::Duration(1/pub_rate);
    ROS_INFO_NAMED("tf_publisher", "TFPublisher: publish period is set to %f", pub_period_.toSec());

    // override world's name if required (useful if the world is called
    // 'JohnDoe' but you want to use 'world' as base frame name)
    world_name_tf_ = world_name_ = model_->GetWorld()->Name();
    if(sdf->HasElement("worldName")) {
      std::string name = sdf->GetElement("worldName")->Get<std::string>();
      if(name.empty())
        ROS_WARN_NAMED("tf_publisher", "TFPublisher: 'worldName' is empty");
      else
        world_name_tf_ = name;
    }
    ROS_INFO_NAMED("tf_publisher", "TFPublisher: world's name is '%s'; '%s' "
      "will be used as the name to publish transformations",
      world_name_.c_str(), world_name_tf_.c_str());

    // get a tf prefix if requested
    tf_prefix_ = "";
    if(sdf->HasElement("tfPrefix")) {
      std::string prefix = sdf->GetElement("tfPrefix")->Get<std::string>();
      if(prefix.empty())
        ROS_WARN_NAMED("tf_publisher", "TFPublisher: 'tfPrefix' is empty");
      else
        tf_prefix_ = prefix + "/";
    }
    if(!tf_prefix_.empty())
      ROS_INFO_NAMED("tf_publisher", "TFPublisher: using prefix '%s' in frame ids", tf_prefix_.c_str());

    // make sure that every time the simulation steps forward we publish transformations
    update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&TFPublisher::update, this));
    tf_.reset( new tf2_ros::TransformBroadcaster() );
    ROS_INFO_NAMED("tf_publisher", "TFPublisher: plugin started in ROS "
      "namespace '%s'", ros::this_node::getNamespace().c_str());
  }


private:
  /// Callback to be executed whenver the simulation advances.
  /** If the plugin failed to load, this function will not be called. */
  void update()
  {
    // check if it is time to send information
    ros::Time now = ros::Time::now();
    if(now - last_ < pub_period_)
      return;

    last_ = now;
    std::vector<geometry_msgs::TransformStamped> tfs;
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();
    const auto& base = model_->RelativePose(); // current pose fo the model
    auto parent = model_->GetParent()->GetName();
    if(parent == world_name_)
      parent = world_name_tf_;
    std::string pframe;

    // fill a vector of transforms (one for each link)
    for(const auto& link : model_->GetLinks()) {
      pframe = link->GetParent()->GetName();
      if(pframe == model_->GetName())
        msg.header.frame_id = parent;
      else
        msg.header.frame_id = tf_prefix_ + pframe;
      msg.child_frame_id = tf_prefix_ + link->GetName();
      // Note: in ignition apparently the multiplication between two poses is
      // "reversed". If aTb and bTc are two poses, then according to ignition
      // we have aTc = bTc * aTb
      pose2transform(link->RelativePose()*base, msg.transform);
      tfs.push_back(msg);
    }
    // all done, send data
    tf_->sendTransform(tfs);
  }

private:
  /// The model this plugin was instanciated for.
  physics::ModelPtr model_;
  /// Connection to publish when simulation is advancing.
  event::ConnectionPtr update_connection_;
  /// Broadcaster to publish transformations over /tf.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  /// The name of the world in which the model lives.
  std::string world_name_;
  /// Name that will be used to represent the world.
  std::string world_name_tf_;
  /// Prefix to be added to each frame (except the world).
  std::string tf_prefix_;
  /// Last time we published.
  ros::Time last_;
  /// Publish period.
  ros::Duration pub_period_;

}; // end of class



GZ_REGISTER_MODEL_PLUGIN(TFPublisher)

} // end of namespace
