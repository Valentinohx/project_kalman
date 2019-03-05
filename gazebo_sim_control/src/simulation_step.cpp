#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <std_msgs/Int32.h>
#include <memory>


namespace gazebo
{

class SimulationStep : public WorldPlugin
{
public:
  SimulationStep() : WorldPlugin() { }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // this plugin needs ROS, so check if it is running in a valid node
    if(ros::isInitialized())
    {
      world_ = _world;
      nh_.reset(new ros::NodeHandle("gazebo_sim_control")); // custom namespace not to mess with "classical" gazebo_ros utilities
      sub_ = nh_->subscribe("sim_step", 10, &SimulationStep::callback, this);
      ROS_INFO_NAMED(
        "simulation_step",
        "SimulationStep: plugin started in ROS namespace '%s'",
        nh_->getNamespace().c_str()
      );
    }
    else
      ROS_ERROR_NAMED(
        "simulation_step",
        "SimulationStep: ros::init not called, cannot create connection to master."
      );
  }

private:
  void callback(const std_msgs::Int32 msg)
  {
    // we perform steps manually only if we are in a paused state
    if(world_->IsPaused())
    {
      if(msg.data > 0) // check that we actually step forward (0 is considered invalid)
      {
        ROS_DEBUG_NAMED(
          "simulation_step",
          "SimulationStep: stepping forward %d time(s)",
          msg.data
        );
        world_->Step(msg.data);
      }
      else
      {
        if(msg.data == 0)
          ROS_WARN_NAMED(
            "simulation_step",
            "SimulationStep: number of requested steps is zero"
          );
        else
          ROS_ERROR_NAMED(
            "simulation_step",
            "SimulationStep: invalid negative step %d",
            msg.data
          );
      }
    }
    else
      ROS_ERROR_NAMED(
        "simulation_step",
        "SimulationStep: simulation is not paused"
      );
  }

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber sub_;
  physics::WorldPtr world_;

}; // end of class

GZ_REGISTER_WORLD_PLUGIN(SimulationStep)

} // end of namespace
