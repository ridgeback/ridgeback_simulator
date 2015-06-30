

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo
{
  class MecanumPlugin : public ModelPlugin
  {
  public:
    MecanumPlugin();
    virtual ~MecanumPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void GazeboUpdate();

  private:
    event::ConnectionPtr update_connection_;
    physics::ModelPtr model_;
    physics::LinkPtr wheel_link_;
    physics::LinkPtr fixed_link_;
    //std::string robot_namespace_;
    //std::string frame_id_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MecanumPlugin);

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  MecanumPlugin::MecanumPlugin()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  MecanumPlugin::~MecanumPlugin()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void MecanumPlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
  {
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    ROS_INFO("Greetings from MecanumPlugin!");

    // Get sdf parameters
    /*if(_sdf->HasElement("robotNamespace")) {
      this->robot_namespace_ = _sdf->Get<std::string>("robotNamespace") + "/";
    }

    if(_sdf->HasElement("frameId")) {
      this->frame_id_ = _sdf->Get<std::string>("frameId");
    }

    if(_sdf->HasElement("kl")) {
      this->kl_ = _sdf->Get<double>("kl");
    }
    if(_sdf->HasElement("ka")) {
      this->ka_ = _sdf->Get<double>("ka");
    }*/

    // Store the model
    model_ = _parent;

    std::string link_name;

    if(_sdf->HasElement("wheelLinkName")) {
      link_name = _sdf->Get<std::string>("wheelLinkName");
    } else {
      ROS_FATAL("The mecanum plugin requires a `wheelLinkName` parameter.");
      return;
    }
    if (!(wheel_link_ = model_->GetLink(link_name)))
    {
      ROS_FATAL_STREAM("Wheel link [" << link_name << "] not found!");
      return;
    }

    if(_sdf->HasElement("fixedLinkName")) {
      link_name = _sdf->Get<std::string>("fixedLinkName");
    } else {
      ROS_FATAL("The mecanum plugin requires a `fixedLinkName` parameter.");
      return;
    }
    if (!(fixed_link_ = model_->GetLink(link_name)))
    {
      ROS_FATAL_STREAM("Fixed link [" << link_name << "] not found!");
      return;
    }

    // Register update event handler
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&MecanumPlugin::GazeboUpdate, this));
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void MecanumPlugin::GazeboUpdate()
  {
    unsigned int collision_index = 0;
    physics::SurfaceParamsPtr surface = wheel_link_->GetCollision(collision_index)->GetSurface();

    // This cast will fail is not using ODE. How to check for this?
    physics::ODESurfaceParams* ode_surface = dynamic_cast<physics::ODESurfaceParams*>(surface.get());

    physics::FrictionPyramid& fric(ode_surface->frictionPyramid);

    fric.SetMuPrimary(0.1);
    fric.SetMuSecondary(0.9);
    fric.direction1.x = 0.7;
    fric.direction1.y = 0.7;
    fric.direction1.z = 0;

    /*
    // Get TF transform relative to the /world link
    geometry_msgs::TransformStamped hog_desired_tform;
    static bool errored = false;
    try{
      hog_desired_tform = tf_buffer_->lookupTransform("world", frame_id_+"_desired", ros::Time(0));
      errored = false;
    } catch (tf2::TransformException ex){
      if(!errored) {
        ROS_ERROR("%s",ex.what());
        errored = true;
      }
      return;
    }

    // Convert TF transform to Gazebo Pose
    const geometry_msgs::Vector3 &p = hog_desired_tform.transform.translation;
    const geometry_msgs::Quaternion &q = hog_desired_tform.transform.rotation;
    gazebo::math::Pose hog_desired(
        gazebo::math::Vector3(p.x, p.y, p.z),
        gazebo::math::Quaternion(q.w, q.x, q.y, q.z));

    // Relative transform from actual to desired pose
    gazebo::math::Pose world_pose = floating_link_->GetDirtyPose();
    gazebo::math::Vector3 err_pos = hog_desired.pos - world_pose.pos;
    // Get exponential coordinates for rotation
    gazebo::math::Quaternion err_rot =  (world_pose.rot.GetAsMatrix4().Inverse() * hog_desired.rot.GetAsMatrix4()).GetRotation();
    gazebo::math::Quaternion not_a_quaternion = err_rot.GetLog();

    floating_link_->AddForce(
        kl_ * err_pos - cl_ * floating_link_->GetWorldLinearVel());

    floating_link_->AddRelativeTorque(
        ka_ * gazebo::math::Vector3(not_a_quaternion.x, not_a_quaternion.y, not_a_quaternion.z) - ca_ * floating_link_->GetRelativeAngularVel());

    // Convert actual pose to TransformStamped message
    geometry_msgs::TransformStamped hog_actual_tform;

    hog_actual_tform.header.frame_id = "world";
    hog_actual_tform.header.stamp = ros::Time::now();

    hog_actual_tform.child_frame_id = frame_id_ + "_actual";

    hog_actual_tform.transform.translation.x = world_pose.pos.x;
    hog_actual_tform.transform.translation.y = world_pose.pos.y;
    hog_actual_tform.transform.translation.z = world_pose.pos.z;

    hog_actual_tform.transform.rotation.w = world_pose.rot.w;
    hog_actual_tform.transform.rotation.x = world_pose.rot.x;
    hog_actual_tform.transform.rotation.y = world_pose.rot.y;
    hog_actual_tform.transform.rotation.z = world_pose.rot.z;

    tf_broadcaster_->sendTransform(hog_actual_tform);*/
  }

}
