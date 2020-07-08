/**
Software License Agreement (BSD)

\file      mecanum_plugin.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>, Vladimir Ivan <v.ivan@ed.ac.uk>
\copyright Copyright (c) 2020, Clearpath Robotics, Inc., The University of Edinburgh, All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/console.h>

#include <string>

namespace gazebo
{

class MecanumPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  virtual void GazeboUpdate();

private:
  event::ConnectionPtr update_connection_;
  physics::ModelPtr model_;
  physics::LinkPtr wheel_link_;
  physics::JointPtr passive_joint_;
  ignition::math::v4::Vector3d axis_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MecanumPlugin);

void MecanumPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the model
  model_ = _parent;

  auto physics = model_->GetWorld()->Physics()->GetType();
  if (physics != "ode" && physics != "dart")
  {
    ROS_FATAL("Only ODE and Dart physics engines are supported with mecanum wheels!");
    return;
  }


  std::string link_name;

  if (_sdf->HasElement("wheelLinkName"))
  {
    link_name = _sdf->Get<std::string>("wheelLinkName");
    wheel_link_ = model_->GetLink(link_name);
    if (!wheel_link_)
    {
      ROS_FATAL_STREAM("Wheel link [" << link_name << "] not found!");
      return;
    }
  }
  else
  {
    ROS_FATAL("The mecanum plugin requires a `wheelLinkName` parameter.");
    return;
  }

  if (_sdf->HasElement("axis"))
  {
    std::istringstream iss(_sdf->Get<std::string>("axis"));
    std::vector<std::string> numbers(std::istream_iterator<std::string>{iss}, 
                                    std::istream_iterator<std::string>());
    if (numbers.size() == 3)
    {
        axis_.X() = std::stod(numbers[0]);
        axis_.Y() = std::stod(numbers[1]);
        axis_.Z() = std::stod(numbers[2]);
        axis_ = axis_.Normalize();
        if (axis_.Length() < 1.0 - std::numeric_limits<double>::epsilon())
        {
          ROS_FATAL_STREAM("The `axis` parameter is not a valid vector.");
          return;
        }
    }
    else
    {
      ROS_FATAL("The mecanum plugin requires a valid `axis` parameter.");
      return;
    }
    
  }
  else
  {
    ROS_FATAL("The mecanum plugin requires an `axis` parameter.");
    return;
  }

  passive_joint_ = wheel_link_->GetChildJoints()[0];

  ROS_INFO_STREAM("Mecanum plugin initialized for " << wheel_link_->GetName() <<
                  ", axis: [" << axis_.X() << ", " <<  axis_.Y() << ", " <<  
                  axis_.Z() <<"]");

  // Register update event handler
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MecanumPlugin::GazeboUpdate, this));
}

void MecanumPlugin::GazeboUpdate()
{
  // Re-align the passive hoint axis
  passive_joint_->SetAxis(0, axis_);
}

}  // namespace gazebo
