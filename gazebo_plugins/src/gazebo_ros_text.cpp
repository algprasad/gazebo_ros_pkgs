/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/**
 *  \author Carlos M. Costa
 *  \desc   Text plugin to display a message on a plane.
 */

#include <gazebo_plugins/gazebo_ros_text.h>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
GazeboRosText::GazeboRosText() {}

////////////////////////////////////////////////////////////////////////////////
GazeboRosText::~GazeboRosText() {}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosText::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  std::string robot_namespace = "text";
  if (!_sdf->HasElement("robotNamespace"))
  {
    ROS_WARN("GazeboRosText plugin missing <robotNamespace>, defaults to \"%s\".",
             robot_namespace.c_str());
  }
  else
  {
    robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  std::string material = "Gazebo/White";
  if (!_sdf->HasElement("material"))
  {
    ROS_WARN("GazeboRosText plugin missing <material>, defaults to \"%s\".",
             material.c_str());
  }
  else
  {
    material = _sdf->GetElement("material")->Get<std::string>();
  }

  float text_height = 0.2;
  if (!_sdf->HasElement("textHeight"))
  {
    ROS_WARN("GazeboRosText plugin missing <textHeight>, defaults to \"%f\".",
             text_height);
  }
  else
  {
    text_height = _sdf->GetElement("textHeight")->Get<float>();
  }

  std::string text_font_name = "Arial";
  if (!_sdf->HasElement("textFontName"))
  {
    ROS_WARN("GazeboRosText plugin missing <textFontName>, defaults to \"%s\".",
             text_font_name.c_str());
  }
  else
  {
    text_font_name = _sdf->GetElement("textFontName")->Get<std::string>();
  }

  bool text_show_on_top = false;
  if (!_sdf->HasElement("textShowOnTop"))
  {
    ROS_WARN("GazeboRosText plugin missing <textShowOnTop>, defaults to \"%s\".",
             text_show_on_top ? "true" : "false");
  }
  else
  {
    text_show_on_top = _sdf->GetElement("textShowOnTop")->Get<bool>();
  }

  std::string horizontal_alignment = "center";
  if (!_sdf->HasElement("horizontalAlignment"))
  {
    ROS_WARN("GazeboRosText plugin missing <horizontalAlignment>, defaults to \"%s\".",
             horizontal_alignment.c_str());
  }
  else
  {
    horizontal_alignment = _sdf->GetElement("horizontalAlignment")->Get<std::string>();
  }

  std::string vertical_alignment = "above";
  if (!_sdf->HasElement("verticalAlignment"))
  {
    ROS_WARN("GazeboRosText plugin missing <verticalAlignment>, defaults to \"%s\".",
             vertical_alignment.c_str());
  }
  else
  {
    vertical_alignment = _sdf->GetElement("verticalAlignment")->Get<std::string>();
  }

  std::stringstream ss;
  ss << "Multiline" << '\n' << "Text" << '\n' << "Message";
  text_ = ss.str();
  if (_sdf->HasElement("defaultText"))
  {
    text_ = _sdf->GetElement("defaultText")->Get<std::string>();
  }

  common::Color matAmbient, matDiffuse, matSpecular, matEmissive;
  rendering::Material::GetMaterialAsColor(material,
      matAmbient, matDiffuse, matSpecular, matEmissive);
  movableText_.Load(robot_namespace + "__TEXT__",
      text_, text_font_name, text_height, matAmbient);
  movableText_.SetShowOnTop(text_show_on_top);
  movableText_.SetTextAlignment(horizontal_alignment == "center" ?
      rendering::MovableText::HorizAlign::H_CENTER :
      rendering::MovableText::HorizAlign::H_LEFT,
      vertical_alignment == "above" ?
      rendering::MovableText::VertAlign::V_ABOVE :
      rendering::MovableText::VertAlign::V_BELOW);
  movableText_.MovableObject::getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(robot_namespace)));

  _parent->GetSceneNode()->attachObject(&(movableText_));

  std::string topic_name = "set_text";
  if (_sdf->HasElement("topicName"))
  {
    topic_name = _sdf->GetElement("topicName")->Get<std::string>();
  }

  // Initialize the ROS node for the gazebo client if necessary
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  rosnode_.reset(new ros::NodeHandle(robot_namespace));

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::String>(topic_name, 1,
        boost::bind(&GazeboRosText::processStringMsg, this, _1),
        ros::VoidPtr(), &queue_);

  string_subscriber_ = rosnode_->subscribe(so);

  this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosText::QueueThread, this));

  this->update_connection_ =
    event::Events::ConnectPreRender(
        boost::bind(&GazeboRosText::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosText::processString(const std::string &str)
{
  boost::mutex::scoped_lock scoped_lock(text_lock_);
  text_ = str;
  new_text_available_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosText::processStringMsg(const std_msgs::StringConstPtr &msg)
{
  processString(msg->data);
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosText::UpdateChild()
{
  boost::mutex::scoped_lock scoped_lock(text_lock_);
  if (new_text_available_)
  {
    movableText_.SetText(text_);
    new_text_available_ = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosText::QueueThread()
{
  static const double timeout = 0.01;
  while (rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(GazeboRosText);
}
