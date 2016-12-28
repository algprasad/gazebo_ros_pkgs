/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Text plugin to display a message on a plane.
 * Author: Carlos M. Costa
 * Date: 28 December 2016
 */

#ifndef GAZEBO_ROS_TEXT_HH
#define GAZEBO_ROS_TEXT_HH

#include <string>
#include <sstream>

#include <boost/thread/mutex.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/rate.h>
#include <std_msgs/String.h>

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/MovableText.hh>

namespace gazebo
{

   class GazeboRosText : public VisualPlugin
   {
      /// \brief Constructor
      public: GazeboRosText();

      /// \brief Destructor
      public: virtual ~GazeboRosText();

      /// \brief Load the controller
      public: void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

      /// \brief Update of message text
      void processString(const std::string &str);

      /// \brief Callback for changing the text message
      void processStringMsg(const std_msgs::StringConstPtr &msg);

      /// \brief Update the controller
      protected: virtual void UpdateChild();

      /// \brief The custom callback queue thread function.
      private: void QueueThread();

      /// \brief A mutex to lock access to the string message
      boost::mutex text_lock_;

      /// \brief Flag to indicate if there is new text to render
      bool new_text_available_;

      /// \brief Text to render
      std::string text_;

      /// \brief Text displaying the message.
      public: rendering::MovableText movableText_;

      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

      /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
      protected: boost::shared_ptr<ros::NodeHandle> rosnode_;

      protected: ros::Subscriber string_subscriber_;

      /// \brief Custom Callback Queue
      private: ros::CallbackQueue queue_;

      /// \brief Thead object for the running callback Thread.
      private: boost::thread callback_queue_thread_;
   };

}

#endif

