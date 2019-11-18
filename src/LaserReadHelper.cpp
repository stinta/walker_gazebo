/* Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file laserReadHelper.cpp
 * @author Sandra Tinta
 * @copyright 2019 BSD License
 *
 * @brief This node subcribes to laserScan topic from the turtleBot simulation
 *       in order to track distance to wall and/or obstacle.  
 *       If a wall/obtacle is not detected, the robot keeps moving forward
 *       If a wall/obsatavle is detected, the robot rotates until it can keep
 *       moving forward. 
 **/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "walker_gazebo/laserReadHelper.h"
/**
 * This class implementes methods to read LaserScan Messages and determine 
 * the precense of an obtacle
 */

   /*
    * @brief constructor
    */
    LaserReadHelper::LaserReadHelper(){};
    /*
     * @brief virtual destructor
     */
   LaserReadHelper::~LaserReadHelper() {
     ROS_INFO_STREAM("LaserReadHelper shutting down");
   }
    
    /*
     * @brief Implements the callback funiton for LaserScan Subscriber
     *
     * @param message published by LaserScan
     */
    void LaserReadHelper::processLaserScan(const sensor_msgs::LaserScan::ConstPtr&) {};



/*void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("I heard: [" <<  msg->data.c_str() <<"]");
}*/

//int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  //ros::init(argc, argv, "walker_gazebo");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  //ros::NodeHandle n = ros::NodeHandle();

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  //auto sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  //ros::spin();
  //ROS_WARN_STREAM("After ros::spin; program is ending");

  //return 0;
//}
