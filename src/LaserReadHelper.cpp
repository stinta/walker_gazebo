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
    LaserReadHelper::LaserReadHelper() {
      wallInFront = false;
    }
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
    void LaserReadHelper::processLaserScan
                    (const sensor_msgs::LaserScan::ConstPtr& scan) {
        ROS_INFO_STREAM("Extracting range distance ahead of range sensor "
                        << scan->ranges[320]);
        if (scan->ranges[320] > 1.0) {
          wallInFront =  false;
        } else {
           wallInFront = true;
           ROS_INFO_STREAM("Wall in front detected");
        }
     }
    /*
     * @brief Implements the callback funiton for LaserScan Subscriber
     *
     */
    bool LaserReadHelper::getWallInFront() {
      return wallInFront;
    }
