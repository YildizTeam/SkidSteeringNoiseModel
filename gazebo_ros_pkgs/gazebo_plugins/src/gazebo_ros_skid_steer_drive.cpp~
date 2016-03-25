/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_skid_steer_drive.cpp
 *
 * \brief A skid steering drive plugin. Inspired by gazebo_ros_diff_drive and SkidSteerDrivePlugin
 *
 * \author  Zdenek Materna (imaterna@fit.vutbr.cz)
 *
 * $ Id: 06/25/2013 11:23:40 AM materna $
    
    To implement non-perfect odometry
    Modified by Muhammet Balcilar, Yildiz Technical University, Istanbul, Turkey
    Any questions please contact muhammetbalcilar@gmail.com 

 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_skid_steer_drive.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo {

  enum {
    RIGHT_FRONT=0,
    LEFT_FRONT=1,
    RIGHT_REAR=2,
    LEFT_REAR=3,
  };

  GazeboRosSkidSteerDrive::GazeboRosSkidSteerDrive()
  {
      first_odom=true;
  }

  // Destructor
  GazeboRosSkidSteerDrive::~GazeboRosSkidSteerDrive() {
    delete rosnode_;
    delete transform_broadcaster_;
  }

  double GazeboRosSkidSteerDrive::pf_ran_gaussian(double sigma)
  {
    double x1, x2, w;
    double r;
    if (sigma==0)
        return 0;

    do
    {
      do { r = drand48(); } while (r == 0.0);
      x1 = 2.0 * r - 1.0;
      do { r = drand48(); } while (r == 0.0);
      x2 = 2.0 * drand48() - 1.0;
      w = x1*x1 + x2*x2;
    } while(w > 1.0 || w==0.0);

    return(sigma * x2 * sqrt(-2.0*log(w)/w));
  }


  // Load the controller
  void GazeboRosSkidSteerDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("GazeboRosSkidSteerDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }
    
    this->broadcast_tf_ = false;
    if (!_sdf->HasElement("broadcastTF")) {
      if (!this->broadcast_tf_)
    	  ROS_INFO("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
      else ROS_INFO("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());
          
    } else {
      this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
    }

    // TODO write error if joint doesn't exist!
    this->left_front_joint_name_ = "left_front_joint";
    if (!_sdf->HasElement("leftFrontJoint")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <leftFrontJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
    } else {
      this->left_front_joint_name_ = _sdf->GetElement("leftFrontJoint")->Get<std::string>();
    }

    this->right_front_joint_name_ = "right_front_joint";
        if (!_sdf->HasElement("rightFrontJoint")) {
          ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <rightFrontJoint>, defaults to \"%s\"",
              this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
        } else {
          this->right_front_joint_name_ = _sdf->GetElement("rightFrontJoint")->Get<std::string>();
        }

	this->left_rear_joint_name_ = "left_rear_joint";
	if (!_sdf->HasElement("leftRearJoint")) {
	  ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <leftRearJoint>, defaults to \"%s\"",
		  this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
	} else {
	  this->left_rear_joint_name_ = _sdf->GetElement("leftRearJoint")->Get<std::string>();
	}

    this->right_rear_joint_name_ = "right_rear_joint";
    if (!_sdf->HasElement("rightRearJoint")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <rightRearJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
    } else {
      this->right_rear_joint_name_ = _sdf->GetElement("rightRearJoint")->Get<std::string>();
    }


    // This assumes that front and rear wheel spacing is identical
    /*this->wheel_separation_ = this->parent->GetJoint(left_front_joint_name_)->GetAnchor(0).Distance(
    		this->parent->GetJoint(right_front_joint_name_)->GetAnchor(0));*/

    this->wheel_separation_ = 0.4;

    if (!_sdf->HasElement("wheelSeparation")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <wheelSeparation>, defaults to value from robot_description: %f",
          this->robot_namespace_.c_str(), this->wheel_separation_);
    } else {
      this->wheel_separation_ =
        _sdf->GetElement("wheelSeparation")->Get<double>();
    }

    // TODO get this from robot_description
    this->wheel_diameter_ = 0.15;
    if (!_sdf->HasElement("wheelDiameter")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
          this->robot_namespace_.c_str(), this->wheel_diameter_);
    } else {
      this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
    }

    this->torque = 5.0;
    if (!_sdf->HasElement("torque")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <torque>, defaults to %f",
          this->robot_namespace_.c_str(), this->torque);
    } else {
      this->torque = _sdf->GetElement("torque")->Get<double>();
    }

    this->command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
      this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->odometry_topic_ = "odom";
    if (!_sdf->HasElement("odometryTopic")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    } else {
      this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->odometry_frame_ = "odom";
    if (!_sdf->HasElement("odometryFrame")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    } else {
      this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }

    this->a1 = 0.0002;
    if (_sdf->HasElement("a1"))
    {
      this->a1 = _sdf->GetElement("a1")->Get<double>();
    }

    this->a2 = 0.0001;
    if (_sdf->HasElement("a2"))
    {
      this->a2 = _sdf->GetElement("a2")->Get<double>();
    }

    this->a3 = 0.002;
    if (_sdf->HasElement("a3"))
    {
      this->a3 = _sdf->GetElement("a3")->Get<double>();
    }

    this->a4 = 0.001;
    if (_sdf->HasElement("a4"))
    {
      this->a4 = _sdf->GetElement("a4")->Get<double>();
    }







    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
    last_update_time_ = this->world->GetSimTime();

    // Initialize velocity stuff
    wheel_speed_[RIGHT_FRONT] = 0;
    wheel_speed_[LEFT_FRONT] = 0;
    wheel_speed_[RIGHT_REAR] = 0;
	wheel_speed_[LEFT_REAR] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    joints[LEFT_FRONT] = this->parent->GetJoint(left_front_joint_name_);
    joints[RIGHT_FRONT] = this->parent->GetJoint(right_front_joint_name_);
    joints[LEFT_REAR] = this->parent->GetJoint(left_rear_joint_name_);
    joints[RIGHT_REAR] = this->parent->GetJoint(right_rear_joint_name_);

    if (!joints[LEFT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[RIGHT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[LEFT_REAR]) {
	 char error[200];
	 snprintf(error, 200,
		 "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
		 this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
	 gzthrow(error);
   }

   if (!joints[RIGHT_REAR]) {
	 char error[200];
	 snprintf(error, 200,
		 "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
		 this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
	 gzthrow(error);
   }

#if GAZEBO_MAJOR_VERSION > 2
    joints[LEFT_FRONT]->SetParam("fmax", 0, torque);
    joints[RIGHT_FRONT]->SetParam("fmax", 0, torque);
    joints[LEFT_REAR]->SetParam("fmax", 0, torque);
    joints[RIGHT_REAR]->SetParam("fmax", 0, torque);
#else
    joints[LEFT_FRONT]->SetMaxForce(0, torque);
    joints[RIGHT_FRONT]->SetMaxForce(0, torque);
    joints[LEFT_REAR]->SetMaxForce(0, torque);
    joints[RIGHT_REAR]->SetMaxForce(0, torque);
#endif

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ROS_INFO("Starting GazeboRosSkidSteerDrive Plugin (ns = %s)!", this->robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosSkidSteerDrive::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = rosnode_->subscribe(so);

    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosSkidSteerDrive::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosSkidSteerDrive::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosSkidSteerDrive::UpdateChild() {
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update =
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

      publishOdometry(seconds_since_last_update);

      // Update robot in case new velocities have been requested
      getWheelVelocities();
#if GAZEBO_MAJOR_VERSION > 2
      joints[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
      joints[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
      joints[LEFT_REAR]->SetParam("vel", 0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
      joints[RIGHT_REAR]->SetParam("vel", 0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
#else
      joints[LEFT_FRONT]->SetVelocity(0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
      joints[RIGHT_FRONT]->SetVelocity(0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
      joints[LEFT_REAR]->SetVelocity(0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
      joints[RIGHT_REAR]->SetVelocity(0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
#endif

      last_update_time_+= common::Time(update_period_);

    }
  }

  // Finalize the controller
  void GazeboRosSkidSteerDrive::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosSkidSteerDrive::getWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);

    double vr = x_;
    double va = rot_;

    wheel_speed_[RIGHT_FRONT] = vr + va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_REAR] = vr + va * wheel_separation_ / 2.0;

    wheel_speed_[LEFT_FRONT] = vr - va * wheel_separation_ / 2.0;
    wheel_speed_[LEFT_REAR] = vr - va * wheel_separation_ / 2.0;

  }

  void GazeboRosSkidSteerDrive::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) {

    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
  }

  void GazeboRosSkidSteerDrive::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosSkidSteerDrive::publishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame =
      tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);

    // create some non-perfect odometry!    
    // written by Muhammet Balcilar, Yildiz Technical University, Istanbul, Turkey
    // Any questions please contact muhammetbalcilar@gmail.com 

    math::Pose pose = this->parent->GetWorldPose();
    if (first_odom)
    {
        first_odom=false;
        prevGroundTPose=pose;
        noisyPose=pose;
    }

   

    double xt, yt, thetat, xt1, yt1, thetat1, xht, yht, thetaht, xht1, yht1, thetaht1;


        // a1 : is the standard deviation of rotation noise for unit rotation
        // a2 : is the standard deviation of rotation noise for unit translation
        // a3 : is the standard deviation of translation noise for unit translation  
        // a4 : is the standard deviation of translation noise for unit rotation 
        // xt : ground truth x at time t
        // yt : ground truth y at time t
        // thetat : ground truth theta at time t
        // xt1 : ground truth x at time t+1
        // yt1 : ground truth y at time t+1
        // thetat1 : ground truth theta at time t+1
        // xht : noisy x at time t
        // yht : noisy y at time t
        // thetaht : noisy theta at time t
        // xht1 : noisy y at time t+1
        // yht1 : t+1 anındaki odom y
        // thetaht1 : noisy theta at time t+1
    xt1=pose.pos.x;
    yt1=pose.pos.y;
    tf::Quaternion q1_for_yaw(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    thetat1 = tf::getYaw(q1_for_yaw);



    xt =prevGroundTPose.pos.x;
    yt =prevGroundTPose.pos.y;
    tf::Quaternion q1_for_yaw1(prevGroundTPose.rot.x, prevGroundTPose.rot.y, prevGroundTPose.rot.z, prevGroundTPose.rot.w);
    thetat = tf::getYaw(q1_for_yaw1);

    xht=noisyPose.pos.x;
    yht=noisyPose.pos.y;
    tf::Quaternion q1_for_yaw2(noisyPose.rot.x, noisyPose.rot.y, noisyPose.rot.z, noisyPose.rot.w);
    thetaht = tf::getYaw(q1_for_yaw2);




    double rt=sqrt( (xt1-xt)*(xt1-xt)+(yt1-yt)*(yt1-yt));
    double r1=atan2(yt1-yt,xt1-xt)-thetat;
    double r2=thetat1-thetat-r1;
    double rh1=r1+pf_ran_gaussian(this->a1*fabs(r1)+this->a2*rt);
    double rht=rt+pf_ran_gaussian(this->a3*rt+this->a4*(fabs(r1)+fabs(r2)));
    double rh2=r2+pf_ran_gaussian(this->a1*fabs(r2)+this->a2*rt);
    xht1=xht+rht*cos(thetaht+rh1);
    yht1=yht+rht*sin(thetaht+rh1);
    thetaht1=thetaht+rh1+rh2;

    noisyPose.pos.x=xht1;
    noisyPose.pos.y=yht1;
    tf::Quaternion q1_for_yaw3(noisyPose.rot.x, noisyPose.rot.y, noisyPose.rot.z, noisyPose.rot.w);
    q1_for_yaw3.setRPY(0,0,thetaht1);
    noisyPose.rot.x = q1_for_yaw3.getX();
    noisyPose.rot.y = q1_for_yaw3.getY();
    noisyPose.rot.z = q1_for_yaw3.getZ();
    noisyPose.rot.w = q1_for_yaw3.getW();

    



    tf::Quaternion qt(noisyPose.rot.x, noisyPose.rot.y, noisyPose.rot.z, noisyPose.rot.w);
    tf::Vector3 vt(noisyPose.pos.x, noisyPose.pos.y, noisyPose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    if (this->broadcast_tf_) {

    	transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time,
            odom_frame, base_footprint_frame));

    }

    // publish odom topic
    odom_.pose.pose.position.x = noisyPose.pos.x;
    odom_.pose.pose.position.y = noisyPose.pos.y;

    odom_.pose.pose.orientation.x = noisyPose.rot.x;
    odom_.pose.pose.orientation.y = noisyPose.rot.y;
    odom_.pose.pose.orientation.z = noisyPose.rot.z;
    odom_.pose.pose.orientation.w = noisyPose.rot.w;
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.01;

    // get velocity in /odom frame
    math::Vector3 linear;
    linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = noisyPose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish(odom_);
    prevGroundTPose = pose;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosSkidSteerDrive)
}
