/*
 * Authors: kss (2022)
 * 
 * Based on the sicklms.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 * 
 * Released under Apache 2.0 license.
 */ 

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "nav350_msg/msg/navtimestamp.hpp"  

//declare standard c++14 headers
#include <iostream>
#include <sstream>
#include <deque>

//declare own ros2 headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <nav_msgs/msg/odometry.hpp>

//declare own package headers
#include <SickNAV350.hh>

//#include "nav350/nav_time_stamp.hpp"
//#include "tutorial_interfaces/msg/nav_time_stamp.hpp"   
//#include "builtin_interfaces/msg/time.hpp"

#define DEG2RAD(x) ((x)*M_PI/180.)

const double TRANSFORM_TIMEOUT = 20.0f;
const double POLLING_DURATION = 0.05f;
const std::string ODOM_TOPIC = "odom";


//brought from nav350 source
//for exit by executing ctr+c
bool need_exit = false;


// A complimentary filter to get a (much) better time estimate, does not
// calibrate out constant network latency delays, but does get rid of 
// timming jitter to get better timing estimates than the 
// communicated clock resolution (which is only 1ms)
class smoothtime { 
    protected:
        rclcpp::Time smoothtime_prev, smoothed_timestamp;
        double time_smoothing_factor;
        double error_threshold;
    public:
    //!
    smoothtime(){
        time_smoothing_factor = 0.95; /// slowly skew the clocks into sync
        error_threshold = .50; /// 50% jitter is acceptable for , discard data otherwise.
    }
    //! Between 0 and 1, bigger is smoother
    void set_smoothing_factor(double smoothing_factor)
    {
        time_smoothing_factor = smoothing_factor;
    }
    //! Between 0 and 1, threshold on jitter acceptability, higher accepts more jitter before discarding
    void set_error_threshold(double err_threshold)
    {
        error_threshold = err_threshold;
    }
    rclcpp::Time smooth_timestamp(rclcpp::Time recv_timestamp, rclcpp::Duration expctd_dur) {
    //if (smoothtime_prev.is_zero() == true) {
    if ((smoothtime_prev.seconds() == 0 && smoothtime_prev.nanoseconds() == 0 ) == true) {
      smoothed_timestamp = recv_timestamp;
    } else {
      smoothed_timestamp = smoothtime_prev + expctd_dur;
      double err = (recv_timestamp - smoothed_timestamp).nanoseconds();            //.toSec-> .nanoseconds
      double time_error_threshold = expctd_dur.nanoseconds() * error_threshold;    //.toSec-> .nanoseconds
      if ((time_smoothing_factor > 0) && (fabs(err) < time_error_threshold)){
        rclcpp::Duration correction = rclcpp::Duration(err * (1 - time_smoothing_factor));
        smoothed_timestamp += correction;
      } else {
        // error too high, or smoothing disabled - set smoothtime to last timestamp
        smoothed_timestamp = recv_timestamp;
      }
    }
    smoothtime_prev = smoothed_timestamp;
    return smoothed_timestamp;
  }
};

void createOdometryMessage(const rclcpp::Duration& time_elapsed, const tf2::Stamped<tf2::Transform>::Transform& prev_transform,
                           const tf2::Stamped<tf2::Transform>::Transform& current_transform, nav_msgs::msg::Odometry& odom_msg)
{
  double dt = time_elapsed.nanoseconds();
  double dx = (current_transform.getOrigin().getX() - prev_transform.getOrigin().getX())/dt;
  double dy = (current_transform.getOrigin().getY() - prev_transform.getOrigin().getY())/dt;
  double dr = (tf2::getYaw(current_transform.getRotation()) - tf2::getYaw(prev_transform.getRotation()))/dt;

  // setting position
  odom_msg.pose.pose.position.x = current_transform.getOrigin().getX();
  odom_msg.pose.pose.position.y = current_transform.getOrigin().getY();
  odom_msg.pose.pose.position.z = 0.0f;
  
  //this is function for more accurately odoms
  //but we will use kalman filter
  //odom_msg.pose.covariance.assign(0.0f);

   //may this change cause unexpected error 
  //tf2::quaternionTFToMsg(current_transform.getRotation(),odom_msg.pose.pose.orientation);
  tf2::convert(current_transform.getRotation() , odom_msg.pose.pose.orientation);

  // set velocity
  odom_msg.twist.twist.linear.x = dx;
  odom_msg.twist.twist.linear.y = dy;
  odom_msg.twist.twist.linear.z = 0.0f;
  odom_msg.twist.twist.angular.x = 0.0f;
  odom_msg.twist.twist.angular.y = 0.0f;
  odom_msg.twist.twist.angular.z = dr;
  //odom_msg.twist.covariance.assign(0.0f);

}

class averager {
protected:
  std::deque<double> deq;
  unsigned int max_len;
  double sum;
public:
  averager(int max_len = 50){
    this->max_len = max_len;
  }
  void add_new(double data) {
    deq.push_back(data);
    sum += data;
    if (deq.size() > max_len) {
      sum -= deq.front();
      deq.pop_front();
    }
  }
  double get_mean() {
    return sum/deq.size();
  }
};

//no functions in tf2
//newly created 
auto createQuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  //return tf2::toMsg(q);
  return q;
}

void PublishReflectorTransform(std::vector<double> x,std::vector<double> y,double th,std::vector<tf2_ros::TransformBroadcaster> odom_broadcaster,std::string frame_id,std::string child_frame_id)
{
  //printf("\npos %.2f %.2f %.2f\n",x,y,th);
  tf2::Transform b_transforms;
  //tf2::Stamped<tf2::Transform> b_transforms;
  //tf2::Quaternion tfquat=createQuaternionFromYaw(th);//th should be 0? no orientation information on reflectors?
  tf2::Quaternion tfquat = createQuaternionFromYaw(th);//th should be 0? no orientation information on reflectors?

  std::ostringstream childframes;
  for (int i=0;i<x.size();i++)
  {
    childframes << child_frame_id << i;
    b_transforms.setRotation(tfquat);
    b_transforms.setOrigin(tf2::Vector3(-x[i] / 1000, -y[i] / 1000, 0.0));
    //RCLCPP_DEBUG_STREAM(PublishReflectorTransform.get_logger(), "Reflector "<<i<<" x pos: "<<-x[i]<<" y pos: "<<-y[i]<<" with frame: "<<childframes.str());
 
    //send the transform (changed by alternatives)
    //odom_broadcaster[i].sendTransform(tf::StampedTransform(b_transforms, rclcpp::Clock().now(), frame_id, childframes.str()));
    //odom_broadcaster[i].sendTransform(geometry_msgs::msg::TransformStamped(b_transforms, rclcpp::Clock().now(), frame_id, childframes.str()));
    
    //Caused by no exist tf::StampedTransform() func.
    geometry_msgs::msg::TransformStamped t;
    auto now = rclcpp::Clock().now();
    //add infomations
    t.header.stamp = now;
    t.header.frame_id = frame_id;
    t.child_frame_id = childframes.str();
    t.transform = tf2::toMsg(b_transforms);

    odom_broadcaster[i].sendTransform(t);
    //odom_broadcaster[i].sendTransform(geometry_msgs::msg::TransformStamped(b_transforms, rclcpp::Clock().now(), frame_id, childframes.str()));
    childframes.str(std::string());
    childframes.clear();

  }
}

namespace OperatingModes
{
  enum OperatingMode
  {
    POWERDOWN = 0,
    STANDBY = 1,
    MAPPING = 2,
    LANDMARK = 3,
    NAVIGATION = 4,
  };
}
typedef OperatingModes::OperatingMode OperatingMode;

using namespace std;
//using namespace SickToolbox;

class sick_nav350: public rclcpp::Node
{
    public:
        sick_nav350() : Node("sick_nav350_node")
        { 
            //rclcpp::QoS(rclcpp::SensorDataQoS())
            scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
            scan_pub1 = this->create_publisher<nav350_msg::msg::Navtimestamp>("navtimestamp", 10);
            //odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);

            tf_buffer =
                  std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener =
                  std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
                  std::vector<tf2_ros::TransformBroadcaster> landmark_broadcasters;

        }

    private:
        int op_mode, port, wait, mask;
        std::string ipaddress;
        std::string frame_id;
        std::string scan;
        bool inverted, do_mapping;
        bool publish_odom;
        int sick_motor_speed = 8;//10; // Hz
        double sick_step_angle = 1.5;//0.5;//0.25; // deg (0.125 = no gaps between spots)
        double active_sector_start_angle = 0;
        double active_sector_stop_angle = 360;//269.75;
        double smoothing_factor, error_threshold;
        std::string sick_frame_id; 
        std::string target_frame_id; // the frame to be publish relative to frame_id
        std::string scan_frame_id;   // The frame reported in the scan message.
                                     // Decoupling this from target_frame_id allows us to
                                     // display nav localization and ROS localization
                                     // simultaneously in RViz.
        std::string mobile_base_frame_id = "";
        std::string reflector_frame_id, reflector_child_frame_id;

        //If you want a counterpart of tf::StampedTransform in tf2, 
        //you need to go with geometry_msgs::TransformStamped. 
        //Please note that tf2::Stamped<tf2::Transform> has no child_frame_id field which is required within the ROS tf/tf2 framework. 
        //geometry_msgs::msg::TransformStamped sickn350_to_target_tf;
        //geometry_msgs::msg::TransformStamped target_to_mobile_base_tf;

        //tf2::Transform sickn350_to_target_tf;
        //tf2::Transform target_to_mobile_base_tf;
        tf2::Stamped<tf2::Transform> sickn350_to_target_tf;
        tf2::Stamped<tf2::Transform> target_to_mobile_base_tf;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
        rclcpp::Publisher<nav350_msg::msg::Navtimestamp>::SharedPtr scan_pub1;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformBroadcaster> landmark_broadcasters;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};


 tf2::Stamped<tf2::Transform> tf2;

    private:
        void init_param()
        {
            //standard config
            this->declare_parameter("mode");
            this->declare_parameter("port");
            this->declare_parameter("ipaddress");
            this->declare_parameter("inverted");
            this->declare_parameter("publish_odom");
            this->declare_parameter("perform_mapping");
            this->declare_parameter("scan");
    
            //frame id enable
            this->declare_parameter("frame_id");
            this->declare_parameter("sick_frame_id");
            this->declare_parameter("scan_frame_id");
            this->declare_parameter("reflector_frame_id");
            this->declare_parameter("reflector_child_frame_id");
            this->declare_parameter("target_frame_id");
            this->declare_parameter("mobile_base_frame_id");

            //command
            this->declare_parameter("wait_command");
            this->declare_parameter("mask_command");

            //etc
            this->declare_parameter("timer_smoothing_factor");
            this->declare_parameter("timer_error_threshold");
            this->declare_parameter("resolution");
            this->declare_parameter("start_angle");
            this->declare_parameter("stop_angle");
            this->declare_parameter("scan_rate");

            //add values in params
            this->get_parameter_or<int>("mode", op_mode, 4);
            this->get_parameter_or<int>("port", port, DEFAULT_SICK_TCP_PORT);
            this->get_parameter_or<std::string>("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
            this->get_parameter_or<bool>("inverted", inverted, false);
            this->get_parameter_or<bool>("publish_odom",publish_odom, false);
            this->get_parameter_or<bool>("perform_mapping", do_mapping, true);
            this->get_parameter_or<std::string>("scan", scan, "scan");

            this->get_parameter_or<std::string>("frame_id", frame_id, "map");
            this->get_parameter_or<std::string>("sick_frame_id", sick_frame_id, "sick_nav350");
            this->get_parameter_or<std::string>("scan_frame_id", scan_frame_id, sick_frame_id);
            this->get_parameter_or<std::string>("reflector_frame_id", reflector_frame_id, "nav350");
            this->get_parameter_or<std::string>("reflector_child_frame_id", reflector_child_frame_id, "reflector");
            this->get_parameter_or<std::string>("target_frame_id",target_frame_id,sick_frame_id);
            this->get_parameter_or<std::string>("mobile_base_frame_id",mobile_base_frame_id, mobile_base_frame_id);
            this->get_parameter_or<int>("wait_command", wait, 1);
            this->get_parameter_or<int>("mask_command", mask, 2);

            this->get_parameter_or<double>("timer_smoothing_factor", smoothing_factor, 0.97);
            this->get_parameter_or<double>("timer_error_threshold", error_threshold, 0.5);
            this->get_parameter_or<double>("resolution", sick_step_angle, 1.0);
            this->get_parameter_or<double>("start_angle",active_sector_start_angle,0.);
            this->get_parameter_or<double>("stop_angle",active_sector_stop_angle,360.);
            this->get_parameter_or<int>("scan_rate",sick_motor_speed,5);
        }

        // TODO: refactor these functions into a common util lib (similar to code in sicklms.cpp)
        void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub, double *range_values,
                  uint32_t n_range_values, int *intensity_values,
                  uint32_t n_intensity_values, rclcpp::Time start,
                  double scan_time, bool inverted, float angle_min,
                  float angle_max, std::string frame_id,
                  //unsigned int sector_start_timestamp)   
                  unsigned int sector_start_timestamp, rclcpp::Publisher<nav350_msg::msg::Navtimestamp>::SharedPtr& pub1)         
                   /////////////////////////////////////////////////////msgs from customized
        {
            static int scan_count = 0;
            
            auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();


            scan_msg->header.stamp = start;
            scan_msg->header.frame_id = frame_id;
            scan_count++;
            
            // assumes scan window at the bottom
            if (inverted) { 
                scan_msg->angle_min = angle_max;
                scan_msg->angle_max = angle_min;
            } 
            else 
            {
                scan_msg->angle_min = angle_min;
                scan_msg->angle_max = angle_max;
            }
            scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(n_range_values-1);
            scan_msg->scan_time = 0.125;//scan_time;
            scan_msg->time_increment = scan_msg->scan_time/*scan_time*/ / n_range_values;
            scan_msg->range_min = 0.1;
            scan_msg->range_max = 250.;
            scan_msg->ranges.resize(n_range_values);


            for (size_t i = 0; i < n_range_values; i++) 
            {
                scan_msg->ranges[i] = (float)range_values[i]/1000;
            }
            scan_msg->intensities.resize(n_intensity_values);
            for (size_t i = 0; i < n_intensity_values; i++) 
            {
                scan_msg->intensities[i] = (float)intensity_values[i];
            }
              pub->publish(*scan_msg);
            /////////////////////////////////////////////////////msgs from customized
             auto st = std::make_shared<nav350_msg::msg::Navtimestamp>();
             st->stamp=start;
             st->navtimestamp=sector_start_timestamp;
             pub1->publish(*st);
            /////////////////////////////////////////////////////msgs from customized
        }

    public:    
        int work_loop()
        {
          init_param();
          /* Define buffers for return values */
          double range_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
          int intensity_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
            /* Define buffers to hold sector specific data */
          unsigned int num_measurements = {0};
          unsigned int sector_start_timestamp = {0};
          unsigned int sector_stop_timestamp = {0};
          double sector_step_angle = {0};
          double sector_start_angle = {0};
          double sector_stop_angle = {0};



          //tf2_ros::Buffer tf_buffer(rclcpp::Clock);
          //tf2_ros::TransformBroadcaster odom_broadcaster();
          //tf2_ros::TransformListener tf_listerner();
          //std::vector<tf2_ros::Buffer> tf_buffer;
          //std::vector<tf2_ros::TransformBroadcaster> odom_broadcaster;
          //std::vector<tf2_ros::TransformListener> tf_listerner;
          //geometry_msgs::msg::TransformStamped odom_broadcaster;
          //geometry_msgs::msg::TransformStamped tf_listerner;

          /* Instantiate the object */
          SickToolbox::SickNav350 sick_nav350(ipaddress.c_str(),port);
          double last_time_stamp=0;
          try
          {
            /* Initialize the device */
            sick_nav350.Initialize();
            sick_nav350.GetSickIdentity();

            // TODO: do some calls to setup the device - e.g. scan rate. Configure mapping. Configure reflectors/landmarks
            if (do_mapping)
            {
              sick_nav350.SetOperatingMode((int)OperatingModes::MAPPING);
              sick_nav350.DoMapping();
              sick_nav350.SetOperatingMode((int)OperatingModes::STANDBY);
              RCLCPP_INFO(this->get_logger(), "Sicknav50 Mapping Completed");
            }

            try
            {
              sick_nav350.SetOperatingMode((int)OperatingModes::STANDBY);
              sick_nav350.SetScanDataFormat();
              sick_nav350.SetOperatingMode(op_mode);
            }

            catch (...)
            {
              RCLCPP_ERROR(this->get_logger(), "Configuration error");
              return -1;
            }

            smoothtime smoothtimer = smoothtime();
            averager avg_fulldur = averager();
            averager avg_scandur = averager();
            smoothtimer.set_smoothing_factor(smoothing_factor);
            smoothtimer.set_error_threshold(error_threshold);
            rclcpp::Time last_start_scan_time;
            unsigned int last_sector_stop_timestamp = 0;
            double full_duration;
            rclcpp::Rate loop_rate(8);
            //std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcasters;
            //tf2_ros::TransformBroadcaster odom_broadcasters; 

            // looking up transform from sicknav350 to target frame
            if(target_frame_id == sick_frame_id)
            {
              sickn350_to_target_tf.setIdentity(); // = tf2::toMsg(test_transforms);
            }

            else
            {
              std::string error_msg;
              rclcpp::Time current_time = rclcpp::Time(0);
              
              try
              {
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
              // the tf2 command is chnaged
              //   tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

              //   tf_listerner = tf_buffer.waitForTransform(target_frame_id, sick_frame_id, rclcpp::Time(0), rclcpp::Duration(TRANSFORM_TIMEOUT), rclcpp::Duration(POLLING_DURATION),&error_msg);
                
              //   //tf_buffer.waitForTransform(target_frame_id, sick_frame_id, rclcpp::Time(0), rclcpp::Duration(TRANSFORM_TIMEOUT), rclcpp::Duration(POLLING_DURATION),&error_msg);

              //  if(!tf_listerner)
              //   RCLCPP_ERROR_STREAM(this->get_logger(), "Transform lookup timed out, error msg: "<<error_msg);
              //   return -1;
              // }
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //test_buffer1 = tf_buffer.lookupTransform(target_frame_id,sick_frame_id,current_time,rclcpp::Duration(TRANSFORM_TIMEOUT));
                // geometry_msgs::msg::TransformStamped test_buffer1 =
                //     tf_buffer->lookupTransform(target_frame_id,sick_frame_id,current_time,rclcpp::Duration(TRANSFORM_TIMEOUT));
              }

              catch(tf2::LookupException &exp)
              {
                  RCLCPP_ERROR_STREAM(this->get_logger(), "Transform lookup between "<<sick_frame_id<<" and "<<target_frame_id<<" failed, exiting"); //nav350, nav350
                  return -1;
              }
              //tf2::convert(test_buffer1, sickn350_to_target_tf);
            }

            rclcpp::Time previous_time = rclcpp::Clock().now()-rclcpp::Duration(0.5f);
            tf2::Stamped<tf2::Transform>::Transform mobile_base_current_tf = tf2::Stamped<tf2::Transform>::Transform::getIdentity();
            tf2::Stamped<tf2::Transform>::Transform mobile_base_prev_tf = tf2::Stamped<tf2::Transform>::Transform::getIdentity();
            nav_msgs::msg::Odometry odom_msg;

            if(publish_odom)
            {
              auto odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);

              if(mobile_base_frame_id == "")
              {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Frame id for mobile base was not set in the parameter list");
                return -1;
              }
              odom_msg.header.frame_id = frame_id;
              odom_msg.child_frame_id = mobile_base_frame_id;
              std::string error_msg;
              rclcpp::Time current_time = rclcpp::Time(0);
              try
              {
                
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
              // the tf2 command is chnaged
              //     if(!tf_listerner.waitForTransform(
              //         target_frame_id,mobile_base_frame_id,rclcpp::Time(0),rclcpp::Duration(TRANSFORM_TIMEOUT),
              //         rclcpp::Duration(POLLING_DURATION),&error_msg))
              //     {
              //          //ROS_ERROR_STREAM("Transform lookup timed out, error msg: "<<error_msg);
              //          return -1;
              //     }

              //     tf_listerner.lookupTransform(target_frame_id,mobile_base_frame_id,current_time,target_to_mobile_base_tf);
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                geometry_msgs::msg::TransformStamped test_buffer1 =
                     tf_buffer->lookupTransform(target_frame_id,mobile_base_frame_id,current_time,rclcpp::Duration(TRANSFORM_TIMEOUT));//base_scan, nav350
                     //tf_listener->sendTransform(test_buffer1);
              }

              catch(tf2::LookupException &exp)
              {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Transform lookup between "<<mobile_base_frame_id<<" and "<<target_frame_id<<" failed, exiting");
                return -1;
              }
              //tf2::convert(test_buffer1, target_to_mobile_base_tf);
            }
            while (rclcpp::ok() && !need_exit)
            {
              //Grab the measurements (from all sectors)
              if (op_mode==3)
              {
                //ROS_INFO_STREAM("Getting landmark data");
                sick_nav350.GetDataLandMark(1,1);
              }

              else if (op_mode==4)
              {
                //ROS_DEBUG_STREAM("Getting nav data");
                sick_nav350.GetDataNavigation(wait,mask);
              }

              else
              {
                RCLCPP_INFO_STREAM(this->get_logger()," Selected operating mode does not return data... try again");
                return -1;
              }

              //ROS_DEBUG_STREAM("Getting sick range/scan measurements");
              sick_nav350.GetSickMeasurements(range_values,
                                              intensity_values,
                                              &num_measurements,
                                              &sector_step_angle,
                                              &sector_start_angle,
                                              &sector_stop_angle,
                                              &sector_start_timestamp,
                                              &sector_stop_timestamp);


              /*
              * Get nav localization data and publish /map to /odom transform
              */
              double x1=(double) sick_nav350.PoseData_.x;
              double y1=(double) sick_nav350.PoseData_.y;
              double phi1=sick_nav350.PoseData_.phi;
              //ROS_DEBUG_STREAM("NAV350 pose in x y alpha:"<<x1<<" "<<y1<<" "<<phi1/1000.0);
              tf2::Transform odom_to_sick_tf;
              tf2::Transform odom_to_target_tf;
              tf2::Quaternion odomquat=createQuaternionFromYaw(DEG2RAD(phi1/1000.0));
              odomquat.inverse();
              odom_to_sick_tf.setRotation(odomquat);
              odom_to_sick_tf.setOrigin(tf2::Vector3(-x1 / 1000, -y1/ 1000, 0.0));

              // converting to target frame
              odom_to_target_tf = odom_to_sick_tf * sickn350_to_target_tf;

              //tf2::Stamped<Transform> transformTf = ...;
              // geometry_msgs::TransformStamped s = tf2::toMsg(transformTf);
                
              auto now = rclcpp::Clock().now();

              //add infomations
              // s.header.stamp = now;
              // s.header.frame_id = frame_id;
              // s.child_frame_id = target_frame_id;
              // s.transform = tf2::toMsg(odom_to_target_tf);
              // s.sendTransform(odom_broadcasters);



              RCLCPP_DEBUG_STREAM(this->get_logger(), "Sending transform from "<<frame_id<<" to "<<target_frame_id);
              // odom_broadcasters->sendTransform(s);            
              ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
              // //odom_broadcasters.sendTransform(tf2_ros::StampedTransform(odom_to_target_tf, rclcpp::Clock().now(), frame_id, target_frame_id));
      
              // publishing odometry
              if(publish_odom)
              {
                  mobile_base_current_tf = odom_to_target_tf * target_to_mobile_base_tf;
                  createOdometryMessage(rclcpp::Clock().now() - previous_time, mobile_base_prev_tf, mobile_base_current_tf, odom_msg);
                  mobile_base_prev_tf = mobile_base_current_tf;
                  previous_time = rclcpp::Clock().now();
                  odom_pub->publish(odom_msg);
              }
              /*
              * Get landmark data and broadcast transforms
              */
              int num_reflectors=sick_nav350.PoseData_.numUsedReflectors;
              int number_reflectors=sick_nav350.ReflectorData_.num_reflector;
              std::vector<double> Rx(number_reflectors), Ry(number_reflectors);
              RCLCPP_DEBUG_STREAM(this->get_logger(), "NAV350 # reflectors seen:"<<number_reflectors);
              RCLCPP_DEBUG_STREAM(this->get_logger(), "NAV350 # reflectors used:"<<num_reflectors);
              for (int r=0;r<number_reflectors; r++)
              {
                  Rx[r]=(double) sick_nav350.ReflectorData_.x[r];
                  Ry[r]=(double) sick_nav350.ReflectorData_.y[r];
                  RCLCPP_DEBUG_STREAM(this->get_logger(), "Reflector "<<r<<" x pos: "<<Rx[r]<<" y pos: "<<Ry[r]);
              }
              //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
              //landmark_broadcasters.resize(number_reflectors);
              ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
              //PublishReflectorTransform(Rx, Ry, DEG2RAD(phi1/1000.0), landmark_broadcasters, reflector_frame_id, reflector_child_frame_id);

              /*
              * Get Scan data and publish scan
              */
              if (sector_start_timestamp<last_time_stamp)
              {
                  loop_rate.sleep();
                  rclcpp::spin_some(shared_from_this());
                  continue;
              }
              last_time_stamp=sector_start_timestamp;
              rclcpp::Time end_scan_time = rclcpp::Clock().now();

              double scan_duration = (sector_stop_timestamp - sector_start_timestamp) * 1e-3;
              avg_scandur.add_new(scan_duration);
              scan_duration = 0.125;//avg_scandur.get_mean();
              if (last_sector_stop_timestamp == 0) {
                  full_duration = 1./((double)sick_motor_speed);
              } else {
                  full_duration = (sector_stop_timestamp - last_sector_stop_timestamp) * 1e-3;
              }
              avg_fulldur.add_new(full_duration);
              full_duration = avg_fulldur.get_mean();

              rclcpp::Time smoothed_end_scan_time = smoothtimer.smooth_timestamp(end_scan_time, rclcpp::Duration(full_duration));
              rclcpp::Time start_scan_time = smoothed_end_scan_time - rclcpp::Duration(scan_duration);
              sector_start_angle-=180;
              sector_stop_angle-=180;
              publish_scan(scan_pub, range_values, num_measurements, intensity_values, num_measurements, start_scan_time, scan_duration, inverted,
                          //DEG2RAD((float)sector_start_angle), DEG2RAD((float)sector_stop_angle), scan_frame_id, sector_start_timestamp);
                          DEG2RAD((float)sector_start_angle), DEG2RAD((float)sector_stop_angle), scan_frame_id, sector_start_timestamp,scan_pub1);


              /*ROS_INFO_STREAM/*DEBUG_STREAM*//*("Num meas: " << num_measurements
              << " smoothed start T: " << start_scan_time
              << " smoothed rate: " << 1./(start_scan_time - last_start_scan_time).toSec()
              << " raw start T: " << sector_start_timestamp
              << " raw stop T: " << sector_stop_timestamp
              << " dur: " << full_duration
              << " step A: " << sector_step_angle
              << " start A: " << sector_start_angle
              << " stop A: " << sector_stop_angle);
              //last_start_scan_time = start_scan_time;
              //last_sector_stop_timestamp = sector_stop_timestamp;*/
              loop_rate.sleep();
              rclcpp::spin_some(shared_from_this());
            }

            /* Uninitialize the device */
            try
            {
              sick_nav350.Uninitialize();
            }

            catch(...)
            {
              cerr << "Uninitialize failed!" << endl;
              return -1;
            }
          }
          catch(...)
          {
            RCLCPP_ERROR(this->get_logger(), "Error");
            return -1;
          }
        }
};




void ExitHandler(int sig)
{
    (void)sig;
    need_exit = true;
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto sick_nav350_publisher = std::make_shared<sick_nav350>();
  signal(SIGINT,ExitHandler);
  int ret = sick_nav350_publisher->work_loop();

  rclcpp::shutdown();
  return ret;
  //return 0;
}

