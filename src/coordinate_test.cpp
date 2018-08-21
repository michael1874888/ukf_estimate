#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
//#include <nav_msgs/Odometry.h>
#include <mavros_msgs/VFR_HUD.h>
//#include <UKF/output.h>
#include <std_msgs/Float64.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <fstream>
#define pi 3.1415926

using namespace std;
int callback_spin_count = 0;
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::Twist host_mocap_vel;
//gazebo_msgs::ModelStates gaz_state;
sensor_msgs::Imu imu_data;
typedef struct
{
    float roll;
    float pitch;
    float yaw;
}rpy;


void mocap_cb(const gazebo_msgs::ModelStates::ConstPtr &msg){
    //gaz_state = *msg;
    host_mocap.pose = msg->pose[2];
    host_mocap_vel = msg->twist[2];
    //ROS_INFO("y:%f",host_mocap.pose.position.y);

}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    imu_data = *msg;//test
    //ROS_INFO("imu y:%f",imu_data.orientation.y);
}

//void quaternionToRPY(){
//  /*
//  imu_data.orientation.x = 0.1;
//  imu_data.orientation.y = 0.05;
//  imu_data.orientation.z = 0.1;
//  imu_data.orientation.w = 1;*/
//   //imu orientation

//  if(imu_data.orientation.w == 0)
//  {
//    imu_data.orientation.w = 1;
//    flag = 0;
//  }
//  if(imu_data.orientation.w != 0 && imu_data.orientation.w != 1){
//    flag = 1;
//  }
//  //ROS_INFO("imu.x = %f", imu_data.orientation.x);

//  //ROS_INFO("flag = %d", flag);
//  //ROS_INFO("imu = %f", imu_data.orientation.w);
//  tf::Quaternion quat(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);

//  if(mocap_pose.pose.orientation.w == 0)
//  {
//    mocap_pose.pose.orientation.w = 1;
//    flag2 = 0;
//  }
//  if(mocap_pose.pose.orientation.w != 0 && mocap_pose.pose.orientation.w != 1)
//    flag2 = 1;

//  if(vfr_data.throttle == 0)
//  {
//    flag3 = 0;
//  }
//  if(vfr_data.throttle !=0)
//  {
//    flag3 = 1;
//  }
//  //ROS_INFO("imu.x = %f", imu_data.orientation.x);

//  //ROS_INFO("flag = %d", flag);
//  //ROS_INFO("imu = %f", imu_data.orientation.w);
//  tf::Quaternion quat1(mocap_pose.pose.orientation.x, mocap_pose.pose.orientation.y, mocap_pose.pose.orientation.z, mocap_pose.pose.orientation.w);

//  double roll, pitch, yaw;
//  double yaw_bias;
//  double roll_mocap, pitch_mocap, yaw_mocap;
//  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
//  tf::Matrix3x3(quat1).getRPY(roll_mocap, pitch_mocap, yaw_mocap);

//  geometry_msgs::Vector3 rpy;
//  geometry_msgs::Vector3 rpy_mocap;
//  rpy.x = roll;
//  rpy.y = pitch;
//  rpy.z = yaw;

//  rpy_mocap.x = roll_mocap;
//  rpy_mocap.y = pitch_mocap;
//  rpy_mocap.z = yaw_mocap;
//  if(imu_data.orientation.w != 0 && imu_data.orientation.w != 1){
//    flag = 1;

//  }

//  imu_roll = rpy.x;
//  imu_pitch = rpy.y;
//  imu_yaw = rpy_mocap.z;
//  /*
//  if(imu_yaw - yaw_bias < 0){
//    imu_yaw = imu_yaw - yaw_bias + 2*3.1415926;
//  }
//  else{
//    imu_yaw = imu_yaw - yaw_bias;
//  }
//  */
//  output.theta.x = imu_roll;
//  output.theta.y = imu_pitch;
//  output.theta.z = imu_yaw;

//  state_[StateMemberRoll] = rpy_mocap.x;
//  state_[StateMemberPitch] = rpy_mocap.y;
//  state_[StateMemberYaw] = rpy_mocap.z;

//  /*
//  state_[StateMemberRoll] = 0;
//  state_[StateMemberPitch] = 0;
//  state_[StateMemberYaw] = 0;
//*/
///*
//  state_[StateMemberAx] = 0;
//  state_[StateMemberAy] = 0;
//  state_[StateMemberAz] = 0;
///*
//  state_[StateMemberAx] = imu_data.linear_acceleration.x;
//  state_[StateMemberAy] = imu_data.linear_acceleration.y;
//  state_[StateMemberAz] = (imu_data.linear_acceleration.z - 9.8);
//*/
//  //ROS_INFO("roll = %f, pitch = %f, yaw = %f", state_[StateMemberRoll],state_[StateMemberPitch],state_[StateMemberYaw]);
///*
//  for (int i = 0; i < 19; i++){
//  printf("%f ", state_[i]);
//  }
//  printf("\n");
//*/
//}
float imuDataRec_accx[500], imuDataRec_accy[500], imuDataRec_accz[500];
float imuDataRec_wx[500], imuDataRec_wy[500], imuDataRec_wz[500];
float mocapDataRec_x[500], mocapDataRec_y[500], mocapDataRec_z[500];
rpy quaternionToRPY(float quat_x, float quat_y, float quat_z, float quat_w)
{
    rpy rpy1;
    double roll, pitch, yaw;
    tf::Quaternion quat1(quat_x,quat_y,quat_z,quat_w);
    tf::Matrix3x3(quat1).getRPY(roll, pitch, yaw);
    rpy1.roll = roll;
    rpy1.pitch = pitch;
    rpy1.yaw = yaw;

    return rpy1;
}

float biasNoiseCal(float data_expect, float* data, int iteration)
{
    float sum = 0, squareSubsum = 0;
    float mean = 0;
    float bias = 0, deviation = 0;
    for(int i=0;i<iteration;i++)
    {
        sum = sum + data[i];
    }
    mean = sum/iteration;
    bias = abs(data_expect - sum);
    for(int i=0;i<iteration;i++)
    {
        squareSubsum = squareSubsum + pow((data[i]-mean),2);
    }
    deviation = sqrt(squareSubsum/iteration);
    ROS_INFO("mean:%.3f deviation:%.3f",mean,deviation);
    return mean;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coordinate_test");
    ros::NodeHandle nh;
    ros::Subscriber host_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",10,mocap_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",10,imu_cb);
    ros::Publisher true_pub = nh.advertise<geometry_msgs::Point>("/true_data", 10);
    ros::Publisher measurement_pub = nh.advertise<geometry_msgs::Point>("/measurement_data", 10);
    ros::Publisher estimate_pub = nh.advertise<geometry_msgs::Point>("/estimate_data", 10);

    ros::AsyncSpinner spinner(2);   //0 means use threads of CPU numbers
    spinner.start();

    ros::Rate rate(50);

    //get callback value before start
    while(ros::ok() && callback_spin_count<=100){
        ROS_INFO("start to spin the callbacks");
        callback_spin_count++;
        rate.sleep();
    }
    //spinner.stop();   //seems that it is not necessary to have this line

    //calculate the bias and noise
    fstream file;
    file.open("noise_cal.txt", ios::out);
    for(int i=0;i<500;i++){
        ROS_INFO("start to calculate bias and noise");
        //spinner.start();
        //ROS_INFO("imu:    x:%.3f y:%.3f z:%.3f",imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
        imuDataRec_accx[i] = imu_data.linear_acceleration.x;
        imuDataRec_accy[i] = imu_data.linear_acceleration.y;
        imuDataRec_accz[i] = imu_data.linear_acceleration.z;
        imuDataRec_wx[i] = imu_data.angular_velocity.x;
        imuDataRec_wy[i] = imu_data.angular_velocity.y;
        imuDataRec_wz[i] = imu_data.angular_velocity.z;
        mocapDataRec_x[i] = host_mocap.pose.position.x;
        mocapDataRec_y[i] = host_mocap.pose.position.y;
        mocapDataRec_z[i] = host_mocap.pose.position.z;
        file << " acc_x: " << imuDataRec_accx[i] << " acc_y: " << imuDataRec_accy[i] << " acc_z: " << imuDataRec_accz[i];
        file << " wx: " << imuDataRec_wx[i] << " wy: " << imuDataRec_wy[i] << " wz: " << imuDataRec_wz[i];
        file << " x: " << mocapDataRec_x[i] << " y: " << mocapDataRec_y[i] << " z: " << mocapDataRec_z[i] <<"\n";
        //spinner.stop();
        rate.sleep();
    }
    //file.close();
    //float test[5] = {9.1,9.3,9.4,9.7,9.8};
    float accxDev, accyDev, acczDev, wxDev, wyDev, wzDev,xDev, yDev, zDev;
    accxDev = biasNoiseCal(0,&imuDataRec_accx[0],500);
    accyDev = biasNoiseCal(0,&imuDataRec_accy[0],500);
    acczDev = biasNoiseCal(0,&imuDataRec_accz[0],500);
    wxDev = biasNoiseCal(0,&imuDataRec_wx[0],500);
    wyDev = biasNoiseCal(0,&imuDataRec_wy[0],500);
    wzDev = biasNoiseCal(0,&imuDataRec_wz[0],500);
    xDev = biasNoiseCal(0,&mocapDataRec_x[0],500);
    yDev = biasNoiseCal(0,&mocapDataRec_y[0],500);
    zDev = biasNoiseCal(0,&mocapDataRec_z[0],500);
    file << " dev_ax: " << accxDev << " dev_ay: " << accyDev << " dev_az: " << acczDev;
    file << " dev_wx: " << wxDev << " dev_wy: " << wyDev << " dev_wz: " << wzDev;
    file << " dev_x: " << xDev << " dev_y: " << yDev << " dev_z: " << zDev <<"\n";
    file.close();

    //main process
    while(ros::ok()){
        //spinner.start();
        rpy rpy_mocap, rpy_imu;
        Eigen::MatrixXd rotation_x, rotation_y, rotation_z;
        rotation_x.setZero(3,3);
        rotation_y.setZero(3,3);
        rotation_z.setZero(3,3);
        Eigen::MatrixXd rotationB2C_x, rotationB2C_y, rotationB2C_z;
        rotationB2C_x.setZero(3,3);
        rotationB2C_y.setZero(3,3);
        rotationB2C_z.setZero(3,3);
        Eigen::VectorXd b2g_acc, b2g_acc2, body_acc, ag_global, ag_body, ag_body2, ag_body3, ag_body4;
        b2g_acc.setZero(3);
        b2g_acc2.setZero(3);
        body_acc.setZero(3);
        ag_global.setZero(3);
        ag_body.setZero(3);
        ag_body2.setZero(3);
        ag_body3.setZero(3);
        ag_body4.setZero(3);
        Eigen::VectorXd acc_bias;
        acc_bias.setZero(3);
        Eigen::VectorXd global_vel, body_vel, camera_vel, global_wvel, body_wvel, camera_wvel;
        global_vel.setZero(3);
        body_vel.setZero(3);
        camera_vel.setZero(3);
        global_wvel.setZero(3);
        body_wvel.setZero(3);
        camera_wvel.setZero(3);

        ROS_INFO("start");

        rpy_mocap = quaternionToRPY(host_mocap.pose.orientation.x,host_mocap.pose.orientation.y,host_mocap.pose.orientation.z,host_mocap.pose.orientation.w);
        rpy_imu = quaternionToRPY(imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w);
        body_acc << imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z;
        acc_bias << -0.251688, -0.264664, 0.2316;
        ag_global(2) = -9.8;

        global_vel << host_mocap_vel.linear.x, host_mocap_vel.linear.y, host_mocap_vel.linear.z;
        global_wvel << host_mocap_vel.angular.x, host_mocap_vel.angular.y, host_mocap_vel.angular.z;

        rotation_x <<  1,                   0,                    0,
                       0, cos(rpy_mocap.roll), sin(rpy_mocap.roll),
                       0, -sin(rpy_mocap.roll),  cos(rpy_mocap.roll);

        rotation_y <<  cos(rpy_mocap.pitch), 0, -sin(rpy_mocap.pitch),
                                          0, 1,                    0,
                      sin(rpy_mocap.pitch), 0, cos(rpy_mocap.pitch);

        rotation_z <<  cos(rpy_mocap.yaw), sin(rpy_mocap.yaw),    0,
                       -sin(rpy_mocap.yaw),  cos(rpy_mocap.yaw),    0,
                                        0,                   0,    1;

        rotationB2C_x <<  1,      0,       0,
                          0, cos(0),  sin(0),
                          0, -sin(0), cos(0);

        rotationB2C_y <<  cos(pi/2), 0, -sin(pi/2),
                                  0, 1,          0,
                          sin(pi/2), 0, cos(pi/2);

        rotationB2C_z <<  cos(-pi/2), sin(-pi/2),    0,
                         -sin(-pi/2), cos(-pi/2),    0,
                                   0,          0,    1;
        b2g_acc = rotation_z*rotation_y*rotation_x*body_acc;
        //b2g_acc2 = rotation_x*rotation_y*rotation_z*body_acc;
        ag_body = rotation_x*rotation_y*rotation_z*ag_global;
        //ag_body2 = rotation_z*rotation_y*rotation_x*ag_global;
        ag_body3 = (rotation_x*rotation_y*rotation_z).inverse()*ag_global;
        ag_body4 = rotation_z*rotation_y*rotation_x*ag_global;
        b2g_acc = rotation_z*rotation_y*rotation_x*(-(body_acc - acc_bias) - ag_body);
        //ag_body3 = (rotation_x*rotation_y*rotation_z).inverse()*(-(body_acc - ag_body));
        body_vel = rotation_x*rotation_y*rotation_z*global_vel;
        body_wvel = rotation_x*rotation_y*rotation_z*global_wvel;
        camera_vel = rotationB2C_z*rotationB2C_y*body_vel;
        camera_wvel = rotationB2C_z*rotationB2C_y*body_wvel;

        ROS_INFO("mocap:  pitch:%.3f roll:%.3f yaw:%.3f",rpy_mocap.pitch/pi*180, rpy_mocap.roll/pi*180, rpy_mocap.yaw/pi*180);
        ROS_INFO("imu:    pitch:%.3f roll:%.3f yaw:%.3f",rpy_imu.pitch/pi*180, rpy_imu.roll/pi*180, rpy_imu.yaw/pi*180);
        ROS_INFO("vel:    x:%.3f y:%.3f z:%.3f",host_mocap_vel.linear.x, host_mocap_vel.linear.y, host_mocap_vel.linear.z);

        //ROS_INFO("tf_acc2:x:%.3f y:%.3f z:%.3f",b2g_acc2(0), b2g_acc2(1), b2g_acc2(2));
        //ROS_INFO("bodyacc:x:%.3f y:%.3f z:%.3f",body_acc(0) - acc_bias(0), body_acc(1) - acc_bias(1), body_acc(2) - acc_bias(2));
        //ROS_INFO("g_body: x:%.3f y:%.3f z:%.3f",ag_body(0), ag_body(1), ag_body(2));
        //ROS_INFO("tf_acc: x:%.3f y:%.3f z:%.3f",b2g_acc(0), b2g_acc(1), b2g_acc(2));
        //ROS_INFO("g_body2:x:%.3f y:%.3f z:%.3f",ag_body2(0), ag_body2(1), ag_body2(2));
        //ROS_INFO("g_body3:x:%.3f y:%.3f z:%.3f",ag_body3(0), ag_body3(1), ag_body3(2));
       // ROS_INFO("g_body4:x:%.3f y:%.3f z:%.3f",ag_body4(0), ag_body4(1), ag_body4(2));
        ROS_INFO("g_vel:  x:%.3f y:%.3f z:%.3f",global_vel(0), global_vel(1), global_vel(2));
        ROS_INFO("b_vel:  x:%.3f y:%.3f z:%.3f",body_vel(0), body_vel(1), body_vel(2));
        ROS_INFO("c_vel:  x:%.3f y:%.3f z:%.3f",camera_vel(0), camera_vel(1), camera_vel(2));
        ROS_INFO("g_wvel: x:%.3f y:%.3f z:%.3f",global_wvel(0), global_wvel(1), global_wvel(2));
        ROS_INFO("b_wvel: x:%.3f y:%.3f z:%.3f",body_wvel(0), body_wvel(1), body_wvel(2));
        ROS_INFO("c_wvel: x:%.3f y:%.3f z:%.3f",camera_wvel(0), camera_wvel(1), camera_wvel(2));

        //spinner.stop();
        ROS_INFO("after spin");
        rate.sleep();


    }
    ros::waitForShutdown();
}

