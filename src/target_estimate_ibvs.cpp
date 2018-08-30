#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/VFR_HUD.h>
#include <ukf_estimate/output.h>
#include <std_msgs/Float64.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <deque>
#include <numeric>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#define pi 3.14159265359

using namespace std;
////////////////////Global variable//////////////////
Eigen::VectorXd global_vel, body_vel, camera_vel, global_wvel, body_wvel, camera_wvel;
Eigen::VectorXd target_gvel, target_g2cvel, target_gwvel, target_g2cwvel;
Eigen::MatrixXd rotation_x, rotation_y, rotation_z;
Eigen::MatrixXd rotationB2C_x, rotationB2C_y, rotationB2C_z;
Eigen::MatrixXd rot_c2g;
Eigen::MatrixXd rot_b2g;
Eigen::MatrixXd rot_b2c;
float fx = 381.36246688113556, fy = 381.36246688113556; //286.02185016085167
float cx = 320.5, cy = 240.5; 							//240.5 135.5
float image_width = 640, image_height = 480;			//480*270
float model_width = 4.6344, model_height = 1.5;
int callback_spin_count = 0;
float KPx = 1;
float KPy = 1;
float KPz = 1;
float KProll = 1;
float KPx_ibvs = 0.0005;
float KPy_ibvs = 0.0005;
float KPz_ibvs = 0.0008;
float KProll_ibvs = 1.6;
////Note : Ti cannot set to zero, set Ti = inf(1000 or higher) to remove I control//////
float Tix_ibvs = 10000;
float Tiy_ibvs = 10000;
float Tiz_ibvs = 10000;
float Tiroll_ibvs = 10000;
///////////////////////////////////////////////////////////////////////////////////////
float Tdx_ibvs = 0;
float Tdy_ibvs = 0;
float Tdz_ibvs = 0;
float Tdroll_ibvs = 0;

float desired_distance = 6500;							//5500
float camera_offset = 0;							//0
float desired_heading = -pi/2;							//-pi/2

float err_ux, err_uy, err_uz, err_uroll;
float err_ux_pre = 0, err_uy_pre = 0, err_uz_pre = 0, err_uroll_pre = 0;	//initialize
float err_ux_dev = 0, err_uy_dev = 0, err_uz_dev = 0, err_uroll_dev = 0;
float err_ux_sum = 0, err_uy_sum = 0, err_uz_sum = 0, err_uroll_sum = 0;
bool ibvs_mode = false;
bool ukf_mode = false;
/////////////////////////////////////////////////////

int drone_idx = 0;
int car_idx = 0;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::Twist host_mocap_vel;
geometry_msgs::PoseStamped car_pose;
geometry_msgs::Twist car_vel;
sensor_msgs::Imu imu_data;
std_msgs::Float32MultiArray box;
typedef struct
{
    double roll;
    double pitch;
    double yaw;
}rpy;

typedef struct
{
    float roll;
    float x;
    float y;
    float z;
}vir;

int sign(double num)
{
    if(num >= 0) return 1;
    else if(num < 0) return -1;
    else return 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void mocap_cb(const gazebo_msgs::ModelStates::ConstPtr &msg){
    //gaz_state = *msg;
    while ((strcmp(msg->name[drone_idx].c_str(),"iris") != 0 ))
    {
        if(drone_idx<(msg->name.size()-1))
            drone_idx++;
    }
    while ((strcmp(msg->name[car_idx].c_str(),"prius") != 0 ))
    {
        if(car_idx<(msg->name.size()-1))
            car_idx++;
    }
    host_mocap.pose = msg->pose[drone_idx];
    host_mocap_vel = msg->twist[drone_idx];
    car_pose.pose = msg->pose[car_idx];
    car_vel = msg->twist[car_idx];
    //ROS_INFO("mocap");

}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    imu_data = *msg;//test
    //ROS_INFO("imu");
}

void box_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    box = *msg;
    //ROS_INFO("box");
}
////////////////////UKF Global variable//////////////////
double L;
double dt;
//vector size
int x_size;
int y_size;
int x_sigmavector_size;
int y_sigmavector_size;
//int window_size;
//virtual Eigen::MatrixXd  dynamics(Eigen::MatrixXd  sigma_state);
//virtual Eigen::MatrixXd   state_to_measure(Eigen::MatrixXd  sigma_state);


Eigen::MatrixXd rotate(double roll , double yaw , double pitch);

void set_initial_state_value(Eigen::VectorXd vector);
void set_process_noise(Eigen::MatrixXd matrix);
void set_measurement_noise(Eigen::MatrixXd matrix);
void set_covariance_matrix(Eigen::MatrixXd matrix);

void set_measurement_matrix(Eigen::MatrixXd matrix);
void set_parameter(double alpha,double beta , double lambda , double kappa);

void predict();
void correct(Eigen::VectorXd measure);


Eigen::VectorXd x ; //states
Eigen::VectorXd y ; //measurements

Eigen::VectorXd x_hat; //x mean
Eigen::VectorXd y_hat; //y mean
Eigen::VectorXd x_hat_;
Eigen::VectorXd y_hat_;

double alpha ;
double kappa ;
double beta ;
double lambda ;

Eigen::VectorXd w_c ; //weight c
Eigen::VectorXd w_m ;  //weight m

Eigen::MatrixXd x_sigmavector ;
Eigen::MatrixXd y_sigmavector ;
Eigen::MatrixXd x_sigmavector_ ;
Eigen::MatrixXd y_sigmavector_ ;
Eigen::MatrixXd H ;    //measurement transform

Eigen::MatrixXd P ; //covariance matrix

Eigen::MatrixXd Q ; //noise matrix
Eigen::MatrixXd R ; //noise matrix
Eigen::VectorXd q ;
Eigen::VectorXd r ;
deque<Eigen::VectorXd> q_window, r_window;
deque<Eigen::MatrixXd> Q_window, R_window;
deque<double> w_window;

Eigen::MatrixXd P_a ; //covariance matrix
Eigen::MatrixXd P_ ; //covariance matrix
Eigen::MatrixXd P_yy ;
Eigen::MatrixXd P_yy_ ;
Eigen::MatrixXd P_xy ;

Eigen::MatrixXd Kalman_gain ;
////////////////////////////////////////////////////////////////


////////////////////define state and measurement////////////////
enum state{
    member_x1=0,
    member_x2,
    member_x3,
    member_xq,
    member_yq,
    member_zq,
    member_vqx,
    member_vqy,
    member_vqz,
    statesize
};

enum measurement{
    member_mu = 0,
    member_mv,
    member_ma,
    member_mxc,
    member_myc,
    member_mzc,
    measurementsize
};
////////////////////////////////////////////////////////////////


////////////////////dynamic(predict state)////////////////
Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state){

    x_size = statesize;
    x_sigmavector_size = 2*x_size+1;
    Eigen::MatrixXd predict_sigma_state(x_size,x_sigmavector_size);
    ROS_INFO("vx:%.3f vy:%.3f vz:%.3f",camera_vel(0),camera_vel(1),camera_vel(2));
    ROS_INFO("wx:%.3f wy:%.3f wz:%.3f",camera_wvel(0),camera_wvel(1),camera_wvel(2));

    for(int i=0;i<x_sigmavector_size;i++){

        double x1 = sigma_state(member_x1,i);
        double x2 = sigma_state(member_x2,i);
        double x3 = sigma_state(member_x3,i);
        Eigen::VectorXd q_pose;
        q_pose.setZero(3);
        q_pose << sigma_state(member_xq,i), sigma_state(member_yq,i), sigma_state(member_zq,i);
        Eigen::VectorXd vq;
        vq.setZero(3);
        vq << sigma_state(member_vqx,i), sigma_state(member_vqy,i), sigma_state(member_vqz,i);
        //vq << target_g2cvel(0), target_g2cvel(1), target_g2cvel(2);
        double x1_ ;
        double x2_ ;
        double x3_ ;
        Eigen::VectorXd q_pose_;
        q_pose_.setZero(3);
        Eigen::VectorXd vq_;
        vq_.setZero(3);

        x1_ = x1 + (vq(0)*x3 - vq(2)*x1*x3 + (camera_vel(2)*x1 - camera_vel(0))*x3 + camera_wvel(2)*x2 - camera_wvel(1) - camera_wvel(1)*x1*x1 + camera_wvel(0)*x1*x2)*dt;
        x2_ = x2 + (vq(1)*x3 - vq(2)*x2*x3 + (camera_vel(2)*x2 - camera_vel(1))*x3 - camera_wvel(2)*x1 + camera_wvel(0) + camera_wvel(0)*x2*x2 - camera_wvel(1)*x1*x2)*dt;
        x3_ = x3 + (-vq(2)*x3*x3 + camera_vel(2)*x3*x3 - (camera_wvel(1)*x1 - camera_wvel(0)*x2)*x3)*dt;
        q_pose_ = q_pose + rot_c2g*vq*dt;
        vq_ = vq;
        //ROS_INFO("vq: x:%.3f y:%.3f z:%.3f",vq_(0),vq_(1),vq_(2));


        predict_sigma_state(member_x1,i) =  x1_;
        predict_sigma_state(member_x2,i) =  x2_;
        predict_sigma_state(member_x3,i) =  x3_;
        predict_sigma_state(member_xq,i) =  q_pose_(0);
        predict_sigma_state(member_yq,i) =  q_pose_(1);
        predict_sigma_state(member_zq,i) =  q_pose_(2);
        predict_sigma_state(member_vqx,i) =  vq_(0);
        predict_sigma_state(member_vqy,i) =  vq_(1);
        predict_sigma_state(member_vqz,i) =  vq_(2);

    }
    ROS_INFO("x1:%.3f x2:%.3f x3:%.3f, z:%.3f",sigma_state(member_x1,0),sigma_state(member_x2,0),sigma_state(member_x3,0),1/sigma_state(member_x3,0));
    ROS_INFO("x1_:%.3f x2_:%.3f x3_:%.3f z:%.3f",predict_sigma_state(member_x1,0),predict_sigma_state(member_x2,0),predict_sigma_state(member_x3,0),1/predict_sigma_state(member_x3,0));
    //ROS_INFO("xq:%.3f yq:%.3f zq:%.3f",sigma_state(member_xq,0),sigma_state(member_yq,0),sigma_state(member_zq,0));
    //ROS_INFO("vqx_:%.3f vqy_:%.3f vqz_:%.3f",predict_sigma_state(member_vqx,0),predict_sigma_state(member_vqy,0),predict_sigma_state(member_vqz,0));

    return predict_sigma_state;


}
///////////////////////////////////////////////////////


////////////////////predict measurement////////////////
Eigen::MatrixXd state_to_measure(Eigen::MatrixXd sigma_state){

    y_size = measurementsize;
    x_sigmavector_size = 2*x_size+1;
    Eigen::MatrixXd predict_sigma_measure(y_size,x_sigmavector_size);
    Eigen::Vector3d rel_pose, rel_posec2g;

    for(int i=0;i<x_sigmavector_size;i++){

        rel_pose << sigma_state(member_x1,i)/sigma_state(member_x3,i), sigma_state(member_x2,i)/sigma_state(member_x3,i), 1/sigma_state(member_x3,i);
        rel_posec2g = rot_c2g*rel_pose;

        predict_sigma_measure( member_mu ,i) =   fx*sigma_state(member_x1,i) + cx;
        predict_sigma_measure( member_mv ,i) =   fy*sigma_state(member_x2,i) + cy;
        predict_sigma_measure( member_ma ,i) =   fx*fy*model_width*model_height*sigma_state(member_x3,i)*sigma_state(member_x3,i)*sign(sigma_state(member_x3,i));
        //a will be positive although x3 is negative, so we need to add sign function
        //add bias to predict measurement
        predict_sigma_measure( member_mxc ,i) =   sigma_state(member_xq,i) - rel_posec2g(0);
        predict_sigma_measure( member_myc ,i) =   sigma_state(member_yq,i) - rel_posec2g(1);
        predict_sigma_measure( member_mzc ,i) =   sigma_state(member_zq,i) - rel_posec2g(2);
        //ROS_INFO("rel_pose: x:%.3f y:%.3f z:%.3f",rel_pose(0),rel_pose(1),rel_pose(2));
    }
    //ROS_INFO("xq:%.3f",sigma_state(member_xq,0));
    return predict_sigma_measure;


}
////////////////////////////////////////////////////



void initialize(){

    ROS_INFO("Initilaize");

    x_size = statesize;
    y_size = measurementsize;
    alpha = 1e-3;
    kappa = 0;
    beta = 2;
    lambda = 0.0;


    L=x_size;
    x_sigmavector_size=2*x_size+1;

    lambda= alpha * alpha * (L + kappa) -L;

    x.setZero(x_size);
    y.setZero(y_size);

    x_hat_.setZero(x_size);
    y_hat_.setZero(y_size);
    x_hat.setZero(x_size);
    y_hat.setZero(y_size);


    x_sigmavector.setZero(x_size,x_sigmavector_size);

    y_sigmavector.setZero(y_size,x_sigmavector_size);

    x_sigmavector_.setZero(x_size,x_sigmavector_size);

    y_sigmavector_.setZero(y_size,x_sigmavector_size);

    H.setZero(y_size,x_size);  // measurement matrix


    y = H*x;

    w_c.setZero(x_sigmavector_size);
    w_m.setZero(x_sigmavector_size);


    w_c(0) = (lambda / (L+lambda))+(1-alpha*alpha+beta);
    w_m(0) = (lambda)/(L+lambda);

    for(int i=1 ; i<x_sigmavector_size ; i++){
        w_c(i) = 1/(2*(L+lambda));
        w_m(i) = 1/(2*(L+lambda));
    }

    // default Q R P matrix and initial value of x
    Q =5e-7*Eigen::MatrixXd::Identity(x_size, x_size);
    R =5e-4*Eigen::MatrixXd::Identity(y_size,y_size);
    P=1e-3*Eigen::MatrixXd::Identity(x_size, x_size);

    q.setZero(x_size);
    r.setZero(y_size);

    P_.setZero(x_size,x_size);
    P_yy.setZero(y_size,y_size);
    P_xy.setZero(x_size,y_size);
    P_yy_.setZero(y_size,y_size);

    //  x<<0,2;
    //  x_hat<<0,0;
}


//time update
void predict(){
    //find sigma point
    P=(lambda+L)*P;
    Eigen::MatrixXd M= (P).llt().matrixL();
    Eigen::MatrixXd buffer;
    x_sigmavector.col(0) = x;
    for(int i=0;i<x_size;i++)
    {

        Eigen::VectorXd sigma =(M.row(i)).transpose();
        x_sigmavector.col(i+1) = x + sigma;

        x_sigmavector.col(i+x_size+1) = x - sigma;
    }
    x_sigmavector_ = dynamics( x_sigmavector);
    x_sigmavector =x_sigmavector_ + q*Eigen::MatrixXd::Constant(1,x_sigmavector_size,1);

    //x_hat (mean)
    x_hat_.setZero(x_size);   //initialize x_hat
    x_hat.setZero(x_size);
    for(int i=0;i<x_sigmavector_size;i++){
        x_hat_ += w_m(i)* x_sigmavector_.col(i);
    }
    x_hat = x_hat_ + q;

    //covariance
    P_.setZero(x_size,x_size);
    P.setZero(x_size,x_size);
    for(int i=0 ; i<x_sigmavector_size ;i++){
        P_+=   w_c(i) * (x_sigmavector.col(i)-x_hat) * ((x_sigmavector.col(i)-x_hat).transpose());
    }
    //add process noise covariance
    P = P_ + Q;

    y_sigmavector_ = state_to_measure(x_sigmavector);
    y_sigmavector = y_sigmavector_ + r*Eigen::MatrixXd::Constant(1,x_sigmavector_size,1);
    //y_sigmavector = H* x_sigmavector;
    //y_hat (mean)
    y_hat_.setZero(y_size);
    y_hat.setZero(y_size);
    for(int i=0;i< x_sigmavector_size;i++){
        y_hat_ += w_m(i) * y_sigmavector_.col(i);
    }
    y_hat = y_hat_ + r;
}


//measurement update
void correct(Eigen::VectorXd measure){

    y=measure;

    P_yy_.setZero(y_size,y_size);
    P_yy.setZero(y_size,y_size);
    P_xy.setZero(x_size,y_size);

    for(int i=0;i<x_sigmavector_size;i++){
        Eigen::MatrixXd err;
        Eigen::MatrixXd err_t;
        err = y_sigmavector.col(i) - y_hat;
        err_t = err.transpose();
        P_yy_ += w_c(i) * err * err_t;
    }
    //add measurement noise covarinace

    P_yy = P_yy_ + R;

    for(int i=0;i<x_sigmavector_size;i++){

        Eigen::VectorXd err_y , err_x;
        err_y = y_sigmavector.col(i) - y_hat;
        err_x = x_sigmavector.col(i) - x_hat;
        P_xy += w_c(i) * err_x * err_y.transpose();
    }

    Kalman_gain = P_xy * (P_yy.inverse());

    x = x_hat + Kalman_gain *(y-y_hat);

    P = P - Kalman_gain*P_yy*(Kalman_gain.transpose());

}


//noise estimate
void noise_estimate(int window_size)
{
    //ROS_INFO("estimate noise");

    Eigen::VectorXd q_window_element, r_window_element;
    Eigen::MatrixXd Q_window_element, R_window_element;
    Eigen::MatrixXd Q_window_element_, R_window_element_;
    double delta_S, w_element;
    deque<double> v_weight;
    float S_gain = 1;
    Q_window_element.setZero(x_size,x_size);
    R_window_element.setZero(y_size,y_size);
    v_weight.resize(window_size);


    Eigen::VectorXd q_window_sum, r_window_sum;
    Eigen::MatrixXd Q_window_sum, R_window_sum;


    q_window_element = x - x_hat_;
    r_window_element = y - y_hat_;
    Q_window_element_ = P + Kalman_gain*(y - y_hat)*((y - y_hat).transpose())*(Kalman_gain.transpose()) - P_;
    R_window_element_ = (y - y_hat)*((y - y_hat).transpose()) - P_yy_;
    Q_window_element = Q_window_element_.cwiseAbs().diagonal().asDiagonal();
    R_window_element = R_window_element_.cwiseAbs().diagonal().asDiagonal();
    delta_S = (S_gain*(P_yy_ + R).trace())/(((y - y_hat).transpose())*(y - y_hat));
    w_element = sqrt(((x - x_hat).transpose())*(x - x_hat))*sqrt(((y_hat - y).transpose())*(y_hat - y))*delta_S;

    q_window.push_front(q_window_element);
    r_window.push_front(r_window_element);
    Q_window.push_front(Q_window_element);
    R_window.push_front(R_window_element);
    w_window.push_front(w_element);


    if(q_window.size()>window_size || r_window.size()>window_size || Q_window.size()>window_size || R_window.size()>window_size)
    {
        //ROS_INFO("estimate noise");
        q_window.resize(window_size);
        r_window.resize(window_size);
        Q_window.resize(window_size);
        R_window.resize(window_size);
        w_window.resize(window_size);
    }


    if(q_window.size()==window_size || r_window.size()==window_size || Q_window.size()==window_size || R_window.size()==window_size)
    {
        //ROS_INFO("estimate noise");
        q_window_sum.setZero(x_size);
        r_window_sum.setZero(y_size);
        Q_window_sum.setZero(x_size,x_size);
        R_window_sum.setZero(y_size,y_size);


        for(int i=0;i<window_size;i++)
        {
            v_weight.at(i) = w_window.at(i)/std::accumulate(w_window.begin(), w_window.end(), 0);
            //cout<<"w"<<i<<": "<<w_window.at(i);
        }

        for(int i=0;i<window_size;i++)
        {
            q_window_sum += q_window.at(i)*v_weight.at(i);
            r_window_sum += r_window.at(i)*v_weight.at(i);
            Q_window_sum += Q_window.at(i)*v_weight.at(i);
            R_window_sum += R_window.at(i)*v_weight.at(i);
            //cout<<"v_weight"<<i<<": "<<v_weight.at(i);
        }
        if(callback_spin_count>250)
        {
            //q = q_window_sum;
            //r = r_window_sum;
            Q = Q_window_sum;
            //R = R_window_sum;
        }
    }
    //cout << "\nq" <<q_window_element;
    //cout << "\nq" <<q_window_sum;
    //cout << "\nr" <<r_window_element;
    //cout << "\nQ\n" <<Q_window_element;
    //cout << "\nQsum\n" <<Q_window_sum<<"\n";
    //cout << "\nR\n" <<R_window_element<<"\n";
    //cout << "\nR" <<R_window_element<<"\n";
    //cout << "\nRsum" <<R_window_sum<<"\n";
    //cout << "\nRsum\n" <<R_window_sum/window_size<<"\n";
    callback_spin_count++;
}

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
rpy rpy_mocap;

void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs, float dis_x, float dis_y)
{
    float errx, erry, errz, err_roll;
    float ux, uy, uz, uroll;
    float local_x, local_y;

    local_x = cos(vir.roll)*dis_x+sin(vir.roll)*dis_y;
    local_y = -sin(vir.roll)*dis_x+cos(vir.roll)*dis_y;

    errx = vir.x - host_mocap.pose.position.x - local_x;
    erry = vir.y - host_mocap.pose.position.y - local_y;
    errz = vir.z - host_mocap.pose.position.z - 0;
    err_roll = vir.roll - rpy_mocap.yaw;
    if(err_roll>pi)
        err_roll = err_roll - 2*pi;
    else if(err_roll<-pi)
        err_roll = err_roll + 2*pi;

    ROS_INFO("err_roll: %.3f",err_roll);

    ux = KPx*errx;
    uy = KPy*erry;
    uz = KPz*errz;
    uroll = KProll*err_roll;

    vs->twist.linear.x = ux;
    vs->twist.linear.y = uy;
    vs->twist.linear.z = uz;
    vs->twist.angular.z = uroll;

}

void ibvs(vir& vir, std_msgs::Float32MultiArray box, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs, float de_time)
{
    float xt, yt;
    float erru, errv, errz, err_roll;
	float err_ux_cam, err_uy_cam, err_uz_cam, err_upitch_cam;
	Eigen::Vector3d errv_cam, errw_cam;
	Eigen::Vector3d errv_global, errw_global;
	Eigen::Vector3d errv_body;//, errv_global_;
	Eigen::Vector3d vq_cam, vq_global;
	Eigen::Vector3d uv_global, uw_global, uv_cam, uw_cam;
    float ux, uy, uz, uroll;
    float ux_trans, uy_trans;
    float box_x_center, box_y_center;
    float model_true_area, model_image_area;
    float depth_Z;
    float fov_x;
    float box_width, box_height;

	vq_cam << x(6), x(7), x(8);
	vq_global = rot_c2g*vq_cam;

	box_width = box.data[4]*image_width;
	box_height = box.data[3]*image_height;

	model_true_area = model_width*model_height;
	model_image_area = box_width*box_height;

	box_x_center = box.data[2]*image_width;
	box_y_center = box.data[1]*image_height;
	fov_x = 2*atan(image_width/(2*fx));

    xt = (box_x_center - cx)/fx;
    yt = (box_y_center - cy)/fy;
    depth_Z = 1000*sqrt((model_true_area/model_image_area)*fx*fy);       //meter to millimeter
    ROS_INFO("x_center:%.2f y_center: %.2f depth_Z:%.2f",box_x_center,box_y_center,depth_Z);
	
	//ROS_INFO("erru: %.2f errv:%.2f",xt,yt);

    erru = xt - (cx - cx)/fx;
    errv = yt - ((cy + 0.1*image_height) - cy)/fy;
	//define a = sqrt(model_true_area/(depth_Z^2)) = sqrt(model_image_area/(fx*fy))
	//err_a = a - a* = a - a*depth_Z/depth_Z*

    err_uy_cam = depth_Z*errv;
    err_uz_cam = -depth_Z*(1 - depth_Z/desired_distance);
    err_upitch_cam = (1/(xt*xt+1))*erru;
	errv_cam << 0, err_uy_cam, err_uz_cam;
    errw_cam << 0, err_upitch_cam, 0;
	errv_body << 0, ((rpy_mocap.yaw - desired_heading)*depth_Z*image_width/(fov_x*fx)/* - erru*depth_Z*/), 0;
	errv_cam = errv_cam + rot_b2c*errv_body;
	//errv_global = rot_c2g*errv_cam + rot_b2g*errv_body;
	//errw_global = rot_c2g*errw_cam;
	//errv_global_ = rot_b2g*errv_body;
	//errv_global = errv_global + 
	err_ux = errv_cam(0);// + 1000*target_gvel(0);
	err_uy = errv_cam(1);// + 1000*target_gvel(1);
	err_uz = errv_cam(2);// + 1000*target_gvel(2);
	err_uroll = errw_cam(1);
	
	ROS_INFO("yaw: %.2f err_uy:%.2f",rpy_mocap.yaw/pi*180,err_uy);
	//cout << "\nerrv_cam\n" <<errv_cam << "\nerrw_cam\n" <<errw_cam;
	//cout << "\nerrv_body\n" <<errv_body;
	//cout << "\nerrv_b2c\n" <<rot_b2c*errv_body;
	//cout << "\nerrv_global\n" <<errv_global << "\nerrw_global\n" <<errw_global;
	cout << "\ntarget_gvel\n" <<vq_global;
	cout << "\nerr_ux: " <<err_ux<< " err_uy: " <<err_uy<< " err_uz: " <<err_uz<< " err_uroll: " <<err_uroll <<"\n";

    err_ux_sum = err_ux_sum + err_ux;
    err_uy_sum = err_uy_sum + err_uy;
    err_uz_sum = err_uz_sum + err_uz;
    err_uroll_sum = err_uroll_sum + err_uroll;

    err_ux_dev = err_ux - err_ux_pre;
    err_uy_dev = err_uy - err_uy_pre;
    err_uz_dev = err_uz - err_uz_pre;
    err_uroll_dev = err_uroll - err_uroll_pre;

    err_ux_pre = err_ux;
    err_uy_pre = err_uy;
    err_uz_pre = err_uz;
    err_uroll_pre = err_uroll;

    ux = KPx_ibvs*err_ux + KPx_ibvs*de_time*err_ux_sum/Tix_ibvs + KPx_ibvs*Tdx_ibvs*err_ux_dev/de_time;
    uy = KPy_ibvs*err_uy + KPy_ibvs*de_time*err_uy_sum/Tiy_ibvs + KPy_ibvs*Tdy_ibvs*err_uy_dev/de_time;
    uz = KPz_ibvs*err_uz + KPz_ibvs*de_time*err_uz_sum/Tiz_ibvs + KPz_ibvs*Tdz_ibvs*err_uz_dev/de_time;
    uroll = KProll_ibvs*err_uroll + KProll_ibvs*de_time*err_uroll_sum/Tiroll_ibvs + KProll_ibvs*Tdroll_ibvs*err_uroll_dev/de_time;
    //if(abs(ux)>0.3)
    //ux = 0.3*ux/abs(ux);
	uv_cam << ux, uy, uz;
	uw_cam << 0, uroll, 0;
	uv_global = rot_c2g*uv_cam;
	uw_global = rot_c2g*uw_cam;


	ROS_INFO("ux: %.3f uy: %.3f uz: %.3f uroll: %.3f",uv_global(0),uv_global(1),uv_global(2),uw_global(2));
    vs->twist.linear.x = uv_global(0) + vq_global(0);//+ target_gvel(0);//_trans;
    vs->twist.linear.y = uv_global(1) + vq_global(1);//+ target_gvel(1);//_trans;
    vs->twist.linear.z = uv_global(2) + vq_global(2);//+ target_gvel(2);
    vs->twist.angular.z = uw_global(2);
    ROS_INFO("vx: %.3f vy: %.3f vz: %.3f",vs->twist.linear.x,vs->twist.linear.y,vs->twist.linear.z);
    vir.x = host_mocap.pose.position.x;
    vir.y = host_mocap.pose.position.y;
    vir.z = host_mocap.pose.position.z;
}

void ibvs_ukf(vir& vir, Eigen::VectorXd state, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs, float de_time)
{
    float xt, yt;
	float erru, errv, errz, err_roll;
    float err_ux_cam, err_uy_cam, err_uz_cam, err_upitch_cam;
	Eigen::Vector3d errv_cam, errw_cam;
	Eigen::Vector3d errv_global, errw_global;
	Eigen::Vector3d errv_body;//, errv_global_;
	Eigen::Vector3d vq_cam, vq_global;
	Eigen::Vector3d uv_global, uw_global, uv_cam, uw_cam;
    float ux, uy, uz, uroll;
    float ux_trans, uy_trans;
    float box_x_center, box_y_center;
    float model_true_area, model_image_area;
    float depth_Z;
    float fov_x;
    float box_width, box_height;

	vq_cam << state(member_vqx), state(member_vqy), state(member_vqz);
	vq_global = rot_c2g*vq_cam;
	cout << "\ntarget_gvel\n" <<vq_global;

    box_x_center = fx*state(member_x1) + cx;
    box_y_center = fy*state(member_x1) + cy;
    fov_x = 2*atan(image_width/(2*fx));

    xt = state(member_x1);
    yt = state(member_x2);
    depth_Z = 1000/state(member_x3);       //meter to millimeter
    ROS_INFO("u:%.2f v: %.2f z:%.2f",box_x_center,box_y_center,depth_Z);

    erru = xt - (cx - cx)/fx;
    errv = yt - ((cy + 0.1*image_height) - cy)/fy;
	//define a = sqrt(model_true_area/(depth_Z^2)) = sqrt(model_image_area/(fx*fy))
	//err_a = a - a* = a - a*depth_Z/depth_Z*

    err_uy_cam = depth_Z*errv;
    err_uz_cam = -depth_Z*(1 - depth_Z/desired_distance);
    err_upitch_cam = (1/(xt*xt+1))*erru;
	errv_cam << 0, err_uy_cam, err_uz_cam;
    errw_cam << 0, err_upitch_cam, 0;
	errv_body << 0, ((rpy_mocap.yaw - desired_heading)*depth_Z*image_width/(fov_x*fx)/* - erru*depth_Z*/), 0;
	errv_cam = errv_cam + rot_b2c*errv_body;
	//errv_global = rot_c2g*errv_cam + rot_b2g*errv_body;
	//errw_global = rot_c2g*errw_cam;
	//errv_global_ = rot_b2g*errv_body;
	//errv_global = errv_global + 
	err_ux = errv_cam(0);// + 1000*target_gvel(0);
	err_uy = errv_cam(1);// + 1000*target_gvel(1);
	err_uz = errv_cam(2);// + 1000*target_gvel(2);
	err_uroll = errw_cam(1);

    err_ux_sum = err_ux_sum + err_ux;
    err_uy_sum = err_uy_sum + err_uy;
    err_uz_sum = err_uz_sum + err_uz;
    err_uroll_sum = err_uroll_sum + err_uroll;

    err_ux_dev = err_ux - err_ux_pre;
    err_uy_dev = err_uy - err_uy_pre;
    err_uz_dev = err_uz - err_uz_pre;
    err_uroll_dev = err_uroll - err_uroll_pre;

    err_ux_pre = err_ux;
    err_uy_pre = err_uy;
    err_uz_pre = err_uz;
    err_uroll_pre = err_uroll;

    ux = KPx_ibvs*err_ux + KPx_ibvs*de_time*err_ux_sum/Tix_ibvs + KPx_ibvs*Tdx_ibvs*err_ux_dev/de_time;
    uy = KPy_ibvs*err_uy + KPy_ibvs*de_time*err_uy_sum/Tiy_ibvs + KPy_ibvs*Tdy_ibvs*err_uy_dev/de_time;
    uz = KPz_ibvs*err_uz + KPz_ibvs*de_time*err_uz_sum/Tiz_ibvs + KPz_ibvs*Tdz_ibvs*err_uz_dev/de_time;
    uroll = KProll_ibvs*err_uroll + KProll_ibvs*de_time*err_uroll_sum/Tiroll_ibvs + KProll_ibvs*Tdroll_ibvs*err_uroll_dev/de_time;
    //if(abs(ux)>0.3)
    //ux = 0.3*ux/abs(ux);
	uv_cam << ux, uy, uz;
	uw_cam << 0, uroll, 0;
	uv_global = rot_c2g*uv_cam;
	uw_global = rot_c2g*uw_cam;


	ROS_INFO("ux: %.3f uy: %.3f uz: %.3f uroll: %.3f",uv_global(0),uv_global(1),uv_global(2),uw_global(2));
    vs->twist.linear.x = uv_global(0) + vq_global(0);//+ target_gvel(0);//_trans;
    vs->twist.linear.y = uv_global(1) + vq_global(1);//+ target_gvel(1);//_trans;
    vs->twist.linear.z = uv_global(2);// + vq_global(2);//+ target_gvel(2);
    vs->twist.angular.z = uw_global(2);
    ROS_INFO("vx: %.3f vy: %.3f vz: %.3f",vs->twist.linear.x,vs->twist.linear.y,vs->twist.linear.z);
    vir.x = host_mocap.pose.position.x;
    vir.y = host_mocap.pose.position.y;
    vir.z = host_mocap.pose.position.z;
}

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}




int main(int argc, char **argv)
{
    string topic_box;
    ros::init(argc, argv, "target_estimate_ibvs");
    ros::NodeHandle nh;
    ros::param::get("~topic_box", topic_box);
    ros::Subscriber host_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1,mocap_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",1,imu_cb);
    ros::Publisher measurement_pub = nh.advertise<ukf_estimate::output>("/measurement_data", 1);
    ros::Publisher estimate_pub = nh.advertise<ukf_estimate::output>("/estimate_data", 1);
    ros::Subscriber bb_box_sub = nh.subscribe<std_msgs::Float32MultiArray>(topic_box, 1, box_cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);

    ros::Rate rate(15);
    //ros::AsyncSpinner spinner(2);   //0 means use threads of CPU numbers
    //spinner.start();

    while (ros::ok() && current_state.connected) {
        mocap_pos_pub.publish(host_mocap);
        ros::spinOnce();
        rate.sleep();
    }

    initialize();
    global_vel.setZero(3);
    body_vel.setZero(3);
    camera_vel.setZero(3);
    global_wvel.setZero(3);
    body_wvel.setZero(3);
    camera_wvel.setZero(3);
    target_gvel.setZero(3);
    target_gwvel.setZero(3);
    target_g2cvel.setZero(3);
    target_g2cwvel.setZero(3);
    ros::Time current_time = ros::Time::now();
    ros::Time previous_time = ros::Time::now();
    geometry_msgs::TwistStamped vs;
    std_msgs::Float32MultiArray ukf_box;
    vir vir1;

    ros::param::get("~KPx_ibvs", KPx_ibvs);
    ros::param::get("~KPy_ibvs", KPy_ibvs);
    ros::param::get("~KPz_ibvs", KPz_ibvs);
    ros::param::get("~KProll_ibvs", KProll_ibvs);
    ros::param::get("~Tix_ibvs", Tix_ibvs);
    ros::param::get("~Tiy_ibvs", Tiy_ibvs);
    ros::param::get("~Tiz_ibvs", Tiz_ibvs);
    ros::param::get("~Tiroll_ibvs", Tiroll_ibvs);
    ros::param::get("~Tdx_ibvs", Tdx_ibvs);
    ros::param::get("~Tdy_ibvs", Tdy_ibvs);
    ros::param::get("~Tdz_ibvs", Tdz_ibvs);
    ros::param::get("~Tdroll_ibvs", Tdroll_ibvs);

    vs.twist.linear.x = 0;
    vs.twist.linear.y = 0;
    vs.twist.linear.z = 0;
    vs.twist.angular.x = 0;
    vs.twist.angular.y = 0;
    vs.twist.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(vs);
        mocap_pos_pub.publish(host_mocap);
        //ROS_INFO("position: %.3f, %.3f, %.3f", host_mocap.pose.position.x, host_mocap.pose.position.y, host_mocap.pose.position.z);
        vir1.x = 0;
        vir1.y = 7.5;
        vir1.z = 0.9;
        vir1.roll = desired_heading;
        callback_spin_count++;
        ros::spinOnce();
        rate.sleep();
    }




    //set initial value of state
    x(0) = 1;
    x(1) = 1;
    x(2) = 0.14;

    //increase the initial value of P can increase the speed of convergence
    Eigen::MatrixXd P_init;
    P_init.setZero(statesize,statesize);
    ros::param::get("~P_init_0", P_init(0,0));
    ros::param::get("~P_init_1", P_init(1,1));
    ros::param::get("~P_init_2", P_init(2,2));
    ros::param::get("~P_init_3", P_init(3,3));
    ros::param::get("~P_init_4", P_init(4,4));
    ros::param::get("~P_init_5", P_init(5,5));
    ros::param::get("~P_init_6", P_init(6,6));
    ros::param::get("~P_init_7", P_init(7,7));
    ros::param::get("~P_init_8", P_init(8,8));
    P = P_init;             //set initial P matrix



    Eigen::MatrixXd measurement_noise;
    measurement_noise.setZero(measurementsize,measurementsize);
    measurement_noise = 1* Eigen::MatrixXd::Identity(measurementsize,measurementsize);
    ros::param::get("~measurement_noise_0", measurement_noise(0,0));
    ros::param::get("~measurement_noise_1", measurement_noise(1,1));
    ros::param::get("~measurement_noise_2", measurement_noise(2,2));
    ros::param::get("~measurement_noise_3", measurement_noise(3,3));
    ros::param::get("~measurement_noise_4", measurement_noise(4,4));
    ros::param::get("~measurement_noise_5", measurement_noise(5,5));
    R = measurement_noise;             //set measurement noise

    Eigen::MatrixXd process_noise;
    process_noise.setZero(statesize,statesize);
    process_noise = 1* Eigen::MatrixXd::Identity(statesize,statesize);
    ros::param::get("~process_noise_0", process_noise(0,0));
    ros::param::get("~process_noise_1", process_noise(1,1));
    ros::param::get("~process_noise_2", process_noise(2,2));
    ros::param::get("~process_noise_3", process_noise(3,3));
    ros::param::get("~process_noise_4", process_noise(4,4));
    ros::param::get("~process_noise_5", process_noise(5,5));
    ros::param::get("~process_noise_6", process_noise(6,6));
    ros::param::get("~process_noise_7", process_noise(7,7));
    ros::param::get("~process_noise_8", process_noise(8,8));
    Q = process_noise;              //set process noise

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    while(ros::ok()){

        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

		if(!isnormal(x(0)))
		{
			initialize();
			x(0) = 1;
			x(1) = 1;
			x(2) = 0.14;
			P = P_init;
			R = measurement_noise;
			Q = process_noise;
			callback_spin_count = 100;
		}

        ukf_estimate::output measure_value, estimate_value;
        float center_u, center_v, box_area;
        rpy_mocap = quaternionToRPY(host_mocap.pose.orientation.x,host_mocap.pose.orientation.y,host_mocap.pose.orientation.z,host_mocap.pose.orientation.w);

        global_vel << host_mocap_vel.linear.x, host_mocap_vel.linear.y, host_mocap_vel.linear.z;
        global_wvel << host_mocap_vel.angular.x, host_mocap_vel.angular.y, host_mocap_vel.angular.z;
        target_gvel << car_vel.linear.x, car_vel.linear.y, car_vel.linear.z;
        target_gwvel << car_vel.angular.x, car_vel.angular.y, car_vel.angular.z;

        rotation_x.setZero(3,3);
        rotation_y.setZero(3,3);
        rotation_z.setZero(3,3);

        rotationB2C_x.setZero(3,3);
        rotationB2C_y.setZero(3,3);
        rotationB2C_z.setZero(3,3);

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

        rot_c2g.setZero(3,3);
		rot_b2g.setZero(3,3);
		rot_b2c.setZero(3,3);
        rot_c2g = (rotationB2C_z*rotationB2C_y*rotation_x*rotation_y*rotation_z).transpose();
		rot_b2g = (rotation_x*rotation_y*rotation_z).transpose();
		rot_b2c = (rotationB2C_z*rotationB2C_y);
        body_vel = rotation_x*rotation_y*rotation_z*global_vel;
        body_wvel = rotation_x*rotation_y*rotation_z*global_wvel;
        camera_vel = rotationB2C_z*rotationB2C_y*body_vel;
        camera_wvel = rotationB2C_z*rotationB2C_y*body_wvel;

        target_g2cvel = rotationB2C_z*rotationB2C_y*rotation_x*rotation_y*rotation_z*target_gvel;
        target_g2cwvel = rotationB2C_z*rotationB2C_y*rotation_x*rotation_y*rotation_z*target_gwvel;

        current_time = ros::Time::now();
        dt = current_time.toSec() - previous_time.toSec();
        previous_time = current_time;

        predict();

        //save measurement data to matrix for correct()
        Eigen::VectorXd measure_vector;
        measure_vector.setZero(measurementsize);
        center_u = box.data[2]*image_width;
        center_v = box.data[1]*image_height;
        box_area = abs(box.data[3]*image_height*box.data[4]*image_width);
        measure_vector<<center_u, center_v, box_area, host_mocap.pose.position.x, host_mocap.pose.position.y, host_mocap.pose.position.z;
        ROS_INFO("u:%.3f v:%.3f a:%.3f", center_u, center_v, box_area);
        //ROS_INFO("mx1:%.3f mx2:%.3f mz:%.3f", (center_u - cx)/fx, (center_v - cy)/fy, 1/sqrt(box_area/(model_height*model_width*fx*fy)));

        //execute correct if the target is detected
        if(box.data[0] != -1)
            correct(measure_vector);

        noise_estimate(20);
        if(Q(6,6)<0.01)
            Q(6,6) = 0.01;
        if(Q(7,7)<0.005)
            Q(7,7) = 0.005;
        if(Q(8,8)<0.0001)
            Q(8,8) = 0.0001;

        int c = getch();
        //ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                vir1.z += 0.3;
                break;
            case 66:    // key down
                vir1.z += -0.3;
                break;
            case 67:    // key CW(->)
                vir1.roll -= 0.3;
                break;
            case 68:    // key CCW(<-)
                vir1.roll += 0.3;
                break;
            case 119:    // key foward
                vir1.x += 0.3;
                break;
            case 120:    // key back
                vir1.x += -0.3;
                break;
            case 97:    // key left
                vir1.y += 0.3;
                break;
            case 100:    // key right
                vir1.y -= 0.3;
                break;
            case 115:    // key origin
            {
                //vir1.x = 0.0;
                //vir1.y = 0.0;
                vir1.z = 0;
                vir1.roll = desired_heading;
                break;
            }
            case 49:    // virtual leader mode
            {
                ibvs_mode = false;
                break;
            }
            case 50:    // ibvs mode
            {
                ibvs_mode = true;
                break;
            }
            case 51:    // ukf mode
            {
                if(ukf_mode == false)
                    ukf_mode = true;
                else
                  ukf_mode = false;

                break;
            }
            case 108:    // close arming
            {
                offb_set_mode.request.custom_mode = "MANUAL";
                set_mode_client.call(offb_set_mode);
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
                break;
            }
            case 63:
                return 0;
                break;
            }
        }
        if(vir1.roll>pi)
            vir1.roll = vir1.roll - 2*pi;
        else if(vir1.roll<-pi)
            vir1.roll = vir1.roll + 2*pi;


        if(box.data[1] == -1 && box.data[2] == -1 && box.data[3] == -1 && box.data[4] == -1 || ibvs_mode == false)
        {
            ROS_INFO("position: %.3f, %.3f, %.3f", host_mocap.pose.position.x, host_mocap.pose.position.y, host_mocap.pose.position.z);
            ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z, vir1.roll/pi*180);

            follow(vir1,host_mocap,&vs,0,0);
            if(box.data[1] != -1 || box.data[2] != -1 || box.data[3] != -1 || box.data[4] != -1)
                ROS_INFO("target detected");
        }
        else
        {
            if(ukf_mode == false)
            {
                vir1.x = 0;
                vir1.y = 7.5;
                vir1.z = 0.7;
                follow(vir1,host_mocap,&vs,0,0);
                ibvs(vir1, box, host_mocap, &vs, dt);
            }
            else
            {
				ROS_INFO("ukf mode");
                vir1.x = 0;
                vir1.y = 7.5;
                vir1.z = 0.7;
                follow(vir1,host_mocap,&vs,0,0);
                ibvs_ukf(vir1, x, host_mocap, &vs, dt);
            }
        }
        //ROS_INFO("drone: x:%.3f  y:%.3f  z:%.3f", host_mocap.pose.position.x, host_mocap.pose.position.y, host_mocap.pose.position.z);
        //ROS_INFO("car:   x:%.3f  y:%.3f  z:%.3f", car_pose.pose.position.x, car_pose.pose.position.y, car_pose.pose.position.z);

        measure_value.feature.x = (host_mocap.pose.position.x - car_pose.pose.position.x)/(host_mocap.pose.position.y - car_pose.pose.position.y);
        measure_value.feature.y = (host_mocap.pose.position.z - car_pose.pose.position.z)/(host_mocap.pose.position.y - car_pose.pose.position.y);
        measure_value.feature.z = (host_mocap.pose.position.y - car_pose.pose.position.y);
        measure_value.target_pose.x = car_pose.pose.position.x + 1.3827;
        measure_value.target_pose.y = car_pose.pose.position.y + 1.03453;
        measure_value.target_pose.z = car_pose.pose.position.z + 0.75;
        measure_value.target_vel.x = target_g2cvel(0);
        measure_value.target_vel.y = target_g2cvel(1);
        measure_value.target_vel.z = target_g2cvel(2);
        estimate_value.feature.x = x(0);
        estimate_value.feature.y = x(1);
        estimate_value.feature.z = (1/x(2));                            //depth = 1/x(2)
        estimate_value.target_pose.x = x(3);
        estimate_value.target_pose.y = x(4);
        estimate_value.target_pose.z = x(5);
        estimate_value.target_vel.x = x(6);
        estimate_value.target_vel.y = x(7);
        estimate_value.target_vel.z = x(8);


        measurement_pub.publish(measure_value);
        estimate_pub.publish(estimate_value);
        mocap_pos_pub.publish(host_mocap);
        local_vel_pub.publish(vs);
        ros::spinOnce();
        rate.sleep();


    }
    return 0;
}
