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
#include <list>
#include <deque>
#include <numeric>
#define pi 3.14159265359

using namespace std;
int callback_spin_count = 0;

////////////////////Global variable////////////////
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
    pos=0,
    velocity,
    acc,
    statesize
};

enum measurement{
    mpos = 0,
    //mvel,
    macc,
    measurementsize
};
////////////////////////////////////////////////////////////////


////////////////////dynamic(predict state)////////////////
Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state){

    x_size = statesize;
    x_sigmavector_size = 2*x_size+1;
    Eigen::MatrixXd predict_sigma_state(x_size,x_sigmavector_size);

    for(int i=0;i<x_sigmavector_size;i++){

        double p = sigma_state(pos,i) ;
        double v = sigma_state(velocity,i);
        double a = sigma_state(acc,i);
        double p_ ;
        double v_ ;
        double a_ ;

        p_ = p+v*dt;
        v_ = v+a*dt;
        a_ = a;

        predict_sigma_state(pos,i) =  p_ ;
        predict_sigma_state(velocity,i) =  v_;
        predict_sigma_state(acc,i) =  a_;

    }
    return predict_sigma_state;


}
///////////////////////////////////////////////////////


////////////////////predict measurement////////////////
Eigen::MatrixXd state_to_measure(Eigen::MatrixXd sigma_state){

    y_size = measurementsize;
    x_sigmavector_size = 2*x_size+1;
    Eigen::MatrixXd predict_sigma_measure(y_size,x_sigmavector_size);

    for(int i=0;i<x_sigmavector_size;i++){

        predict_sigma_measure( mpos ,i) =   sigma_state(pos,i);
        //predict_sigma_measure( mvel ,i) =  sigma_state(velocity,i);
        predict_sigma_measure( macc ,i) =   sigma_state(acc,i);

    }
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
    Eigen::VectorXd buffer;
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
    ROS_INFO("estimate noise");

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
    delta_S = (S_gain*((P_yy_ + R).trace()))/(((y - y_hat).transpose())*(y - y_hat));
    w_element = sqrt(((x - x_hat).transpose())*(x - x_hat))*sqrt(((y_hat - y).transpose())*(y_hat - y))*delta_S;

    q_window.push_back(q_window_element);
    r_window.push_back(r_window_element);
    Q_window.push_back(Q_window_element);
    R_window.push_back(R_window_element);
    w_window.push_back(w_element);


    if(q_window.size()>window_size || r_window.size()>window_size || Q_window.size()>window_size || R_window.size()>window_size)
    {
        //ROS_INFO("estimate noise");
        q_window.pop_front();
        r_window.pop_front();
        Q_window.pop_front();
        R_window.pop_front();
        w_window.pop_front();
    }


    if(q_window.size()==window_size || r_window.size()==window_size || Q_window.size()==window_size || R_window.size()==window_size)
    {
        ROS_INFO("estimate noise");

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
        if(callback_spin_count>200)
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
    cout << "\nQsum\n" <<Q_window_sum<<"\n";
    //cout << "\nR\n" <<R_window_element<<"\n";
    //cout << "\nR" <<R_window_element<<"\n";
    cout << "\nRsum" <<R_window_sum<<"\n";
    //cout << "\nRsum\n" <<R_window_sum/window_size<<"\n";
    callback_spin_count++;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_estimate");
    ros::NodeHandle nh;

    ros::Publisher true_pub = nh.advertise<geometry_msgs::Point>("/true_data", 10);
    ros::Publisher measurement_pub = nh.advertise<geometry_msgs::Point>("/measurement_data", 10);
    ros::Publisher estimate_pub = nh.advertise<geometry_msgs::Point>("/estimate_data", 10);
    ros::Rate rate(50);


    initialize();

    //test data
    dt =0.02;               //set dt
    double T=0.0;
    double pos = 0.0;
    double velocity = 0.0 ;
    double acc = 0.0;
    //double pre_pos = 0;
    double mpos=0.0;
    double mvelocity=0.0;
    double macc=0.0;

    //increase the initial value of P can increase the speed of convergence
    Eigen::MatrixXd P_init;
    P_init.setZero(statesize,statesize);
    P_init(0,0) = 1;
    P_init(1,1) = 1000;
    P_init(2,2) = 1;
    P = P_init;             //set initial P matrix



    Eigen::MatrixXd noise;
    noise.setZero(measurementsize,measurementsize);
    noise = 1* Eigen::MatrixXd::Identity(measurementsize,measurementsize);
    noise(0,0) = 0.5;
    noise(1,1) = 50;
    R = noise;             //set measurement noise


    noise.setZero(statesize,statesize);
    noise = 1* Eigen::MatrixXd::Identity(statesize,statesize);
    noise(0,0) = 1;
    noise(1,1) = 1;
    noise(2,2) = 1;
    Q = noise;              //set process noise

    while(ros::ok()){

        geometry_msgs::Point true_value, measure_value, estimate_value;

        T+=dt;
        //pre_pos = pos;
        pos = sin(5*T);
        velocity = 5*cos(5*T);
        //velocity = (pos-pre_pos)/dt;
        acc = -25*sin(5*T);



        mpos = pos+ (rand()%100-50)*0.01;
        mvelocity = velocity + (rand()%100-50)*0.01;
        macc = acc + (rand()%100-50)*0.5;



        predict();

        //save measurement data to matrix for correct()
        Eigen::VectorXd measure_vector;
        measure_vector.setZero(measurementsize);
        measure_vector<<mpos, macc;// , mvelocity  ;

        correct(measure_vector);

        noise_estimate(50);


        true_value.x = pos;
        true_value.y = velocity;
        true_value.z = acc;
        measure_value.x = mpos;
        measure_value.y = mvelocity;
        measure_value.z = macc;
        estimate_value.x = x(0);
        estimate_value.y = x(1);
        estimate_value.z = x(2);


        true_pub.publish(true_value);
        measurement_pub.publish(measure_value);
        estimate_pub.publish(estimate_value);
        ros::spinOnce();
        rate.sleep();


    }
}

