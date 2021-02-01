#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

/*-------------------------Functions----------------------*/
float c(float x);		//-------Short cosine function
float s(float x);		//-------Short sine function
float sat(float _x,float val);	//--Saturation function
float db(float _x,float val);	//--Dead band for control
MatrixXf hat(float _wx, float _wy, float _wz); //--Hat operator
Matrix3d QuatToMat(Vector4d Quat);//--Quaternion to matrix
Vector3d R2XYZ(Matrix3d R);//--Rotation to XYZ angles
void save();			//-------Saving data function
/*-------------------------Functions----------------------*/

/*--------------------------Constants-----------------------*/
float pi = 3.1416;		//-------pi
float e  = 0.1;//float e  = 0.05;	    //-------epsilon
int k = 0;
/*--------------------------Constants-----------------------*/


/*--------------------Observer variables------------------*/
float x11 = 0.0, x12 = 0.0, x11g = 0.0, x12g = 0.0, x13g = 0.0, dx11g= 0.0, dx12g= 0.0, dx13g= 0.0;
float x21 = 0.0, x22 = 0.0, x21g = 0.0, x22g = 0.0, x23g = 0.0, dx21g= 0.0, dx22g= 0.0, dx23g= 0.0;
float x31 = 0.0, x32 = 0.0, x31g = 0.0, x32g = 0.0, x33g = 0.0, dx31g= 0.0, dx32g= 0.0, dx33g= 0.0;
float x41 = 0.0, x42 = 0.0, x41g = 0.0, x42g = 0.0, x43g = 0.0, dx41g= 0.0, dx42g= 0.0, dx43g= 0.0;
float x51 = 0.0, x52 = 0.0, x51g = 0.0, x52g = 0.0, x53g = 0.0, dx51g= 0.0, dx52g= 0.0, dx53g= 0.0;
float x61 = 0.0, x62 = 0.0, x61g = 0.0, x62g = 0.0, x63g = 0.0, dx61g= 0.0, dx62g= 0.0, dx63g= 0.0;

Vector3d _orientation;
/*--------------------Observer variables------------------*/

/*---------------------ROS variables---------------------*/
void odometry_callback(const nav_msgs::Odometry odometry_msg);
void control_callback(const std_msgs::Float32MultiArray control_msg);
std_msgs::Float32MultiArray u;
std_msgs::Float32MultiArray x3g;
nav_msgs::Odometry odom;
/*---------------------ROS variables---------------------*/

int main(int argc, char **argv)
{
	/*------ROS Setup-------*/
	ros::init(argc, argv,"robust_observer");
	ros::NodeHandle nh;
	
	ros::Publisher observer_publisher = nh.advertise<std_msgs::Float32MultiArray>("disturbances",0);
	
	ros::Subscriber feedback = nh.subscribe("/firefly_tilt/odometry", 1, odometry_callback);
	ros::Subscriber control  = nh.subscribe("/control_signals", 1, control_callback);
	cout<<"ROS OK!"<<endl;
	ros::Rate loop_rate(1000);
	u.data.resize(6);
	x3g.data.resize(6);
	/*---Initialization----*/
	//Rd.setZero(); R.setZero(); wRb.setZero();
	/*------ROS Loop-------*/
	while (ros::ok())
	{
		cout<<"-------------------------------------"<<endl;
		/*-----Feedback-----------*/
		x11 =  odom.pose.pose.position.x;
		x21 = -odom.pose.pose.position.y;
		x31 = -odom.pose.pose.position.z;
		
		x12 =  odom.twist.twist.linear.x;
		x22 = -odom.twist.twist.linear.y;
		x32 = -odom.twist.twist.linear.z;
		
		_orientation = R2XYZ( QuatToMat ( Vector4d( odom.pose.pose.orientation.w,  odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, -odom.pose.pose.orientation.z ) ) );
		
		x41 = _orientation[0];
		x51 = _orientation[1];
		x61 = _orientation[2];
		
		/*----Airframe velocities---*/
		x42 = odom.twist.twist.angular.x;
		x52 = odom.twist.twist.angular.y;
		x62 = -odom.twist.twist.angular.z;
		/*----Robust Observer---*/
		
		/*----------X-----------*/
		dx11g = x12g + 1*e*( x11-x11g + tanh( 100*(x11-x11g) ) )/pow(1*e,2);
		dx12g = x13g +   ( x11-x11g + tanh( 100*(x11-x11g) ) )/pow(1*e,2) + u.data[0];
		dx13g =    (1/(1*e))*( x11-x11g + tanh( 100*(x11-x11g) ) )/pow(1*e,2);
		
		x11g = x11g + 0.01*dx11g;
		x12g = x12g + 0.001*dx12g;
		x13g = x13g + 0.0001*dx13g;
		
		/*----------Y-----------*/
		dx21g = x22g + 1*e*( x21-x21g + tanh( 100*(x21-x21g) ) )/pow(1*e,2);
		dx22g = x23g +   ( x21-x21g + tanh( 100*(x21-x21g) ) )/pow(1*e,2) + u.data[1];
		dx23g =    (1/(1*e))*( x21-x21g + tanh( 100*(x21-x21g) ) )/pow(1*e,2);
		
		x21g = x21g + 0.01*dx21g;
		x22g = x22g + 0.001*dx22g;
		x23g = x23g + 0.0001*dx23g;
		
		/*----------Z-----------*/
		dx31g = x32g + e*( x31-x31g + tanh( 100*(x31-x31g) ) )/pow(e,2);
		dx32g = x33g +   ( x31-x31g + tanh( 100*(x31-x31g) ) )/pow(e,2) + u.data[2];
		dx33g =    (1/e)*( x31-x31g + tanh( 100*(x31-x31g) ) )/pow(e,2);
		
		x31g = x31g + 0.01*dx31g;
		x32g = x32g + 0.001*dx32g;
		x33g = x33g + 0.0001*dx33g;
		
		/*--------ROLL----------*/
		dx41g = x42g + 0.1*e*( x41-x41g + tanh( 100*(x41-x41g) ) )/pow(0.1*e,2);
		dx42g = x43g +   ( x41-x41g + tanh( 100*(x41-x41g) ) )/pow(0.1*e,2) + u.data[3];
		dx43g =    (1/(0.1*e))*( x41-x41g + tanh( 100*(x41-x41g) ) )/pow(0.1*e,2);
		
		x41g = x41g + 0.001*dx41g;
		x42g = x42g + 0.0001*dx42g;
		x43g = x43g + 0.00001*dx43g;
		
		/*--------PITCH----------*/
		dx51g = x52g + 0.1*e*( x51-x51g + tanh( 100*(x51-x51g) ) )/pow(0.1*e,2);
		dx52g = x53g +   ( x51-x51g + tanh( 100*(x51-x51g) ) )/pow(0.1*e,2) + u.data[4];
		dx53g =    (1/(0.1*e))*( x51-x51g + tanh( 100*(x51-x51g) ) )/pow(0.1*e,2);
		
		x51g = x51g + 0.001*dx51g;
		x52g = x52g + 0.0001*dx52g;
		x53g = x53g + 0.00001*dx53g;
		
		/*--------YAW-----------*/
		dx61g = x62g + 0.1*e*( x61-x61g + tanh( 100*(x61-x61g) ) )/pow(0.1*e,2);
		dx62g = x63g +   ( x61-x61g + tanh( 100*(x61-x61g) ) )/pow(0.1*e,2) + u.data[5];
		dx63g =    (1/(0.1*e))*( x61-x61g + tanh( 100*(x61-x61g) ) )/pow(0.1*e,2);
		
		x61g = x61g + 0.001*dx61g;
		x62g = x62g + 0.0001*dx62g;
		x63g = x63g + 0.00001*dx63g;
		
		/*---Observer Pubilsher--*/
		x3g.data[0] = x13g; x3g.data[1] = x23g; x3g.data[2] = x33g;
		x3g.data[3] = x43g; x3g.data[4] = x53g; x3g.data[5] = x63g;
		
		observer_publisher.publish(x3g);
		
		/*-----Control info-----*/
		cout<<"x3g = \n"<<x3g<<endl;
		cout<<"\n-------------"<<endl;
		
		ros::spinOnce();
		loop_rate.sleep();
		k++;
		if(k==100)
		{
			//save();
			k = 0;
		}
		
	}
	return 0;
}

void odometry_callback(const nav_msgs::Odometry odometry_msg) {
    odom = odometry_msg;
}

void control_callback(const std_msgs::Float32MultiArray control_msg) {
    u = control_msg;    
}

void save(){
	ofstream myfile;
	myfile.open ("resultsSMO.txt",std::ios::app);
	myfile <<x11<<","<<x11g<<","<<x12g<<","<<x13g<<","<<x21<<","<<x21g<<","<<x22g<<","<<x23g<< ","<<x31<<","<<x31g<<","<<x32g<<","<<x33g<<","<<x41<<","<<x41g<<","<<x42g<<","<<x43g<< ","<<x51<<","<<x51g<<","<<x52g<<","<<x53g<<","<<x61<<","<<x61g<<","<<x62g<<","<<x63g<<endl;
	//myfile <<x21<<","<<x21g<<","<<x22g<<","<<x23g<<endl;
	//myfile <<x31<<","<<x31g<<","<<x32g<<","<<x33g<<endl;
	//myfile <<x41<<","<<x41g<<","<<x42g<<","<<x43g<<endl;
	//myfile <<x51<<","<<x51g<<","<<x52g<<","<<x53g<<endl;
	//myfile <<x61<<","<<x61g<<","<<x62g<<","<<x63g<<endl;
	myfile.close();
}

float c(float _x){
	return cos(_x);
}

float s(float _x){
	return sin(_x);
}

MatrixXf hat(float _wx, float _wy, float _wz){
	MatrixXf _hat(3,3);
	_hat(0,0) =   0 ; _hat(0,1) = -_wz; _hat(0,2) =  _wy;
	_hat(1,0) =  _wz; _hat(1,1) =   0 ; _hat(1,2) = -_wx;
	_hat(2,0) = -_wy; _hat(2,1) =  _wx; _hat(2,2) =   0 ;
	return _hat;
}

Matrix3d QuatToMat(Vector4d Quat){
    Matrix3d Rot;
    float _s = Quat[0];
    float _x = Quat[1];
    float _y = Quat[2];
    float _z = Quat[3];
    Rot << 1-2*(_y*_y+_z*_z),2*(_x*_y-_s*_z),2*(_x*_z+_s*_y),
    2*(_x*_y+_s*_z),1-2*(_x*_x+_z*_z),2*(_y*_z-_s*_x),
    2*(_x*_z-_s*_y),2*(_y*_z+_s*_x),1-2*(_x*_x+_y*_y);
    return Rot;
}

Vector3d R2XYZ(Matrix3d R) {
    double _phi=0.0, _theta=0.0, _psi=0.0;
    Vector3d XYZ = Vector3d::Zero();
    
    _theta = asin(R(0,2));
    
    if(fabsf(cos(_theta))>pow(10.0,-10.0))
    {
        _phi=atan2(-R(1,2)/cos(_theta), R(2,2)/cos(_theta));
        _psi=atan2(-R(0,1)/cos(_theta), R(0,0)/cos(_theta));
    }
    else
    {
        if(fabsf(_theta-pi/2.0)<pow(10.0,-5.0))
        {
            _psi = 0.0;
            _phi = atan2(R(1,0), R(2,0));
            _theta = pi/2.0;
        }
        else
        {
            _psi = 0.0;
            _phi = atan2(-R(1,0), R(2,0));
            _theta = -pi/2.0;
        }
    }
    
    XYZ << _phi,_theta,_psi;
    return XYZ;
}


		/*T = J*( -9*eR-3*eO ) +w.cross(Jw)-J*( hat( w(0),w(1),w(2) )*R.transpose()*Rd*wd - R.transpose()*Rd*dwd );
		tx = 15*(phid-phi)+13*(dphid-dphi)+40*tanh( 10*( 30*(phid-phi)+(dphid-dphi) ) );
		ty = 15*(thed-the)+13*(dthed-dthe)+40*tanh( 10*( 30*(thed-the)+(dthed-dthe) ) );
		tz = 15*(psid-psi)+13*(dpsid-dpsi)+60*tanh( 10*( 30*(psid-psi)+(dpsid-dpsi) ) );
		fx = 0.05*(xd-x)+0.03*(dxd-dx)+0.1*tanh( 10*( 30*(xd-x)+(dxd-dx) ) );
		fy = 0.05*(yd-y)+0.03*(dyd-dy)+0.1*tanh( 10*( 30*(yd-y)+(dyd-dy) ) );
		fz = 15*(zd-z)+13*(dzd-dz)+20*tanh( 10*( 10*(zd-z)+(dzd-dz) ) );*/
		
