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
Matrix3d QuatToMat(Vector4d Quat);//--Quaternion to matrix
Vector3d R2XYZ(Matrix3d R);//--Rotation to XYZ angles
void save();			//-------Saving data function
/*-------------------------Functions----------------------*/

/*--------------------------Matrices----------------------*/
MatrixXf wRb(6,6); 		//-------6D rotation of airframe respect to the world
Matrix3f R(3,3);   		//-------3D rotation of airframe respect to the world
Matrix3f dR(3,3); 		//-------3D  derivative of rotation R
Matrix3f Rd(3,3); 		//-------Desired orientation matrix
Matrix3f dRd(3,3);		//-------Derivative of the desired orientation matrix
MatrixXf F1(float a);	//-------Translational allocation
MatrixXf F2(float a);	//-------Rotational allocation
MatrixXf hat(float _wx, float _wy, float _wz); //--Hat operator
MatrixXf F(6,6);		//-------6D allocation
MatrixXf M(6,6);		//-------6D Inertial matrix
MatrixXf G(6,6);		//-------Gamma = M*wRb
MatrixXf invF(6,6);		//-------inverse of 6D allocation
MatrixXf invM(6,6);		//-------inverse of 6D Inertial matrix
MatrixXf invG(6,6);		//-------inverse of Gamma
MatrixXf J(3,3);		//-------Inertia tensor
/*--------------------------Matrices----------------------*/

/*--------------------------Vectors-----------------------*/
Vector3f eR;			//-------Orientation error
Vector3f eO;			//-------Orientation velocity error
VectorXf v(6);			//-------Linear regulator
VectorXf u(6);			//-------Nonlinear control
VectorXf b(6);			//-------Non inertial dynamics
VectorXf f(6);			//-------Allocation forces
Vector3f w;				//-------Angular velocities
Vector3f wd;			//-------Desired angular velocities
Vector3f Jw;			//-------J*w
float pi = 3.1416;		//-------pi
/*--------------------------Vectors-----------------------*/

/*------------------Parameters of dynamics---------------*/
float g = 9.81, m = 1.5;
float Ixx = 0.0347563, Iyy = 0.0458929, Izz = 0.0977;
float kf = 8.54858e-6, km = 1.6e-2, d = km/kf;
float l = 0.215;
//float tilt_angle = 0.1*pi/180.0;//no tilt
float tilt_angle = 20*pi/180.0;
/*------------------Parameters of dynamics---------------*/


/*--------------------Control variables------------------*/
float x=0.0,y=0.0,z=0.0,dx=0.0,dy=0.0,dz=0.0,psi=0.0,the=0.0,phi=0.0,dphi=0.0,dthe=0.0,dpsi=0.0;
float fx,fy,fz,tx,ty,tz;

float ex,ey,ez,ephi,ethe,epsi;
float dex,dey,dez,dephi,dethe,depsi;

Vector3d _orientation;
/*--------------------Control variables------------------*/

/*--------------------References------------------*/
float xd=1.0,yd=-1.0,zd=3.0;
float phid=0.0*pi/180.0,thed=0.0*pi/180.0,psid= -30.0*pi/180.0;

float dxd=0.0,dyd=0.0,dzd=0.0;
float dphid=0.0,dthed=0.0,dpsid=0.0;
/*--------------------References------------------*/

/*---------------------ROS variables---------------------*/
void odometry_callback(const nav_msgs::Odometry odometry_msg);
std_msgs::Float64 t_msg0;
std_msgs::Float64 t_msg1;
std_msgs::Float64 t_msg2;
std_msgs::Float64 t_msg3;
std_msgs::Float64 t_msg4;
std_msgs::Float64 t_msg5;
std_msgs::Float32MultiArray motor_vel;
nav_msgs::Odometry odom;
/*---------------------ROS variables---------------------*/

int main(int argc, char **argv)
{
	/*------ROS Setup-------*/
	ros::init(argc, argv,"simple_controller");
	ros::NodeHandle nh;
	
	ros::Publisher rotors_publisher = nh.advertise<std_msgs::Float32MultiArray>("/firefly_tilt/cmd/motor_vel",0);
	
	ros::Publisher tilt_0_publisher = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_0_joint_controller/command",0);
	ros::Publisher tilt_1_publisher = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_1_joint_controller/command",0);
	ros::Publisher tilt_2_publisher = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_2_joint_controller/command",0);
	ros::Publisher tilt_3_publisher = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_3_joint_controller/command",0);
	ros::Publisher tilt_4_publisher = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_4_joint_controller/command",0);
	ros::Publisher tilt_5_publisher = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_5_joint_controller/command",0);

	ros::Subscriber feedback = nh.subscribe("/firefly_tilt/odometry", 1, odometry_callback);
	
	motor_vel.data.resize(6);
	ros::Rate loop_rate(10);
	/*---Initialization----*/
	Rd.setZero(); R.setZero(); M.setZero(); J.setZero(); wRb.setZero();
	/*------ROS Loop-------*/
	while (ros::ok())
	{
		cout<<"-------------------------------------"<<endl;
		/*-----Feedback-----------*/
		x =  odom.pose.pose.position.x;
		y = -odom.pose.pose.position.y;
		z = -odom.pose.pose.position.z;
		
		dx =  odom.twist.twist.linear.x;
		dy = -odom.twist.twist.linear.y;
		dz = -odom.twist.twist.linear.z;
		
		_orientation = R2XYZ( QuatToMat ( Vector4d( odom.pose.pose.orientation.w,  odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, -odom.pose.pose.orientation.z ) ) );
		
		phi = _orientation[0];
		the = _orientation[1];
		psi = _orientation[2];
		
		/*----Airframe velocities---*/
		dphi = odom.twist.twist.angular.x;
		dthe = odom.twist.twist.angular.y;
		dpsi = -odom.twist.twist.angular.z;
		
		w<<dphi,dthe,dpsi;
		wd<<dphid,dthed,dpsid;
		Vector3f dwd(0,0,0);
			
		/*-----Error signals--------*/
		ex  = xd - x;   ey  = yd - y;   ez  = zd - z;
		dex = dxd - dx; dey = dyd - dy; dez = dzd - dz;
		ephi  = phid - phi;   ethe  = thed  - the;  epsi  = psid  - psi;
		dephi = dphid - dphi; dethe = dthed - dthe; depsi = dpsid - dpsi;
		
		/*-----Control-----------*/
		/*--------WRB--------*/
		wRb(0,0) = c(psi)*c(the); wRb(0,1) = c(psi)*s(the)*s(phi)-s(psi)*c(phi); wRb(0,2) = c(psi)*s(the)*c(phi)+s(psi)*s(phi);
		wRb(1,0) = s(psi)*c(the); wRb(1,1) = s(psi)*s(the)*s(phi)+c(psi)*c(phi); wRb(1,2) = s(psi)*s(the)*c(phi)-c(psi)*s(phi);
		wRb(2,0) =    -s(the)   ; wRb(2,1) =             c(the)*s(phi)         ; wRb(2,2) =            c(the)*c(phi);
		wRb(3,3) = 1 ; wRb(4,4) = 1 ; wRb(5,5) = 1 ;
		/*------Inertial matrices--------*/
		M(0,0) = m; M(1,1) = m; M(2,2) = m; M(3,3) = Ixx; M(4,4) = Iyy; M(5,5) = Izz;
		J(0,0) = Ixx; J(1,1) = Iyy; J(2,2) = Izz;
		invM = M.inverse();
		/*------Gamma------*/
		G = M.inverse()*wRb;
		invG = G.inverse();
		/*-----Angular velocities------*/
		Jw = J*w;
		/*-----Airframe dynamics------*/
		//b<<0,0,g,w.cross(Jw);
		b<<0,0,g,0,0,0;
		/*------Stabilizer------------*/
		fx =  sat( m*( 0.02*db(ex,5e-1) + 0.015*db(dex,5e-1) ),0.1 );
		fy = -sat( m*( 0.02*db(ey,5e-1) + 0.015*db(dey,5e-1) ),0.1 );
		fz = (1.15*m)*( 5.0*db(ez,1e-2) + 4.0*db(dez,1e-2)+g )/wRb(2,2);
		//fz = (1.15*m)*( 10.0*db(ez,1e-2) + 8.0*db(dez,1e-2)+g )/wRb(2,2);//non tilted
		//tx = -sat( Ixx*( 80.0*db(ephi,5e-3) + 70.0*db(dephi,1e-2) ),1 );//non tilted
		//ty = -sat( Iyy*( 80.0*db(ethe,5e-3) + 70.0*db(dethe,1e-2) ),1 );//non tilted
		tx = -sat( Ixx*( 10000.0*db(ephi,5e-3) + 8000.0*db(dephi,1e-2) ),100 );
		ty = -sat( Iyy*( 10000.0*db(ethe,5e-3) + 8000.0*db(dethe,1e-2) ),100 );
		tz =  sat( 1000.0*db(epsi,1e-2) + 800.0*db(depsi,1e-2), 1000);
		v<<fx,fy,fz,tx,ty,tz;
		u = v;//+b;
		/*-----Allocation------------*/
		F<<F1(tilt_angle),F2(tilt_angle);
		invF = F.inverse();
		f = invF*u;
		
		/*-----Tilting----------*/
		t_msg0.data = tilt_angle;
		t_msg1.data = -tilt_angle;
		t_msg2.data = tilt_angle;
		t_msg3.data = -tilt_angle;
		t_msg4.data = tilt_angle;
		t_msg5.data = -tilt_angle;
	
		tilt_0_publisher.publish(t_msg0);
		tilt_1_publisher.publish(t_msg1);
		tilt_2_publisher.publish(t_msg2);
		tilt_3_publisher.publish(t_msg3);
		tilt_4_publisher.publish(t_msg4);
		tilt_5_publisher.publish(t_msg5);
		
		/*---Rotor velocities--*/
		cout<<"rotor velocities:"<<endl;
		
		for(int i=0; i<6; i++ ) {
			motor_vel.data[i] = sat( sqrt(abs(f[i])/kf),1700 );
			cout<<"rotor"<<i+1<<" = "<<motor_vel.data[i]<<endl;
		}
		rotors_publisher.publish(motor_vel);
		/*-----Control info-----*/
		cout<<"x = "<<x<<", y = "<<y<<", z = "<<z<<endl;
		cout<<"fx = "<<fx<<", fy = "<<fy<<", fz = "<<fz<<endl;
		cout<<"phi = "<<phi*180.0/pi<<", the = "<<the*180.0/pi<<", psi = "<<psi*180.0/pi<<endl;
		cout<<"tx = "<<tx<<", ty = "<<ty<<", tz = "<<tz<<endl;
		cout<<"\n-------------"<<endl;
		
		ros::spinOnce();
		loop_rate.sleep();
		save();
	}
	return 0;
}

void odometry_callback(const nav_msgs::Odometry odometry_msg) {
    odom = odometry_msg;
}

void save(){
	ofstream myfile;
	myfile.open ("results_simple.txt",std::ios::app);
	myfile <<x<<","<<y<<","<<z<<","<<phi<<","<<the<<","<<psi<<","<<f[0]<<","<<f[1]<<","<<f[2]<<"," <<f[3]<<","<<f[4]<<","<<f[5]<<","<<fx<<","<<fy<<","<<fz<<","<<tx<<","<<ty<<","<<tz<<endl;
	myfile.close();
}

float c(float _x){
	return cos(_x);
}

float s(float _x){
	return sin(_x);
}

float sat(float _x,float val){
	if(_x>val) return val;
	else if(_x<-val) return -val;
	else return _x;
}

float db(float _x,float val){
	if(abs(_x)<val) return 0;
	else return _x;
}

MatrixXf F1(float a){
	MatrixXf f1(3,6);
	float r3_2 = sqrt(3)/2.;
	/*f1(0,4) = -s(a); f1(0,5) = -s(-a)/2.;  f1(0,0) = s(a)/2.;   f1(0,1) = s(-a); f1(0,2) = s(a)/2.;    f1(0,3) = -s(-a)/2.;
	f1(1,4) =    0 ; f1(1,5) = r3_2*s(-a); f1(1,0) = r3_2*s(a); f1(1,1) = 0;     f1(1,2) = -r3_2*s(a); f1(1,3) = -r3_2*s(-a);
	f1(2,4) =  c(a); f1(2,5) = c(-a);      f1(2,0) = c(a);      f1(2,1) = c(-a); f1(2,2) = c(a);       f1(2,3) = c(-a);*/
	f1(0,0) =  s (a)/2.; f1(0,1) =  s(-a); f1(0,2) = s (a)/2.;
	f1(0,3) = -s(-a)/2.; f1(0,4) = -s (a); f1(0,5) =-s(-a)/2.;
	f1(1,0) =  r3_2*s (a); f1(1,1) = 0; f1(1,2) = -r3_2*s( a);
	f1(1,3) = -r3_2*s(-a); f1(1,4) = 0; f1(1,5) =  r3_2*s(-a);
	f1(2,0) = c( a); f1(2,1) = c(-a); f1(2,2) = c( a);
	f1(2,3) = c(-a); f1(2,4) = c( a); f1(2,5) = c(-a);
	return f1;
}
MatrixXf F2(float a){
	MatrixXf f2(3,6);
	float r3_2 = sqrt(3)/2.;
	/*f2(0,4)= l*c( a)+d*s( a); f2(0,5)= -(1/2.)*(l*c(-a)+d*s(-a)); f2(0,0)=  (1/2.)*(l*c( a)-d*s( a));
	f2(0,1)=-l*c(-a)+d*s(-a); f2(0,2)=  (1/2.)*(l*c( a)-d*s( a)); f2(0,3)= -(1/2.)*(l*c(-a)+d*s(-a));
	
	f2(1,4) = 0.0; f2(1,5) =  r3_2*(l*c(-a)+d*s(-a)); f2(1,0) =  r3_2*(l*c( a)-d*s( a));
	f2(1,1) = 0.0; f2(1,2) = -r3_2*(l*c( a)-d*s( a)); f2(1,3) = -r3_2*(l*c(-a)+d*s(-a));

	f2(2,4) = -l*s( a)-d*c( a); f2(2,5) = -l*s(-a)+d*c(-a); f2(2,0) = -l*s( a)-d*c( a);
	f2(2,1) = -l*s(-a)+d*c(-a); f2(2,2) = -l*s( a)-d*c( a); f2(2,3) = -l*s(-a)+d*c(-a);*/
	f2(0,0) =  (1/2.)*(l*c( a)-d*s( a)); f2(0,1) =  l*c(-a)+d*s(-a);
	f2(0,2) =  (1/2.)*(l*c( a)-d*s( a));
	f2(0,3) = -(1/2.)*(l*c(-a)+d*s(-a)); f2(0,4) = -l*c( a)+d*s( a);
	f2(0,5) = -(1/2.)*(l*c(-a)+d*s(-a));

	f2(1,0) =  r3_2*(l*c( a)-d*s( a)); f2(1,1) = 0.0; f2(1,2) = -r3_2*(l*c( a)-d*s( a));
	f2(1,3) = -r3_2*(l*c(-a)+d*s(-a)); f2(1,4) = 0.0; f2(1,5) =  r3_2*(l*c(-a)+d*s(-a)); 

	f2(2,0) = -l*s( a)-d*c( a); f2(2,1) = -l*s(-a)+d*c(-a); f2(2,2) = -l*s( a)-d*c( a);
	f2(2,3) = -l*s(-a)+d*c(-a); f2(2,4) = -l*s( a)-d*c( a); f2(2,5) = -l*s(-a)+d*c(-a);
	return f2;
	
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

