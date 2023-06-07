#include <stdio.h>      /* printf */
      /* sin */
#include "hdl_grabber/quaternion_fnc.h"

QuaternionFnc::QuaternionFnc()
{

}
QuaternionFnc:: ~ QuaternionFnc()
{

}
// converts quaterion to euler angle
bool QuaternionFnc::quaternion2euler(euler_t &euler,quat_t quat)
{ 
/*
q0=q(1);  q.w
q1=q(2);  q.x
q2=q(3);  q.y
q3=q(4);  q.z
if(nargin<2)
    mode='';
end
    
switch(mode)
    case 'all'
        roll=atan2(2*(q2*q3 +q0*q1),(2*q0*q0 +2*q3*q3 -1));
        pitch=(-1)*asin((2*q1*q3-2*q0*q2));
        yaw=atan2((2*q1*q2+2*q0*q3),(2*q0*q0 + 2*q1*q1 -1));
    otherwise
        roll=atan(2*(q2*q3 +q0*q1)/(2*q0*q0 +2*q3*q3 -1));
        pitch=(-1)*asin((2*q1*q3-2*q0*q2));
        yaw=atan((2*q1*q2+2*q0*q3)/(2*q0*q0 + 2*q1*q1 -1));
end
*/
	bool ret=true;
	euler.r = atan2(2*(quat.y*quat.z +quat.w*quat.x),(2*quat.w*quat.w +2*quat.z*quat.z -1));
	euler.p = (-1)*asin((2*quat.x*quat.z-2*quat.w*quat.y));
	euler.y = atan2((2*quat.x*quat.y+2*quat.w*quat.z),(2*quat.w*quat.w + 2*quat.x*quat.x -1));

	return ret;
}
bool QuaternionFnc::euler2quaternion(quat_t& quat,euler_t eurler)
{

bool ret=true;
/*c1= cos(roll/2);
c2 =cos(pitch/2);
c3=cos(yaw/2);

s1= sin(roll/2);
s2 =sin(pitch/2);
s3=sin(yaw/2);

Q1=[c1 s1 0 0];
Q2 =[c2 0 s2 0];
Q3 =[c3 0 0 s3];
% inverse multiplication
% q= Q3*Q2*Q1
Q32=quatMultiplication(Q3,Q2);

q=quatMultiplication(Q32,Q1);
 */
quat_t q1,q2,q3,q32;
double c1,c2,c3,s1,s2,s3;

	//get sines and cosine from of angles
	c1= cos(eurler.r/2);
	c2 =cos(eurler.p/2);
	c3= cos(eurler.y/2);
	s1= sin(eurler.r/2);
	s2 =sin(eurler.p/2);
	s3= sin(eurler.y/2);

	// create quaterions
	//Q1=[c1 s1 0 0];
	//Q2 =[c2 0 s2 0];
	//Q3 =[c3 0 0 s3];
	q1.x=c1; q1.y=s1; q1.z=0;  q1.w=0;
	q2.x=c2; q2.y=0;  q2.z=s2; q2.w=0;
	q3.x=c3; q3.y=0;  q3.z=0;  q3.w=s3;

	/* inverse multiplication
	% q= Q3*Q2*Q1
	Q32=quatMultiplication(Q3,Q2);
	q=quatMultiplication(Q32,Q1);*/

	quatProduct(q32,q3,q2);
	quatProduct(quat,q32,q1);

	return(ret);
}
bool QuaternionFnc::quatProduct(quat_t& quat,quat_t q1,quat_t q2)
{
/*
 *  q= [q1(1)*q2(1) - q1(2)*q2(2)- q1(3)*q2(3)- q1(4)*q2(4);
        q1(1)*q2(2) + q1(2)*q2(1)+ q1(3)*q2(4)- q1(4)*q2(3);
        q1(1)*q2(3) - q1(2)*q2(4)+ q1(3)*q2(1)+ q1(4)*q2(2);
        q1(1)*q2(4) + q1(2)*q2(3)- q1(3)*q2(2)+ q1(4)*q2(1)];
 */
bool ret=true;

    quat.x= (q1.x*q2.x) - (q1.y*q2.y) - (q1.z*q2.z) - (q1.w*q2.w);
    quat.y= (q1.x*q2.y) + (q1.y*q2.x) + (q1.z*q2.w) - (q1.w*q2.z);
    quat.z= (q1.x*q2.z) - (q1.y*q2.w) + (q1.z*q2.x) + (q1.w*q2.y);
    quat.w= (q1.x*q2.w) + (q1.y*q2.z) - (q1.z*q2.y) + (q1.w*q2.x);
	return(ret);
}
bool QuaternionFnc::quat2Rotation3DPoint(point_t& rot_point,point_t point,quat_t quat)
{
bool ret=true;
/*
w=q(1);
x=q(2);
y=q(3);
z=q(4);
pt_out(1)=w*w*pt_in(1) + 2*y*w*pt_in(3) -2*z*w*pt_in(2)+x*x*pt_in(1)+ ...
          2*y*x*pt_in(2)+ 2*z*x*pt_in(3) -z*z*pt_in(1) - y*y*pt_in(1);

pt_out(2)= 2*x*y*pt_in(1) + y*y*pt_in(2)+2*z*y*pt_in(3) + 2*w*z*pt_in(1)...
           -z*z*pt_in(2)+ w*w*pt_in(2) - 2*x*w*pt_in(3) - x*x*pt_in(2);
pt_out(3)= 2*x*z*pt_in(1) + 2*y*z*pt_in(2) + z*z*pt_in(3) - 2*w*y*pt_in(1) ...
           -y*y*pt_in(3) + 2*w*x*pt_in(2) - x*x*pt_in(3) + w*w*pt_in(3);
*/

double w,x,y,z;

	w=quat.x; x=quat.y; y=quat.z; z=quat.w;

    rot_point.x= w*w*point.x + 2*y*w*point.z -2*z*w*point.y + x*x*point.x +
                 2*y*x*point.y + 2*z*x*point.z -z*z*point.x - y*y*point.x;
    rot_point.y= 2*x*y*point.x + y*y*point.y+2*z*y*point.z + 2*w*z*point.x -
               z*z*point.y+ w*w*point.y - 2*x*w*point.z - x*x*point.y;
    rot_point.z= 2*x*z*point.x + 2*y*z*point.y + z*z*point.z - 2*w*y*point.x -
               y*y*point.z + 2*w*x*point.y - x*x*point.z + w*w*point.z;
	return(ret);
}
