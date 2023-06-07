 
#ifndef QUATERNION_FNC_H_
#define QUATERNION_FNC_H_



#include <string>
#include <iostream>
#include <math.h>

#define toRadians(x) ((x) * M_PI / 180.0)
#define toDegrees(x) ((x) * 180.0 / M_PI)

class QuaternionFnc {

public:


	QuaternionFnc();
	virtual ~ QuaternionFnc();

	typedef struct{
		double x;
		double y;
		double z;
		double w;
	}quat_t;

	typedef struct{
		double r;
		double p;
		double y;
	}euler_t;
	typedef struct{
		double x;
		double y;
		double z;
	}point_t;

    // converts euler angle to quaterion
	bool euler2quaternion(quat_t& quat,euler_t eurler);
	// converts quaterion to euler angle
	bool quaternion2euler(euler_t &euler,quat_t quat);
	// product of two quaterions
	bool quatProduct(quat_t& quat,quat_t q1,quat_t q2);
	// rotates 3D point w.r.t to a quaterion
	bool quat2Rotation3DPoint(point_t& rot_point,point_t point,quat_t quat);




};














#endif /* QUATERNION_FNC_H_ */
