#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>


class quaternions
{	
	public:	
	
		const geometry_msgs::Quaternion norm(geometry_msgs::Quaternion q)
		{
			float q_l = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
			q.x = q.x/q_l; q.y = q.y/q_l; q.z = q.z/q_l; q.w = q.w/q_l;
			
			return q;
		}
	
		const geometry_msgs::Quaternion rotation(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
		{
			
			geometry_msgs::Quaternion qr;
			qr.w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z;
			qr.x = q2.w*q1.x + q2.x*q1.w - q2.y*q1.z + q2.z*q1.y;
			qr.y = q2.w*q1.y + q2.x*q1.z + q2.y*q1.w - q2.z*q1.x;
			qr.z = q2.w*q1.z - q2.x*q1.y + q2.y*q1.x + q2.z*q1.w;
			
			return qr;			
		};
		
		const geometry_msgs::Quaternion conjugate(geometry_msgs::Quaternion q)
		{
			geometry_msgs::Quaternion q_conj;
			q_conj.x = -q.x;
			q_conj.y = -q.y;
			q_conj.z = -q.z;
			q_conj.w = q.w;
			
			return q_conj;	
		};
		
		const geometry_msgs::Vector3 vector_rotation(geometry_msgs::Quaternion q, geometry_msgs::Vector3 v)
		{
			geometry_msgs::Vector3 vr;
			vr.x = (1-2*q.y*q.y-2*q.z*q.z)*v.x + 2*(q.x*q.y+q.w*q.z)*v.y + 2*(q.x*q.z-q.w*q.y)*v.z;
			vr.y = 2*(q.x*q.y-q.w*q.z)*v.x + (1-2*q.x*q.x-2*q.z*q.z)*v.y + 2*(q.y*q.z+q.w*q.x)*v.z;
			vr.z = 2*(q.x*q.z+q.w*q.y)*v.x + 2*(q.y*q.z-q.w*q.x)*v.y + (1-2*q.x*q.x-2*q.y*q.y)*v.z;
			
			return vr;	
		};
};

#endif //QUATERNIONS_H