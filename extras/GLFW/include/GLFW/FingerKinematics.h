//
// Created by SuYuan on 2022/10/15
//


#ifndef _FINGER_KINEMATICS_
#define _FINGER_KINEMATICS_

#include "Eigen/Eigen"
#include <math.h>
#include <iostream>
#define PI 3.1415926

namespace chai3d
{
	namespace FingerMath
	{

		Eigen::Matrix4f T_10;		//1 relative 0
		Eigen::Matrix4f T_21;		//2 relative 1
		Eigen::Matrix4f T_32;
		Eigen::Matrix4f T_43;
		Eigen::Matrix4f T_54;
		Eigen::Matrix4f T_65;
		Eigen::Matrix4f T_tip_6;

		float L1;
		float L2;
		float L3;
		float L4;
		float L5;

		Eigen::Matrix4f T_60;
		Eigen::Matrix4f T_tip_0;
		Eigen::Matrix4f T_tip_W;

		Eigen::Matrix4f T_0W;					//device frame{0} relative world frame{W}
		Eigen::Matrix4f T_finger_0W;			//finger frame{0} relative world frame{W}
		Eigen::Matrix4f T_finger_0W_inv;		//inverse
		Eigen::Matrix4f T_finger_tip_0;

		Eigen::Matrix3f Rx;
		Eigen::Matrix3f Ry;
		Eigen::Matrix3f Rz;

		Eigen::Matrix3f Rxyz;

		//calculate single finger joint angles
		float e1;
		float d1;
		float d2;
		float a;
		float b;

		Eigen::Vector4f finger_joint_angle;

		Eigen::Matrix3f getRx(float theta) {
			Rx(0, 0) = 1;
			Rx(0, 1) = 0;
			Rx(0, 2) = 0;
			Rx(1, 0) = 0;
			Rx(1, 1) = cos(theta * PI / 180);
			Rx(1, 2) = (-1) * sin(theta * PI / 180);
			Rx(2, 0) = 0;
			Rx(2, 1) = sin(theta * PI / 180);
			Rx(2, 2) = cos(theta * PI / 180);
			return Rx;
		}

		Eigen::Matrix3f getRy(float theta) {
			Ry(0, 0) = cos(theta * PI / 180);
			Ry(0, 1) = 0;
			Ry(0, 2) = sin(theta * PI / 180);
			Ry(1, 0) = 0;
			Ry(1, 1) = 1;
			Ry(1, 2) = 0;
			Ry(2, 0) = (-1) * sin(theta * PI / 180);
			Ry(2, 1) = 0;
			Ry(2, 2) = cos(theta * PI / 180);
			return Ry;
		}

		Eigen::Matrix3f getRz(float theta) {
			Rz(0, 0) = cos(theta * PI / 180);
			Rz(0, 1) = (-1) * sin(theta * PI / 180);
			Rz(0, 2) = 0;
			Rz(1, 0) = sin(theta * PI / 180);
			Rz(1, 1) = cos(theta * PI / 180);
			Rz(1, 2) = 0;
			Rz(2, 0) = 0;
			Rz(2, 1) = 0;
			Rz(2, 2) = 1;
			return Rz;
		}

		Eigen::Matrix4f getTransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			//
			theta1 = (-1) * theta1;
			theta2 = (-1) * theta2;
			theta3 = (-1) * theta3;
			theta4 = (-1) * theta4;
			theta5 = theta5;
			theta6 = (-1) * theta6;

			//
			theta1 = theta1 - 78.97;
			theta2 = theta2 + 180;
			theta3 = theta3 + 90;
			theta4 = theta4;
			theta5 = theta5 - 90;
			theta6 = theta6 - 70;

			//
			L1 = 74.01;
			L2 = 33.30;
			L3 = 43.80;
			L5 = 12.99;

			//
			T_10(0, 0) = cos(theta1 * PI / 180);
			T_10(0, 1) = (-1) * sin(theta1 * PI / 180);
			T_10(0, 2) = 0;
			T_10(0, 3) = 0;
			T_10(1, 0) = sin(theta1 * PI / 180) * cos(0 * PI / 180);
			T_10(1, 1) = cos(theta1 * PI / 180) * cos(0 * PI / 180);
			T_10(1, 2) = (-1) * sin(0 * PI / 180);
			T_10(1, 3) = (-1) * sin(0 * PI / 180) * 0;
			T_10(2, 0) = sin(theta1 * PI / 180) * sin(0 * PI / 180);
			T_10(2, 1) = cos(theta1 * PI / 180) * sin(0 * PI / 180);
			T_10(2, 2) = cos(0 * PI / 180);
			T_10(2, 3) = cos(0 * PI / 180) * 0;
			T_10(3, 0) = 0;
			T_10(3, 1) = 0;
			T_10(3, 2) = 0;
			T_10(3, 3) = 1;
			//
			T_21(0, 0) = cos(theta2 * PI / 180);
			T_21(0, 1) = (-1) * sin(theta2 * PI / 180);
			T_21(0, 2) = 0;
			T_21(0, 3) = L1;											//73.9
			T_21(1, 0) = sin(theta2 * PI / 180) * cos(0 * PI / 180);
			T_21(1, 1) = cos(theta2 * PI / 180) * cos(0 * PI / 180);
			T_21(1, 2) = (-1) * sin(0 * PI / 180);
			T_21(1, 3) = (-1) * sin(0 * PI / 180) * 0;
			T_21(2, 0) = sin(theta2 * PI / 180) * sin(0 * PI / 180);
			T_21(2, 1) = cos(theta2 * PI / 180) * sin(0 * PI / 180);
			T_21(2, 2) = cos(0 * PI / 180);
			T_21(2, 3) = cos(0 * PI / 180) * 0;
			T_21(3, 0) = 0;
			T_21(3, 1) = 0;
			T_21(3, 2) = 0;
			T_21(3, 3) = 1;
			//
			T_32(0, 0) = cos(theta3 * PI / 180);
			T_32(0, 1) = (-1) * sin(theta3 * PI / 180);
			T_32(0, 2) = 0;
			T_32(0, 3) = 0;
			T_32(1, 0) = sin(theta3 * PI / 180) * cos(90 * PI / 180);
			T_32(1, 1) = cos(theta3 * PI / 180) * cos(90 * PI / 180);
			T_32(1, 2) = (-1) * sin(90 * PI / 180);
			T_32(1, 3) = (-1) * sin(90 * PI / 180) * L2;
			T_32(2, 0) = sin(theta3 * PI / 180) * sin(90 * PI / 180);
			T_32(2, 1) = cos(theta3 * PI / 180) * sin(90 * PI / 180);
			T_32(2, 2) = cos(90 * PI / 180);
			T_32(2, 3) = cos(90 * PI / 180) * L2;
			T_32(3, 0) = 0;
			T_32(3, 1) = 0;
			T_32(3, 2) = 0;
			T_32(3, 3) = 1;
			//
			T_43(0, 0) = cos(theta4 * PI / 180);
			T_43(0, 1) = (-1) * sin(theta4 * PI / 180);
			T_43(0, 2) = 0;
			T_43(0, 3) = 0;
			T_43(1, 0) = sin(theta4 * PI / 180) * cos(60 * PI / 180);
			T_43(1, 1) = cos(theta4 * PI / 180) * cos(60 * PI / 180);
			T_43(1, 2) = (-1) * sin(60 * PI / 180);
			T_43(1, 3) = (-1) * sin(60 * PI / 180) * L3;
			T_43(2, 0) = sin(theta4 * PI / 180) * sin(60 * PI / 180);
			T_43(2, 1) = cos(theta4 * PI / 180) * sin(60 * PI / 180);
			T_43(2, 2) = cos(60 * PI / 180);
			T_43(2, 3) = cos(60 * PI / 180) * L3;
			T_43(3, 0) = 0;
			T_43(3, 1) = 0;
			T_43(3, 2) = 0;
			T_43(3, 3) = 1;
			//
			T_54(0, 0) = cos(theta5 * PI / 180);
			T_54(0, 1) = (-1) * sin(theta5 * PI / 180);
			T_54(0, 2) = 0;
			T_54(0, 3) = 0;
			T_54(1, 0) = sin(theta5 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_54(1, 1) = cos(theta5 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_54(1, 2) = (-1) * sin((-1) * 90 * PI / 180);
			T_54(1, 3) = (-1) * sin((-1) * 90 * PI / 180) * 0;
			T_54(2, 0) = sin(theta5 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_54(2, 1) = cos(theta5 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_54(2, 2) = cos((-1) * 90 * PI / 180);
			T_54(2, 3) = cos((-1) * 90 * PI / 180) * 0;
			T_54(3, 0) = 0;
			T_54(3, 1) = 0;
			T_54(3, 2) = 0;
			T_54(3, 3) = 1;
			//
			T_65(0, 0) = cos(theta6 * PI / 180);
			T_65(0, 1) = (-1) * sin(theta6 * PI / 180);
			T_65(0, 2) = 0;
			T_65(0, 3) = L5;
			T_65(1, 0) = sin(theta6 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_65(1, 1) = cos(theta6 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_65(1, 2) = (-1) * sin((-1) * 90 * PI / 180);
			T_65(1, 3) = (-1) * sin((-1) * 90 * PI / 180) * 0;
			T_65(2, 0) = sin(theta6 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_65(2, 1) = cos(theta6 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_65(2, 2) = cos((-1) * 90 * PI / 180);
			T_65(2, 3) = cos((-1) * 90 * PI / 180) * 0;
			T_65(3, 0) = 0;
			T_65(3, 1) = 0;
			T_65(3, 2) = 0;
			T_65(3, 3) = 1;

			//
			T_tip_6(0, 0) = 1;		T_tip_6(0, 1) = 0;		T_tip_6(0, 2) = 0;		T_tip_6(0, 3) = 20;
			T_tip_6(1, 0) = 0;		T_tip_6(1, 1) = 0;		T_tip_6(1, 2) = -1;		T_tip_6(1, 3) = 28;
			T_tip_6(2, 0) = 0;		T_tip_6(2, 1) = 1;		T_tip_6(2, 2) = 0;		T_tip_6(2, 3) = 0;
			T_tip_6(3, 0) = 0;		T_tip_6(3, 1) = 0;		T_tip_6(3, 2) = 0;		T_tip_6(3, 3) = 1;

			//
			T_60 = T_10 * T_21 * T_32 * T_43 * T_54 * T_65;
			T_tip_0 = T_60 * T_tip_6;

			return T_tip_0;
		}

		Eigen::Matrix4f getMIDDLETransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			//
			theta1 = (-1) * theta1;
			theta2 = (-1) * theta2;
			theta3 = (-1) * theta3;
			theta4 = (-1) * theta4;
			theta5 = theta5;
			theta6 = theta6;

			//
			theta1 = theta1 - 78.97;
			theta2 = theta2 + 180;
			theta3 = theta3 + 90;
			theta4 = theta4;
			theta5 = theta5 - 90;
			theta6 = theta6 - 70;

			//
			L1 = 74.01;
			L2 = 33.30;
			L3 = 43.80;
			L5 = 12.99;

			//
			T_10(0, 0) = cos(theta1 * PI / 180);
			T_10(0, 1) = (-1) * sin(theta1 * PI / 180);
			T_10(0, 2) = 0;
			T_10(0, 3) = 0;
			T_10(1, 0) = sin(theta1 * PI / 180) * cos(0 * PI / 180);
			T_10(1, 1) = cos(theta1 * PI / 180) * cos(0 * PI / 180);
			T_10(1, 2) = (-1) * sin(0 * PI / 180);
			T_10(1, 3) = (-1) * sin(0 * PI / 180) * 0;
			T_10(2, 0) = sin(theta1 * PI / 180) * sin(0 * PI / 180);
			T_10(2, 1) = cos(theta1 * PI / 180) * sin(0 * PI / 180);
			T_10(2, 2) = cos(0 * PI / 180);
			T_10(2, 3) = cos(0 * PI / 180) * 0;
			T_10(3, 0) = 0;
			T_10(3, 1) = 0;
			T_10(3, 2) = 0;
			T_10(3, 3) = 1;
			//
			T_21(0, 0) = cos(theta2 * PI / 180);
			T_21(0, 1) = (-1) * sin(theta2 * PI / 180);
			T_21(0, 2) = 0;
			T_21(0, 3) = L1;											//73.9
			T_21(1, 0) = sin(theta2 * PI / 180) * cos(0 * PI / 180);
			T_21(1, 1) = cos(theta2 * PI / 180) * cos(0 * PI / 180);
			T_21(1, 2) = (-1) * sin(0 * PI / 180);
			T_21(1, 3) = (-1) * sin(0 * PI / 180) * 0;
			T_21(2, 0) = sin(theta2 * PI / 180) * sin(0 * PI / 180);
			T_21(2, 1) = cos(theta2 * PI / 180) * sin(0 * PI / 180);
			T_21(2, 2) = cos(0 * PI / 180);
			T_21(2, 3) = cos(0 * PI / 180) * 0;
			T_21(3, 0) = 0;
			T_21(3, 1) = 0;
			T_21(3, 2) = 0;
			T_21(3, 3) = 1;
			//
			T_32(0, 0) = cos(theta3 * PI / 180);
			T_32(0, 1) = (-1) * sin(theta3 * PI / 180);
			T_32(0, 2) = 0;
			T_32(0, 3) = 0;
			T_32(1, 0) = sin(theta3 * PI / 180) * cos(90 * PI / 180);
			T_32(1, 1) = cos(theta3 * PI / 180) * cos(90 * PI / 180);
			T_32(1, 2) = (-1) * sin(90 * PI / 180);
			T_32(1, 3) = (-1) * sin(90 * PI / 180) * L2;
			T_32(2, 0) = sin(theta3 * PI / 180) * sin(90 * PI / 180);
			T_32(2, 1) = cos(theta3 * PI / 180) * sin(90 * PI / 180);
			T_32(2, 2) = cos(90 * PI / 180);
			T_32(2, 3) = cos(90 * PI / 180) * L2;
			T_32(3, 0) = 0;
			T_32(3, 1) = 0;
			T_32(3, 2) = 0;
			T_32(3, 3) = 1;
			//
			T_43(0, 0) = cos(theta4 * PI / 180);
			T_43(0, 1) = (-1) * sin(theta4 * PI / 180);
			T_43(0, 2) = 0;
			T_43(0, 3) = 0;
			T_43(1, 0) = sin(theta4 * PI / 180) * cos(60 * PI / 180);
			T_43(1, 1) = cos(theta4 * PI / 180) * cos(60 * PI / 180);
			T_43(1, 2) = (-1) * sin(60 * PI / 180);
			T_43(1, 3) = (-1) * sin(60 * PI / 180) * L3;
			T_43(2, 0) = sin(theta4 * PI / 180) * sin(60 * PI / 180);
			T_43(2, 1) = cos(theta4 * PI / 180) * sin(60 * PI / 180);
			T_43(2, 2) = cos(60 * PI / 180);
			T_43(2, 3) = cos(60 * PI / 180) * L3;
			T_43(3, 0) = 0;
			T_43(3, 1) = 0;
			T_43(3, 2) = 0;
			T_43(3, 3) = 1;
			//
			T_54(0, 0) = cos(theta5 * PI / 180);
			T_54(0, 1) = (-1) * sin(theta5 * PI / 180);
			T_54(0, 2) = 0;
			T_54(0, 3) = 0;
			T_54(1, 0) = sin(theta5 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_54(1, 1) = cos(theta5 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_54(1, 2) = (-1) * sin((-1) * 90 * PI / 180);
			T_54(1, 3) = (-1) * sin((-1) * 90 * PI / 180) * 0;
			T_54(2, 0) = sin(theta5 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_54(2, 1) = cos(theta5 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_54(2, 2) = cos((-1) * 90 * PI / 180);
			T_54(2, 3) = cos((-1) * 90 * PI / 180) * 0;
			T_54(3, 0) = 0;
			T_54(3, 1) = 0;
			T_54(3, 2) = 0;
			T_54(3, 3) = 1;
			//
			T_65(0, 0) = cos(theta6 * PI / 180);
			T_65(0, 1) = (-1) * sin(theta6 * PI / 180);
			T_65(0, 2) = 0;
			T_65(0, 3) = L5;
			T_65(1, 0) = sin(theta6 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_65(1, 1) = cos(theta6 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_65(1, 2) = (-1) * sin((-1) * 90 * PI / 180);
			T_65(1, 3) = (-1) * sin((-1) * 90 * PI / 180) * 0;
			T_65(2, 0) = sin(theta6 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_65(2, 1) = cos(theta6 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_65(2, 2) = cos((-1) * 90 * PI / 180);
			T_65(2, 3) = cos((-1) * 90 * PI / 180) * 0;
			T_65(3, 0) = 0;
			T_65(3, 1) = 0;
			T_65(3, 2) = 0;
			T_65(3, 3) = 1;

			//
			T_tip_6(0, 0) = 1;		T_tip_6(0, 1) = 0;		T_tip_6(0, 2) = 0;		T_tip_6(0, 3) = 20;
			T_tip_6(1, 0) = 0;		T_tip_6(1, 1) = 0;		T_tip_6(1, 2) = -1;		T_tip_6(1, 3) = 28;
			T_tip_6(2, 0) = 0;		T_tip_6(2, 1) = 1;		T_tip_6(2, 2) = 0;		T_tip_6(2, 3) = 0;
			T_tip_6(3, 0) = 0;		T_tip_6(3, 1) = 0;		T_tip_6(3, 2) = 0;		T_tip_6(3, 3) = 1;

			//
			T_60 = T_10 * T_21 * T_32 * T_43 * T_54 * T_65;
			T_tip_0 = T_60 * T_tip_6;

			return T_tip_0;
		}

		Eigen::Matrix4f getTHUMBTransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			//
			theta1 = (-1) * theta1;
			theta2 = (-1) * theta2;
			theta3 = (-1) * theta3;
			theta4 = (-1) * theta4;
			theta5 = theta5;
			theta6 = theta6;

			//
			theta1 = theta1 - 78.97;
			theta2 = theta2 + 180;
			theta3 = theta3 + 90;
			theta4 = theta4;
			theta5 = theta5 - 90;
			theta6 = theta6 - 70;

			//
			L1 = 74.01;
			L2 = 33.30;
			L3 = 43.80;
			L5 = 12.99;

			//
			T_10(0, 0) = cos(theta1 * PI / 180);
			T_10(0, 1) = (-1) * sin(theta1 * PI / 180);
			T_10(0, 2) = 0;
			T_10(0, 3) = 0;
			T_10(1, 0) = sin(theta1 * PI / 180) * cos(0 * PI / 180);
			T_10(1, 1) = cos(theta1 * PI / 180) * cos(0 * PI / 180);
			T_10(1, 2) = (-1) * sin(0 * PI / 180);
			T_10(1, 3) = (-1) * sin(0 * PI / 180) * 0;
			T_10(2, 0) = sin(theta1 * PI / 180) * sin(0 * PI / 180);
			T_10(2, 1) = cos(theta1 * PI / 180) * sin(0 * PI / 180);
			T_10(2, 2) = cos(0 * PI / 180);
			T_10(2, 3) = cos(0 * PI / 180) * 0;
			T_10(3, 0) = 0;
			T_10(3, 1) = 0;
			T_10(3, 2) = 0;
			T_10(3, 3) = 1;
			//
			T_21(0, 0) = cos(theta2 * PI / 180);
			T_21(0, 1) = (-1) * sin(theta2 * PI / 180);
			T_21(0, 2) = 0;
			T_21(0, 3) = L1;											//73.9
			T_21(1, 0) = sin(theta2 * PI / 180) * cos(0 * PI / 180);
			T_21(1, 1) = cos(theta2 * PI / 180) * cos(0 * PI / 180);
			T_21(1, 2) = (-1) * sin(0 * PI / 180);
			T_21(1, 3) = (-1) * sin(0 * PI / 180) * 0;
			T_21(2, 0) = sin(theta2 * PI / 180) * sin(0 * PI / 180);
			T_21(2, 1) = cos(theta2 * PI / 180) * sin(0 * PI / 180);
			T_21(2, 2) = cos(0 * PI / 180);
			T_21(2, 3) = cos(0 * PI / 180) * 0;
			T_21(3, 0) = 0;
			T_21(3, 1) = 0;
			T_21(3, 2) = 0;
			T_21(3, 3) = 1;
			//
			T_32(0, 0) = cos(theta3 * PI / 180);
			T_32(0, 1) = (-1) * sin(theta3 * PI / 180);
			T_32(0, 2) = 0;
			T_32(0, 3) = 0;
			T_32(1, 0) = sin(theta3 * PI / 180) * cos(90 * PI / 180);
			T_32(1, 1) = cos(theta3 * PI / 180) * cos(90 * PI / 180);
			T_32(1, 2) = (-1) * sin(90 * PI / 180);
			T_32(1, 3) = (-1) * sin(90 * PI / 180) * L2;
			T_32(2, 0) = sin(theta3 * PI / 180) * sin(90 * PI / 180);
			T_32(2, 1) = cos(theta3 * PI / 180) * sin(90 * PI / 180);
			T_32(2, 2) = cos(90 * PI / 180);
			T_32(2, 3) = cos(90 * PI / 180) * L2;
			T_32(3, 0) = 0;
			T_32(3, 1) = 0;
			T_32(3, 2) = 0;
			T_32(3, 3) = 1;
			//
			T_43(0, 0) = cos(theta4 * PI / 180);
			T_43(0, 1) = (-1) * sin(theta4 * PI / 180);
			T_43(0, 2) = 0;
			T_43(0, 3) = 0;
			T_43(1, 0) = sin(theta4 * PI / 180) * cos(60 * PI / 180);
			T_43(1, 1) = cos(theta4 * PI / 180) * cos(60 * PI / 180);
			T_43(1, 2) = (-1) * sin(60 * PI / 180);
			T_43(1, 3) = (-1) * sin(60 * PI / 180) * L3;
			T_43(2, 0) = sin(theta4 * PI / 180) * sin(60 * PI / 180);
			T_43(2, 1) = cos(theta4 * PI / 180) * sin(60 * PI / 180);
			T_43(2, 2) = cos(60 * PI / 180);
			T_43(2, 3) = cos(60 * PI / 180) * L3;
			T_43(3, 0) = 0;
			T_43(3, 1) = 0;
			T_43(3, 2) = 0;
			T_43(3, 3) = 1;
			//
			T_54(0, 0) = cos(theta5 * PI / 180);
			T_54(0, 1) = (-1) * sin(theta5 * PI / 180);
			T_54(0, 2) = 0;
			T_54(0, 3) = 0;
			T_54(1, 0) = sin(theta5 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_54(1, 1) = cos(theta5 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_54(1, 2) = (-1) * sin((-1) * 90 * PI / 180);
			T_54(1, 3) = (-1) * sin((-1) * 90 * PI / 180) * 0;
			T_54(2, 0) = sin(theta5 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_54(2, 1) = cos(theta5 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_54(2, 2) = cos((-1) * 90 * PI / 180);
			T_54(2, 3) = cos((-1) * 90 * PI / 180) * 0;
			T_54(3, 0) = 0;
			T_54(3, 1) = 0;
			T_54(3, 2) = 0;
			T_54(3, 3) = 1;
			//
			T_65(0, 0) = cos(theta6 * PI / 180);
			T_65(0, 1) = (-1) * sin(theta6 * PI / 180);
			T_65(0, 2) = 0;
			T_65(0, 3) = L5;
			T_65(1, 0) = sin(theta6 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_65(1, 1) = cos(theta6 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_65(1, 2) = (-1) * sin((-1) * 90 * PI / 180);
			T_65(1, 3) = (-1) * sin((-1) * 90 * PI / 180) * 0;
			T_65(2, 0) = sin(theta6 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_65(2, 1) = cos(theta6 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_65(2, 2) = cos((-1) * 90 * PI / 180);
			T_65(2, 3) = cos((-1) * 90 * PI / 180) * 0;
			T_65(3, 0) = 0;
			T_65(3, 1) = 0;
			T_65(3, 2) = 0;
			T_65(3, 3) = 1;

			//
			T_tip_6(0, 0) = 1;		T_tip_6(0, 1) = 0;		T_tip_6(0, 2) = 0;		T_tip_6(0, 3) = 20;
			T_tip_6(1, 0) = 0;		T_tip_6(1, 1) = 0;		T_tip_6(1, 2) = -1;		T_tip_6(1, 3) = 28;
			T_tip_6(2, 0) = 0;		T_tip_6(2, 1) = 1;		T_tip_6(2, 2) = 0;		T_tip_6(2, 3) = 0;
			T_tip_6(3, 0) = 0;		T_tip_6(3, 1) = 0;		T_tip_6(3, 2) = 0;		T_tip_6(3, 3) = 1;

			//
			T_60 = T_10 * T_21 * T_32 * T_43 * T_54 * T_65;
			T_tip_0 = T_60 * T_tip_6;

			return T_tip_0;
		}

		Eigen::Matrix4f getF1TransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			
			T_tip_0 = getTransMatrix(theta1, theta2, theta3, theta4, theta5, theta6);
			//std::cout << T_tip_0(0, 3) << "," << T_tip_0(1, 3) << "," << T_tip_0(2, 3) << std::endl;

			Rx(0, 0) = 1;
			Rx(0, 1) = 0;
			Rx(0, 2) = 0;
			Rx(1, 0) = 0;
			Rx(1, 1) = cos((-1) * 180 * PI / 180);
			Rx(1, 2) = (-1) * sin((-1) * 180 * PI / 180);
			Rx(2, 0) = 0;
			Rx(2, 1) = sin((-1) * 180 * PI / 180);
			Rx(2, 2) = cos((-1) * 180 * PI / 180);

			//
			Ry(0, 0) = cos((-1) * 45 * PI / 180);
			Ry(0, 1) = 0;
			Ry(0, 2) = sin((-1) * 45 * PI / 180);
			Ry(1, 0) = 0;
			Ry(1, 1) = 1;
			Ry(1, 2) = 0;
			Ry(2, 0) = (-1) * sin((-1) * 45 * PI / 180);
			Ry(2, 1) = 0;
			Ry(2, 2) = cos((-1) * 45 * PI / 180);

			//
			Rxyz = Rx * Ry;

			//
			T_0W(0, 0) = Rxyz(0, 0);	T_0W(0, 1) = Rxyz(0, 1);	T_0W(0, 2) = Rxyz(0, 2);	T_0W(0, 3) = (-1) * 5.50;
			T_0W(1, 0) = Rxyz(1, 0);	T_0W(1, 1) = Rxyz(1, 1);	T_0W(1, 2) = Rxyz(1, 2);	T_0W(1, 3) = 46.5;
			T_0W(2, 0) = Rxyz(2, 0);	T_0W(2, 1) = Rxyz(2, 1);	T_0W(2, 2) = Rxyz(2, 2);	T_0W(2, 3) = (-1) * 28.00;		//�Ѳ���
			T_0W(3, 0) = 0;				T_0W(3, 1) = 0;				T_0W(3, 2) = 0;				T_0W(3, 3) = 1;

			//
			T_tip_W = T_0W * T_tip_0;

			return T_tip_W;

		}

		Eigen::Matrix4f getF2TransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			
			T_tip_0 = getTransMatrix(theta1, theta2, theta3, theta4, theta5, theta6);

			//check T_tip_0
			//std::cout << T_tip_0(0, 3) << "," << T_tip_0(1, 3) << "," << T_tip_0(2, 3) << std::endl;

			Rx = getRx(-90);

			Rxyz = Rx;

			//
			T_0W(0, 0) = Rxyz(0, 0);	T_0W(0, 1) = Rxyz(0, 1);	T_0W(0, 2) = Rxyz(0, 2);	T_0W(0, 3) = 54.00;		//�Ѽ�� ʵ������54
			T_0W(1, 0) = Rxyz(1, 0);	T_0W(1, 1) = Rxyz(1, 1);	T_0W(1, 2) = Rxyz(1, 2);	T_0W(1, 3) = 21.00;		//�Ѽ�� ʵ������21
			T_0W(2, 0) = Rxyz(2, 0);	T_0W(2, 1) = Rxyz(2, 1);	T_0W(2, 2) = Rxyz(2, 2);	T_0W(2, 3) = 18.00;		//�Ѽ�� ʵ������18
			T_0W(3, 0) = 0;				T_0W(3, 1) = 0;				T_0W(3, 2) = 0;				T_0W(3, 3) = 1;

			//
			T_tip_W = T_0W * T_tip_0;

			return T_tip_W;

		}

		Eigen::Matrix4f getF3TransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			//
			T_tip_0 = getMIDDLETransMatrix(theta1, theta2, theta3, theta4, theta5, theta6);
			//std::cout << T_tip_0(0, 3) << "," << T_tip_0(1, 3) << "," << T_tip_0(2, 3) << std::endl;

			//
			Rx = getRx(-90);

			Rxyz = Rx;

			//
			T_0W(0, 0) = Rxyz(0, 0);	T_0W(0, 1) = Rxyz(0, 1);	T_0W(0, 2) = Rxyz(0, 2);	T_0W(0, 3) = 54.00;
			T_0W(1, 0) = Rxyz(1, 0);	T_0W(1, 1) = Rxyz(1, 1);	T_0W(1, 2) = Rxyz(1, 2);	T_0W(1, 3) = (-1) * 24.00;		 //���ܶ�-24���ܶ�-25�����1mm
			T_0W(2, 0) = Rxyz(2, 0);	T_0W(2, 1) = Rxyz(2, 1);	T_0W(2, 2) = Rxyz(2, 2);	T_0W(2, 3) = 18.00;
			T_0W(3, 0) = 0;				T_0W(3, 1) = 0;				T_0W(3, 2) = 0;				T_0W(3, 3) = 1;

			//
			T_tip_W = T_0W * T_tip_0;

			return T_tip_W;

		}






		//
		// Created by YuanSu on 01/02/23.
		// Hand model kinematic solution.
		//

		Eigen::Vector3f aux_xyz;
		
		///index finger	
		Eigen::Vector3f index_1_motion(float &theta1 , float &theta2)	
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180);
			aux_xyz(1) = 0.23 + 0.45 * sin(theta1 * PI / 180);
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180);		
			return aux_xyz;
		}
		Eigen::Vector3f index_2_motion(float& theta1, float& theta2, float& theta3)
		{
	
			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180) + 0.28 * sin((theta2 + theta3) * PI / 180);
			aux_xyz(1) = 0.23 + (0.45 + 0.28) * sin(theta1 * PI / 180);
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180) + 0.28 * cos((theta2 + theta3) * PI / 180);
		
			return aux_xyz;
		}
		Eigen::Vector3f index_3_motion(float&theta1, float& theta2, float& theta3, float& theta4)
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180) + 0.28 * sin((theta2 + theta3) * PI / 180) + 0.21 * sin((theta2 + theta3 + theta4) * PI / 180);
			aux_xyz(1) = 0.23 + (0.45 + 0.28 + 0.21) * sin(theta1 * PI / 180);
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180) + 0.28 * cos((theta2 + theta3) * PI / 180) + 0.21 * cos((theta2 + theta3 + theta4) * PI / 180);
			
			return aux_xyz;
		}

		///middle finger
		Eigen::Vector3f middle_1_motion(float& theta1, float& theta2)
		{						
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180);
			aux_xyz(1) = 0 + 0.48 * sin(theta1 * PI / 180);;
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180);
							
			return aux_xyz;
		}
		Eigen::Vector3f middle_2_motion(float& theta1, float& theta2, float& theta3)
		{							
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180) + 0.32 * sin((theta2 + theta3) * PI / 180);
			aux_xyz(1) = 0 + (0.48 + 0.32) * sin(theta1 * PI / 180);
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180) + 0.32 * cos((theta2 + theta3) * PI / 180);
				
			return aux_xyz;
		}
		Eigen::Vector3f middle_3_motion(float& theta1, float& theta2, float& theta3, float& theta4)
		{
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180) + 0.32 * sin((theta2 + theta3) * PI / 180) + 0.21 * sin((theta2 + theta3 + theta4) * PI / 180);
			aux_xyz(1) = 0 + (0.48 + 0.32 + 0.21) * sin(theta1 * PI / 180);
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180) + 0.32 * cos((theta2 + theta3) * PI / 180) + 0.21 * cos((theta2 + theta3 + theta4) * PI / 180);
							
			return aux_xyz;
		}

		/*
		Eigen::Vector3f index_1_motion(float& theta2)
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180);
			aux_xyz(1) = 0.27;
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180);
			return aux_xyz;
		}
		Eigen::Vector3f index_2_motion(float& theta2, float& theta3)
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180) + 0.28 * sin((theta2 + theta3) * PI / 180);
			aux_xyz(1) = 0.28;
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180) + 0.28 * cos((theta2 + theta3) * PI / 180);

			return aux_xyz;
		}
		Eigen::Vector3f index_3_motion(float& theta2, float& theta3, float& theta4)
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180) + 0.28 * sin((theta2 + theta3) * PI / 180) + 0.21 * sin((theta2 + theta3 + theta4) * PI / 180);
			aux_xyz(1) = 0.24;
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180) + 0.28 * cos((theta2 + theta3) * PI / 180) + 0.21 * cos((theta2 + theta3 + theta4) * PI / 180);

			return aux_xyz;
		}

		///middle finger
		Eigen::Vector3f middle_1_motion(float& theta2)
		{
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180);
			aux_xyz(1) = 0;
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180);

			return aux_xyz;
		}
		Eigen::Vector3f middle_2_motion(float& theta2, float& theta3)
		{
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180) + 0.32 * sin((theta2 + theta3) * PI / 180);
			aux_xyz(1) = 0;
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180) + 0.32 * cos((theta2 + theta3) * PI / 180);

			return aux_xyz;
		}
		Eigen::Vector3f middle_3_motion(float& theta2, float& theta3, float& theta4)
		{
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180) + 0.32 * sin((theta2 + theta3) * PI / 180) + 0.21 * sin((theta2 + theta3 + theta4) * PI / 180);
			aux_xyz(1) = 0;
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180) + 0.32 * cos((theta2 + theta3) * PI / 180) + 0.21 * cos((theta2 + theta3 + theta4) * PI / 180);

			return aux_xyz;
		}
		*/
		

		//
		Eigen::Vector3f thumb_1_motion(float& theta2)
		{	
			Ry = getRy(-90);
			Rz = getRz(45);
			Rx = getRx(-45);

			Rxyz = Ry * Rz * Rx;

			Eigen::Matrix4f T0W;
			T0W(0, 0) = Rxyz(0, 0);		T0W(0, 1) = Rxyz(0, 1);		T0W(0, 2) = Rxyz(0, 2);		T0W(0, 3) = 0;
			T0W(1, 0) = Rxyz(1, 0);		T0W(1, 1) = Rxyz(1, 1);		T0W(1, 2) = Rxyz(1, 2);		T0W(1, 3) = 0.19;
			T0W(2, 0) = Rxyz(2, 0);		T0W(2, 1) = Rxyz(2, 1);		T0W(2, 2) = Rxyz(2, 2);		T0W(2, 3) = -0.72;
			T0W(3, 0) = 0;				T0W(3, 1) = 0;				T0W(3, 2) = 0;				T0W(3, 3) = 1;
			Eigen::Matrix4f TP0;
			TP0(0, 0) = 1;				TP0(0, 1) = 0;				TP0(0, 2) = 0;				TP0(0, 3) = 0 + sqrt(0.2 * 0.2 + 0.32 * 0.32)*cos(theta2*PI/180);
			TP0(1, 0) = 0;				TP0(1, 1) = 1;				TP0(1, 2) = 0;				TP0(1, 3) = 0;
			TP0(2, 0) = 0;				TP0(2, 1) = 0;				TP0(2, 2) = 1;				TP0(2, 3) = 0 - sqrt(0.2 * 0.2 + 0.32 * 0.32)*sin(theta2*PI/180);
			TP0(3, 0) = 0;				TP0(3, 1) = 0;				TP0(3, 2) = 0;				TP0(3, 3) = 1;

			Eigen::Matrix4f TPW;
			TPW = T0W * TP0;

			aux_xyz(0) = TPW(0,3);
			aux_xyz(1) = TPW(1,3);
			aux_xyz(2) = TPW(2,3);

			return aux_xyz;
		}
		Eigen::Vector3f thumb_2_motion(float& theta2, float& theta3)
		{
			Ry = getRy(-90);
			Rz = getRz(45);
			Rx = getRx(-45);

			Rxyz = Ry * Rz * Rx;

			Eigen::Matrix4f T0W;
			T0W(0, 0) = Rxyz(0, 0);		T0W(0, 1) = Rxyz(0, 1);		T0W(0, 2) = Rxyz(0, 2);		T0W(0, 3) = 0;
			T0W(1, 0) = Rxyz(1, 0);		T0W(1, 1) = Rxyz(1, 1);		T0W(1, 2) = Rxyz(1, 2);		T0W(1, 3) = 0.19;
			T0W(2, 0) = Rxyz(2, 0);		T0W(2, 1) = Rxyz(2, 1);		T0W(2, 2) = Rxyz(2, 2);		T0W(2, 3) = -0.72;
			T0W(3, 0) = 0;				T0W(3, 1) = 0;				T0W(3, 2) = 0;				T0W(3, 3) = 1;
			Eigen::Matrix4f TP0;
			TP0(0, 0) = 1;				TP0(0, 1) = 0;				TP0(0, 2) = 0;				TP0(0, 3) = 0 + sqrt(0.2 * 0.2 + 0.32 * 0.32) * cos(theta2 * PI / 180) + sqrt(0.14 * 0.14 + 0.34 * 0.34) * cos((theta2 + theta3) * PI / 180);
			TP0(1, 0) = 0;				TP0(1, 1) = 1;				TP0(1, 2) = 0;				TP0(1, 3) = 0;
			TP0(2, 0) = 0;				TP0(2, 1) = 0;				TP0(2, 2) = 1;				TP0(2, 3) = 0 - sqrt(0.2 * 0.2 + 0.32 * 0.32) * sin(theta2 * PI / 180) - sqrt(0.14 * 0.14 + 0.34 * 0.34) * sin((theta2 + theta3) * PI / 180);
			TP0(3, 0) = 0;				TP0(3, 1) = 0;				TP0(3, 2) = 0;				TP0(3, 3) = 1;

			Eigen::Matrix4f TPW;
			TPW = T0W * TP0;

			aux_xyz(0) = TPW(0, 3);
			aux_xyz(1) = TPW(1, 3);
			aux_xyz(2) = TPW(2, 3);

			return aux_xyz;
		}
		Eigen::Vector3f thumb_3_motion(float& theta2, float& theta3, float& theta4)
		{
			Ry = getRy(-90);
			Rz = getRz(45);
			Rx = getRx(-45);

			Rxyz = Ry * Rz * Rx;

			Eigen::Matrix4f T0W;
			T0W(0, 0) = Rxyz(0, 0);		T0W(0, 1) = Rxyz(0, 1);		T0W(0, 2) = Rxyz(0, 2);		T0W(0, 3) = 0;
			T0W(1, 0) = Rxyz(1, 0);		T0W(1, 1) = Rxyz(1, 1);		T0W(1, 2) = Rxyz(1, 2);		T0W(1, 3) = 0.19;
			T0W(2, 0) = Rxyz(2, 0);		T0W(2, 1) = Rxyz(2, 1);		T0W(2, 2) = Rxyz(2, 2);		T0W(2, 3) = -0.72;
			T0W(3, 0) = 0;				T0W(3, 1) = 0;				T0W(3, 2) = 0;				T0W(3, 3) = 1;
			Eigen::Matrix4f TP0;
			TP0(0, 0) = 1;				TP0(0, 1) = 0;				TP0(0, 2) = 0;				TP0(0, 3) = 0 + sqrt(0.2 * 0.2 + 0.32 * 0.32) * cos(theta2 * PI / 180) + sqrt(0.14 * 0.14 + 0.34 * 0.34) * cos((theta2 + theta3) * PI / 180)+ sqrt(0.11 * 0.11 + 0.3 * 0.3) * cos((theta2 + theta3 + theta4) * PI / 180);
			TP0(1, 0) = 0;				TP0(1, 1) = 1;				TP0(1, 2) = 0;				TP0(1, 3) = 0;
			TP0(2, 0) = 0;				TP0(2, 1) = 0;				TP0(2, 2) = 1;				TP0(2, 3) = 0 - sqrt(0.2 * 0.2 + 0.32 * 0.32) * sin(theta2 * PI / 180) - sqrt(0.14 * 0.14 + 0.34 * 0.34) * sin((theta2 + theta3) * PI / 180)- sqrt(0.11 * 0.11 + 0.3 * 0.3) * sin((theta2+theta3+theta4)*PI/180);
			TP0(3, 0) = 0;				TP0(3, 1) = 0;				TP0(3, 2) = 0;				TP0(3, 3) = 1;

			Eigen::Matrix4f TPW;
			TPW = T0W * TP0;

			aux_xyz(0) = TPW(0, 3);
			aux_xyz(1) = TPW(1, 3);
			aux_xyz(2) = TPW(2, 3);

			return aux_xyz;
		}





		Eigen::Matrix4f auxF1TransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			//for simple calculate thumb

			T_tip_0 = getTransMatrix(theta1, theta2, theta3, theta4, theta5, theta6);
			
			Rx(0, 0) = 1;
			Rx(0, 1) = 0;
			Rx(0, 2) = 0;
			Rx(1, 0) = 0;
			Rx(1, 1) = cos(-90 * PI / 180);
			Rx(1, 2) = (-1) * sin(-90 * PI / 180);
			Rx(2, 0) = 0;
			Rx(2, 1) = sin(-90 * PI / 180);
			Rx(2, 2) = cos(-90 * PI / 180);

			Rxyz = Rx;

			//
			T_0W(0, 0) = Rxyz(0, 0);	T_0W(0, 1) = Rxyz(0, 1);	T_0W(0, 2) = Rxyz(0, 2);	T_0W(0, 3) = 0;
			T_0W(1, 0) = Rxyz(1, 0);	T_0W(1, 1) = Rxyz(1, 1);	T_0W(1, 2) = Rxyz(1, 2);	T_0W(1, 3) = 0;
			T_0W(2, 0) = Rxyz(2, 0);	T_0W(2, 1) = Rxyz(2, 1);	T_0W(2, 2) = Rxyz(2, 2);	T_0W(2, 3) = 0;		//�Ѳ���
			T_0W(3, 0) = 0;				T_0W(3, 1) = 0;				T_0W(3, 2) = 0;				T_0W(3, 3) = 1;

			//
			T_tip_W = T_0W*T_tip_0;
			
			return T_tip_W;

		}

        Eigen::Vector3f getThumbTipPosi(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6){
            Eigen::Matrix4f aux_matrix = getF1TransMatrix(theta1,theta2,theta3,theta4,theta5,theta6);
            Eigen::Vector3f ret_vector;
            ret_vector[0] = aux_matrix(0,3);
            ret_vector[1] = aux_matrix(1,3);
            ret_vector[2] = aux_matrix(2,3);
            return ret_vector;
        }

        Eigen::Matrix3f getThumbTipOri(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6){
            Eigen::Matrix4f aux_matrix = getF1TransMatrix(theta1,theta2,theta3,theta4,theta5,theta6);
            Eigen::Matrix3f ret_matrix;
            ret_matrix(0,0) = aux_matrix(0,0);
            ret_matrix(0,1) = aux_matrix(0,1);
            ret_matrix(0,2) = aux_matrix(0,2);
            ret_matrix(1,0) = aux_matrix(1,0);
            ret_matrix(1,1) = aux_matrix(1,1);
            ret_matrix(1,2) = aux_matrix(1,2);
            ret_matrix(2,0) = aux_matrix(2,0);
            ret_matrix(2,1) = aux_matrix(2,1);
            ret_matrix(2,2) = aux_matrix(2,2);
            return ret_matrix;
        }

        Eigen::Vector3f getIndexTipPosi(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6){
            Eigen::Matrix4f aux_matrix = getF2TransMatrix(theta1,theta2,theta3,theta4,theta5,theta6);
            Eigen::Vector3f ret_vector;
            ret_vector[0] = aux_matrix(0,3);
            ret_vector[1] = aux_matrix(1,3);
            ret_vector[2] = aux_matrix(2,3);
            return ret_vector;
        }

        Eigen::Matrix3f getIndexTipOri(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6){
            Eigen::Matrix4f aux_matrix = getF2TransMatrix(theta1,theta2,theta3,theta4,theta5,theta6);
            Eigen::Matrix3f ret_matrix;
            ret_matrix(0,0) = aux_matrix(0,0);
            ret_matrix(0,1) = aux_matrix(0,1);
            ret_matrix(0,2) = aux_matrix(0,2);
            ret_matrix(1,0) = aux_matrix(1,0);
            ret_matrix(1,1) = aux_matrix(1,1);
            ret_matrix(1,2) = aux_matrix(1,2);
            ret_matrix(2,0) = aux_matrix(2,0);
            ret_matrix(2,1) = aux_matrix(2,1);
            ret_matrix(2,2) = aux_matrix(2,2);
            return ret_matrix;
        }

        Eigen::Vector3f getMiddleTipPosi(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6){
            Eigen::Matrix4f aux_matrix = getF3TransMatrix(theta1,theta2,theta3,theta4,theta5,theta6);
            Eigen::Vector3f ret_vector;
            ret_vector[0] = aux_matrix(0,3);
            ret_vector[1] = aux_matrix(1,3);
            ret_vector[2] = aux_matrix(2,3);
            return ret_vector;
        }

        Eigen::Matrix3f getMiddleTipOri(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6){
            Eigen::Matrix4f aux_matrix = getF3TransMatrix(theta1,theta2,theta3,theta4,theta5,theta6);
            Eigen::Matrix3f ret_matrix;
            ret_matrix(0,0) = aux_matrix(0,0);
            ret_matrix(0,1) = aux_matrix(0,1);
            ret_matrix(0,2) = aux_matrix(0,2);
            ret_matrix(1,0) = aux_matrix(1,0);
            ret_matrix(1,1) = aux_matrix(1,1);
            ret_matrix(1,2) = aux_matrix(1,2);
            ret_matrix(2,0) = aux_matrix(2,0);
            ret_matrix(2,1) = aux_matrix(2,1);
            ret_matrix(2,2) = aux_matrix(2,2);
            return ret_matrix;
        }


        Eigen::Vector4f getThumbJoints(Eigen::Matrix4f &aux_finger1_pose, float &thumb_base_x, float &thumb_base_y, float &thumb_base_z, float &thumb_d){

            Eigen::Matrix4f F1_T_tip_w = aux_finger1_pose;
            Eigen::Matrix4f F1_0_W;

            Eigen::Matrix3f Rot_y;
            Rot_y(0, 0) = cos((-1) * 45 * PI / 180);
            Rot_y(0, 1) = 0;
            Rot_y(0, 2) = sin((-1) * 45 * PI / 180);
            Rot_y(1, 0) = 0;
            Rot_y(1, 1) = 1;
            Rot_y(1, 2) = 0;
            Rot_y(2, 0) = (-1) * sin((-1) * 45 * PI / 180);
            Rot_y(2, 1) = 0;
            Rot_y(2, 2) = cos((-1) * 45 * PI / 180);

            Eigen::Matrix3f Rot_z;
            Rot_z(0, 0) = cos(-45*PI/180);
            Rot_z(0, 1) = (-1) * sin(-45*PI/180);
            Rot_z(0, 2) = 0;
            Rot_z(1, 0) = sin(-45*PI/180);
            Rot_z(1, 1) = cos(-45*PI/180);
            Rot_z(1, 2) = 0;
            Rot_z(2, 0) = 0;
            Rot_z(2, 1) = 0;
            Rot_z(2, 2) = 1;

            Eigen::Matrix3f Rot_xyz = Rot_y * Rot_z;

            F1_0_W(0, 0) = Rot_xyz(0, 0);   F1_0_W(0, 1) = Rot_xyz(0, 1);   F1_0_W(0, 2) = Rot_xyz(0, 2);   F1_0_W(0, 3) = thumb_base_x;
            F1_0_W(1, 0) = Rot_xyz(1, 0);   F1_0_W(1, 1) = Rot_xyz(1, 1);   F1_0_W(1, 2) = Rot_xyz(1, 2);   F1_0_W(1, 3) = thumb_base_y;
            F1_0_W(2, 0) = Rot_xyz(2, 0);   F1_0_W(2, 1) = Rot_xyz(2, 1);   F1_0_W(2, 2) = Rot_xyz(2, 2);   F1_0_W(2, 3) = thumb_base_z;//-20
            F1_0_W(3, 0) = 0;   F1_0_W(3, 1) = 0;   F1_0_W(3, 2) = 0;   F1_0_W(3, 3) = 1;
            Eigen::Matrix4f F1_0_W_inv = F1_0_W.inverse();
            Eigen::Matrix4f F1_T_tip_0 = F1_0_W_inv * F1_T_tip_w;

            float thumb_d1 = thumb_d * 0.251 / (0.251 + 0.196 + 0.158);
            float thumb_d2 = thumb_d * 0.196 / (0.251 + 0.196 + 0.158);
            float thumb_d3 = thumb_d * 0.158 / (0.251 + 0.196 + 0.158);

            //theta1
            float thumb_theta1 = atan2(F1_T_tip_0(1, 3), F1_T_tip_0(0, 3)) * 180 / PI;

            //theta3
            float thumb_x_4 = sqrt((F1_T_tip_0(0, 3) * F1_T_tip_0(0, 3)) + (F1_T_tip_0(1, 3) * F1_T_tip_0(1, 3))) - (thumb_d3 * F1_T_tip_0(2, 2));

            float thumb_y_4 = thumb_d3 * F1_T_tip_0(2, 0) - F1_T_tip_0(2, 3);

            float thumb_cos_3 = (-1) * (((thumb_d1 * thumb_d1) + (thumb_d2 * thumb_d2) - (thumb_x_4 * thumb_x_4) - (thumb_y_4 * thumb_y_4)) / (2 * thumb_d1 * thumb_d2));

            if (thumb_cos_3 < -1) {
                thumb_cos_3 = -1;
            }
            else if (thumb_cos_3 > 1) {
                thumb_cos_3 = 1;
            }

            float thumb_sin_3 = sqrt(1 - (thumb_cos_3 * thumb_cos_3));

            float thumb_theta3 = atan2(thumb_sin_3, thumb_cos_3) * 180 / PI;

            //theta2
            float thumb_beta = atan2(thumb_y_4, thumb_x_4) * 180 / PI;

            float thumb_cos_gama = (thumb_d1 * thumb_d1 + thumb_x_4 * thumb_x_4 + thumb_y_4 * thumb_y_4 - thumb_d2 * thumb_d2) / (2 * thumb_d1 * (sqrt((thumb_x_4 * thumb_x_4 + thumb_y_4 * thumb_y_4))));

            if (thumb_cos_gama < -1) {
                thumb_cos_gama = -1;
            }
            else if (thumb_cos_gama > 1) {
                thumb_cos_gama = 1;
            }

            float thumb_sin_gama = sqrt((1 - (thumb_cos_gama * thumb_cos_gama)));

            float thumb_gama = atan2(thumb_sin_gama, thumb_cos_gama) * 180 / PI;

            float thumb_theta2 = thumb_beta - thumb_gama;

            //theta4
            float thumb_theta4 = 0.2 * thumb_theta3;

            Eigen::Vector4f ret_thumb_joints;
            ret_thumb_joints[0] = thumb_theta1;
            ret_thumb_joints[1] = thumb_theta2;
            ret_thumb_joints[2] = thumb_theta3;
            ret_thumb_joints[3] = thumb_theta4;

            return ret_thumb_joints;
        }

        Eigen::Vector4f getIndexJoints(Eigen::Matrix4f &finger2_pose, float &index_base_x, float &index_base_y, float &index_base_z, float & index_d){

            Eigen::Matrix4f F2_T_tip_w = finger2_pose;
            Eigen::Matrix4f F2_0_W;
            F2_0_W(0, 0) = 1;   F2_0_W(0, 1) = 0;   F2_0_W(0, 2) = 0;    F2_0_W(0, 3) = index_base_x;
            F2_0_W(1, 0) = 0;   F2_0_W(1, 1) = 1;   F2_0_W(1, 2) = 0;    F2_0_W(1, 3) = index_base_y;
            F2_0_W(2, 0) = 0;   F2_0_W(2, 1) = 0;   F2_0_W(2, 2) = 1;    F2_0_W(2, 3) = index_base_z;
            F2_0_W(3, 0) = 0;   F2_0_W(3, 1) = 0;   F2_0_W(3, 2) = 0;    F2_0_W(3, 3) = 1;
            Eigen::Matrix4f F2_0_W_inv = F2_0_W.inverse();
            Eigen::Matrix4f F2_T_tip_0 = F2_0_W_inv * F2_T_tip_w;

            float index_d1 = index_d * 0.245/(0.245 + 0.143 + 0.097);    //46
            float index_d2 = index_d * 0.143/(0.245 + 0.143 + 0.097);    //19
            float index_d3 = index_d * 0.097/(0.245 + 0.143 + 0.097);    //21

            //theta1
            float index_theta1 = atan2(F2_T_tip_0(1, 3), F2_T_tip_0(0, 3)) * 180 / PI;

            //theta3
            float index_x_4 = sqrt((F2_T_tip_0(0, 3) * F2_T_tip_0(0, 3)) + (F2_T_tip_0(1, 3) * F2_T_tip_0(1, 3))) - (index_d3 * F2_T_tip_0(2, 2));

            float index_y_4 = index_d3 * F2_T_tip_0(2, 0) - F2_T_tip_0(2, 3);

            float index_cos_3 = (-1) * (((index_d1 * index_d1) + (index_d2 * index_d2) - (index_x_4 * index_x_4) - (index_y_4 * index_y_4)) / (2 * index_d1 * index_d2));

            if (index_cos_3 < -1) {
                index_cos_3 = -1;
            }
            else if (index_cos_3 > 1) {
                index_cos_3 = 1;
            }

            float index_sin_3 = sqrt(1 - (index_cos_3 * index_cos_3));

            float index_theta3 = atan2(index_sin_3, index_cos_3) * 180 / PI;

            //theta2
            float index_beta = atan2(index_y_4, index_x_4) * 180 / PI;

            float index_cos_gama = (index_d1 * index_d1 + index_x_4 * index_x_4 + index_y_4 * index_y_4 - index_d2 * index_d2) / (2 * index_d1 * (sqrt((index_x_4 * index_x_4 + index_y_4 * index_y_4))));

            if (index_cos_gama < -1) {
                index_cos_gama = -1;
            }
            else if (index_cos_gama > 1) {
                index_cos_gama = 1;
            }

            float index_sin_gama = sqrt((1 - (index_cos_gama * index_cos_gama)));

            float index_gama = atan2(index_sin_gama, index_cos_gama) * 180 / PI;

            float index_theta2 = index_beta - index_gama;

            //theta4
            float index_theta4 = 0.32 * index_theta3;

            Eigen::Vector4f ret_index_joints;
            ret_index_joints[0] = index_theta1;
            ret_index_joints[1] = index_theta2;
            ret_index_joints[2] = index_theta3;
            ret_index_joints[3] = index_theta4;

            return ret_index_joints;
        }

        Eigen::Vector4f getMiddleJoints(Eigen::Matrix4f &finger3_pose, float &middle_base_x, float &middle_base_y, float middle_base_z, float middle_d){

            Eigen::Matrix4f F3_T_tip_w = finger3_pose;
            Eigen::Matrix4f F3_0_W;
            F3_0_W(0, 0) = 1;   F3_0_W(0, 1) = 0;   F3_0_W(0, 2) = 0;   F3_0_W(0, 3) = middle_base_x;
            F3_0_W(1, 0) = 0;   F3_0_W(1, 1) = 1;   F3_0_W(1, 2) = 0;   F3_0_W(1, 3) = middle_base_y;
            F3_0_W(2, 0) = 0;   F3_0_W(2, 1) = 0;   F3_0_W(2, 2) = 1;   F3_0_W(2, 3) = middle_base_z;
            F3_0_W(3, 0) = 0;   F3_0_W(3, 1) = 0;   F3_0_W(3, 2) = 0;   F3_0_W(3, 3) = 1;
            Eigen::Matrix4f F3_0_W_inv = F3_0_W.inverse();
            Eigen::Matrix4f F3_T_tip_0 = F3_0_W_inv * F3_T_tip_w;

            float middle_d1 = middle_d * 0.266 / (0.266 + 0.170 + 0.108);
            float middle_d2 = middle_d * 0.170 / (0.266 + 0.170 + 0.108);
            float middle_d3 = middle_d * 0.108 / (0.266 + 0.170 + 0.108);

            //theta1
            float middle_theta1 = atan2(F3_T_tip_0(1, 3), F3_T_tip_0(0, 3)) * 180 / PI;

            //theta3
            float middle_x_4 = sqrt((F3_T_tip_0(0, 3) * F3_T_tip_0(0, 3)) + (F3_T_tip_0(1, 3) * F3_T_tip_0(1, 3))) - (middle_d3 * F3_T_tip_0(2, 2));

            float middle_y_4 = middle_d3 * F3_T_tip_0(2, 0) - F3_T_tip_0(2, 3);

            float middle_cos_3 = (-1) * (((middle_d1 * middle_d1) + (middle_d2 * middle_d2) - (middle_x_4 * middle_x_4) - (middle_y_4 * middle_y_4)) / (2 * middle_d1 * middle_d2));

            if (middle_cos_3 < -1) {
                middle_cos_3 = -1;
            }
            else if (middle_cos_3 > 1) {
                middle_cos_3 = 1;
            }

            float middle_sin_3 = sqrt(1 - (middle_cos_3 * middle_cos_3));

            float middle_theta3 = atan2(middle_sin_3, middle_cos_3) * 180 / PI;

            //theta2
            float middle_beta = atan2(middle_y_4, middle_x_4) * 180 / PI;

            float middle_cos_gama = (middle_d1 * middle_d1 + middle_x_4 * middle_x_4 + middle_y_4 * middle_y_4 - middle_d2 * middle_d2) / (2 * middle_d1 * (sqrt((middle_x_4 * middle_x_4 + middle_y_4 * middle_y_4))));

            if (middle_cos_gama < -1) {
                middle_cos_gama = -1;
            }
            else if (middle_cos_gama>1) {
                middle_cos_gama = 1;
            }

            float middle_sin_gama = sqrt((1 - (middle_cos_gama * middle_cos_gama)));

            float middle_gama = atan2(middle_sin_gama, middle_cos_gama) * 180 / PI;

            float middle_theta2 = middle_beta - middle_gama;

            //theta4
            float middle_theta4 = 0.36 * middle_theta3;

            Eigen::Vector4f ret_middle_joints;
            ret_middle_joints[0] = middle_theta1;
            ret_middle_joints[1] = middle_theta2;
            ret_middle_joints[2] = middle_theta3;
            ret_middle_joints[3] = middle_theta4;

            return ret_middle_joints;
        }





	}
}

#endif // !_FINGER_KINEMATICS_

