#pragma once
#include <stdio.h>
#include "Definitions.h"

// quat

Vector4 quatFromEul(const Vector3& eul);

Vector3 quat2Eul(const Vector4& q);

Vector3 quatToQuatVec(const Vector4& quat); // assume quat scalar part quat[0] > 0;

Vector4 quatVecToQuat(const Vector3& quat); // assume quat scalar part quat[0] > 0;

Vector4 quatMultiply(const Vector4& q1, const Vector4& q2);

Vector4 quatInverse(const Vector4& q);

Vector3 quatRotate(const Vector4& q, const Vector3& v);

Eigen::Matrix<double, 3, 3> quatToMatrix(const Vector4& q);

Eigen::Matrix<double, 3, 3> crossOperator( const Vector3& v);

// lin

Eigen::Matrix<double, 3, 4> quatRotateLinearizationQ(const Vector4& q, const Vector3& v);

Eigen::Matrix<double, 4, 4> poissonEqLinearizationQ(const Vector3& w);

Eigen::Matrix<double, 4, 3> poissonEqLinearizationW(const Vector4& q, const Vector3& w);

Eigen::Matrix<double, 1, 3> normVect3Linearization(const Vector3& v);

// ll2ne

void LLtoNE(double dLat, double dLon, double*NE);



