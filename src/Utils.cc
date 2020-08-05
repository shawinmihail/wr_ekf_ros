#include "Utils.h"
#include <math.h>

const double UTILS_EPS = 1e-6f;

Vector4 quatFromEul(const Vector3& eul)
{
	 double roll = eul[0];
	 double pitch = eul[1];
	 double yaw = eul[2];

	 double cy = cos(yaw * 0.5);
	 double sy = sin(yaw * 0.5);
	 double cr = cos(roll * 0.5);
	 double sr = sin(roll * 0.5);
	 double cp = cos(pitch * 0.5);
	 double sp = sin(pitch * 0.5);

	 double q0 = cy * cr * cp + sy * sr * sp;
	 double q1 = cy * sr * cp - sy * cr * sp;
	 double q2 = cy * cr * sp + sy * sr * cp;
	 double q3 = sy * cr * cp - cy * sr * sp;

	 return Vector4(q0, q1, q2 ,q3);
}

Vector3 quat2Eul(const Vector4& q)
{
    double sinr = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    double cosr = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    double roll = atan2(sinr, cosr);

    double sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);
    double pitch = 0.;
    if (abs(sinp) >= 1) {
        pitch = sinp / fabs (sinp) * 3.1415 / 2.0;
    }
    else {
        pitch = asin(sinp);
    }

    double siny = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    double cosy = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
    double yaw = atan2(siny, cosy);
    return Vector3(roll, pitch, yaw);
}

Vector3 quatToQuatVec(const Vector4& q) // assume quat scalar part quat[0] > 0;
{
	Vector3 qv(q[1], q[2], q[3]);

	if (q[0] < 0.0)
	{
		double sinHalfAlpha = qv.norm();
		if (sinHalfAlpha < UTILS_EPS)
		{
			qv = Vector3(0.0, 0.0, 0.0);
			return qv;
		};
		if (sinHalfAlpha > 1.0)
		{
			sinHalfAlpha = 1.0 - UTILS_EPS; // garanteed for asin exists
		}
		qv = qv / sinHalfAlpha; // pin
		double alpha = 2.0 * asin(sinHalfAlpha);
		double pi = 3.1415; // use WGS4 PI here;
		double alphaNew = 2.0 * pi - alpha; // rotate to another dir

		double sinHalfNewAlpha = sin(alphaNew / 2.0);
		qv = -qv * sinHalfNewAlpha;
	}
	return qv;
}

Vector4 quatVecToQuat(const Vector3& qv) // assume quat scalar part quat[0] > 0;
{
	double q0Square = 1 - qv[0] * qv[0] - qv[1] * qv[1] - qv[2] * qv[2];
	if (q0Square < 0.0) // possible in case of numerical integration error
	{
		q0Square = UTILS_EPS;
	} 
	double q0 = sqrt(q0Square);

	Vector4 q(q0, qv[0], qv[1], qv[2]);
	q = q / q.norm();

	return q;
}

Vector4 quatMultiply(const Vector4& q, const Vector4& r)
{
	Vector4 p;
	p[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
	p[1] = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
	p[2] = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
	p[3] = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];
	return p;
}

Vector4 quatInverse(const Vector4& q)
{
	Vector4 qDual (q[0], -q[1], -q[2], -q[3]);
	return qDual;
}

Vector3 quatRotate(const Vector4& q, const Vector3& v)
{
	Vector4 qv(0.0f, v[0], v[1], v[2]);

	Vector4 qDual = quatInverse(q);
	Vector4 qv1 = quatMultiply(qv, qDual);
	Vector4 qv2 = quatMultiply(q, qv1);

	return Vector3(qv2[1], qv2[2], qv2[3]);
}

Eigen::Matrix<double, 3, 3> quatToMatrix(const Vector4& q)
{
	double R11 = 1.0 - 2.0 * q(2) * q(2) - 2.0 * q(3) * q(3);
	double R12 = 2.0 * q(1) * q(2) - 2.0 * q(3) * q(0);
	double R13 = 2.0 * q(1) * q(3) + 2.0 * q(2) * q(0);

	double R21 = 2.0 * q(1) * q(2) + 2.0 * q(3) * q(0);
	double R22 = 1.0 - 2.0 * q(1) * q(1) - 2.0 * q(3) * q(3);
	double R23 = 2.0 * q(2) * q(3) - 2.0 * q(1) * q(0);

	double R31 = 2.0 * q(1) * q(3) - 2.0 * q(2) * q(0);
	double R32 = 2.0 * q(2) * q(3) + 2.0 * q(1) * q(0);
	double R33 = 1.0 - 2.0 * q(1) * q(1) - 2.0 * q(2) * q(2);

	Eigen::Matrix<double, 3, 3> R;
	R << R11, R12, R13, R21, R22, R23, R31, R32, R33;
	return R;
}

Eigen::Matrix<double, 3, 3> crossOperator(const Vector3& v)
{
	Eigen::Matrix<double, 3, 3> R;
	R << 0.0, -v(2), v(1),
		 v(2), 0.0, -v(0),
		-v(1), v(0), 0.0;
	return R;
}

Eigen::Matrix<double, 3, 4> quatRotateLinearizationQ(const Vector4& q, const Vector3& v)
{
	// f = q * v * q_dual
	// res = df/dq

	/*
	rddot_q_j(q1, q2, q3, q4) =
	[2 * a1 * q1 - 2 * a2 * q4 + 2 * a3 * q3, 2 * a1 * q2 + 2 * a2 * q3 + 2 * a3 * q4, 2 * a2 * q2 - 2 * a1 * q3 + 2 * a3 * q1, 2 * a3 * q2 - 2 * a1 * q4 - 2 * a2 * q1]
	[2 * a2 * q1 + 2 * a1 * q4 - 2 * a3 * q2, 2 * a1 * q3 - 2 * a2 * q2 - 2 * a3 * q1, 2 * a1 * q2 + 2 * a2 * q3 + 2 * a3 * q4, 2 * a1 * q1 - 2 * a2 * q4 + 2 * a3 * q3]
	[2 * a2 * q2 - 2 * a1 * q3 + 2 * a3 * q1, 2 * a2 * q1 + 2 * a1 * q4 - 2 * a3 * q2, 2 * a2 * q4 - 2 * a1 * q1 - 2 * a3 * q3, 2 * a1 * q2 + 2 * a2 * q3 + 2 * a3 * q4]
	*/

	Eigen::Matrix<double, 3, 4> res;
	res << v(0) * q(0) - v(1) * q(3) + v(2) * q(2), v(0)* q(1) + (2) * v(1) * q(2) + v(2) * q(3), v(1)* q(1) - v(0) * q(2) + v(2) * q(0), v(2)* q(1) - v(0) * q(3) - v(1) * q(0),
		   v(1) * q(0) + v(0) * q(3) - v(2) * q(1), v(0)* q(2) - (1) * v(1) * q(1) - v(2) * q(0), v(0)* q(1) + v(1) * q(2) + v(2) * q(3), v(0)* q(0) - v(1) * q(3) + v(2) * q(2),
		   v(1) * q(1) - v(0) * q(2) + v(2) * q(0), v(1)* q(0) + (1) * v(0) * q(3) - v(2) * q(1), v(1)* q(3) - v(0) * q(0) - v(2) * q(2), v(0)* q(1) + v(1) * q(2) + v(2) * q(3);

	return 2.f*res;
}

Eigen::Matrix<double, 4, 4> poissonEqLinearizationQ(const Vector3& w)
{
	// f = 0.5 * q0 * qw0
	// res = df/dq

	/*
	qdot_q_j(q1, q2, q3, q4) =
	[    0, -w1/2, -w2/2, -w3/2]
	[ w1/2,     0,  w3/2, -w2/2]
	[ w2/2, -w3/2,     0,  w1/2]
	[ w3/2,  w2/2, -w1/2,     0]
	*/

	Eigen::Matrix<double, 4, 4> res;
	res << 0, -w(0) / 2.0, -w(1) / 2.0, -w(2) / 2.0,
		  w(0) / 2.0, 0, w(2) / 2.0, -w(1) / 2.0,
		  w(1) / 2.0, -w(2) / 2.0, 0, w(0) / 2.0,
		  w(2) / 2.0, w(1) / 2.0, -w(0) / 2.0, 0;

	return res;
}

Eigen::Matrix<double, 4, 3> poissonEqLinearizationW(const Vector4& q, const Vector3& w)
{
	// f = 0.5 * q0 * qw0
	// res = df/dq

	/*
	qdot_w_j(w1, w2, w3) =

	[ -q2/2, -q3/2, -q4/2]
	[  q1/2, -q4/2,  q3/2]
	[  q4/2,  q1/2, -q2/2]
	[ -q3/2,  q2/2,  q1/2]
	*/

	Eigen::Matrix<double, 4, 3> res;
	res << -q(1) / 2.0, -q(2) / 2.0, -q(3) / 2.0,
		    q(0) / 2.0, -q(3) / 2.0, q(2) / 2.0,
			q(3) / 2.0, q(0) / 2.0, -q(1) / 2.0,
			-q(2) / 2.0, q(1) / 2.0, q(0) / 2.0;

	return res;
}

Eigen::Matrix<double, 1, 3> normVect3Linearization(const Vector3& v)
{
	Eigen::Matrix<double, 1, 3> res;
	double vn = v.norm();
	res << v(0) / vn, v(1) / vn, v(2) / vn;
	return res;
}

void LLtoNE(double dLat, double dLon, double* NE)
{
  double PI = 3.14159265358979;
  int zone = (int)dLon/6+1;

  double a = 6378245.0;
  double b = 6356863.019;
  double e2 = (a*a-b*b)/(a*a);
  double n = (a-b)/(a+b);

  double F = 1.0;
  double Lat0 = 0.0;
  double Lon0 = (zone*6-3)*PI/180;
  double N0 = 0.0;
  double E0 = zone*1000000+500000.0;

  double Lat = dLat*PI/180.0;
  double Lon = dLon*PI/180.0;

  double sinLat = sin(Lat);
  double cosLat = cos(Lat);
  double tanLat = tan(Lat);

  double v = a*F*pow(1-e2*pow(sinLat,2),-0.5);
  double p = a*F*(1-e2)*pow(1-e2*pow(sinLat,2),-1.5);
  double n2 = v/p-1;
  double M1 = (1+n+5.0/4.0*pow(n,2)+5.0/4.0*pow(n,3))*(Lat-Lat0);
  double M2 = (3*n+3*pow(n,2)+21.0/8.0*pow(n,3))*sin(Lat - Lat0)*cos(Lat + Lat0);
  double M3 = (15.0/8.0*pow(n,2)+15.0/8.0*pow(n,3))*sin(2*(Lat - Lat0))*cos(2*(Lat + Lat0));
  double M4 = 35.0/24.0*pow(n,3)*sin(3*(Lat - Lat0))*cos(3*(Lat + Lat0));
  double M = b*F*(M1-M2+M3-M4);
  double I = M+N0;
  double II = v/2 * sinLat * cosLat;
  double III = v/24 * sinLat * pow(cosLat,3) * (5-pow(tanLat,2)+9*n2);
  double IIIA = v/720 * sinLat * pow(cosLat,5) * (61-58*pow(tanLat,2)+pow(tanLat,4));
  double IV = v * cosLat;
  double V = v/6 * pow(cosLat,3) * (v/p-pow(tanLat,2));
  double VI = v/120 * pow(cosLat,5) * (5-18*pow(tanLat,2)+pow(tanLat,4)+14*n2-58*pow(tanLat,2)*n2);

  double N = I+II*pow(Lon-Lon0,2)+III*pow(Lon-Lon0,4)+IIIA*pow(Lon-Lon0,6);
  double E = E0+IV*(Lon-Lon0)+V*pow(Lon-Lon0,3)+VI*pow(Lon-Lon0,5);

  NE[0] = N;
  NE[1] = E;
}

