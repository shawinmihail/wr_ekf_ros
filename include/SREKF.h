#pragma once
#include "Definitions.h"
#include "Utils.h"

// state X = [r v a q w]
#define SREKF_STATE_DIM 16
typedef Eigen::Matrix<double, SREKF_STATE_DIM, 1> EkfStateVector;


class SREKF
{
public:
	SREKF();
	void predictImu(const Vector3& aMes, const Vector3& wMes, double dt);
	void correctPv(const Vector6& pv);
	void correctV(const Vector3& v);
	void correctA(const Vector3& a);
    void correctP3(const Vector3& dr1, const Vector3& dr2);
	EkfStateVector getEstState();

private:
	EkfStateVector _X;
	Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM> _Q;
	Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM> _sqrtP;

	Eigen::Matrix<double, 6, 6> _sqrtR_pv;
	Eigen::Matrix<double, 6, 6> _sqrtR_p3;
	Eigen::Matrix<double, 3, 3> _sqrtR_v;
	Eigen::Matrix<double, 3, 3> _sqrtR_a;
	Eigen::Matrix<double, 12, 12> _sqrtR_z;

	Vector3 _gpsAttachmentShift;
    Vector3 _gpsSlave1;
	Vector3 _gpsSlave2;

	// constants
	Eigen::Matrix<double, 3, 3> O33;
	Eigen::Matrix<double, 3, 4> O34;
	Eigen::Matrix<double, 4, 3> O43;
	Eigen::Matrix<double, 3, 3> E33;
};

