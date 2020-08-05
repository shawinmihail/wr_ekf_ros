#include "SREKF.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>

template <int a, int b>
Eigen::Matrix<double, a, a> getLowTriang(const Eigen::Matrix<double, a, b>& A)
{
	Eigen::Matrix<double, b, a> A_transposed = A.transpose();
	Eigen::HouseholderQR<Eigen::Matrix<double, b, a>> qrHolder;
	qrHolder.compute(A_transposed);
	Eigen::Matrix<double, b, a> R = qrHolder.matrixQR().template triangularView<Eigen::Upper>();
	Eigen::Matrix<double, a, b> L = R.transpose();
	Eigen::Matrix<double, a, a> cutedL = L.block(0, 0, a, a);

	return -cutedL;
}

template <int a>
Eigen::Matrix<double, a, a> cholUpdate(const Eigen::Matrix<double, a, a>& A)
{
	Eigen::LLT<Eigen::Matrix<double, a, a>> cdA(A);
	return cdA.matrixL();
}

SREKF::SREKF() :
	// consts
	O33(Eigen::Matrix<double, 3, 3>::Zero()),
	O34(Eigen::Matrix<double, 3, 4>::Zero()),
	O43(Eigen::Matrix<double, 4, 3>::Zero()),
	E33(Eigen::Matrix<double, 3, 3>::Identity())
{
	_X = EkfStateVector::Zero();
	_X(9) = 1.0;

	// init Q
	EkfStateVector qDiag;
	qDiag << /*r*/ 1e-2, 1e-2, 1e-2, /*v*/ 1e-2, 1e-2, 1e-2, /*a*/ 1e-2, 1e-2, 1e-2, /*q*/ 1e-4, 1e-4, 1e-4, 1e-4, /*w*/ 1e-2, 1e-2, 1e-2;
	_Q = qDiag.asDiagonal();

	// init P
	EkfStateVector pDiag = 30 * qDiag;
	Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM> P = pDiag.asDiagonal();
	_sqrtP = cholUpdate<SREKF_STATE_DIM>(P);

	// init R_pv
	Vector6 rDiag_pv;
	rDiag_pv << /*r*/ 1e-4, 1e-4, 1e-4, /*v*/ 2e-4, 2e-4, 2e-4;
	Eigen::Matrix<double, 6, 6> R_pv = rDiag_pv.asDiagonal();
	_sqrtR_pv = cholUpdate<6>(R_pv);

	// init R_v
	Vector3 rDiag_v;
	rDiag_v <<  /*v*/ 2e-4, 2e-4, 2e-4;
	Eigen::Matrix<double, 3, 3> R_v = rDiag_v.asDiagonal();
	_sqrtR_v = cholUpdate<3>(R_v);

	// init R_a
	Vector3 rDiag_a;
	rDiag_a << /*a*/ 0.0025, 0.0025, 0.0025;
	Eigen::Matrix<double, 3, 3> R_a = rDiag_a.asDiagonal();
	_sqrtR_a = cholUpdate<3>(R_a);

	// init R_p3
	Vector6 rDiag_p3;
	rDiag_p3 <<  1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
	Eigen::Matrix<double, 6, 6> R_p3 = rDiag_p3.asDiagonal();
	_sqrtR_p3 = cholUpdate<6>(R_p3);

	//gpsAttachmentShift
	_gpsAttachmentShift << 0.3, 0.0, 0.5;
	_gpsSlave1 << -0.5, 0.2, 0.0;
	_gpsSlave2 << -0.5, -0.2, 0.0;
}

void SREKF::predictImu(const Vector3& aMes, const Vector3& wMes, double dt)
{

	/* state */
	Vector3 g;
	g << 0.0, 0.0, -9.8;
	Vector3 r0 = _X.segment(0, 3);
	Vector3 v0 = _X.segment(3, 3);
	Vector4 q0 = _X.segment(9, 4);
	Vector3 w0 = wMes;
	Vector3 a0 = quatRotate(q0, aMes) + g;

	Vector4 qw0(0.0f, w0[0], w0[1], w0[2]);
	Vector3 r1 = r0 + v0 * dt;
	Vector3 v1 = v0 + a0 * dt;
	Vector3 a1 = a0;
	Vector4 q1 = q0 + 0.5 * quatMultiply(q0, qw0) * dt;
	//Vector3 q1 = q_next / norm(q_next);
	Vector3 w1 = w0;

	_X << r1, v1, a1, q1, w1;

	/* cov */
	Eigen::Matrix<double, 3, 4> Maq = quatRotateLinearizationQ(q0, aMes);
	Eigen::Matrix<double, 4, 4> Mqq = poissonEqLinearizationQ(wMes);
	Eigen::Matrix<double, 4, 3> Mqw = poissonEqLinearizationW(q0, wMes);

	Eigen::Matrix<double, 3, SREKF_STATE_DIM> Fr;
	Eigen::Matrix<double, 3, SREKF_STATE_DIM> Fv;
	Eigen::Matrix<double, 3, SREKF_STATE_DIM> Fa;
	Eigen::Matrix<double, 4, SREKF_STATE_DIM> Fq;
	Eigen::Matrix<double, 3, SREKF_STATE_DIM> Fw;

	Fr << O33, E33, O33, O34, O33;
	Fv << O33, O33, E33, Maq, O33;
	Fa << O33, O33, O33, O34, O33;
	Fq << O43, O43, O43, Mqq, Mqw;
	Fw << O33, O33, O33, O34, O33;

	Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM> F;
	F << Fr, Fv, Fa, Fq, Fw;

	Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM> Enn(Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM>::Identity());

	Eigen::Matrix<double, SREKF_STATE_DIM, 2 * SREKF_STATE_DIM> triaArg;
	triaArg << (Enn + F * dt) * _sqrtP, _Q * sqrt(dt);

	_sqrtP = getLowTriang<SREKF_STATE_DIM, 2 * SREKF_STATE_DIM>(triaArg);
}

void SREKF::correctPv(const Vector6& pv)
{
	Vector3 r = _X.segment(0, 3);
	Vector3 v = _X.segment(3, 3);
	Vector3 a = _X.segment(6, 3);
	Vector4 q = _X.segment(9, 4);
	Vector3 w = _X.segment(13, 3);

	// mes model
	// Z[rgnns vgnns]
	Vector3 Zr = r + quatRotate(q, _gpsAttachmentShift);
	Vector3 Zv = v + quatRotate(q, w.cross(_gpsAttachmentShift));
	Vector6 Zx;
	Zx << Zr, Zv;
	Vector6 dz = pv - Zx;

	// H
	Eigen::Matrix<double, 3, 4> Zrq = quatRotateLinearizationQ(q, _gpsAttachmentShift);
	Eigen::Matrix<double, 3, 4> Zvq = quatRotateLinearizationQ(q, w.cross(_gpsAttachmentShift));
	Eigen::Matrix<double, 3, 3> Zvw = quatToMatrix(q) * crossOperator(-_gpsAttachmentShift);

	Eigen::Matrix<double, 3, SREKF_STATE_DIM> H1;
	Eigen::Matrix<double, 3, SREKF_STATE_DIM> H2;
	Eigen::Matrix<double, 6, SREKF_STATE_DIM> H;
	H1 << E33, O33, O33, Zrq, O33;
	H2 << O33, E33, O33, Zvq, Zvw;
	H << H1, H2;

	// square-root
	Eigen::Matrix<double, SREKF_STATE_DIM + 6, SREKF_STATE_DIM + 6> triaArg;
	Eigen::Matrix<double, 6, SREKF_STATE_DIM + 6> triaArg1;
	Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM + 6> triaArg2;
	triaArg1 << _sqrtR_pv, H * _sqrtP;

	triaArg2 << Eigen::Matrix<double, SREKF_STATE_DIM, 6>::Zero(), _sqrtP;
	triaArg << triaArg1, triaArg2;

	Eigen::Matrix<double, SREKF_STATE_DIM + 6, SREKF_STATE_DIM + 6> M = getLowTriang<SREKF_STATE_DIM + 6, SREKF_STATE_DIM + 6>(triaArg);
	Eigen::Matrix<double, 6, 6> sqrtRk = M.block<6, 6>(0, 0);
	Eigen::Matrix<double, SREKF_STATE_DIM, 6> K = M.block<SREKF_STATE_DIM, 6>(6, 0);
	_sqrtP = M.block<SREKF_STATE_DIM, SREKF_STATE_DIM>(6, 6);

	_X = _X + K * (sqrtRk.transpose().inverse())*dz;
	_X.segment(9, 4).normalize();
}

void SREKF::correctV(const Vector3& vMes)
{
	Vector3 r = _X.segment(0, 3);
	Vector3 v = _X.segment(3, 3);
	Vector3 a = _X.segment(6, 3);
	Vector4 q = _X.segment(9, 4);
	Vector3 w = _X.segment(13, 3);

	Vector3 ex(1.0, 0.0, 0.0);

	// mes model
	// Z[vgnns]
	Vector3 Zx = quatRotate(q, ex * v.norm()) + quatRotate(q, w.cross(_gpsAttachmentShift));
	Vector3 dz = vMes - Zx;

	// H
	Eigen::Matrix<double, 3, 4> Zuq = quatRotateLinearizationQ(q, ex * v.norm() + w.cross(_gpsAttachmentShift));
	Eigen::Matrix<double, 3, 3> Zuv = Eigen::Matrix<double, 3, 3>::Zero();
	Zuv.row(0) = normVect3Linearization(v); // !!! v not zero chek add
	Zuv = quatToMatrix(q) * Zuv;
	Eigen::Matrix<double, 3, 3> Zuw = quatToMatrix(q) * crossOperator(-_gpsAttachmentShift);

	Eigen::Matrix<double, 3, SREKF_STATE_DIM> H;
	H << O33, Zuv, O33, Zuq, Zuw;

	// square-root
	Eigen::Matrix<double, SREKF_STATE_DIM + 3, SREKF_STATE_DIM + 3> triaArg;
	Eigen::Matrix<double, 3, SREKF_STATE_DIM + 3> triaArg1;
	Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM + 3> triaArg2;
	triaArg1 << _sqrtR_v, H * _sqrtP;

	triaArg2 << Eigen::Matrix<double, SREKF_STATE_DIM, 3>::Zero(), _sqrtP;
	triaArg << triaArg1, triaArg2;

	Eigen::Matrix<double, SREKF_STATE_DIM + 3, SREKF_STATE_DIM + 3> M = getLowTriang<SREKF_STATE_DIM + 3, SREKF_STATE_DIM + 3>(triaArg);
	Eigen::Matrix<double, 3, 3> sqrtRk = M.block<3, 3>(0, 0);
	Eigen::Matrix<double, SREKF_STATE_DIM, 3> K = M.block<SREKF_STATE_DIM, 3>(3, 0);
	_sqrtP = M.block<SREKF_STATE_DIM, SREKF_STATE_DIM>(3, 3);

	_X = _X + K * (sqrtRk.transpose().inverse()) * dz;
	_X.segment(9, 4).normalize();
}

void SREKF::correctA(const Vector3& aMes)
{
	Vector3 g(0.0, 0.0, -9.8);

	Vector3 r = _X.segment(0, 3);
	Vector3 v = _X.segment(3, 3);
	Vector3 a = _X.segment(6, 3);
	Vector4 q = _X.segment(9, 4);
	Vector3 w = _X.segment(13, 3);

	Vector3 ex(1.0, 0.0, 0.0);

	// mes model
	// Z[vgnns]
	Vector3 Zx = quatRotate(quatInverse(q), a - g);
	Vector3 dz = aMes - Zx;

	// H
	Eigen::Matrix<double, 3, 4> Zaq = quatRotateLinearizationQ(quatInverse(q), a - g);
	Zaq.block<3, 3>(0, 1) = -Zaq.block<3, 3>(0, 1);
	Eigen::Matrix<double, 3, 3> Zaa = quatToMatrix(quatInverse(q));

	Eigen::Matrix<double, 3, SREKF_STATE_DIM> H;
	H << O33, O33, Zaa, Zaq, O33;

	// square-root
	Eigen::Matrix<double, SREKF_STATE_DIM + 3, SREKF_STATE_DIM + 3> triaArg;
	Eigen::Matrix<double, 3, SREKF_STATE_DIM + 3> triaArg1;
	Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM + 3> triaArg2;
	triaArg1 << _sqrtR_a, H* _sqrtP;

	triaArg2 << Eigen::Matrix<double, SREKF_STATE_DIM, 3>::Zero(), _sqrtP;
	triaArg << triaArg1, triaArg2;

	Eigen::Matrix<double, SREKF_STATE_DIM + 3, SREKF_STATE_DIM + 3> M = getLowTriang<SREKF_STATE_DIM + 3, SREKF_STATE_DIM + 3>(triaArg);
	Eigen::Matrix<double, 3, 3> sqrtRk = M.block<3, 3>(0, 0);
	Eigen::Matrix<double, SREKF_STATE_DIM, 3> K = M.block<SREKF_STATE_DIM, 3>(3, 0);
	_sqrtP = M.block<SREKF_STATE_DIM, SREKF_STATE_DIM>(3, 3);

	_X = _X + K * (sqrtRk.transpose().inverse()) * dz;
	_X.segment(9, 4).normalize();
}

void SREKF::correctP3(const Vector3& dr1, const Vector3& dr2)
{
	Vector3 r = _X.segment(0, 3);
	Vector3 v = _X.segment(3, 3);
	Vector3 a = _X.segment(6, 3);
	Vector4 q = _X.segment(9, 4);
	Vector3 w = _X.segment(13, 3);

	// mes model
	Vector3 Z1 = quatRotate(q, _gpsSlave1);
	Vector3 Z2 = quatRotate(q, _gpsSlave2);
	Vector6 Zx;
	Zx << Z1, Z2;

	Vector6 Z;
	Z << dr1, dr2;
	Vector6 dz = Z - Zx;

	// H
	Eigen::Matrix<double, 3, 4> Zdr1 = quatRotateLinearizationQ(q, _gpsSlave1);
	Eigen::Matrix<double, 3, 4> Zdr2 = quatRotateLinearizationQ(q, _gpsSlave2);

	Eigen::Matrix<double, 3, SREKF_STATE_DIM> H1;
	Eigen::Matrix<double, 3, SREKF_STATE_DIM> H2;
	Eigen::Matrix<double, 6, SREKF_STATE_DIM> H;
	H1 << O33, O33, O33, Zdr1, O33;
	H2 << O33, O33, O33, Zdr2, O33;
	H << H1, H2;

	// square-root
	Eigen::Matrix<double, SREKF_STATE_DIM + 6, SREKF_STATE_DIM + 6> triaArg;
	Eigen::Matrix<double, 6, SREKF_STATE_DIM + 6> triaArg1;
	Eigen::Matrix<double, SREKF_STATE_DIM, SREKF_STATE_DIM + 6> triaArg2;
	triaArg1 << _sqrtR_p3, H * _sqrtP;

	triaArg2 << Eigen::Matrix<double, SREKF_STATE_DIM, 6>::Zero(), _sqrtP;
	triaArg << triaArg1, triaArg2;

	Eigen::Matrix<double, SREKF_STATE_DIM + 6, SREKF_STATE_DIM + 6> M = getLowTriang<SREKF_STATE_DIM + 6, SREKF_STATE_DIM + 6>(triaArg);
	Eigen::Matrix<double, 6, 6> sqrtRk = M.block<6, 6>(0, 0);
	Eigen::Matrix<double, SREKF_STATE_DIM, 6> K = M.block<SREKF_STATE_DIM, 6>(6, 0);
	_sqrtP = M.block<SREKF_STATE_DIM, SREKF_STATE_DIM>(6, 6);

	_X = _X + K * (sqrtRk.transpose().inverse())*dz;
	_X.segment(9, 4).normalize();
}

EkfStateVector SREKF::getEstState()
{
	return _X;
}

