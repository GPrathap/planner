/*
* ExtendedLQR.h
* ExtendedLQR Library
*
* Copyright (c) 2013 University of Utah.
* All rights reserved.
*
* Permission to use, copy, modify, and distribute this software and its
* documentation for educational, research, and non-profit purposes, without
* fee, and without a written agreement is hereby granted, provided that the
* above copyright notice, this paragraph, and the following four paragraphs
* appear in all copies.
*
* Permission to incorporate this software into commercial products may be
* obtained by contacting the authors <berg@cs.utah.edu> or the Technology
* and Venture Commercialization office at the University of Utah
* <801-581-7792> <http://tvc.utah.edu>.
*
* This software program and documentation are copyrighted by the University of
* Utah. The software program and documentation are supplied "as is," without 
* any accompanying services from the University of Utah or the authors. The 
* University of Utah and the authors do not warrant that the operation of
* the program will be uninterrupted or error-free. The end-user understands
* that the program was developed for research purposes and is advised not to
* rely exclusively on the program for any reason.
*
* IN NO EVENT SHALL THE UNIVERSITY OF UTAH OR THE AUTHORS BE LIABLE TO ANY 
* PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
* INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS 
* DOCUMENTATION, EVEN IF THE UNIVERSITY OF UTAH OR THE AUTHORS HAVE BEEN 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* THE UNIVERSITY OF UTAH AND THE AUTHORS SPECIFICALLY DISCLAIM ANY WARRANTIES, 
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
* FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY OF NON-INFRINGEMENT. 
* THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF 
* UTAH AND THE AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, 
* UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
*
* Please send all bug reports to <berg@cs.unc.edu>.
*
* The authors may be contacted via:
*
* Jur van den Berg
* School of Computing
* 50 S. Central Campus Drive, 
* Salt Lake City, UT 84112 
* United States of America
*
* <http://arl.cs.utah.edu/research/extendedlqr/>
*/

#ifndef __MATRIXExtended_LQR_H__
#define __MATRIXExtended_LQR_H__

#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include "matrix.h"
#include <cnpy.h>
#include <stdio.h> 
#include <Eigen/Dense>
namespace loto {
namespace hagen{
// Set dimensions
#define X_DIM 13
#define U_DIM 4
#define DIM 3

	struct Obstacle {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Matrix<DIM> pos;
		double radius;
		size_t dim;
	};

	class ExtendedLQR {
      public:
	  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ExtendedLQR();
        // Dynamics() = default;
        ~ExtendedLQR() = default;
		size_t ell;
		SymmetricMatrix<X_DIM> Q;
		double rotCost;
		Matrix<X_DIM> xGoal, xStart;
		SymmetricMatrix<U_DIM> R;
		Matrix<U_DIM> uNominal;
		double obstacleFactor;
		double timeFactor;
		double scaleFactor;

		std::vector<Obstacle> obstacles;
		Matrix<DIM> bottomLeft;
		Matrix<DIM> topRight;
		double robotRadius;

		double gravity;         // gravity,   m/s^2
		Matrix<3> eX, eY, eZ;   // unit vectors
		// quadrotor constants
		double mass;            // mass, kg
		SymmetricMatrix<3> inertia;    // moment of inertia matrix 
		double momentConst;     // ratio between force and moment of rotor
		double dragConst;       // ratio between speed and opposite drag force
		double length;          // distance between center and rotor, m
		double minForce;        // minimum force of rotor, N
		double maxForce;        // maximum force of rotor, N
		// derivative constants
		SymmetricMatrix<3> invInertia; // inverse of intertia matrix
        double initdt;

		

		static constexpr double DEFAULTSTEPSIZE = 0.0009765625;
		void extendedLQRIterator(const size_t& ell, const Matrix<X_DIM>& startState,
				 const Matrix<U_DIM>& uNominal, std::vector<Matrix<U_DIM, X_DIM> >& L
				 , std::vector<Matrix<U_DIM> >& l, double dt);
		double extendedLQRItr(const size_t& ell, const Matrix<X_DIM>& startState
			, const Matrix<U_DIM>& uNominal, std::vector<Matrix<U_DIM,X_DIM> >& L
			, std::vector<Matrix<U_DIM> >& l, size_t& iter);

		
		inline Matrix<3,3> skewSymmetric(const Matrix<3>& vector) {
			Matrix<3,3> result = zeros<3,3>();
			result(0,1) = -vector[2]; result(0,2) = vector[1];
			result(1,0) = vector[2];    result(1,2) = -vector[0];
			result(2,0) = -vector[1]; result(2,1) = vector[0];
			return result;
		}

		template <size_t xDim>
		void regularize(SymmetricMatrix<xDim>& Q) {
			SymmetricMatrix<xDim> D;
			Matrix<xDim,xDim> V;
			jacobi(Q, V, D);
			for (size_t i = 0; i < xDim; ++i) {
				if (D(i,i) < 0) {
					D(i,i) = 0;
				}
			}
			Q = SymProd(V,D*~V);
		}

		inline Matrix<X_DIM,X_DIM> jacobian1Forward(const Matrix<X_DIM>& a, const Matrix<U_DIM>& b, double jStep) {
				Matrix<X_DIM,X_DIM> A;
				// std::cout<< "====" << yDim << "  " << aDim << std::endl;
				// std::cout << "================================================q" << yDim << "  "<< aDim << std::endl;
				Matrix<X_DIM> ar(a), al(a);
				for (size_t i = 0; i < X_DIM; ++i) {
					ar[i] += jStep; al[i] -= jStep;
					// std::cout << "=========-----=========" << std::endl;
					// A.insert(0,i, (f(ar, b) - f(al, b)) / (2*jStep));
					A.insert(0,i, (gBar(ar, b) - gBar(al, b)) / (2*jStep));
					ar[i] = al[i] = a[i];
				}
				// std::cout<< "====after====" << (f(ar, b) - f(al, b)) / (2*jStep) << std::endl;
				return A;
			}

		inline Matrix<X_DIM,U_DIM> jacobian2Forward(const Matrix<X_DIM>& a, const Matrix<U_DIM>& b, double jStep) {
				Matrix<X_DIM,U_DIM> B;
				Matrix<U_DIM> br(b), bl(b);
				// std::cout<< "======================" << yDim << " " << bDim << " " << std::endl;
				for (size_t i = 0; i < U_DIM; ++i) {
					br[i] += jStep; bl[i] -= jStep;
					B.insert(0, i, (gBar(a, br) - gBar(a, bl)) / (2*jStep));
					br[i] = bl[i] = b[i];
				}
				return B;
		}

		inline Matrix<X_DIM,X_DIM> jacobian1Backward(const Matrix<X_DIM>& a, const Matrix<U_DIM>& b, double jStep) {
				Matrix<X_DIM,X_DIM> A;
				// std::cout<< "====" << yDim << "  " << aDim << std::endl;
				// std::cout << "================================================q" << yDim << "  "<< aDim << std::endl;
				Matrix<X_DIM> ar(a), al(a);
				for (size_t i = 0; i < X_DIM; ++i) {
					ar[i] += jStep; al[i] -= jStep;
					// std::cout << "=========-----=========" << std::endl;
					// A.insert(0,i, (f(ar, b) - f(al, b)) / (2*jStep));
					A.insert(0,i, (g(ar, b) - g(al, b)) / (2*jStep));
					ar[i] = al[i] = a[i];
				}
				// std::cout<< "====after====" << (f(ar, b) - f(al, b)) / (2*jStep) << std::endl;
				return A;
		}

		inline Matrix<X_DIM,U_DIM> jacobian2Backward(const Matrix<X_DIM>& a, const Matrix<U_DIM>& b, double jStep) {
				Matrix<X_DIM,U_DIM> B;
				Matrix<U_DIM> br(b), bl(b);
				// std::cout<< "======================" << yDim << " " << bDim << " " << std::endl;
				for (size_t i = 0; i < U_DIM; ++i) {
					br[i] += jStep; bl[i] -= jStep;
					B.insert(0, i, (g(a, br) - g(a, bl)) / (2*jStep));
					br[i] = bl[i] = b[i];
				}
				return B;
		}

		// inline Matrix<X_DIM,X_DIM> jacobian1(const Matrix<X_DIM>& a, const Matrix<U_DIM>& b, double jStep) {
		// 	Matrix<X_DIM,X_DIM> A;
		// 	// std::cout<< "====" << yDim << "  " << aDim << std::endl;
		// 	// std::cout << "================================================q" << yDim << "  "<< aDim << std::endl;
		// 	Matrix<X_DIM> ar(a), al(a);
		// 	for (size_t i = 0; i < X_DIM; ++i) {
		// 		ar[i] += jStep; al[i] -= jStep;
		// 		// std::cout << "=========-----=========" << std::endl;
		// 		// A.insert(0,i, (f(ar, b) - f(al, b)) / (2*jStep));
		// 		A.insert(0,i, (gBar(ar, b) - gBar(al, b)) / (2*jStep));
		// 		ar[i] = al[i] = a[i];
		// 	}
		// 	// std::cout<< "====after====" << (f(ar, b) - f(al, b)) / (2*jStep) << std::endl;
		// 	return A;
		// }

		// inline Matrix<X_DIM,U_DIM> jacobian2(const Matrix<X_DIM>& a, const Matrix<U_DIM>& b, double jStep) {
		// 	Matrix<X_DIM,U_DIM> B;
		// 	Matrix<U_DIM> br(b), bl(b);
		// 	// std::cout<< "======================" << yDim << " " << bDim << " " << std::endl;
		// 	for (size_t i = 0; i < U_DIM; ++i) {
		// 		br[i] += jStep; bl[i] -= jStep;
		// 		B.insert(0, i, (f(a, br) - f(a, bl)) / (2*jStep));
		// 		br[i] = bl[i] = b[i];
		// 	}
		// 	return B;
		// }

		// Obstacle-cost term in local cost function
		inline double obstacleCost(const Matrix<X_DIM>& x) {
			double cost = 0;
			Matrix<DIM> pos = x.subMatrix<DIM>(0,0);

			for (size_t i = 0; i < obstacles.size(); ++i) {
				Matrix<DIM> d = pos - obstacles[i].pos;
				//d[obstacles[i].dim] = 0;
				double dist = std::sqrt(scalar(~d*d)) - robotRadius - obstacles[i].radius;
				cost += obstacleFactor * std::exp(-scaleFactor*dist);
			}
			for (size_t i = 0; i < DIM; ++i) {
				double dist = (pos[i] - bottomLeft[i]) - robotRadius;
				cost += obstacleFactor * std::exp(-scaleFactor*dist);
			}
			for (size_t i = 0; i < DIM; ++i) {
				double dist = (topRight[i] - pos[i]) - robotRadius;
				cost += obstacleFactor * std::exp(-scaleFactor*dist);
			}
			return cost;	
		}

		inline void quadratizeObstacleCost(const Matrix<X_DIM>& x, SymmetricMatrix<X_DIM>& Q, Matrix<X_DIM>& q) {
			SymmetricMatrix<DIM> QObs = zeros<DIM>();
			Matrix<DIM> qObs = zero<DIM>();

			Matrix<DIM> pos = x.subMatrix<DIM>(0,0);

			for (size_t i = 0; i < obstacles.size(); ++i) {
				Matrix<DIM> d = pos - obstacles[i].pos;
				// d[obstacles[i].dim] = 0;
				double distr = std::sqrt(scalar(~d*d));
				d /= distr;
				double dist = distr - robotRadius - obstacles[i].radius;

				Matrix<DIM> n = zero<DIM>();
				// n[obstacles[i].dim] = 1.0;
				Matrix<DIM> d_ortho = skewSymmetric(n)*d;

				double a0 = obstacleFactor * std::exp(-scaleFactor*dist);
				double a1 = -scaleFactor*a0;
				double a2 = -scaleFactor*a1;

				double b2 = a1 / distr;

				QObs += a2*SymProd(d,~d) + b2*SymProd(d_ortho,~d_ortho);
				qObs += a1*d;
			}
			for (size_t i = 0; i < DIM; ++i) {
				double dist = (pos[i] - bottomLeft[i]) - robotRadius;

				Matrix<DIM> d = zero<DIM>();
				d[i] = 1.0;

				double a0 = obstacleFactor * std::exp(-scaleFactor*dist);
				double a1 = -scaleFactor*a0;
				double a2 = -scaleFactor*a1;

				QObs += a2*SymProd(d,~d);
				qObs += a1*d;
			}
			for (size_t i = 0; i < DIM; ++i) {
				double dist = (topRight[i] - pos[i]) - robotRadius;

				Matrix<DIM> d = zero<DIM>();
				d[i] = -1.0;

				double a0 = obstacleFactor * std::exp(-scaleFactor*dist);
				double a1 = -scaleFactor*a0;
				double a2 = -scaleFactor*a1;

				QObs += a2*SymProd(d,~d);
				qObs += a1*d;
			}
			regularize(QObs);
			Q.insert(0, QObs + Q.subSymmetricMatrix<3>(0));
			q.insert(0,0, qObs - QObs*x.subMatrix<3>(0,0) + q.subMatrix<3>(0,0));
		}

		// Local cost-function c_t(x_t, u_t)
		inline double ct(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u, const size_t& t) {
			double cost = 0;
			if (t == 0) {
				cost += scalar(~(x - xStart)*Q*(x - xStart));
			} else {
				cost += obstacleCost(x);
			}
			cost += scalar(~(u - uNominal)*R*(u - uNominal));
			cost += timeFactor * std::exp(x[X_DIM-1]);
			return cost;
		}

		inline void quadratizeCost(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u, const size_t& t, Matrix<U_DIM,X_DIM>& Pt, SymmetricMatrix<X_DIM>& Qt, SymmetricMatrix<U_DIM>& Rt, Matrix<X_DIM>& qt, Matrix<U_DIM>& rt, const size_t& iter) {

			if (t == 0) {
				Qt = Q;
				qt = -(Q*xStart);
				if (iter < 1) {
					Qt(X_DIM-1,X_DIM-1) = 100;
					qt[X_DIM-1] = -Qt(X_DIM-1,X_DIM-1)*x[X_DIM-1];
				} else {
					Qt(X_DIM-1,X_DIM-1) = timeFactor*std::exp(x[X_DIM-1]); 
					qt[X_DIM-1] = timeFactor*std::exp(x[X_DIM-1]) - Qt(X_DIM-1,X_DIM-1)*x[X_DIM-1];
				}
			} else {
				Qt.reset();	
				qt.reset();

				quadratizeObstacleCost(x, Qt, qt);

				Qt(X_DIM-1,X_DIM-1) = timeFactor*std::exp(x[X_DIM-1]); 
				qt[X_DIM-1] = timeFactor*std::exp(x[X_DIM-1]) - Qt(X_DIM-1,X_DIM-1)*x[X_DIM-1];

				if (iter < 1) {
					Qt.insert(6,rotCost*identity<3>());
				}
			}

			Rt = R; 
			rt = -(R*uNominal);
			Pt.reset();
		}

		// Final cost function c_\ell(x_\ell)
		inline double cell(const Matrix<X_DIM>& x) {
			double cost = 0;
			cost += scalar(~(x - xGoal)*Q*(x - xGoal));
			return cost;
		}

		inline void quadratizeFinalCost(const Matrix<X_DIM>& x, SymmetricMatrix<X_DIM>& Qell, Matrix<X_DIM>& qell, const size_t& iter) {
			/*Qell = hessian(x, cell); 
			qell = jacobian(x, cell) - Qell*x;*/

			Qell = Q;
			qell = -(Q*xGoal);

			if (iter < 1) {
				Qell(X_DIM-1,X_DIM-1) = 100;
				qell[X_DIM-1] = -Qell(X_DIM-1,X_DIM-1)*x[X_DIM-1];
			} 
		}

		// Continuous-time dynamics \dot{x} = f(x,u)
		inline Matrix<X_DIM> f(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u) {
			Matrix<X_DIM> xDot;

			Matrix<3> p = x.subMatrix<3,1>(0,0);
			Matrix<3> v = x.subMatrix<3,1>(3,0);
			Matrix<3> r = x.subMatrix<3,1>(6,0);
			Matrix<3> w = x.subMatrix<3,1>(9,0);

			// \dot{p} = v
			xDot.insert(0, 0, v);

			// \dot{v} = [0,0,-g]^T + R*exp([r])*[0,0,(f_1 + f_2 + f_3 + f_4) / m]^T; 
			xDot.insert(3, 0, -gravity*eZ + exp(skewSymmetric(r))*eZ*((u[0]+u[1]+u[2]+u[3])/mass) - v*(dragConst/mass)); 

			// \dot{r} = w + 0.5*skewSymmetric(r)*w + (1.0/tr(~r*r))*(1.0 - 0.5*sqrt(tr(~r*r))/tan(0.5*sqrt(tr(~r*r))))*skewSymmetric(r)*(skewSymmetric(r)*w)
			double l = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
			if (0.5*l > 0.0) {
				xDot.insert(6, 0, w + 0.5*skewSymmetric(r)*w + (1.0 - 0.5*l/tan(0.5*l))*skewSymmetric(r / l)*(skewSymmetric(r / l)*w));
			} else {
				xDot.insert(6, 0, w);
			}

			// \dot{w} = J^{-1}*([l*(f_2 - f_4), l*(f_3 - f_1), (f_1 - f_2 + f_3 - f_4)*k_M]^T - [w]*J*w)
			xDot.insert(9, 0, invInertia*( length*(u[1] - u[3])*eX + length*(u[2] - u[0])*eY + (u[0] - u[1] + u[2] - u[3])*momentConst*eZ - skewSymmetric(w)*inertia*w));


			xDot[X_DIM-1] = 0.0;

			return xDot;
		}

		// Discrete-time dynamics x_{t+1} = g(x_t, u_t)
		inline Matrix<X_DIM> g(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u) {
			double dt = std::exp(x[X_DIM-1]);
			// TODO try to find why dt goes higher than 1.0
			Matrix<X_DIM> k1 = f(x, u);
			Matrix<X_DIM> k2 = f(x + 0.5*dt*k1, u);
			Matrix<X_DIM> k3 = f(x + 0.5*dt*k2, u);
			Matrix<X_DIM> k4 = f(x + dt*k3, u);
			return x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
		}

		// Discrete-time inverse dynamics x_t = \bar{g}(x_{t+1}, u_t)
		inline Matrix<X_DIM> gBar(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u) {
			double dt = std::exp(x[X_DIM-1]);
			Matrix<X_DIM> k1 = f(x, u);
			Matrix<X_DIM> k2 = f(x - 0.5*dt*k1, u);
			Matrix<X_DIM> k3 = f(x - 0.5*dt*k2, u);
			Matrix<X_DIM> k4 = f(x - dt*k3, u);
			return x - (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
		}

	};
}
}

#endif