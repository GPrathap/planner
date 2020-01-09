/*
* ExtendedLQR - quadrotor - timeOpt.cpp
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

#include <stdio.h>
#include "ExtendedLQR.h"

namespace loto {
namespace hagen{
	
	ExtendedLQR::ExtendedLQR(){
		
		gravity = 9.80665;           // gravity,   m/s^2
		eX[0] = 1; eX[1] = 0; eX[2] = 0;
		eY[0] = 0; eY[1] = 1; eY[2] = 0;
		eZ[0] = 0; eZ[1] = 0; eZ[2] = 1;

		// Quadrotor parameters
		mass        = 0.5;             // mass, kg  (source: paper)
		inertia     = 0.05*identity<3>(); // moment of inertia matrix 
		momentConst = 1.5e-9 / 6.11e-8;  // ratio between force and moment of rotor
		dragConst   = 0.15; //0.15;
		length      = 0.3429/2;          // distance between center and rotor, m

		invInertia   = !inertia;
		uNominal[0] = uNominal[1] = uNominal[2] = uNominal[3] = gravity*mass/4;

		// Control parameters
		ell = 30;
		initdt = 0.05;

		timeFactor = 3;

		Q = 500*identity<X_DIM>();
		Q(X_DIM-1,X_DIM-1) = 0;

		rotCost = 0.3; 

		obstacleFactor = 1;
		scaleFactor = 10;

		R = 20*identity<U_DIM>();

		// Environment parameters
		robotRadius = length + 0.1;

		bottomLeft[0] = -15.0; bottomLeft[1] = -15.0; bottomLeft[2] = -15.0;
		topRight[0] = 15.0; topRight[1] = 15.0; topRight[2] = 15.0;
	}	

	void ExtendedLQR::extendedLQRIterator(const size_t& ell, const Matrix<X_DIM>& startState,
				 const Matrix<U_DIM>& uNominal, std::vector<Matrix<U_DIM, X_DIM> >& L
				 , std::vector<Matrix<U_DIM> >& l, double dt){
			
			size_t maxIter = 10;
			// Initialization
			L.resize(ell, zeros<U_DIM,X_DIM>());
			l.resize(ell, uNominal);
			std::vector<Matrix<X_DIM>> lx_state;
			std::vector<SymmetricMatrix<X_DIM>> S(ell + 1, zeros<X_DIM>());
			// std::cout<< "======S " << S << std::endl;
			std::vector<Matrix<X_DIM>> s(ell + 1, zero<X_DIM>());
			std::vector<SymmetricMatrix<X_DIM>> SBar(ell + 1);
			std::vector<Matrix<X_DIM> > sBar(ell + 1);
			double oldCost = -log(0.0);
			Matrix<X_DIM> xHat = startState;
			SBar[0] = zeros<X_DIM>();
			sBar[0] = zero<X_DIM>();
			// for (iter = 0; iter < maxIter; ++iter) {
			// forward pass
			for (size_t t = 0; t < ell; ++t) {
				// std::cout<< "==========xHat====\n"<< xHat << std::endl;
				// std::cout<< "========l[t]======\n"<< l[t]  << std::endl;
				// xHat[3] = 0;
				// xHat[4] = 2.5;
				// xHat[5] = 0;
				// std::cout<< "========L[t]*xHat======\n"<< L[t]*xHat << std::endl;
				// std::cout<< "========l[t]======\n"<< l[t] << std::endl;
				// const Matrix<U_DIM> uHat = L[t]*xHat + l[t];
				const Matrix<U_DIM> uHat = l[t];
				// std::cout<< "======uHat "<< uHat << std::endl;
				const Matrix<X_DIM> xHatPrime = g(xHat, uHat);
				// std::cout<< "======xHatPrime "<< xHatPrime << std::endl;
				const Matrix<X_DIM, X_DIM> ABar = jacobian1Forward(xHatPrime, uHat, DEFAULTSTEPSIZE);
				// std::cout<< "======jacobian1 "<< ABar << std::endl;
				const Matrix<X_DIM, U_DIM> BBar = jacobian2Forward(xHatPrime, uHat, DEFAULTSTEPSIZE);
				// std::cout<< "======jacobian2\n"<< BBar << std::endl;
				const Matrix<X_DIM> cBar = xHat - ABar*xHatPrime - BBar*uHat;
					// std::cout<< "======cBar\n"<< cBar << std::endl;
				Matrix<U_DIM, X_DIM> P;
				SymmetricMatrix<X_DIM> Q;
				SymmetricMatrix<U_DIM> R;
				Matrix<X_DIM> q;
				Matrix<U_DIM> r;
				quadratizeCost(xHat, uHat, t, P, Q, R, q, r, 0);
				// std::cout<< "======P\n" << P << std::endl;
				// std::cout<< "======Q\n" << Q << std::endl;
				// std::cout<< "======R\n" << R << std::endl;
				// std::cout<< "======q\n" << q << std::endl;
				// std::cout<< "======r\n" << r << std::endl;
				// std::cout<< "======SBar[t]\n" << SBar[t] << std::endl;
				const SymmetricMatrix<X_DIM> SBarQ = SBar[t] + Q;
				// std::cout<< "======q\n" << q << std::endl;
				// std::cout<< "======SBarQ\n" << SBarQ << std::endl;
				// std::cout<< "======cBar\n" << cBar << std::endl;
				// std::cout<< "======sBar[t]\n" << sBar[t] << std::endl;
				// std::cout<< "====== SBarQ*cBar\n" <<  SBarQ*cBar << std::endl;
				// std::cout<< " SBar[t] + Q "<< SBar[t] + Q << std::endl;
				// std::cout<< " SBar[t] + Q "<< SBarQ << std::endl;
				const Matrix<X_DIM> sBarqSBarQcBar = sBar[t] + q + SBarQ*cBar;
				// std::cout<< " sBarqSBarQcBar "<< sBarqSBarQcBar << std::endl;
				const Matrix<U_DIM,X_DIM> CBar = ~BBar*SBarQ*ABar + P*ABar;
				//  std::cout<< " CBar "<< CBar << std::endl;
				const SymmetricMatrix<X_DIM> DBar = SymProd(~ABar,SBarQ*ABar);
				//  std::cout<< " DBar "<< DBar << std::endl;
				// // std::cout<< "--------->" << std::endl;
				const SymmetricMatrix<U_DIM> EBar = SymProd(~BBar,SBarQ*BBar) + R + SymSum(P*BBar);
				// std::cout<< EBar << std::endl;
				const Matrix<X_DIM> dBar = ~ABar*sBarqSBarQcBar;
				// std::cout<< dBar << std::endl;
				const Matrix<U_DIM> eBar = ~BBar*sBarqSBarQcBar + r + P*cBar;
				// std::cout<< eBar << std::endl;
				L[t] = -(EBar%CBar);
				l[t] = -(EBar%eBar);
				// std::cout<< L[t]  << std::endl; 
				// std::cout<< l[t]  << std::endl; 
				SBar[t+1] = DBar + SymProd(~CBar, L[t]);
				sBar[t+1] = dBar + ~CBar*l[t];
				// std::cout<< SBar[t+1]  << std::endl; 
				// std::cout<< sBar[t+1] << std::endl; 
				xHat = -((S[t+1] + SBar[t+1])%(s[t+1] + sBar[t+1]));
				// std::cout<< xHat << std::endl;
				lx_state.push_back(xHat);
				std::cout<< "==========xHat====\n"<< xHat << std::endl;
				std::cout<< "==========xHat====\n"<< l[t] << std::endl;
				// std::cout<< "========== xHat ====\n"<< xHat << std::endl;
				// std::cout<< "======== l[t] ======\n"<< l[t]  << std::endl;
				// std::cout<< "======== xHat ======\n"<< xHat  << std::endl;
			}

			std::vector<double> sates_sequeance; 
			int count = lx_state.size();
			for(auto item : lx_state){
				for(int i=0; i<X_DIM; i++){
					sates_sequeance.push_back(item[i]);
				}		
			}

			std::vector<double> control_sequeance; 
			count = l.size();
			for(auto item : l){
				for(int i=0; i<U_DIM; i++){
					control_sequeance.push_back(item[i]);
				}		
			}
			std::cout<< "==1" << sates_sequeance.size() << std::endl;
			cnpy::npy_save("/home/geesara/Desktop/ler/0_state_vector.npy", &sates_sequeance[0],{(unsigned int)1, (unsigned int)count, (unsigned int)X_DIM},"w");
			cnpy::npy_save("/home/geesara/Desktop/ler/0_control_vector.npy", &control_sequeance[0],{(unsigned int)1, (unsigned int)count, (unsigned int)U_DIM},"w");
	}

double ExtendedLQR::extendedLQRItr(const size_t& ell, const Matrix<X_DIM>& startState
			, const Matrix<U_DIM>& uNominal, std::vector<Matrix<U_DIM,X_DIM> >& L
			, std::vector<Matrix<U_DIM> >& l, size_t& iter) {

	size_t maxIter = 20;

	// Initialization
	L.resize(ell, zeros<U_DIM,X_DIM>());
	l.resize(ell, uNominal);

	std::vector<SymmetricMatrix<X_DIM> > S(ell + 1, zeros<X_DIM>());
	std::vector<Matrix<X_DIM> > s(ell + 1, zero<X_DIM>());
	std::vector<SymmetricMatrix<X_DIM> > SBar(ell + 1);
	std::vector<Matrix<X_DIM> > sBar(ell + 1);
	double oldCost = -log(0.0);
	Matrix<X_DIM> xHat = startState;

	SBar[0] = zeros<X_DIM>();
	sBar[0] = zero<X_DIM>();
    // std::cout<< "=======1=======" << std::endl;
	for (iter = 0; iter < maxIter; ++iter) {
		// forward pass
		for (size_t t = 0; t < ell; ++t) {
			// std::cout<< "=======11=======" << std::endl;
			const Matrix<U_DIM> uHat = L[t]*xHat + l[t];
			//  std::cout<< "=======121=======" << std::endl;
			const Matrix<X_DIM> xHatPrime = g(xHat, uHat);
            // std::cout<< "=======12=======" << std::endl;
			const Matrix<X_DIM, X_DIM> ABar = jacobian1Forward(xHatPrime, uHat, DEFAULTSTEPSIZE);
			const Matrix<X_DIM, U_DIM> BBar = jacobian2Forward(xHatPrime, uHat, DEFAULTSTEPSIZE);
			const Matrix<X_DIM> cBar = xHat - ABar*xHatPrime - BBar*uHat;

			Matrix<U_DIM, X_DIM> P;
			SymmetricMatrix<X_DIM> Q;
			SymmetricMatrix<U_DIM> R;
			Matrix<X_DIM> q;
			Matrix<U_DIM> r;
            //  std::cout<< "=======123=======" << std::endl;
			quadratizeCost(xHat, uHat, t, P, Q, R, q, r, iter);
			// std::cout<< "=======14=======" << std::endl;
			const SymmetricMatrix<X_DIM> SBarQ = SBar[t] + Q;
			const Matrix<X_DIM> sBarqSBarQcBar = sBar[t] + q + SBarQ*cBar;
			const Matrix<U_DIM,X_DIM> CBar = ~BBar*SBarQ*ABar + P*ABar;
			const SymmetricMatrix<X_DIM> DBar = SymProd(~ABar,SBarQ*ABar);
			const SymmetricMatrix<U_DIM> EBar = SymProd(~BBar,SBarQ*BBar) + R + SymSum(P*BBar);
			const Matrix<X_DIM> dBar = ~ABar*sBarqSBarQcBar;
			const Matrix<U_DIM> eBar = ~BBar*sBarqSBarQcBar + r + P*cBar;
			L[t] = -(EBar%CBar);
			l[t] = -(EBar%eBar);
			SBar[t+1] = DBar + SymProd(~CBar, L[t]);
			sBar[t+1] = dBar + ~CBar*l[t];
			// std::cout<< "================S[t+1]==========7" << S[t+1] << std::endl;
			// std::cout<< "==============SBar[t+1]============7" << SBar[t+1] << std::endl;
			// std::cout<< "=============s[t+1]=============7" << s[t+1] << std::endl;
			// std::cout<< "=============sBar[t+1]=============7" << sBar[t+1] << std::endl;

			xHat = -((S[t+1] + SBar[t+1])%(s[t+1] + sBar[t+1]));
		}
		// std::cout<< "=======2=======" << std::endl;
		// backward pass
		quadratizeFinalCost(xHat, S[ell], s[ell], iter);
		// std::cout<< "=======3=======" << std::endl;
		xHat = -((S[ell] + SBar[ell])%(s[ell] + sBar[ell]));
		// std::cout<< "=======4=======" << std::endl;
		for (size_t t = ell - 1; t != -1; --t) {
			const Matrix<U_DIM> uHat = L[t]*xHat + l[t];
			const Matrix<X_DIM> xHatPrime = gBar(xHat, uHat);

			const Matrix<X_DIM, X_DIM> A = jacobian1Backward(xHatPrime, uHat, DEFAULTSTEPSIZE);
			const Matrix<X_DIM, U_DIM> B = jacobian2Backward(xHatPrime, uHat, DEFAULTSTEPSIZE);
			const Matrix<X_DIM> c = xHat - A*xHatPrime - B*uHat;

			Matrix<U_DIM, X_DIM> P;
			SymmetricMatrix<X_DIM> Q;
			SymmetricMatrix<U_DIM> R;
			Matrix<X_DIM> q;
			Matrix<U_DIM> r;

			quadratizeCost(xHatPrime, uHat, t, P, Q, R, q, r, iter);

			const Matrix<U_DIM,X_DIM> C = ~B*S[t+1]*A + P;
			const SymmetricMatrix<X_DIM> D = SymProd(~A,S[t+1]*A) + Q;
			const SymmetricMatrix<U_DIM> E = SymProd(~B,S[t+1]*B) + R;
			const Matrix<X_DIM> d = ~A*(s[t+1] + S[t+1]*c) + q;
			const Matrix<U_DIM> e = ~B*(s[t+1] + S[t+1]*c) + r;

			L[t] = -(E%C);
			l[t] = -(E%e);

			S[t] = D + SymProd(~C, L[t]);
			s[t] = d + ~C*l[t];

			xHat = -((S[t] + SBar[t])%(s[t] + sBar[t]));
		}
		// std::cout<< "=======5=======" << std::endl;
		// compute cost
		double newCost = 0;
		Matrix<X_DIM> x = xHat;
		for (size_t t = 0; t < ell; ++t) {
			Matrix<U_DIM> u = L[t]*x + l[t];
			newCost += ct(x, u, t);
			x = g(x, u);
		}
		// std::cout<< "=======6=======" << std::endl;
		newCost += cell(x);

		// if (vis) {
		// 	std::cerr << "Iter: " << iter << " Rel. progress: " << (oldCost - newCost) / newCost << " Cost: " << newCost << " Time step: " << exp(xHat[xDim-1]) << std::endl;
		// }
		if (abs((oldCost - newCost) / newCost) < 1e-4) {
			++iter;
			return std::exp(xHat[X_DIM-1]);
		}
		oldCost = newCost;
	}
	return std::exp(xHat[X_DIM-1]);
}

}
}

// int main(int argc, char *argv[])
// {
//     loto::hagen::ExtendedLQR extendedLQR;
// 	std::vector<loto::hagen::Matrix<U_DIM, X_DIM> > L;
// 	std::vector<loto::hagen::Matrix<U_DIM> > l;
// 	double initdt = 0.25;
// 	extendedLQR.xGoal = loto::hagen::zero<X_DIM>();
// 	extendedLQR.xStart[0] = 4;
// 	extendedLQR.xStart[1] = 4;
// 	extendedLQR.xStart[2] = 4;
// 	extendedLQR.xStart[3] = 0;
// 	extendedLQR.xStart[4] = 2.5;
// 	extendedLQR.xStart[5] = 0;
// 	int number_of_tries = 300;
// 	loto::hagen::Matrix<X_DIM> xStartInit = extendedLQR.xStart;
// 	xStartInit[X_DIM-1] = log(initdt);
// 	size_t numIter;
// 	clock_t beginTime = clock();
// 	std::cout<< "xStart" << extendedLQR.xStart << std::endl;
// 	std::cout<< "xGoal" << extendedLQR.xGoal << std::endl;
// 	extendedLQR.extendedLQRIterator(number_of_tries, xStartInit, extendedLQR.uNominal, L, l, initdt);
// 	clock_t endTime = clock();
// 	std::cerr << "Extended LQR: NumIter: " << numIter << " Time: " << (endTime - beginTime) / (double) CLOCKS_PER_SEC << std::endl;
// 	return 0;
// }
