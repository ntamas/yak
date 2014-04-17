/*
 * The MIT License (MIT)
 * Copyright (c) 2014 Tamas Nepusz
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Simple test for the Kalman filter based on the cannonball example found on
 * the following webpage:
 *
 * http://greg.czerniak.info/guides/kalman1/
 *
 * The process tracks the motion of a cannon ball fired at a 45-degree angle
 * and a muzzle velocity of 100 units/second. The position of the cannon ball
 * is measured by a camera whose position estimate is noisy. The velocity of
 * the cannon ball is measured by accurate detectors in the ball. The state
 * variables are therefore as follows: x and y coordinates of the ball and
 * velocity along the x and y axes.
 */

#include <yak/yak.hpp>
#include <cmath>
#include <iostream>

#define GRAVITY 9.81
#define DT 0.25

using namespace Eigen;
using namespace std;
using namespace yak;

typedef Eigen::Matrix<double, 1, 1> Vector1d;

class CannonBallState : public Vector4d {
public:
	double xPosition() { return (*this)(0); }
	double yPosition() { return (*this)(1); }
	double xVelocity() { return (*this)(2); }
	double yVelocity() { return (*this)(3); }
};

class CannonBallStateEstimate : public Gaussian<4> {
public:
	double xPosition() { return mean(0); }
	double yPosition() { return mean(1); }
	double xVelocity() { return mean(2); }
	double yVelocity() { return mean(3); }
};

class CannonBallControlVector : public SimpleControlVector<1> {
public:
	double gravity() { return value(0); }
};

class CannonBallProcessModel {
public:
	typedef CannonBallStateEstimate StateEstimate;
	typedef CannonBallControlVector ControlVector;

	Matrix4d calculateJacobian(double dt) {
		Matrix4d result;
		result <<  1,  0, dt,  0,
			       0,  1,  0, dt,
				   0,  0,  1,  0,
				   0,  0,  0,  1;
		return result;
	}

	Vector4d calculateControlMatrix(double dt) {
		Vector4d result;
		result << 0, 0, -0.5*dt*dt, -dt;
		return result;
	}

	Matrix4d calculateNoiseCovarianceMatrix(double dt) {
		return Matrix4d::Zero();
	}
};

typedef SimpleMeasurement<4> CannonBallMeasurement;
typedef IndependentNoisyMeasurementModel<CannonBallStateEstimate, CannonBallMeasurement>
	CannonBallMeasurementModelBase;
class CannonBallMeasurementModel : public CannonBallMeasurementModelBase {

public:

	CannonBallState initialState;
	double time;

	CannonBallMeasurementModel(const CannonBallState& initialState)
		: CannonBallMeasurementModelBase(), initialState(initialState), time(0) {}

	CannonBallState getRealState() {
		CannonBallState result;
		result = initialState;
		result(0) += initialState(2)*time;
		result(1) += initialState(3)*time - 0.5 * GRAVITY * time * time;
		result(3) -= GRAVITY * time;
		return result;
	}

	Measurement measure() {
		Measurement result;
		result.value = getMeasurementMatrix() * getRealState() + generateNoise();
		return result;
	}

	void step(double dt) {
		time += dt;
	}
};

int main(int argc, char* argv[]) {
	CannonBallState initialState;
	CannonBallStateEstimate initialEstimate;

	initialState << 0, 0, 100*cos(M_PI/4), 100*sin(M_PI/4);
	initialEstimate.mean << 0, 500, 100*cos(M_PI/4), 100*sin(M_PI/4);

	CannonBallProcessModel cannonBallProcessModel;
	CannonBallMeasurementModel cannonBallMeasurementModel(initialState);
	CannonBallMeasurement measurement;
	CannonBallControlVector gravity;

	KalmanFilter<CannonBallProcessModel, CannonBallMeasurementModel> filter(
			&cannonBallProcessModel, &cannonBallMeasurementModel);
	filter.reset(initialEstimate);

	cannonBallMeasurementModel.variances << 0.2, 0.2, 0.2, 0.2;
	gravity.value << GRAVITY;

	cout << "time\tactual_x\tactual_y\tmeasured_x\tmeasured_y\testimated_x\testimated_y\n";
	while (true) {
		cannonBallMeasurementModel.step(DT);
		measurement = cannonBallMeasurementModel.measure();
		filter.update(DT, gravity, measurement);

		cout << cannonBallMeasurementModel.time << '\t';
		cout << cannonBallMeasurementModel.getRealState()(0) << '\t';
		cout << cannonBallMeasurementModel.getRealState()(1) << '\t';
		cout << measurement.value(0) << '\t';
		cout << measurement.value(1) << '\t';
		cout << filter.lastEstimate.mean(0) << '\t';
		cout << filter.lastEstimate.mean(1) << '\n';

		if (cannonBallMeasurementModel.getRealState()(1) < 0)
			break;
	}

}

