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

#ifndef YAK_KALMAN_FILTER_HPP
#define YAK_KALMAN_FILTER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

namespace yak {

/**
 * The main Kalman filter class.
 */
template <typename TProcessModel, typename TMeasurementModel>
class KalmanFilter {
public:
	
	typedef TProcessModel ProcessModel;
	typedef TMeasurementModel MeasurementModel;

	typedef typename ProcessModel::StateEstimate StateEstimate;
	typedef typename ProcessModel::ControlVector ControlVector;
	typedef typename MeasurementModel::Measurement Measurement;

	typedef Eigen::Matrix<typename StateEstimate::DataType,
		StateEstimate::DIMENSIONS, StateEstimate::DIMENSIONS> JacobianMatrix;
	typedef Eigen::Matrix<typename StateEstimate::DataType,
		StateEstimate::DIMENSIONS, ControlVector::DIMENSIONS> ControlMatrix;
	typedef Eigen::Matrix<typename StateEstimate::DataType,
		Measurement::DIMENSIONS, StateEstimate::DIMENSIONS> MeasurementMatrix;
	typedef Eigen::Matrix<typename StateEstimate::DataType,
		Measurement::DIMENSIONS, Measurement::DIMENSIONS> InnovationInverseMatrix;
	typedef Eigen::Matrix<typename StateEstimate::DataType,
		StateEstimate::DIMENSIONS, Measurement::DIMENSIONS> KalmanGainMatrix;

	/**
	 * The process model that this filter is using.
	 */
	ProcessModel* const processModel;

	/**
	 * The measurement model that this filter is using.
	 */
	MeasurementModel* const measurementModel;

	/**
	 * The last estimate of the state of the process.
	 */
	StateEstimate lastEstimate;

	/**
	 * Constructor.
	 */
	KalmanFilter(ProcessModel* const processModel_,
			MeasurementModel* const measurementModel_)
		: processModel(processModel_), measurementModel(measurementModel_),
		lastEstimate() {}

	/**
	 * Resets the Kalman filter with the given initial state estimate.
	 *
	 * \param  initial  the initial state estimate
	 */
	void reset(const StateEstimate& initial) {
		lastEstimate = initial;
	}

	/**
	 * Updates the Kalman filter with the latest measurements and the
	 * time that has passed since the last measurement.
	 *
	 * \param  dt           the time elapsed since the last measurement
	 * \param  control      the value of the current control vector
	 * \param  measurement  the value of the current measurement
	 */
	void update(double dt, const ControlVector& control,
			const Measurement& measurement) {
		StateEstimate predictedState;
		JacobianMatrix jacobian;
		ControlMatrix controlMatrix;
		MeasurementMatrix measurementMatrix;
		Gaussian<Measurement::DIMENSIONS> innovation;
		InnovationInverseMatrix innovationInverse;
		KalmanGainMatrix partialResult;
		KalmanGainMatrix kalmanGain;

		// Calculate the process Jacobian
		jacobian = processModel->calculateJacobian(dt);

		// Calculate the control matrix
		controlMatrix = processModel->calculateControlMatrix(dt);

		// Preliminary state prediction
		predictedState.mean = jacobian * lastEstimate.mean;
		if (ControlVector::DIMENSIONS > 0) {
			predictedState.mean += controlMatrix * control.value;
		}
		predictedState.covariance = jacobian * lastEstimate.covariance *
			jacobian.transpose() +
			processModel->calculateNoiseCovarianceMatrix(dt);

		// Calculate innovation
		measurementMatrix = measurementModel->getMeasurementMatrix();
		partialResult = predictedState.covariance * measurementMatrix.transpose();
		innovation.mean = measurement.value - measurementMatrix * predictedState.mean;
		innovation.covariance = measurementMatrix * partialResult +
			measurementModel->getNoiseCovarianceMatrix();

		// Calculate Kalman gain
		// TODO: check which LU decomposition is the best here; choices are
		// here:
		// http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
		innovationInverse = innovation.covariance.fullPivLu().inverse();
		kalmanGain = partialResult * innovationInverse;

		// Combine predicted state and observation
		lastEstimate.mean = predictedState.mean + kalmanGain * innovation.mean;
		lastEstimate.covariance = (
				StateEstimate::CovarianceMatrix::Identity() -
				kalmanGain * measurementMatrix
		) * predictedState.covariance;
	}
};

}       // end of namespaces

#endif