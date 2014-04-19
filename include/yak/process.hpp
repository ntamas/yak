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

#ifndef YAK_PROCESS_HPP
#define YAK_PROCESS_HPP

namespace yak {

/**
 * Implementation of a "process" that does not change its state and does not
 * react to control signals. Use this class template to implement your own process
 * in a similar way.
 */
template <typename TStateEstimate>
class ConstantProcessModel {

public:

	typedef TStateEstimate StateEstimate;

protected:

	typedef typename StateEstimate::ColumnVector StateVector;
	typedef Eigen::Matrix<
		typename StateEstimate::DataType,
		StateEstimate::DIMENSIONS, StateEstimate::DIMENSIONS
	> StateStateMatrix;

public:

	/**
	 * The constant state that this process assumes in each step.
	 */
	StateVector constantState;

	/**
	 * Default constructor.
	 */
	ConstantProcessModel()
		: constantState(StateVector::Zero()) {}

	/**
	 * Constructor that also sets the constant state.
	 */
	explicit ConstantProcessModel(const StateVector& initialState)
		: constantState(initialState) {}
	
	/**
	 * Calculate the control matrix of the process, assuming that
	 * \c dt seconds have passed since the last Kalman filter step.
	 * This matrix is usually denoted by B in textbooks.
	 *
	 * This function is optional; if you don't have a control vector
	 * for your process, there is no need to add an implementation.
	 *
	 * \param  dt     the length of the time interval
	 */
	template <typename ControlMatrix>
	ControlMatrix calculateControlMatrix(double dt) {
		return ControlMatrix::Zero();
	}

	/**
	 * Calculate the Jacobian matrix of the process, integrated over
	 * \c dt seconds. This matrix is usually denoted by A in textbooks.
	 *
	 * \param  dt     the length of the time interval
	 */
	StateStateMatrix calculateJacobian(double dt) {
		return StateStateMatrix::Identity();
	}

	/**
	 * Returns the covariance matrix of the process noise, integrated over
	 * \c dt seconds. If you have a constant noise covariance matrix
	 * (per second) here, just return it multiplied by \c dt in your own
	 * implementation.
	 *
	 * \param  dt  the length of the time interval
	 */
	StateStateMatrix calculateNoiseCovarianceMatrix(double dt) {
		return StateStateMatrix::Zero();
	}

	/**
	 * Returns the current state of the process. This function is not used by
	 * the Kalman filter directly (obviously) but may be useful for debugging.
	 */
	StateVector getCurrentState() const {
		return constantState;
	}
};

}

#endif


