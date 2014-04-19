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

#ifndef YAK_MEASUREMENT_HPP
#define YAK_MEASUREMENT_HPP

#include <yak/util.hpp>

namespace yak {

/**
 * Simple class that implements an n-dimensional measurement.
 */
template <int dimension, typename T=double>
class SimpleMeasurement {
public:
	/**
	 * Static constant that can be used to retrieve the dimensions of the
	 * measurement vector at compile time.
	 */
	static const int DIMENSIONS = dimension;

protected:
	/**
	 * Typedef for the datatype used in this class.
	 */
	typedef T DataType;

	/**
	 * Typedef for a column vector containing exactly \c DIMENSIONS coordinates.
	 */
	typedef Eigen::Matrix<T, dimension, 1> ColumnVector;
	
public:
	/**
	 * The value of the measurement in vector form.
	 */
	ColumnVector value;

	/**
	 * Default constructor.
	 */
	SimpleMeasurement() : value(ColumnVector::Zero()) {}

	/**
	 * Constructor that sets the measurement to a given value.
	 */
	explicit SimpleMeasurement(const ColumnVector& value_) : value(value_) {}
};

/**
 * Implementation of a measurement model that measures each variable
 * independently and adds independent noise to each variable.
 */
template <typename TState, typename TMeasurement>
class IndependentNoisyMeasurementModel {

public:
	/**
	 * The number of dimensions of the measurement vector.
	 */
	static const int DIMENSIONS = TMeasurement::DIMENSIONS;

protected:
	/**
	 * Convenience typedef; points to the measurement type used by this model.
	 */
	typedef TMeasurement Measurement;

	/**
	 * Convenience typedef; points to the type of the matrix that transforms a
	 * state into a measurement.
	 */
	typedef Eigen::Matrix<typename TState::DataType,
			DIMENSIONS, TState::DIMENSIONS> MeasurementMatrix;

	/**
	 * Convenience typedef; points to the type of the matrix that describes the
	 * covariance of a measurement.
	 */
	typedef Eigen::Matrix<typename TState::DataType,
			DIMENSIONS, DIMENSIONS> CovarianceMatrix;

	/**
	 * Convenience typedef; points to a vector that contains the variances of the
	 * individual noise components.
	 */
	typedef Eigen::Matrix<typename TState::DataType, DIMENSIONS, 1>
		NoiseVarianceVector;

public:
	/**
	 * Contains the variances of the individual noise components.
	 */
	NoiseVarianceVector variances;

	/**
	 * Returns the matrix that transforms a state variable vector into a
	 * measurement vector when the matrix is multiplied by the state from the
	 * right.
	 */
	MeasurementMatrix getMeasurementMatrix() const {
		return MeasurementMatrix::Identity();
	}

	/**
	 * Returns the covariance matrix of the measurement noise.
	 */
	CovarianceMatrix getNoiseCovarianceMatrix() const {
		return variances.asDiagonal();
	}

	/**
	 * Utility function that generates a noise vector using the variances
	 * given in this measurement model.
	 */
	NoiseVarianceVector generateNoise() const {
		NoiseVarianceVector result;
		int i, n = result.size();
		for (i = 0; i < n; i++) {
			result(i) = standard_normal<typename TState::DataType>() * variances(i);
		}
		return result;
	}
};

}       // end of namespaces

#endif
