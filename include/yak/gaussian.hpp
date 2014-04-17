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

#ifndef YAK_GAUSSIAN_HPP
#define YAK_GAUSSIAN_HPP

#include <Eigen/Core>

namespace yak {

/**
 * Simple class that implements an n-dimensional Gaussian with mean and
 * covariances.
 */
template <int dimension, typename T=double>
class Gaussian {
public:
	/**
	 * Static constant that can be used to retrieve the dimensions of the
	 * Gaussian at compile time.
	 */
	static const int DIMENSIONS = dimension;

	/**
	 * Typedef for the datatype used in this class.
	 */
	typedef T DataType;

	/**
	 * Typedef for a column vector containing exactly \c DIMENSIONS coordinates.
	 */
	typedef Eigen::Matrix<T, dimension, 1> ColumnVector;
	
	/**
	 * Typedef for a square matrix of size \c DIMENSIONS .
	 */
	typedef Eigen::Matrix<T, dimension, dimension> CovarianceMatrix;

	/**
	 * The mean of the Gaussian.
	 */
	ColumnVector mean;

	/**
	 * The covariance matrix of the Gaussian.
	 */
	CovarianceMatrix covariance;

	/**
	 * Default constructor. Constructs a Gaussian with zero mean and unit
	 * variance along each dimension.
	 */
	Gaussian()
		: mean(ColumnVector::Zero()), covariance(CovarianceMatrix::Identity()) {}
};

}       // end of namespaces

#endif
