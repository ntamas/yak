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
