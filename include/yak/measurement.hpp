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

	/**
	 * Typedef for the datatype used in this class.
	 */
	typedef T DataType;

	/**
	 * Typedef for a column vector containing exactly \c DIMENSIONS coordinates.
	 */
	typedef Eigen::Matrix<T, dimension, 1> ColumnVector;
	
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
	 * Required typedef; points to the measurement type used by this model.
	 */
	typedef TMeasurement Measurement;

	/**
	 * Convenience typedef; points to the type of the matrix that transforms a
	 * state into a measurement.
	 */
	typedef Eigen::Matrix<typename TState::DataType,
			TMeasurement::DIMENSIONS, TState::DIMENSIONS> MeasurementMatrix;

	/**
	 * Convenience typedef; points to the type of the matrix that describes the
	 * covariance of a measurement.
	 */
	typedef Eigen::Matrix<typename TState::DataType,
			TMeasurement::DIMENSIONS, TMeasurement::DIMENSIONS> CovarianceMatrix;

	/**
	 * Convenience typedef; points to a vector that contains the variances of the
	 * individual noise components.
	 */
	typedef Eigen::Matrix<typename TState::DataType, TMeasurement::DIMENSIONS, 1>
		NoiseVarianceVector;

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
		int i, n = sizeof(result);
		for (i = 0; i < n; i++) {
			result(i) = standard_normal<typename TState::DataType>() * variances(i);
		}
		return result;
	}
};

}       // end of namespaces

#endif