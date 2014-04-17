#ifndef YAK_CONTROL_HPP
#define YAK_CONTROL_HPP

#include <Eigen/Core>

namespace yak {

/**
 * Simple class that implements an n-dimensional control vector.
 */
template <int dimension, typename T=double>
class SimpleControlVector {
public:
	/**
	 * Static constant that can be used to retrieve the dimensions of the
	 * control vector at compile time.
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
	 * The value of the control vector.
	 */
	ColumnVector value;

	/**
	 * Default constructor.
	 */
	SimpleControlVector() : value(ColumnVector::Zero()) {}
};

}       // end of namespaces

#endif
