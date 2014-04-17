#ifndef YAK_PROCESS_HPP
#define YAK_PROCESS_HPP

namespace yak {

/**
 * Implementation of a "process" that does not change its state and does not
 * react to control signals. Use this class template to implement your own process
 * in a similar way.
 */
template <typename TStateEstimate, typename TControlVector=SimpleControlVector<0> >
class ConstantProcessModel {
public:

	typedef TStateEstimate StateEstimate;
	typedef TControlVector ControlVector;
	typedef typename StateEstimate::ColumnVector StateVector;

	typedef Eigen::Matrix<
		typename StateEstimate::DataType,
		StateEstimate::DIMENSIONS, StateEstimate::DIMENSIONS
	> StateStateMatrix;
	typedef Eigen::Matrix<
		typename StateEstimate::DataType,
		StateEstimate::DIMENSIONS, ControlVector::DIMENSIONS
	> ControlMatrix;

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
	 * \param  dt     the length of the time interval
	 */
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


