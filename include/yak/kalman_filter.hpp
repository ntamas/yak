/* vim:set ts=4 sw=4 sts=4 et: */
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

    /**
     * Typedef for the process model used by the filter.
     */
    typedef TProcessModel ProcessModel;

    /**
     * Typedef for the measurement model used by the filter.
     */
    typedef TMeasurementModel MeasurementModel;

    /**
     * Convenience alias for the state estimate type used by the filter.
     */
    typedef typename ProcessModel::StateEstimate StateEstimate;

    /**
     * Convenience alias for the data type of predicted measurements.
     */
    typedef Gaussian<MeasurementModel::DIMENSIONS> MeasurementEstimate;

    typedef Eigen::Matrix<typename StateEstimate::DataType,
        StateEstimate::DIMENSIONS, StateEstimate::DIMENSIONS> JacobianMatrix;
    typedef Eigen::Matrix<typename StateEstimate::DataType,
        MeasurementModel::DIMENSIONS, StateEstimate::DIMENSIONS> MeasurementMatrix;
    typedef Eigen::Matrix<typename StateEstimate::DataType,
        MeasurementModel::DIMENSIONS, MeasurementModel::DIMENSIONS> MeasurementCovarianceMatrix;
    typedef Eigen::Matrix<typename StateEstimate::DataType,
        StateEstimate::DIMENSIONS, MeasurementModel::DIMENSIONS> KalmanGainMatrix;
    typedef Eigen::Matrix<typename StateEstimate::DataType,
        StateEstimate::DIMENSIONS, MeasurementModel::DIMENSIONS> StateMeasurementCrossCovarianceMatrix;

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

public:

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
     * time that has passed since the last measurement, assuming that
     * there is no control signal.
     *
     * \param  dt           the time elapsed since the last measurement
     * \param  measurement  the value of the current measurement
     */
    template <typename Measurement>
    void update(double dt, const Measurement& measurement) {
        StateEstimate predictedState = predictNextState(dt);
        MeasurementEstimate predictedMeasurement = predictNextMeasurement(
                predictedState);
        StateMeasurementCrossCovarianceMatrix crossCovarianceMatrix =
            calculateStateMeasurementCrossCovarianceMatrix(predictedState);
        correctPredictedState(predictedState, predictedMeasurement,
                crossCovarianceMatrix, measurement);
    }

    /**
     * Updates the Kalman filter with the latest measurements, the current
     * control signal and the time that has passed since the last measurement.
     *
     * \param  dt           the time elapsed since the last measurement
     * \param  control      the value of the current control vector
     * \param  measurement  the value of the current measurement
     */
    template <typename Measurement, typename ControlVector>
    void update(double dt, const ControlVector& control,
            const Measurement& measurement) {
        StateEstimate predictedState = predictNextState(dt, control);
        MeasurementEstimate predictedMeasurement = predictNextMeasurement(
                predictedState);
        StateMeasurementCrossCovarianceMatrix crossCovarianceMatrix =
            calculateStateMeasurementCrossCovarianceMatrix(predictedState);
        correctPredictedState(predictedState, predictedMeasurement,
                crossCovarianceMatrix, measurement);
    }

protected:

    /**
     * Predicts the next state of the model based on the previous state
     * estimate and the time that has passed since the last measurement.
     *
     * \param  dt  the time that has passed since the last measurement
     * \return the \em "a priori" state estimate
     */
    StateEstimate predictNextState(double dt) const {
        JacobianMatrix jacobianMatrix;
        StateEstimate predictedState;

        // Calculate the process Jacobian
        jacobianMatrix = processModel->calculateJacobian(dt);

        // Preliminary state prediction
        predictedState.mean = jacobianMatrix * lastEstimate.mean;
        predictedState.covariance = jacobianMatrix * lastEstimate.covariance *
            jacobianMatrix.transpose() +
            processModel->calculateNoiseCovarianceMatrix(dt);

        return predictedState;
    }

    /**
     * Predicts the next state of the model based on the previous state
     * estimate, the time that has passed since the last measurement, and
     * the current control vector.
     *
     * \param  dt       the time that has passed since the last measurement
     * \param  control  the current control vector
     * \return the \em "a priori" state estimate
     */
    template <typename ControlVector>
    StateEstimate predictNextState(double dt, const ControlVector& control) const {
        JacobianMatrix jacobianMatrix;
        StateEstimate predictedState;

        // Calculate the process Jacobian
        jacobianMatrix = processModel->calculateJacobian(dt);

        // Preliminary state prediction
        predictedState.mean = jacobianMatrix * lastEstimate.mean;
        if (control.size() > 0) {
            predictedState.mean += processModel->calculateControlMatrix(dt) * control;
        }
        predictedState.covariance = jacobianMatrix *
            lastEstimate.covariance * jacobianMatrix.transpose() +
            processModel->calculateNoiseCovarianceMatrix(dt);

        return predictedState;
    }

    /**
     * Predicts the next measurement from the \em "a priori" state estimate.
     *
     * \param  predictedState  the \em "a priori" state estimate.
     */
    MeasurementEstimate predictNextMeasurement(const StateEstimate& predictedState)
        const {
        MeasurementEstimate predictedMeasurement;
        MeasurementMatrix measurementMatrix;

        // Calculate the measurement matrix
        measurementMatrix = measurementModel->getMeasurementMatrix();

        // Calculate the predicted measurement from the predicted state
        predictedMeasurement.mean = measurementMatrix * predictedState.mean;
        predictedMeasurement.covariance = measurementMatrix *
            predictedState.covariance * measurementMatrix.transpose() +
            measurementModel->getNoiseCovarianceMatrix();

        return predictedMeasurement;
    }

    /**
     * Calculates the state-measurement cross-covariance matrix from the
     * \em "a priori" state estimate.
     *
     * \param  predictedState  the \em "a priori" state estimate.
     */
    StateMeasurementCrossCovarianceMatrix calculateStateMeasurementCrossCovarianceMatrix(
            const StateEstimate& predictedState) const {
        MeasurementMatrix measurementMatrix =
            measurementModel->getMeasurementMatrix();
        return predictedState.covariance * measurementMatrix.transpose();
    }

    /**
     * Corrects the given \em "a priori" predicted state with the information
     * gained from the given measurement.
     *
     * \param  predictedState  the \em "a priori" predicted state of the system
     *                         that was calculated from the previous state
     *                         estimate using the process model
     * \param  predictedMeasurement  the \em "a priori" predicted measurement
     *                               that was calculated from the predicted
     *                               state using the measurement model
     * \param  crossCovarianceMatrix the state-measurement cross-covariance
     *                               matrix
     * \param  measurement     the most recent \em actual state measurement
     */
    template <typename Measurement>
    void correctPredictedState(const StateEstimate& predictedState,
            const MeasurementEstimate& predictedMeasurement,
            const StateMeasurementCrossCovarianceMatrix& crossCovarianceMatrix,
            const Measurement& actualMeasurement) {
        MeasurementMatrix measurementMatrix;
        Gaussian<Measurement::DIMENSIONS> innovation;
        MeasurementCovarianceMatrix innovationInverse;
        KalmanGainMatrix kalmanGain;

        // Calculate innovation
        innovation.mean = actualMeasurement.value - predictedMeasurement.mean;
        innovation.covariance = predictedMeasurement.covariance;

        // Calculate Kalman gain
        // TODO: check which LU decomposition is the best here; choices are
        // here:
        // http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
        innovationInverse = innovation.covariance.fullPivLu().inverse();
        kalmanGain = crossCovarianceMatrix * innovationInverse;

        // Combine predicted state and observation
        lastEstimate.mean = predictedState.mean + kalmanGain * innovation.mean;
        lastEstimate.covariance = predictedState.covariance -
            kalmanGain * predictedMeasurement.covariance * kalmanGain.transpose();
    }
};

}       // end of namespaces

#endif