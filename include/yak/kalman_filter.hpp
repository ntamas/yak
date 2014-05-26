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
#include <yak/measurement_predictors.hpp>
#include <yak/state_predictors.hpp>

namespace yak {

/**
 * Base class for Kalman filter implementations.
 */
template <typename TProcessModel, typename TMeasurementModel,
         typename TStatePredictor, typename TMeasurementPredictor>
class KalmanFilterBase {
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
     * Convenience alias for the measurement estimate type used by the filter.
     */
    typedef typename MeasurementModel::MeasurementEstimate MeasurementEstimate;

    typedef Eigen::Matrix<typename StateEstimate::DataType,
        MeasurementEstimate::DIMENSIONS, MeasurementEstimate::DIMENSIONS> MeasurementCovarianceMatrix;
    typedef Eigen::Matrix<typename StateEstimate::DataType,
        StateEstimate::DIMENSIONS, MeasurementEstimate::DIMENSIONS> KalmanGainMatrix;
    typedef Eigen::Matrix<typename StateEstimate::DataType,
        StateEstimate::DIMENSIONS, MeasurementEstimate::DIMENSIONS> StateMeasurementCrossCovarianceMatrix;

    /**
     * The process model that this filter is using.
     */
    ProcessModel* const processModel;

    /**
     * The measurement model that this filter is using.
     */
    MeasurementModel* const measurementModel;

    /**
     * The state predictor that this filter is using.
     */
    TStatePredictor statePredictor;

    /**
     * The measurement predictor that this filter is using.
     */
    TMeasurementPredictor measurementPredictor;

    /**
     * The last estimate of the state of the process.
     */
    StateEstimate lastEstimate;

public:

    /**
     * Constructor.
     */
    KalmanFilterBase(ProcessModel* const processModel_,
            MeasurementModel* const measurementModel_)
        : processModel(processModel_), measurementModel(measurementModel_),
        statePredictor(processModel_), measurementPredictor(measurementModel_),
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
        StateMeasurementCrossCovarianceMatrix crossCovarianceMatrix;

        StateEstimate predictedState =
            statePredictor.predictNextState(lastEstimate, dt);
        MeasurementEstimate predictedMeasurement =
            measurementPredictor.predictNextMeasurement(predictedState);
        measurementPredictor.calculateStateMeasurementCrossCovarianceMatrix(predictedState,
                crossCovarianceMatrix);
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
        StateMeasurementCrossCovarianceMatrix crossCovarianceMatrix;

        StateEstimate predictedState =
            statePredictor.predictNextState(lastEstimate, dt, control);
        MeasurementEstimate predictedMeasurement =
            measurementPredictor.predictNextMeasurement(predictedState);
        measurementPredictor.calculateStateMeasurementCrossCovarianceMatrix(predictedState,
                crossCovarianceMatrix);
        correctPredictedState(predictedState, predictedMeasurement,
                crossCovarianceMatrix, measurement);
    }

protected:

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
        MeasurementEstimate innovation;
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

template <typename TProcessModel, typename TMeasurementModel>
class KalmanFilter : public KalmanFilterBase<TProcessModel, TMeasurementModel,
    LinearStatePredictor<TProcessModel>,
    LinearMeasurementPredictor<TMeasurementModel> > {

    /**
     * Typedef of the superclass to avoid having to type the entire template
     * every time it is needed.
     */
    typedef KalmanFilterBase<TProcessModel, TMeasurementModel,
        LinearStatePredictor<TProcessModel>,
        LinearMeasurementPredictor<TMeasurementModel> > Super;

public:
    /**
     * Constructor.
     */
    KalmanFilter(TProcessModel* const processModel_,
            TMeasurementModel* const measurementModel_)
        : Super(processModel_, measurementModel_) {}

};

}       // end of namespaces

#endif