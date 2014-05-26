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

#ifndef YAK_MEASUREMENT_PREDICTORS_HPP
#define YAK_MEASUREMENT_PREDICTORS_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

namespace yak {

template <typename TMeasurementModel>
class LinearMeasurementPredictor
{

protected:

    /**
     * The measurement model that the measurement predictor uses.
     */
    TMeasurementModel* const measurementModel;

    /**
     * Convenience typedef for the measurement estimate type of the process.
     */
    typedef typename TMeasurementModel::MeasurementEstimate MeasurementEstimate;

public:

    /**
     * Constructor.
     */
    LinearMeasurementPredictor(TMeasurementModel* measurementModel_) :
        measurementModel(measurementModel_) {}

    /**
     * Predicts the next measurement coming from the model based on the
     * current \em "a priori" state estimate, using the assumption that the
     * measurement is truly linear.
     *
     * \param  predictedState  the current \em "a priori" state estimate from
     *                         the Kalman filter
     * \return the current measurement estimate
     */
    template <typename StateEstimate>
    MeasurementEstimate predictNextMeasurement(const StateEstimate& predictedState) const {
        MeasurementEstimate predictedMeasurement;
        Eigen::Matrix<typename StateEstimate::DataType,
            MeasurementEstimate::DIMENSIONS, StateEstimate::DIMENSIONS> measurementMatrix;

        // Calculate the measurement matrix
        measurementMatrix = measurementModel->getMeasurementMatrix();

        // Calculate the predicted measurement from the predicted state
        predictedMeasurement.mean = measurementMatrix * predictedState.mean;
        predictedMeasurement.covariance = measurementMatrix *
            predictedState.covariance * measurementMatrix.transpose() +
            measurementModel->getNoiseCovarianceMatrix();

        return predictedMeasurement;
    }

public:

    /**
     * Calculates the state-measurement cross-covariance matrix from the
     * \em "a priori" state estimate.
     *
     * \param  predictedState  the \em "a priori" state estimate.
     * \param  result          reference to an object where the
     *                         cross-covariance matrix is returned. This is
     *                         needed because C++ cannot infer template
     *                         arguments when they are only used as return
     *                         values.
     * 
     */
    template <typename StateEstimate, typename StateMeasurementCrossCovarianceMatrix>
    void calculateStateMeasurementCrossCovarianceMatrix(
            const StateEstimate& predictedState,
            StateMeasurementCrossCovarianceMatrix& result) const {
        result = predictedState.covariance *
            (measurementModel->getMeasurementMatrix()).transpose();
    }

};

}       // end of namespaces

#endif
