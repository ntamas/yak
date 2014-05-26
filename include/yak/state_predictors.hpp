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

#ifndef YAK_STATE_PREDICTORS_HPP
#define YAK_STATE_PREDICTORS_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

namespace yak {

template <typename TProcessModel>
class LinearStatePredictor
{

protected:

    /**
     * The process model that the state predictor uses.
     */
    TProcessModel* const processModel;

    /**
     * Convenience typedef for the state estimate type of the process.
     */
    typedef typename TProcessModel::StateEstimate StateEstimate;

    /**
     * Typedef for the Jacobian matrix of the process.
     */
    typedef Eigen::Matrix<typename StateEstimate::DataType,
        StateEstimate::DIMENSIONS, StateEstimate::DIMENSIONS> JacobianMatrix;

public:

    /**
     * Constructor.
     */
    LinearStatePredictor(TProcessModel* processModel_) :
        processModel(processModel_) {}

    /**
     * Predicts the next state of the model based on the previous state
     * estimate and the time that has passed since the last measurement,
     * using the assumption that the process is truly linear.
     *
     * \param  lastEstimate  the last \em "a posteriori" state estimate
     *                       from the Kalman filter
     * \param  dt  the time that has passed since the last measurement
     * \return the next \em "a priori" state estimate
     */
    StateEstimate predictNextState(const StateEstimate& lastEstimate,
            double dt) const {
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
     * \param  lastEstimate  the last \em "a posteriori" state estimate
     *                       from the Kalman filter
     * \param  dt       the time that has passed since the last measurement
     * \param  control  the current control vector
     * \return the next \em "a priori" state estimate
     */
    template <typename ControlVector>
    StateEstimate predictNextState(const StateEstimate& lastEstimate,
            double dt, const ControlVector& control) const {
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
};

}       // end of namespaces

#endif
