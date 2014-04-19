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

/**
 * Simple test for the Kalman filter based on the "voltage reading" example
 * found on the following webpage:
 *
 * http://bilgin.esme.org/BitsBytes/KalmanFilterforDummies.aspx
 *
 * The process we filter here is a voltage meter that measures a constant
 * voltage of 0.4 V. The voltage meter is assumed to have a noise variance of
 * 0.1. The measurements are hardcoded to reproduce the example on the above
 * webpage exactly.
 */

#include <iostream>
#include <yak/yak.hpp>

using namespace Eigen;
using namespace yak;

typedef Eigen::Matrix<double, 1, 1> Matrix1d;

typedef Gaussian<1> VoltageReadingState;
typedef SimpleMeasurement<1> VoltageMeasurement;
typedef ConstantProcessModel<VoltageReadingState> VoltageProcessModel;

class VoltageMeasurementModel {

public:
	static const int DIMENSIONS = 1;

	static const int NUM_READINGS = 10;
	static const double predefinedReadings[NUM_READINGS];
	
	int time;

	VoltageMeasurementModel() : time(0) {}

	Matrix1d getMeasurementMatrix() const {
		return Matrix1d::Identity();
	}

	Matrix1d getNoiseCovarianceMatrix() const {
		return Matrix1d::Identity() * 0.1;
	}

	VoltageMeasurement measure() {
		VoltageMeasurement result;
		result.value(0) = predefinedReadings[time % NUM_READINGS];
		return result;
	}

	void step() {
		time += 1;
	}
};

const double VoltageMeasurementModel::predefinedReadings[] = {
	0.45, 0.39, 0.50, 0.48, 0.29, 0.25, 0.32, 0.34, 0.48, 0.41
};

int main(int argc, char* argv[]) {
	VoltageReadingState::ColumnVector constantVoltage;
	constantVoltage << 0.4;

	VoltageProcessModel voltageProcessModel(constantVoltage);
	VoltageMeasurementModel voltageMeasurementModel;
	VoltageMeasurement measurement;

	KalmanFilter<VoltageProcessModel, VoltageMeasurementModel> filter(
			&voltageProcessModel, &voltageMeasurementModel);

	std::cout << "time\tactual\tmeasurement\testimate\n";
	while (voltageMeasurementModel.time < 50) {
		voltageMeasurementModel.step();
		measurement = voltageMeasurementModel.measure();
		filter.update(1, measurement);

		std::cout << voltageMeasurementModel.time << '\t';
		std::cout << voltageProcessModel.getCurrentState()(0) << '\t';
		std::cout << measurement.value(0) << '\t';
		std::cout << filter.lastEstimate.mean(0) << '\n';
	}
}

