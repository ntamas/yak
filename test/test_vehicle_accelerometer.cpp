/**
 * Simple test for the Kalman filter to predict the location of a vehicle
 * fitted with an inaccurate GPS and a not-so-inaccurate accelerometer.
 *
 * The vehicle moves along a Lissajous curve centered at the origin, with
 * predefined A and B coefficients and a maximum horizontal and vertical
 * extent of 40 metres (from -20 to 20 on the X/Y axis). The vehicle is fitted
 * with a GPS that provides an inaccurate reading of the location with a
 * given standard deviation (default: 1.5 metres). It is also fitted with an
 * accelerometer that measures the acceleration of the device along the X and
 * Y axes fairly accurately. The catch is that the state model fed into the
 * Kalman filter assumes standard Newtonian motion with no knowledge about the
 * external forces that keep the vehicle on its path; in other words, the
 * acceleration is assumed to be constant by the Kalman filter instead of being
 * fed into the filter as a control signal.
 */

#include <iostream>
#include <yak/yak.hpp>

#define DT 0.1
#define RADIUS 20
#define GPS_SD 1.5
#define ACCEL_SD 0.05
#define LISSAJOUS_A 0.5
#define LISSAJOUS_B 1
#define LISSAJOUS_DELTA M_PI/2

using namespace Eigen;
using namespace std;
using namespace yak;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class VehicleState : public Vector6d {
public:
	VehicleState() : Vector6d() {}
	double xPosition() { return (*this)(0); }
	double yPosition() { return (*this)(1); }
	double xVelocity() { return (*this)(2); }
	double yVelocity() { return (*this)(3); }
	double xAcceleration() { return (*this)(4); }
	double yAcceleration() { return (*this)(5); }
};

class VehicleStateEstimate : public Gaussian<6> {
public:
	VehicleStateEstimate() : Gaussian<6>() {}
	double xPosition() { return mean(0); }
	double yPosition() { return mean(1); }
	double xVelocity() { return mean(2); }
	double yVelocity() { return mean(3); }
	double xAcceleration() { return mean(4); }
	double yAcceleration() { return mean(5); }
};

typedef SimpleControlVector<0> VehicleControlVector;

class VehicleProcessModel {
public:
	typedef VehicleStateEstimate StateEstimate;
	typedef VehicleControlVector ControlVector;

	Matrix6d calculateJacobian(double dt) {
		Matrix6d result;
		result << 1,  0, dt,  0,  0,  0,
			      0,  1,  0, dt,  0,  0,
				  0,  0,  1,  0, dt,  0,
				  0,  0,  0,  1,  0, dt,
				  0,  0,  0,  0,  1,  0,
				  0,  0,  0,  0,  0,  1;
		return result;
	}

	Eigen::Matrix<double, 6, 0> calculateControlMatrix(double dt) {
		return Eigen::Matrix<double, 6, 0>::Zero();
	}

	Matrix6d calculateNoiseCovarianceMatrix(double dt) {
		/*
		Vector6d variances;
		variances << 0.01, 0.01, 0.01, 0.01, 0.1, 0.1;
		return variances.asDiagonal();
		*/
		return Matrix6d::Identity() * 0.1;
	}
};

typedef SimpleMeasurement<4> VehicleMeasurement;
class VehicleMeasurementModel {

public:

	typedef VehicleMeasurement Measurement;
	typedef Eigen::Matrix<double, 4, 6> MeasurementMatrix;

	double time;
	Vector4d variances;

	VehicleMeasurementModel() : time(0), variances() {
		variances << GPS_SD*GPS_SD, GPS_SD*GPS_SD, ACCEL_SD*ACCEL_SD, ACCEL_SD*ACCEL_SD;
	}

	MeasurementMatrix getMeasurementMatrix() const {
		MeasurementMatrix result;
		result << 1, 0, 0, 0, 0, 0,
			      0, 1, 0, 0, 0, 0,
				  0, 0, 0, 0, 1, 0,
				  0, 0, 0, 0, 0, 1;
		return result;
	}

	Matrix4d getNoiseCovarianceMatrix() const {
		return variances.asDiagonal();
	}

	VehicleState getRealState() {
		VehicleState result;
		result(0) = RADIUS * sin(LISSAJOUS_A*time+LISSAJOUS_DELTA);
		result(1) = RADIUS * sin(LISSAJOUS_B*time);
		result(2) = RADIUS * LISSAJOUS_A * cos(LISSAJOUS_A*time+LISSAJOUS_DELTA);
		result(3) = RADIUS * LISSAJOUS_B * cos(LISSAJOUS_B*time);
		result(4) = -LISSAJOUS_A*LISSAJOUS_A*result(0);
		result(5) = -LISSAJOUS_B*LISSAJOUS_B*result(1);
		return result;
	}

	Vector4d generateNoise() const {
		Vector4d result;
		for (int i = 0; i < 4; i++) {
			result(i) = standard_normal<double>() * variances(i);
		}
		return result;
	}

	Measurement measure() {
		Measurement result;
		result.value = getMeasurementMatrix() * getRealState() + generateNoise();
		return result;
	}

	void step(double dt) {
		time += dt;
	}
};

int main(int argc, char* argv[]) {
	VehicleProcessModel vehicleProcessModel;
	VehicleMeasurementModel vehicleMeasurementModel;
	VehicleMeasurement measurement;
	VehicleControlVector nullVector;

	KalmanFilter<VehicleProcessModel, VehicleMeasurementModel> filter(
			&vehicleProcessModel, &vehicleMeasurementModel);

	cout << "time\tactual_x\tactual_y\tmeasured_x\tmeasured_y\testimated_x\testimated_y\n";
	while (vehicleMeasurementModel.time < 4*M_PI) {
		vehicleMeasurementModel.step(DT);
		measurement = vehicleMeasurementModel.measure();
		filter.update(DT, nullVector, measurement);

		cout << vehicleMeasurementModel.time << '\t';
		cout << vehicleMeasurementModel.getRealState()(0) << '\t';
		cout << vehicleMeasurementModel.getRealState()(1) << '\t';
		cout << measurement.value(0) << '\t';
		cout << measurement.value(1) << '\t';
		cout << filter.lastEstimate.mean(0) << '\t';
		cout << filter.lastEstimate.mean(1) << '\n';
	}

}

