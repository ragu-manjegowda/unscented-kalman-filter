#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    /**
     * End DO NOT MODIFY section for measurement noise values
     */

    /**
     * Done: Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */
    bool is_initialized_ = false;

    n_x_ = 5;

    n_aug_ = n_x_ + 2;

    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    time_us_ = 0;

    weights_ = VectorXd((2 * n_aug_) + 1);

    lambda_ = 3 - n_aug_;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    /**
     * Done: Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */

    /**
     * If not initialized, initialize!
     */
    if (!is_initialized_)
    {
        if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            // clang-format off
            P_ <<  std_laspx_ * 2,               0, 0, 0, 0,
                                0,  std_laspy_ * 2, 0, 0, 0,
                                0,               0, 1, 0, 0,
                                0,               0, 0, 1, 0,
                                0,               0, 0, 0, 1;

            // set the state with the initial location and zero velocity
            x_ <<   meas_package.raw_measurements_[0],
                    meas_package.raw_measurements_[1],
                    0,
                    0;
            // clang-format on

            time_us_ = meas_package.timestamp_;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            // clang-format off
            P_ << std_radr_*2,              0,             0, 0, 0,
                            0,  std_radphi_*2,             0, 0, 0,
                            0,              0,  std_radrd_*2, 0, 0,
                            0,              0,             0, 1, 0,
                            0,              0,             0, 0, 1;
            // clang-format on

            // Convert radar from polar to cartesian coordinates and initialize state.
            float rho = meas_package.raw_measurements_(0);
            float phi = meas_package.raw_measurements_(1);
            float rho_dot = meas_package.raw_measurements_(2);
            float vx = rho_dot * cos(phi);
            float vy = rho_dot * sin(phi);
            float v = sqrt(vx * vx + vy * vy);

            // clang-format off
            x_ << rho * cos(phi),
                  rho * sin(phi),
                  v,
                  0;
            // clang-format on

            time_us_ = meas_package.timestamp_;
        }
        else
        {
            std::cout << "Unknown sensor type, not able to initialize ukf" << std::endl;
            return;
        }

        is_initialized_ = true;
        return;
    }

    /**
     * If initialized, predict and update!
     */

    // compute the time elapsed between the current and previous measurements
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    Prediction(dt);

    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        UpdateLidar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        UpdateRadar(meas_package);
    }
    else
    {
        std::cout << "Unknown sensor type, not able to process ukf" << std::endl;
    }
}

void UKF::Prediction(double delta_t)
{
    /**
     * TODO: Complete this function! Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /**
     * TODO: Complete this function! Use lidar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the lidar NIS, if desired.
     */
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    /**
     * TODO: Complete this function! Use radar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the radar NIS, if desired.
     */
}