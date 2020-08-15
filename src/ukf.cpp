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

    n_x_ = 5;

    n_aug_ = n_x_ + 2;

    n_sig_ = 2 * n_aug_ + 1;

    // initial state vector
    x_ = VectorXd(n_x_);

    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    // Since we are on highway and as we take measurements less 1/10th of a second,
    // we can choose longitudinal acceleration deviation of 2 m/s^2
    std_a_ = 2;

    // Process noise standard deviation yaw acceleration in rad/s^2
    // Since we are on a highway we don't expect yawdd to be
    // much larger so we are good with smaller value of PI/4 rad/s^2
    std_yawdd_ = M_PI / 4;

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

    is_initialized_ = false;

    Xsig_pred_ = MatrixXd(n_x_, n_sig_);

    time_us_ = 0;

    lambda_ = 3 - n_aug_;

    weights_ = VectorXd(n_sig_);

    // Set weights
    weights_.fill(0.5 / (lambda_ + n_aug_));
    weights_(0) = lambda_ / (lambda_ + n_aug_);

    // Radar measurement degrees of freedom is 3 (r, phi, rd)
    R_radar_ = MatrixXd(3, 3);

    // Lidar measurement degrees of freedom is 2 (x, y)
    R_lidar_ = MatrixXd(2, 2);

    // clang-format off
    R_radar_ << std_radr_ * std_radr_,                         0,                         0,
                                    0, std_radphi_ * std_radphi_,                         0,
                                    0,                         0,   std_radrd_ * std_radrd_;

    R_lidar_ << std_laspx_ * std_laspx_,                       0,
                                        0, std_laspy_ * std_laspy_;
    // clang-format on

    NIS_radar_ = 0.;
    NIS_lidar_ = 0.;
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
            P_ <<   std_laspx_ * std_laspx_,                        0, 0, 0, 0,
                                          0,  std_laspy_ * std_laspy_, 0, 0, 0,
                                          0,                        0, 1, 0, 0,
                                          0,                        0, 0, 1, 0,
                                          0,                        0, 0, 0, 1;

            // set the state with the initial location and zero velocity
            x_ <<   meas_package.raw_measurements_[0],
                    meas_package.raw_measurements_[1],
                                                   0.,
                                                   0.,
                                                   0.;
            // clang-format on
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            // clang-format off
            P_ << std_radr_ * std_radr_,                          0,                        0, 0, 0,
                                      0,  std_radphi_ * std_radphi_,                        0, 0, 0,
                                      0,                          0,  std_radrd_ * std_radrd_, 0, 0,
                                      0,                          0,                        0, 1, 0,
                                      0,                          0,                        0, 0, 1;
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
                              0.,
                              0.;
            // clang-format on
        }
        else
        {
            std::cout << "Unknown sensor type, not able to initialize ukf" << std::endl;
            return;
        }

        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    /**
     * If initialized, predict and update!
     */

    // compute the time elapsed between the current and previous measurements
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    Prediction(dt);

    if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        UpdateLidar(meas_package);
    }
    else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
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
     * Done: Complete this function! Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */

    /**
     * Create augmented Sigma Points
     */

    // create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

    // create augmented mean state
    x_aug.head(n_x_) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; ++i)
    {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }

    /**
     * Predict Sigma Points
     */

    for (int i = 0; i < n_sig_; i++)
    {
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        // avoid division by zero
        if (fabs(yawd) > 0.001)
        {
            Xsig_pred_(0, i) = p_x + (v * (sin(yaw + yawd * delta_t) - sin(yaw)) / yawd) +
                               (.5 * delta_t * delta_t * cos(yaw) * nu_a);
            Xsig_pred_(1, i) = p_y +
                               (v * (-cos(yaw + yawd * delta_t) + cos(yaw)) / yawd) +
                               (.5 * delta_t * delta_t * sin(yaw) * nu_a);
            Xsig_pred_(2, i) = v + 0 + (delta_t * nu_a);
            Xsig_pred_(3, i) =
                yaw + (yawd * delta_t) + (0.5 * delta_t * delta_t * nu_yawdd);
            Xsig_pred_(4, i) = yawd + 0 + (delta_t * nu_yawdd);
        }
        else
        {
            Xsig_pred_(0, i) = p_x + (v * cos(yaw) * delta_t) +
                               (.5 * delta_t * delta_t * cos(yaw) * nu_a);
            Xsig_pred_(1, i) = p_y + (v * sin(yaw) * delta_t) +
                               (.5 * delta_t * delta_t * sin(yaw) * nu_a);
            Xsig_pred_(2, i) = v + 0 + (delta_t * nu_a);
            Xsig_pred_(3, i) =
                yaw + (yawd * delta_t) + (0.5 * delta_t * delta_t * nu_yawdd);
            Xsig_pred_(4, i) = yawd + 0 + (delta_t * nu_yawdd);
        }
    }

    /**
     * Predict mean and covariance
     */

    // predicted state mean
    x_.fill(0.0);

    for (int i = 0; i < n_sig_; ++i)
    {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // predicted state covariance matrix
    P_.fill(0);

    for (int i = 0; i < n_sig_; ++i)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // angle normalization
        while (x_diff(3) > M_PI)
        {
            x_diff(3) -= 2. * M_PI;
        }

        while (x_diff(3) < -M_PI)
        {
            x_diff(3) += 2. * M_PI;
        }

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /**
     * Done: Complete this function! Use lidar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the lidar NIS, if desired.
     */

    // set measurement dimension, radar can measure x and y
    int n_z = 2;

    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sig_);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);

    // transform sigma points into measurement space
    for (int i = 0; i < n_sig_; i++)
    {
        Zsig(0, i) = Xsig_pred_(0, i);
        Zsig(1, i) = Xsig_pred_(1, i);
    }

    // calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < n_sig_; i++)
    {
        z_pred += weights_(i) * Zsig.col(i);
    }

    // calculate innovation covariance matrix S
    S.fill(0);
    for (int i = 0; i < n_sig_; ++i)
    {
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    S += R_lidar_;

    /**
     * Update state and covariance
     */

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);  // 5 * 2

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++)
    {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;  // 5 * 1
        VectorXd z_diff = Zsig.col(i) - z_pred;    // 2 * 1

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    // calculate Kalman gain K;
    MatrixXd K = MatrixXd(n_x_, n_z);  // 5 * 2
    K = Tc * S.inverse();

    // create vector for incoming radar measurement
    VectorXd z = meas_package.raw_measurements_;

    VectorXd z_diff = z - z_pred;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;             // (5 * 2) (2 * 1)
    P_ = P_ - K * S * K.transpose();  // (5 * 2) (2 * 2) (2 * 5)

    // calculate NIS
    NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{

    /**
     * Done: Complete this function! Use radar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the radar NIS, if desired.
     */

    /**
     * Predict measurement state
     */

    // set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);

    // transform sigma points into measurement space
    for (int i = 0; i < n_sig_; i++)
    {
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
        Zsig(1, i) = atan2(p_y, p_x);
        Zsig(2, i) = v * (p_x * cos(yaw) + p_y * sin(yaw)) / Zsig(0, i);
    }

    // calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < n_sig_; i++)
    {
        z_pred += weights_(i) * Zsig.col(i);
    }

    // calculate innovation covariance matrix S
    S.fill(0);
    for (int i = 0; i < n_sig_; ++i)
    {
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while (z_diff(1) > M_PI)
        {
            z_diff(1) -= 2. * M_PI;
        }

        while (z_diff(1) < -M_PI)
        {
            z_diff(1) += 2. * M_PI;
        }

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    S += R_radar_;

    /**
     * Update state and covariance
     */

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);  // 5 * 3

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++)
    {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;  // 5 * 1
        VectorXd z_diff = Zsig.col(i) - z_pred;    // 3 * 1

        while (x_diff(3) > M_PI)
        {
            x_diff(3) -= 2. * M_PI;
        }

        while (x_diff(3) < -M_PI)
        {
            x_diff(3) += 2. * M_PI;
        }

        while (z_diff(1) > M_PI)
        {
            z_diff(1) -= 2. * M_PI;
        }

        while (z_diff(1) < -M_PI)
        {
            z_diff(1) += 2. * M_PI;
        }

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    // calculate Kalman gain K;
    MatrixXd K = MatrixXd(n_x_, n_z);  // 5 * 3
    K = Tc * S.inverse();

    // create vector for incoming radar measurement
    VectorXd z = meas_package.raw_measurements_;

    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI)
    {
        z_diff(1) -= 2. * M_PI;
    }

    while (z_diff(1) < -M_PI)
    {
        z_diff(1) += 2. * M_PI;
    }

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;             // (5 * 3) (3 * 1)
    P_ = P_ - K * S * K.transpose();  // (5 * 3) (3 * 3) (3 * 5)

    // calculate NIS
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}