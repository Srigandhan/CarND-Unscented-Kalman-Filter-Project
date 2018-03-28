#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.9;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.85;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  
  x_ << 1, 1, 0, 0, 1;

  P_ << 1, 0, 0, 0, 0,
	    0, 1, 0, 0, 0,
		0, 0, 1, 0, 0,
		0, 0, 0, 1, 0,
		0, 0, 0, 0, 1;
		
  //set weights
  weights_ = VectorXd(2*n_aug_+1);
  double weight = lambda_/(lambda_+n_aug_);
  weights_(0) = weight;
  for(int i=1;i<2*n_aug_+1;i++)
  {   weight = 1/(2*(lambda_+n_aug_));
      weights_(i)= weight;
  }
  

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
	//cout<<"Inside ProcessMeasurement"<<endl;
	
    if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "EKF: " << endl;

	
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	float rho = meas_package.raw_measurements_(0);
	float phi = meas_package.raw_measurements_(1);
	//float rho_dot = meas_package.raw_measurements_(2);
	
	x_(0) = rho * cos (phi);
	x_(1) = rho * sin (phi);
	/*
	float vx = rho_dot * cos(phi);
	float vy = rho_dot * sin(phi);
	float v  = sqrt(vx * vx + vy * vy);
	x_(2) = v;
	*/
    }
	
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */			
	  
	x_(0) = meas_package.raw_measurements_(0);

    x_(1) = meas_package.raw_measurements_(1);

	
    }
	time_us_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  
    //compute the time elapsed between the current and previous measurements
	float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
	time_us_ = meas_package.timestamp_;
	
	//cout<<"Time diff"<<dt;
	Prediction(dt);
	
	if(use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
	{
		UpdateRadar(meas_package);
	}
	
	else if(use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
	{
		UpdateLidar(meas_package);
	}
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //cout<<"Inside Prediction"<<endl;
  
  /*
	double lambda_init_ = 3 - n_x_;
	
 	
	//create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

    //calculate square root of P
    MatrixXd A = P_.llt().matrixL();
	double sqrt_l = sqrt(lambda_init_+n_x_);
  
	Xsig.col(0) = x_;
	for(int i=0; i<n_x_; i++)
  
	{
      Xsig.col(i+1) = x_ + sqrt_l * A.col(i);
      Xsig.col(i+1+n_x_) = x_ - sqrt_l * A.col(i);
	}
	*/
	
	 //create augmented mean vector
	VectorXd x_aug = VectorXd(7);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(7, 7);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
	
	//create augmented mean state
	x_aug.head(n_x_) = x_;
	x_aug(n_x_) = 0;
	x_aug(n_x_+1) = 0;

  
	//create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(n_x_,n_x_) = P_;
	P_aug(n_x_,n_x_) = std_a_*std_a_;
	P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;
  

	//create square root matrix
	MatrixXd B = P_aug.llt().matrixL();
  

	//create augmented sigma points
	float sqrt_la=sqrt(lambda_+n_aug_);
  
	Xsig_aug.col(0)  = x_aug;
  
  
	for (int i = 0; i< n_aug_; i++)
	{
		Xsig_aug.col(i+1)       = x_aug + sqrt_la * B.col(i);
		Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt_la * B.col(i);
	}
	
	//create matrix with predicted sigma points as columns
	for(int i=0; i<2*n_aug_+1; i++)
{
   double px = Xsig_aug(0,i);
   double py = Xsig_aug(1,i);
   double v = Xsig_aug(2,i);
   double yaw = Xsig_aug(3,i);
   double yawdot = Xsig_aug(4,i);
   double nua = Xsig_aug(5,i);
   double nuyaw = Xsig_aug(6,i);
   
   double row1=0;
   double row2=0;
   //double row3=0;
   //double row4=0;
   //double row5=0;
   
 // Calculating the first matrix values alone which differs based on the yawdot value to avoid divide by zero error
   
   if(yawdot>0.001)
   {
   row1 = v / yawdot * (sin(yaw + yawdot*delta_t) - sin(yaw));
   row2 = v / yawdot * (-cos(yaw + yawdot*delta_t) + cos(yaw));
	}
	else
	{
   row1 = v * cos(yaw) * delta_t;
   row2 = v * sin(yaw) * delta_t;
	}

// Adding the remaining left out values and assigning each values to the corresponding row of the current column.
Xsig_pred_(0,i) = px + row1 + 0.5 * delta_t * delta_t * cos(yaw) * nua;
Xsig_pred_(1,i) = py + row2 + 0.5 * delta_t * delta_t * sin(yaw) * nua;
Xsig_pred_(2,i) = v + 0 + delta_t * nua;
Xsig_pred_(3,i) = yaw + yawdot * delta_t + 0.5 * delta_t * delta_t * nuyaw;
Xsig_pred_(4,i) = yawdot + 0 + delta_t * nuyaw;

}

  x_.fill(0.0);
  for(int i=0;i<2*n_aug_+1;i++)
  {
      x_ = x_ + weights_(i)*Xsig_pred_.col(i);
  }
  
  //predict state covariance matrix
  P_.fill(0.0);
  for(int i=0;i<2*n_aug_+1;i++)
  {
    VectorXd diff = Xsig_pred_.col(i)-x_;
    //angle normalization
    while (diff(3)> M_PI) diff(3)-=2.*M_PI;
    while (diff(3)<-M_PI) diff(3)+=2.*M_PI;
    P_ = P_ + weights_(i) * diff * diff.transpose();
  }
  
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
    
  //Initialize H Matrix for Lidar Measurement and Update.
  MatrixXd H_ = MatrixXd(2,5);

  H_ << 1, 0, 0, 0, 0,

       0, 1, 0, 0, 0;
	   
  // Define the measurement noise matrix for LIDAR
  MatrixXd R_lidar_ = MatrixXd(2,2);

  R_lidar_.fill(0.0);

  R_lidar_(0,0) = std_laspx_ * std_laspx_;

  R_lidar_(1,1) = std_laspy_ * std_laspy_;

  
  const int n_z = 2; // A LASER measurement has only two Cartesian coordinates, p_x and p_y

  VectorXd z = VectorXd(n_z);

  // Extract the measurement z
  z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);

  	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_lidar_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
  
    
  // Compute the normalized innovation squared (NIS)
  double nis = y.transpose() * Si * y;
  cout << "LIDAR NIS: " << nis << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  //cout<<"Inside Update Radar"<<endl;
  
   int n_z = 3;
     //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  
  for(int i=0; i<2*n_aug_+1;i++)
{
VectorXd h = VectorXd(3);
h(0) = sqrt(Xsig_pred_(0,i)*Xsig_pred_(0,i)+Xsig_pred_(1,i)*Xsig_pred_(1,i));
 if(h(0) < 0.001) {

      std::cout << "(px, py) is too close to zero. Skipping this measurement.";
      return;
    }
h(1) = atan2(Xsig_pred_(1,i),Xsig_pred_(0,i));
h(2) = (Xsig_pred_(0,i)*cos(Xsig_pred_(3,i))*Xsig_pred_(2,i) +  Xsig_pred_(1,i)*sin(Xsig_pred_(3,i))*Xsig_pred_(2,i))/h(0);
Zsig.col(i)= h;
}

//std::cout << "zsig: " << std::endl << Zsig << std::endl;
  
  //calculate mean predicted measurement
  
z_pred.fill(0.0);
for(int i=0;i<2*n_aug_+1;i++)
{
      z_pred = z_pred + weights_(i)*Zsig.col(i);
}
  
  
  //calculate innovation covariance matrix S

  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    VectorXd diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (diff(1)> M_PI) diff(1)-=2.*M_PI;
    while (diff(1)<-M_PI) diff(1)+=2.*M_PI;

    S = S + weights_(i) * diff * diff.transpose();
  }

//Define the measurement noise matrix for RADAR
 MatrixXd R_radar_ = MatrixXd(3,3);
 
 R_radar_ <<std_radr_*std_radr_, 0, 0,
			0, std_radphi_*std_radphi_, 0,
			0, 0,std_radrd_*std_radrd_;
 
 S = S + R_radar_;
 
 //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // Incoming Radar Measurement
  VectorXd z = VectorXd(n_z);
  
  float rho = meas_package.raw_measurements_(0);
  float phi = meas_package.raw_measurements_(1);
  float rho_dot = meas_package.raw_measurements_(2);
  z << rho, phi, rho_dot;
      
  //residual
  VectorXd z_diffs = z - z_pred;

  //angle normalization
  while (z_diffs(1)> M_PI) z_diffs(1)-=2.*M_PI;
  while (z_diffs(1)<-M_PI) z_diffs(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diffs;
  P_ = P_ - K*S*K.transpose();
  
  // Compute the normalized innovation squared (NIS)
  double nis = z_diffs.transpose() * S.inverse() * z_diffs;
  cout << "RADAR NIS: " << nis << endl;
}
