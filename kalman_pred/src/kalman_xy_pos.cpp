#include <kalman_xy_pos.hpp>
#include <duav_dock/Marker_find_info.h>
#include <duav_dock/Kalman_info.h>

using NEWMAT::ColumnVector;
using NEWMAT::IdentityMatrix;
using NEWMAT::Matrix;

duav_dock::Marker_find_info current_dUAV_pos;
duav_dock::Kalman_info kalpos;

KalmanXYPos::KalmanXYPos()
{

  ros::NodeHandle nh_in("~");

  dUAV_pos_sub = nh_in.subscribe("/dUAV_dock/pos_xy", 1, &KalmanXYPos::dUAV_pos_cb, this);
  kalman_pub = nh_in.advertise<duav_dock::Kalman_info>("/kalman_pred/kalman_pos", 1);

  model_covariance = 2.0;
  observation_covariance = 1.0;
  time_step = 0.5;

  filter_initialized = false;

  state_vec_size = 4;
  observation_vec_size = 2;

  I = IdentityMatrix(state_vec_size);

  // Process error covariance
  // (measure of the estimated accuracy of the state estimate)
  P_ = Matrix(state_vec_size, state_vec_size);
  P_pred_ = Matrix(state_vec_size, state_vec_size);

  P_ = I;
  P_pred_ = I;

  // kalman gain
  K = Matrix(state_vec_size, observation_vec_size);
  K << 1 << 0
    << 0 << 1
    << 0 << 0
    << 0 << 0;

  // innovation (z - H*x)
  y = ColumnVector(observation_vec_size);
  y << 0
    << 0;

  // covariance of observation noise
  R = IdentityMatrix(observation_vec_size);
  R = R * observation_covariance;

  // observation model (which maps the true state space
  //                    into the observed space)
  H = Matrix(observation_vec_size, state_vec_size);
  H << 1 << 0 << 0 << 0
    << 0 << 1 << 0 << 0;

  // State transition model (sometimes called F)
  A = Matrix(state_vec_size, state_vec_size);
  A << 1 << 0 << time_step << 0
    << 0 << 1 << 0 << time_step
    << 0 << 0 << 1 << 0
    << 0 << 0 << 0 << 1;

  // Process noise covariance matrix Q
  Q = IdentityMatrix(state_vec_size);
  Q = Q * model_covariance;
  //computeProcessNoiseCovariance();

  ROS_INFO("[KALMAN XY POS] - Kalman filter init OK!");
}

void KalmanXYPos::dUAV_pos_cb(const duav_dock::Marker_find_info::ConstPtr &msg)
{
  current_dUAV_pos = *msg;
}

void KalmanXYPos::computeProcessNoiseCovariance()
{
  ColumnVector col_vec_G = ColumnVector(observation_vec_size);
  Matrix Q_top = Matrix(observation_vec_size, observation_vec_size);

  double delta_t_2;
  delta_t_2 = pow(time_step, 2.0);
  delta_t_2 = delta_t_2 / 2;
  col_vec_G << delta_t_2
            << time_step;

  Q_top = col_vec_G * col_vec_G.t();
  Q_top = Q_top * model_covariance;

  ROS_INFO_STREAM(Q_top(1, 1) << " " << Q_top(2, 2) << "\n");

  Q << Q_top(1, 1) << 0 << 0 << 0
    << 0 << Q_top(2, 2) << 0 << 0
    << 0 << 0 << Q_top(1, 1) << 0
    << 0 << 0 << 0 << Q_top(2, 2);
}

void KalmanXYPos::predict()
{
  P_ = A * P_ * A.t() + Q;
  //ROS_INFO_STREAM("P_： " << P_);
  x_ = A * x_;
  ROS_INFO_STREAM("x_1： " << x_.element(0));
}

void KalmanXYPos::update(ColumnVector z)
{
  float modZ;
  Matrix S; // innovation covariance

  y = z - H * x_; // innovation
  ROS_INFO_STREAM("y： " << y.element(0));
  S = R + H * P_ * H.t();
  K = P_ * H.t() * S.i();
  //ROS_INFO_STREAM("K： " << K);
  z0_ = z;
}

void KalmanXYPos::correct()
{
  x_ = x_ + K * y;
  P_ = (I - K * H) * P_;
   x_pred_ = x_;
  P_pred_ = P_;
  ROS_INFO_STREAM("x_2： " << x_.element(0));
  //ROS_INFO_STREAM("P_： " << P_);
}

void KalmanXYPos::correct1()
{
  x_ = x_ + K * y;
  P_ = (I - K * H) * P_;
  x_pred_ = x_;
  P_pred_ = P_;
  ROS_INFO_STREAM("x_2： " << x_.element(0));
  //ROS_INFO_STREAM("P_： " << P_);
  kalpos.x = x_.element(0);
  kalpos.y = x_.element(1);
  ROS_INFO_STREAM("kalpos.x:" << kalpos.x << "kalpos.y" << kalpos.y);
  kalman_pub.publish(kalpos);
}

void KalmanXYPos::predictPred()
{
  P_pred_ = A * P_pred_ * A.t() + Q;
  x_pred_ = A * x_pred_;
}

void KalmanXYPos::correctPred()
{
  x_pred_ = x_pred_ + K * y;
  P_pred_ = (I - K * H) * P_pred_;
  
}

void KalmanXYPos::correctPred1()
{
  x_pred_ = x_pred_ + K * y;
  P_pred_ = (I - K * H) * P_pred_;
  kalpos.x = x_pred_.element(0);
  kalpos.y = x_pred_.element(1);
  ROS_INFO_STREAM("kalpos.x:" << kalpos.x << "kalpos.y" << kalpos.y);
  kalman_pub.publish(kalpos);
}

void KalmanXYPos::initializeFilter(ColumnVector z_initial)
{
  z0_ = ColumnVector(2);
  x_ = ColumnVector(4);
  x_pred_ = ColumnVector(4);

  z0_ = z_initial;
  x_ << z_initial.element(0)
     << z_initial.element(1)
     << 0
     << 0;
  x_pred_ = x_;
  P_ = I;
  P_pred_ = I;

  // kalman gain
  K = Matrix(state_vec_size, observation_vec_size);
  K << 1 << 0
    << 0 << 1
    << 0 << 0
    << 0 << 0;

  // innovation (z - H*x)
  y = ColumnVector(observation_vec_size);
  y << 0
    << 0;

  // covariance of observation noise
  R = IdentityMatrix(observation_vec_size);
  R = R * observation_covariance;

  // observation model (which maps the true state space
  //                    into the observed space)
  H = Matrix(observation_vec_size, state_vec_size);
  H << 1 << 0 << 0 << 0
    << 0 << 1 << 0 << 0;

  // State transition model (sometimes called F)
  A = Matrix(state_vec_size, state_vec_size);
  A << 1 << 0 << time_step << 0
    << 0 << 1 << 0 << time_step
    << 0 << 0 << 1 << 0
    << 0 << 0 << 0 << 1;

  // Process noise covariance matrix Q
  Q = IdentityMatrix(state_vec_size);
  Q = Q * model_covariance;

  ROS_INFO("[KALMAN XY POS - initializeFilter] Initial state x: %f, %f, %f, %f",
           x_.element(0), x_.element(1),
           x_.element(2), x_.element(3));
}

void KalmanXYPos::compute()
{
  ros::Rate loop_rate(10);
  ColumnVector z = ColumnVector(2); // measurement
  z.element(0) = current_dUAV_pos.x;
  z.element(1) = current_dUAV_pos.y;
  while (ros::ok())
  {
    if (filter_initialized && !current_dUAV_pos.reset && !current_dUAV_pos.pred)
    {//当子机看到Apriltag时、且初始化时，可用此预测母鸡位置
      z.element(0) = current_dUAV_pos.x;//观测值即为用相机坐标系下的x,y
      z.element(1) = current_dUAV_pos.y;
      ROS_INFO_STREAM("current_dUAV_pos.x:" << current_dUAV_pos.x << "current_dUAV_pos.y" << current_dUAV_pos.y);
      predict();
      update(z);
      correct();
      predictPred();
      correctPred();
      predictPred();
      correctPred();
      predictPred();
      correctPred1();//预测位置
      //correct1();
    }
    else if (!current_dUAV_pos.firstattach)
    {//当子机未第一次看到apriltag时，一直continue
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }
    else if (filter_initialized && !current_dUAV_pos.reset && current_dUAV_pos.pred)
    {//当子机短时间没看到Apriltag时，执行预测模块，即y、K的参数不变，相当于告诉子机与母机的距离的观测值一直为y，按这个去预测母机未来的位置。
      predictPred();
      correctPred1();
    }
    
    else
    {//当子机第一次看到Apriltag时，以及重新定位重新看到时，初始化kalman
      initializeFilter(z);
      filter_initialized = true;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_node");

  KalmanXYPos kalmanXYPos;

  kalmanXYPos.compute();

  return 0;
}
