/*

  https://nitinjsanket.github.io/tutorials/attitudeest/madgwick.html

*/
#include "BerryIMU_v3.h"
#include "vector"
std::vector<float> get_euler_angles_from_accel(float accelX, float accelY, float accelZ);
std::vector<float> get_euler_angles_from_quat(float q1, float q2, float q3, float q4);
std::vector<float> q_est = {1, 0, 0, 0}; //Assumed initial orientation of IMU
std::vector<float> q = {1, 0, 0, 0}; //Final quaterion per loop
float init_time;

BerryIMU_v3 BerryIMU;

void setup() {

  Serial.begin(115200);  // start serial for output

  //Finding an approximate initial orientation and converting it to a quaternion
  BerryIMU.IMU_read();

  //  //Find the yaw, pitch, and roll using the accelerometer readings
  std::vector<float> angles = get_euler_angles_from_accel(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);
  float x_rot  = angles[0];
  float y_rot = angles[1];
  float z_rot = angles[2];
  
//    Serial.print(x_rot);
//    Serial.print(",");
//    Serial.print(y_rot);
//    Serial.print(",");
//    Serial.println(z_rot);

//would ignore the z_rot and conver to quat for initial guess

  float init_time = micros();

}

void loop() {

  BerryIMU.IMU_read();

  //Gravity, gyro, and accel quaterions
  std::vector<float> g_W = {0, 0, 0, 1}; //May need to make neg depending on orientation
  std::vector<float> gyro_I = {0, BerryIMU.gyr_rateXraw * (3.1415/180), BerryIMU.gyr_rateYraw * (3.1415/180), BerryIMU.gyr_rateZraw * (3.1415/180)}; // in rad/s(converted from deg/s)
  float ax = BerryIMU.AccXraw;
  float ay = BerryIMU.AccYraw;
  float az = BerryIMU.AccZraw;
  float mag_accel = sqrt(pow(ax,2)+pow(ay,2)+pow(az,2));
  std::vector<float> a_I = {0, ax/mag_accel, ay/mag_accel, az/mag_accel}; //Normalized Accel

//    Serial.print(a_I[0]);
//    Serial.print(",");
//    Serial.print(a_I[1]);
//    Serial.print(",");
//    Serial.print(a_I[2]);
//    Serial.print(",");
//    Serial.println(a_I[3]);
  
  //q_est components
  float q1 = q_est[0];
  float q2 = q_est[1];
  float q3 = q_est[2];
  float q4 = q_est[3];

  //Orientation from Acceleration
  std::vector<float> f = {(2 * (q2 * q4 - q1 * q3)) - a_I[1],
                          (2 * (q1 * q2 - q3 * q4)) - a_I[2],
                          (2 * (0.5 - pow(q2, 2) - pow(q3, 2))) - a_I[3]};  //3x1 Matrix
  std::vector<float> J = { -2 * q3, 2 * q4, -2 * q1, 2 * q2,
                           2 * q2, 2 * q1, 2 * q4, 2 * q3,
                           0, -4 * q2, -4 * q3, 0};  //3x4 Matrix
  std::vector<float> J_trans = { -2 * q3, 2 * q2, 0,  
                                 2 * q4, 2 * q1, -4 * q2,
                                 -2 * q1, 2 * q4, -4 * q3,
                                 2 * q2, 2 * q3, 0}; //4x3 Matrix

  //Matrix Multiplication (done by multiplying each element)
  //J_trans*f 4x3 *3x1 = 4x1
  std::vector<float> del_f = {J_trans[0]*f[0] + J_trans[1]*f[1] + J_trans[2]*f[2],
                              J_trans[3]*f[0] + J_trans[4]*f[1] + J_trans[5]*f[2],
                              J_trans[6]*f[0] + J_trans[7]*f[1] + J_trans[8]*f[2],
                              J_trans[9]*f[0] + J_trans[10]*f[1] + J_trans[11]*f[2]}; //4x1 Matrix

  //Update Term
  //float beta = 0.033;  //Beta is a tuning parameter
  float beta = 2;  //Beta is a tuning parameter
  float del_f_norm = sqrt(pow(del_f[0], 2) + pow(del_f[1], 2) + pow(del_f[2], 2)+pow(del_f[3], 2));
  std::vector<float> del_q_est = { -beta*(del_f[0] / del_f_norm),
                                   -beta*(del_f[1] / del_f_norm),
                                   -beta*(del_f[2] / del_f_norm),
                                   -beta*(del_f[3] / del_f_norm)}; //4x1 Matrix

  //Orientation from Gyroscope
  //quaternion product wikepedia  //Appears wrong
  std::vector<float> q_dot_w2 = {q_est[0]*gyro_I[0] - q_est[1]*gyro_I[1] - q_est[2]*gyro_I[2] - q_est[3]*gyro_I[3],
                                q_est[0]*gyro_I[1] + q_est[1]*gyro_I[0] + q_est[2]*gyro_I[3] - q_est[3]*gyro_I[2],
                                q_est[0]*gyro_I[2] - q_est[1]*gyro_I[3] + q_est[2]*gyro_I[0] + q_est[3]*gyro_I[1],
                                q_est[0]*gyro_I[3] + q_est[1]*gyro_I[2] - q_est[2]*gyro_I[1] + q_est[3]*gyro_I[0]}; //4x1 Matrix
  q_dot_w2 = {0.5*q_dot_w2[0],
             0.5*q_dot_w2[1],
             0.5*q_dot_w2[2],
             0.5*q_dot_w2[3]};                             

  //quat product another source
  //https://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf  //Seems to work
  std::vector<float> q_dot_w = {q_est[0]*gyro_I[0]-(q_est[1]*gyro_I[1]+q_est[2]*gyro_I[2]+q_est[3]*gyro_I[3])+q_est[0]*(gyro_I[1]+gyro_I[2]+gyro_I[3])+gyro_I[0]*(q_est[1]+q_est[2]+q_est[3]),
                                 q_est[2]*gyro_I[3]-q_est[3]*gyro_I[2],
                                 q_est[3]*gyro_I[1]-q_est[1]*gyro_I[3],
                                 q_est[1]*gyro_I[2]-q_est[2]*gyro_I[1]}; //4x1 Matrix
   q_dot_w = {0.5*q_dot_w[0],
              0.5*q_dot_w[1],
              0.5*q_dot_w[2],
              0.5*q_dot_w[3]};   

  //For comparison try the multiplication from the craine textbook as well.

  //They are different need to look at again
//  Serial.print(q_dot_w[0]);
//  Serial.print(",");
//  Serial.print(q_dot_w[1]);
//  Serial.print(",");
//  Serial.print(q_dot_w[2]);
//  Serial.print(",");
//  Serial.println(q_dot_w[3]);
//  Serial.print(q_dot_w2[0]);
//  Serial.print(",");
//  Serial.print(q_dot_w2[1]);
//  Serial.print(",");
//  Serial.print(q_dot_w2[2]);
//  Serial.print(",");
//  Serial.println(q_dot_w2[3]);
//  Serial.println("Next");
  
  //Fuse Measurements
  std::vector<float> q_est_dot = {q_dot_w[0] + del_q_est[0],
                                  q_dot_w[1] + del_q_est[1],
                                  q_dot_w[2] + del_q_est[2],
                                  q_dot_w[3] + del_q_est[3]};
  //Time Interval
  float final_time = micros();
  float t_interval = (final_time - init_time) / 1000000; //in seconds
  init_time = micros();

  q = {q_est[0] + q_est_dot[0]*t_interval,
       q_est[1] + q_est_dot[1]*t_interval,
       q_est[2] + q_est_dot[2]*t_interval,
       q_est[3] + q_est_dot[3]*t_interval};  //Current estimate

  float q_mag = sqrt(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2));
  std::vector<float> q_norm = {q[0]/q_mag,
                               q[1]/q_mag,
                               q[2]/q_mag,
                               q[3]/q_mag};//normalize the quaternion before next iteration
                               //will drift if not normalized

//  Serial.print(q[0]);
//  Serial.print(",");
//  Serial.print(q[1]);
//  Serial.print(",");
//  Serial.print(q[2]);
//  Serial.print(",");
//  Serial.println(q[3]);
//  Serial.print(q_norm[0]);
//  Serial.print(",");
//  Serial.print(q_norm[1]);
//  Serial.print(",");
//  Serial.print(q_norm[2]);
//  Serial.print(",");
//  Serial.println(q_norm[3]);
//  Serial.println("Next");
  
  q_est = q_norm; //Update the previous estimate

  float q1_euler = q_norm[0];
  float q2_euler = q_norm[1];
  float q3_euler = q_norm[2];
  float q4_euler = q_norm[3];

  //Seems to work is in the Z-Y-X config
  //Need to add function that converts to euler angles to output
  std::vector<float> angles_euler = get_euler_angles_from_quat(q1_euler, q2_euler, q3_euler, q4_euler);
  float roll = angles_euler[0] * (180 / (3.1415)); //converted to degrees //x-axis rot
  float pitch = angles_euler[1] * (180 / (3.1415)); //converted to degrees //y-axis rot
  float yaw = angles_euler[2] * (180 / (3.1415)); //converted to degrees   //z-axis rot

  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);
//  Serial.print(q_est[0]);
//  Serial.print(",");
//  Serial.print(q_est[1]);
//  Serial.print(",");
//  Serial.print(q_est[2]);
//  Serial.print(",");
//  Serial.println(q_est[3]);
//  Serial.println("Next");

}

std::vector<float> get_euler_angles_from_accel(float accelX, float accelY, float accelZ) {
  //angles in radians
  float x_rot = atan2(accelY, sqrt((accelX * accelX) + (accelZ * accelZ))); // Rotation around y
  float y_rot = atan2(accelX, sqrt((accelY * accelY) + (accelZ * accelZ))); // Rotation around x
  float z_rot = atan2(sqrt((accelX * accelX) + (accelY * accelY)), accelZ); // Rotation around z
  std::vector<float> angles;
  angles.push_back(x_rot);
  angles.push_back(y_rot);
  angles.push_back(z_rot);
  return angles;
}

std::vector<float> get_euler_angles_from_quat(float q1, float q2, float q3, float q4) {

  //Still need to add some stuff to account for the singularities and gymbol lock
  
  float roll = atan2(2 * (q1 * q2 + q3 * q4), 1 - 2 * (pow(q2, 2) + pow(q3, 2))); // in radians
  //float pitch = asin(2 * (q1 * q3 - q4 * q2)); // in radians
  float pitch = 2 * (q1 * q3 - q4 * q2);
  //if (abs(pitch) >= 1) {
  //  pitch = 3.1415/2; //??????
  //}
  //else {
    pitch = asin(pitch);
  //}
  float yaw = atan2(2 * (q1 * q4 + q2 * q3), 1 - 2 * (pow(q3, 2) + pow(q4, 2))); // in radians
  std::vector<float> angles_euler;
  angles_euler.push_back(roll);
  angles_euler.push_back(pitch);
  angles_euler.push_back(yaw);
  return angles_euler;
}
