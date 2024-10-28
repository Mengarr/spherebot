#include "../include/dfr_10_dof_pkg/imu_data_node.hpp"

ImuPublisher::ImuPublisher()
    : Node("imu_data"),
    gyro(ITG3200_ADDR_AD0_LOW),
    accel(ADXL345_ADDR_ALT_LOW)
{   
    RCLCPP_INFO(this->get_logger(), "IMU node has been started");

    // Imu publisher
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    // Timer for data publishing
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), // 20 Hz
    std::bind(&ImuPublisher::publish_data, this));

    // initialize quaternion
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0;
    eyInt = 0.0;
    ezInt = 0.0;
    twoKp = twoKpDef;
    twoKi = twoKiDef;
    integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

    // Gyro data:
    RCLCPP_INFO(this->get_logger(), "Initalising Gyro");
    gyro.init();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // Sleep for one seccond
    gyro.zeroCalibrate(128,5);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Sleep for one seccond
    
    // Accelerometer data
    RCLCPP_INFO(this->get_logger(), "Initalising Accelerometer");
    accel.init();

    RCLCPP_INFO(this->get_logger(), "IMU Node Initalisation Complete");
}

float invSqrt(float number) {
    return 1.0f / std::sqrt(number);
}

// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from Sebastian Madgwick filter which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
//=====================================================================================================
void ImuPublisher::AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q1q3, q2q3, q3q3;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float qa, qb, qc;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q1q3 = q1 * q3;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
        float halfvx, halfvy, halfvz;
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Estimated direction of gravity
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
    
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex += (ay * halfvz - az * halfvy);
        halfey += (az * halfvx - ax * halfvz);
        halfez += (ax * halfvy - ay * halfvx);
    }

    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
        integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
        integralFBy += twoKi * halfey * (1.0f / sampleFreq);
        integralFBz += twoKi * halfez * (1.0f / sampleFreq);
        gx += integralFBx;  // apply integral feedback
        gy += integralFBy;
        gz += integralFBz;
        }
        else {
        integralFBx = 0.0f; // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void ImuPublisher::getValues(float * values) {  
    int16_t accval[3];
    accel.readAccel(&accval[0], &accval[1], &accval[2]);
    values[0] = ((float) accval[0]);
    values[1] = ((float) accval[1]);
    values[2] = ((float) accval[2]);

    gyro.readGyro(&values[3]);

    _AccelX = accval[0];
    _AccelY = accval[1];
    _AccelZ = values[2];

    _GyroX = values[3];
    _GyroY = values[4];
    _GyroZ = values[5];
}

void ImuPublisher::getQ(float * q) {
    float val[9];
    getValues(val);
    
    now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - lastUpdate);
    double elapsedTime = static_cast<double>(duration.count());  // Time difference in microseconds
    //micros_now = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    sampleFreq = 1.0 / ((elapsedTime) / 1000000.0);
    lastUpdate = now;
    // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
    //AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
    // use the call below when using a 6DOF IMU
    AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2]);
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

void ImuPublisher::getQ() {
    float val[9];
    getValues(val);
    
    now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - lastUpdate);
    double elapsedTime = static_cast<double>(duration.count());  // Time difference in microseconds
    //micros_now = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    sampleFreq = 1.0 / ((elapsedTime) / 1000000.0);
    lastUpdate = now;
    // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
    //AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
    // use the call below when using a 6DOF IMU
    AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2]);
}

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
void ImuPublisher::getEuler(float * angles) {
    float q[4]; // quaternion
    getQ(q);
    angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI; // psi
    angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180/M_PI; // theta
    angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180/M_PI; // phi
}

void ImuPublisher::getAngles(float * angles) {
    float a[3]; //Euler
    getEuler(a);

    angles[0] = a[0];
    angles[1] = a[1];
    angles[2] = a[2];
    
    if(angles[0] < 0)angles[0] += 360;
    if(angles[1] < 0)angles[1] += 360;
    if(angles[2] < 0)angles[2] += 360;
}

void ImuPublisher::getYawPitchRoll(float * ypr) {
    float q[4]; // quaternion
    float gx, gy, gz; // estimated gravity direction
    getQ(q);
    
    gx = 2 * (q[1]*q[3] - q[0]*q[2]);
    gy = 2 * (q[0]*q[1] + q[2]*q[3]);
    gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
    ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI;
    ypr[1] = atan(gx / sqrt(gy*gy + gz*gz))  * 180/M_PI;
    ypr[2] = atan(gy / sqrt(gx*gx + gz*gz))  * 180/M_PI;
}

void ImuPublisher::publish_data()
{
    auto imu_msg = sensor_msgs::msg::Imu();

    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";
    
    // gyro.readGyro(&_GyroX, &_GyroY, &_GyroZ);
    // accel.readAccel(&_AccelX, &_AccelY, &_AccelZ);
    getQ();

    // Example values, replace these with actual data
    imu_msg.orientation.x = q0;
    imu_msg.orientation.y = q1;
    imu_msg.orientation.z = q2;
    imu_msg.orientation.w = q3;
    
    imu_msg.angular_velocity.x = _GyroX;
    imu_msg.angular_velocity.y = _GyroY;
    imu_msg.angular_velocity.z = _GyroZ;

    imu_msg.linear_acceleration.x = _AccelX * 0.00390625f;
    imu_msg.linear_acceleration.y = _AccelY * 0.00390625f;
    imu_msg.linear_acceleration.z = _AccelZ * 0.00390625f;

    // Publish the IMU data
    imu_publisher_->publish(imu_msg);
    
    //RCLCPP_INFO(this->get_logger(), "Published IMU data: [_GyroX: %f, _GyroY: %f, _GyroZ: %f], [_AccelX %d, _AccelY %d, _AccelZ %d]", _GyroX, _GyroY, _GyroZ, _AccelX, _AccelY, _AccelZ);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPublisher>());
    rclcpp::shutdown();
    return 0;
}