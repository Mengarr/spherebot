#include "../include/dfr_10_dof_pkg/calibrate_magnetometer_node.hpp"

MagnetometerCalibration::MagnetometerCalibration()
    : Node("imu_data"),
    magnetometer(VCM5883L_ADDRESS)
{   

    RCLCPP_INFO(this->get_logger(), "Initalising Mag Calibration");
    magnetometer.init();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Sleep for one seccond

    // eSamples_t samples = magnetometer.getSamples();
    // eMode_t mm  = magnetometer.getMeasurementMode();
    // eDataRate_t dr = magnetometer.getDataRate();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Sleep for one seccond
    RCLCPP_INFO(this->get_logger(), "Init Complete");

    // Open the file in append mode, creating it if it doesn't exist
    data_file_.open("/home/rohan/spherebot/src/dfr_10_dof_pkg/data/mag_out.txt", std::ios::out | std::ios::trunc);
    if (!data_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing.");
    }

    // Initialize a timer that runs at 10 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 20 ms interval corresponds to 10 Hz
        std::bind(&MagnetometerCalibration::timerCallback, this)
    );

}

// Destructor to close the file
MagnetometerCalibration::~MagnetometerCalibration()
{
    if (data_file_.is_open()) {
        data_file_.close();
    }
}

void MagnetometerCalibration::timerCallback()
{
    if (!data_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "File is not open for writing.");
        return;
    }

    sVector_t magData = magnetometer.readRaw();
    float Mx_uT = -static_cast<float>(magData.XAxis) * conv_factor;
    float My_uT = -static_cast<float>(magData.YAxis) * conv_factor;
    float Mz_uT = -static_cast<float>(magData.ZAxis) * conv_factor;

     // Write the data to the file without headers
    data_file_ << Mx_uT << "," << My_uT << "," << Mz_uT << "\n";
    data_file_.flush(); // Ensure data is written immediately
    
    // RCLCPP_INFO(this->get_logger(), "Magnetometer data written: x=%f, y=%f, z=%f", Mx_uT, My_uT, Mz_uT);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MagnetometerCalibration>());
    rclcpp::shutdown();
    return 0;
}

