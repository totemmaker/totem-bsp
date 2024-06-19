void run_example_board();
void run_example_can();
void run_example_dc();
void run_example_gpio_rev10();
void run_example_gpio();
void run_example_imu();
void run_example_qwiic();
void run_example_rgb();
void run_example_servo();

void app_main(void) {
    /**
     * !! Select only one example to run !!
     */
    run_example_board(); // Print board info
    // run_example_can();   // CANbus communication
    // run_example_dc();    // DC motor
    // run_example_gpio_rev10(); // RoboBoard X4 v1.0 GPIO
    // run_example_gpio();  // RoboBoard X4 v1.1 GPIO
    // run_example_imu();   // Read acc & gyro
    // run_example_qwiic(); // Scan Qwiic (I2C) connector
    // run_example_rgb();   // RGB lights
    // run_example_servo(); // Servo motor
}
