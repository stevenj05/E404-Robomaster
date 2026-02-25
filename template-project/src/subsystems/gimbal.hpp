#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include "../drivers_singleton.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"

class Gimbal
{
public:
    Gimbal(src::Drivers *drivers);
    ~Gimbal() = default;

    /**
     * Initialize gimbal subsystem
     */
    void initialize();

    /**
     * Update gimbal control
     */
    void update();

    /**
     * Send motor commands via CAN
     */
    void sendMotorCommands();
    
    /**
     * Get current pitch encoder position
     */
    float getPitchEncoderPosition() const;
    
    /**
     * Get relative yaw angle from startup (degrees)
     * Positive = right/counterclockwise
     * Uses synced encoder (calibrated to gyro reference frame)
     */
    float getYawAngleDegrees() const;
    
    /**
     * Get current gyro heading (world-frame compass)
     * @return Heading in degrees (0 = field forward direction)
     */
    float getGyroHeadingDegrees() const;
    
    /**
     * Get gyro heading corrected for mounting offset
     * This accounts for the structural difference between gyro and encoder orientations
     * @return Corrected gyro heading in degrees
     */
    float getCorrectedGyroHeadingDegrees() const;
    
    /**
     * Get encoder yaw angle corrected for mounting offset
     * This is the primary output for turret-centric driving
     * Uses encoder (never drifts mechanically) with offset correction applied
     * @return Corrected encoder yaw angle in degrees
     */
    float getCorrectedEncoderYawDegrees() const;
    
    /**
     * Calibrate encoder to gyro reference frame
     * Call when turret points forward to sync both sensors
     * After calibration, encoder provides accurate turret-centric angles
     */
    void calibrateEncoderToGyro();
    
    /**
     * Check for manual recalibration via remote button
     * Should be called from gimbal update to allow in-game recalibration
     * Automatically triggers if right_switch goes to DOWN position
     */
    void checkAndHandleRecalibration();
    
    /**
     * Check if gyro/encoder calibration has been performed
     * @return True if calibrated, false otherwise
     */
    bool isCalibrated() const { return encoderCalibrated; }
    
    /**
     * Set chassis angular velocity (enables counter-rotation in Beyblade mode)
     * @param angularVelocityDegPerSec Chassis angular velocity in degrees/second
     */
    void setChassisAngularVelocity(float angularVelocityDegPerSec) { 
        chassisAngularVelocity = angularVelocityDegPerSec;
    }


private:
    src::Drivers *drivers;  // Must be declared first since it's needed by motors
    
    static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
    
    // Motor IDs: Yaw = MOTOR5 (ID 0x205), Pitch = MOTOR8 (ID 0x208)
    // Both motors are INVERTED (true)
    tap::motor::DjiMotor *yawMotor;
    tap::motor::DjiMotor *pitchMotor;
    
    // PID controllers for each motor
    tap::algorithms::SmoothPid *pidYaw;
    tap::algorithms::SmoothPid *pidPitch;
    tap::algorithms::SmoothPid *pidBeyblade;  // Dedicated PID for gyro heading hold in Beyblade mode
    
    // Gimbal target positions (encoder-based)
    float gimbalYawTargetPos = 0.0f;
    float gimbalPitchTargetPos = 0.0f;
    bool firstPitchUpdate = true;
    
    // PID activation delay (3 seconds at ~500Hz update rate = 1500 ticks)
    static constexpr uint32_t PID_ACTIVATION_DELAY = 1500;
    uint32_t updateCounter = 0;
    bool pidYawActive = false;
    bool pidPitchActive = false;
    
    // === DUAL REFERENCE: Gyro + Encoder Synchronization ===
    // Gyro: World-frame compass (reference, handles drift detection)
    // Encoder: Local-frame tracker (high-speed, precise, needs calibration)
    
    // Yaw calibration for turret-centric drive
    float initialYawEncoder = 0.0f;
    float gyroHeadingAtCalibration = 0.0f;  // Gyro heading when encoder was calibrated
    bool encoderCalibrated = false;          // Has calibrateEncoderToGyro() been called?
    
    // MOUNTING OFFSET: Fixed structural difference between gyro and encoder orientation
    // Once calculated at first calibration, this never changes
    // corrected_gyro = raw_gyro - gyroEncoderMountingOffset
    float gyroEncoderMountingOffset = 0.0f;
    bool mountingOffsetCalculated = false;
    
    tap::communication::serial::Remote::SwitchState prevLeftSwitchState = 
        tap::communication::serial::Remote::SwitchState::UNKNOWN;  // Track switch state for edge detection
    
    // === BEYBLADE MODE: Gyro-Locked Counter-Rotation ===
    float targetGyroHeading = 0.0f;     // Gyro heading to maintain in Beyblade mode
    bool wasInBeyblade = false;         // Track mode transitions to handle setup and cleanup
    uint32_t beybladeStartupCounter = 0;  // Delays motor output for first few frames (prevents jerk)
    uint32_t beybladeDecelCounter = 0;  // Counts down during deceleration phase
    float chassisAngularVelocity = 0.0f;  // Updated by setChassisAngularVelocity()
    float lastGyroHeadingDeg = 0.0f;  // Tracks previous gyro reading for delta integration
    
    // === GYRO BIAS CALIBRATION ===
    // Calibrate gyro offset during beyblade startup to eliminate constant drift
    float gyroHeadingDeltaAccumulator = 0.0f;  // Sum of heading deltas during startup
    uint32_t gyroCalibrationSampleCount = 0;   // Number of samples accumulated
    float gyroBiasOffset = 0.0f;               // Final calculated bias to subtract from headings
    
    static constexpr float ENCODER_COUNTS_PER_DEGREE = 8192.0f / 360.0f;  // GM6020
    static constexpr float YAW_GEAR_RATIO = 2.5f;  // 2.5:1 gear ratio (motor spins 2.5x per turret rotation)
    
    // === BEYBLADE FEED-FORWARD CONSTANTS ===
    // FF_GAIN: scales the feed-forward gyro compensation (experiment with 0.5-2.0)
    // Upside-down IMU sign: +1 if normal, -1 if board mounted upside down
    static constexpr float BEYBLADE_FF_GAIN = 0.65f;  // Reduced significantly - PID doing heavy lifting
    // IMU yaw sign already matches the world frame we want for counter-rotation
    static constexpr float IMU_UPSIDE_DOWN_CORRECTION = 1.0f;
    static constexpr float GYRO_ORIENTATION_MAGIC = 1.0f;  // Leave at 1.0 unless further mounting inversion is observed
    
    // Beyblade safety deadband: ignore chassis spin if |velocity| < this threshold to prevent jitter
    static constexpr float BEYBLADE_CHASSIS_DEADBAND = 5.0f;  // degrees/sec
    
    // === BEYBLADE SPEED SCALING ===
    // Scales gimbal counter-rotation speed to match chassis speed
    // If gimbal spins too fast: DECREASE this value (e.g., 0.03, 0.02, 0.01)
    // If gimbal spins too slow: INCREASE this value (e.g., 0.05, 0.1)
    // Tune until gimbal and chassis rotate at same visual speed
    static constexpr float BEYBLADE_SPEED_SCALE = 1.0f;  // Gyro-driven scale (leave at 1.0 unless fine tuning)
    
    // === BEYBLADE DECELERATION ===
    // Smooth stop instead of instant snap when exiting beyblade mode
    // Duration in milliseconds for gimbal to decelerate (500ms = gentle ramp)
    static constexpr uint32_t BEYBLADE_DECEL_TIME_MS = 500;
    static constexpr uint32_t BEYBLADE_DECEL_FRAMES = (BEYBLADE_DECEL_TIME_MS / 2);  // ~500Hz update = 2ms per frame
    static constexpr float BEYBLADE_UPDATE_PERIOD = 0.002f;  // 500Hz update rate (in seconds)
    
    // Motor constants
    static constexpr int MAX_GIMBAL_RPM = 500;

};

#endif  // GIMBAL_HPP
