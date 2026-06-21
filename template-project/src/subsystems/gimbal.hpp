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
    
    // === BEYBLADE MODE ===
    bool wasInBeyblade = false;
    uint32_t beybladeDecelCounter = 0;
    float chassisAngularVelocity = 0.0f;
    float lastGyroHeadingDeg = 0.0f;        // prev frame gyro heading for rate differentiation
    float beybladeTargetHeadingDeg = 0.0f; // world-frame heading the turret is trying to hold

    static constexpr float ENCODER_COUNTS_PER_DEGREE = 8192.0f / 360.0f;
    static constexpr float YAW_GEAR_RATIO = 19.0f / 48.0f;  // 19-tooth motor : 48-tooth turret

    // D gain: opposes drift rate (same term confirmed to produce counter-rotation)
    // P gain: restoring force that grows as heading error accumulates
    // Increase KD_RATE if counter-rotation is too slow.
    // Increase KP_HEADING if turret drifts back to chassis heading over time.
    static constexpr float BEYBLADE_KD_RATE    = 60.0f;
    static constexpr float BEYBLADE_KP_HEADING = 70.0f;

    // User input in beyblade mode
    // BEYBLADE_USER_ACCEL  — how fast the position target moves per frame (normal mode = 12)
    // BEYBLADE_USER_FF     — direct motor feedforward per unit stick input (normal mode = 4000)
    // Raise both to increase aiming speed in beyblade
    static constexpr float BEYBLADE_USER_ACCEL    = 40.0f;
    static constexpr float BEYBLADE_USER_FF       = 25000.0f;
    static constexpr float BEYBLADE_USER_DEADBAND = 0.05f;

    // Motor RPM damping applied in hold mode to prevent oscillation.
    // The gyro can't see turret vibration (IMU is on chassis), so this uses
    // encoder velocity instead. Raise if turret still vibrates after input.
    static constexpr float BEYBLADE_HOLD_DAMP = 5.0f;
    
    // === BEYBLADE DECELERATION ===
    // Smooth stop instead of instant snap when exiting beyblade mode
    // Duration in milliseconds for gimbal to decelerate (500ms = gentle ramp)
    static constexpr uint32_t BEYBLADE_DECEL_TIME_MS = 500;
    static constexpr uint32_t BEYBLADE_DECEL_FRAMES = (BEYBLADE_DECEL_TIME_MS / 2);  // ~500Hz update = 2ms per frame
    static constexpr float BEYBLADE_UPDATE_PERIOD = 0.002f;  // 500Hz update rate (in seconds)
    
    // Motor constants
    static constexpr int MAX_GIMBAL_RPM = 1000;  // Increased from 500 to match higher motor output capacity

};

#endif  // GIMBAL_HPP
