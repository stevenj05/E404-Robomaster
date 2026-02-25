#include "gimbal.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

Gimbal::Gimbal(src::Drivers *drivers)
    : drivers(drivers),
      // Initialize motors on CAN1 - MOTOR5=Yaw, MOTOR8=Pitch (both INVERTED)
      yawMotor(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR5, CAN_BUS, true, "Yaw Motor")),
      pitchMotor(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR8, CAN_BUS, true, "Pitch Motor")),
      // Initialize PID controllers matching the working code
      // YAW PID: Normal Mode Control
      // Diagnosis: P=28 oscillates. P=25 is stable.
      // Fix: Revert to P=25 (Stable/Honey). We will improve responsiveness
      // by adding "Stick FeedForward" in the update() function instead of increasing P.
      pidYaw(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{25.0f, 0.0f, 0.0f, 0, 16384, 1, 0, 1, 0})),
      pidPitch(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{70, 0.15, 0.2, 1200, 15000, 1, 0, 1, 0})),
      // PID for Beyblade gyro heading hold - I-GAIN SET TO 0.0 to kill integral windup
      // P=3.0: reduced significantly from 8.0, I=0.0: NO integral, D=1.5: damping
      pidBeyblade(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{3.0f, 0.0f, 1.5f, 0, 16384, 1, 0, 1, 0}))
{
}

void Gimbal::initialize()
{
    // Resets the encoder value to determine the fixed, absolute orientation
    // of the chassis on startup.
    yawMotor->resetEncoderValue();

    // Initialize the yaw and pitch motors.
    yawMotor->initialize();
    pitchMotor->initialize();
    
    // NOTE: Do NOT calibrate here—calibration happens when user presses LEFT_SWITCH
    // This avoids the race condition where gimbal PID hasn't activated yet
}

void Gimbal::calibrateEncoderToGyro()
{
    // === ABSOLUTE CALIBRATION WITH MECHANICAL OFFSET CORRECTION ===
    // User points turret forward and presses LEFT_SWITCH DOWN
    // This moment becomes the new absolute zero: "Forward = 0°"
    //
    // HOWEVER: The encoder is physically mounted 90° rotated from turret front
    // So when turret points forward, encoder reads X, but motor thinks that's 90° offset
    // We correct for this by shifting the baseline by the mechanical offset
    
    // Capture the encoder position RIGHT NOW
    float rawEncoderReading = yawMotor->getEncoderUnwrapped();
    
    // Convert the known 90° mechanical offset into encoder counts
    // The turret's 90° rotation = motor_counts for (90° * gear_ratio)
    static constexpr float MECHANICAL_OFFSET_DEG = 90.0f;  // Encoder is 90° rotated from turret front
    float mechanicalOffsetCounts = MECHANICAL_OFFSET_DEG * ENCODER_COUNTS_PER_DEGREE * YAW_GEAR_RATIO;
    
    // Apply correction: subtract the offset so that "forward turret" = "0° encoder math"
    initialYawEncoder = rawEncoderReading - mechanicalOffsetCounts;
    
    // Record gyro heading for reference
    gyroHeadingAtCalibration = getGyroHeadingDegrees();
    
    // Mark as calibrated
    encoderCalibrated = true;
    gimbalYawTargetPos = rawEncoderReading;  // Use raw reading for motor control
    
    // Measure mounting offset on first calibration
    if (!mountingOffsetCalculated)
    {
        float encoderAtCal = getYawAngleDegrees();          // Should be ~0 now  
        float gyroAtCal = getGyroHeadingDegrees();
        gyroEncoderMountingOffset = gyroAtCal - encoderAtCal;
        mountingOffsetCalculated = true;
        
        printf("[FIRST CALIBRATION WITH OFFSET CORRECTION]\n");
        printf("  Raw Encoder Reading: %.0f counts\n", rawEncoderReading);
        printf("  90° Mechanical Offset: %.0f counts\n", mechanicalOffsetCounts);
        printf("  Corrected Baseline: %.0f counts\n", initialYawEncoder);
        printf("  Turret NOW at Forward = 0° (mechanically corrected)\n");
        printf("  Encoder Yaw Angle: %.2f deg\n", encoderAtCal);
        printf("  Gyro Heading: %.2f deg\n", gyroAtCal);
        printf("================================================\n");
    }
    else
    {
        printf("[RECALIBRATION]\n");
        printf("  Raw Encoder: %.0f counts → Corrected: %.0f counts\n", rawEncoderReading, initialYawEncoder);
        printf("  Turret realigned to Forward = 0°\n");
        printf("  Ready for turret-centric drive\n");
        printf("====================================\n");
    }
}


void Gimbal::checkAndHandleRecalibration()
{
    // Allow player to recalibrate at any time by pressing LEFT_SWITCH down
    // This syncs encoder to gyro while robot is running
    // Useful if turret belt skips or after battery swap
    
    tap::communication::serial::Remote::SwitchState currentLeftSwitch = 
        drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
    
    // Detect transition from not-DOWN to DOWN
    if (currentLeftSwitch == tap::communication::serial::Remote::SwitchState::DOWN &&
        prevLeftSwitchState != tap::communication::serial::Remote::SwitchState::DOWN)
    {
        // Button just pressed - recalibrate!
        calibrateEncoderToGyro();
    }
    
    // Remember this state for next frame
    prevLeftSwitchState = currentLeftSwitch;
}

float Gimbal::getGyroHeadingDegrees() const
{
    // Type-C board provides BMI088 IMU via drivers->bmi088
    // Z-axis rotation (yaw): positive = counterclockwise when viewed from above
    // The board's gyro gives us the world-frame heading (0 = field forward direction)
    
    // Try to get gyro Z-angle (yaw) in degrees
    // BMI088 provides integrated yaw angle via Mahony algorithm
    if (drivers != nullptr)
    {
        // Using Type-C's BMI088 IMU - provides absolute yaw via Mahony AHRS
        // getYaw() returns integrated heading in degrees
        return drivers->bmi088.getYaw();
    }
    
    // Fallback: if gyro unavailable, return 0 (will still work, just no world-frame correction)
    return 0.0f;
}

float Gimbal::getCorrectedGyroHeadingDegrees() const
{
    // Return gyro heading with mounting offset applied
    // The mounting offset is discovered once at calibration and never changes
    return getGyroHeadingDegrees() + gyroEncoderMountingOffset;
}

float Gimbal::getCorrectedEncoderYawDegrees() const
{
    // With absolute calibration, the encoder angle IS the corrected angle
    // We set the baseline such that "Forward" = 0° at calibration time
    // No additional offset needed
    return getYawAngleDegrees();
}

void Gimbal::update()
{
    // Get joystick input from DR16 remote
    float yawInput = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    float pitchInput = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    
    // Check for manual recalibration request (LEFT_SWITCH down = recalibrate)
    checkAndHandleRecalibration();
    
    // Increment counter for PID activation timing
    updateCounter++;
    
    // Check if PID should be activated (after 3 second delay)
    if (!pidYawActive && updateCounter >= PID_ACTIVATION_DELAY)
    {
        pidYawActive = true;
        gimbalYawTargetPos = yawMotor->getEncoderUnwrapped();
    }
    if (!pidPitchActive && updateCounter >= PID_ACTIVATION_DELAY)
    {
        pidPitchActive = true;
        gimbalPitchTargetPos = pitchMotor->getEncoderUnwrapped();
    }
    
    // === BEYBLADE MODE: Position-Based with PID Control ===
    // Smart counter-rotation like the reference team:
    // 1. Integrate chassis angular velocity → target position
    // 2. Let PID chase the moving target → smooth, adaptive motor control
    // No hardcoded scaling factors - PID naturally adapts to speed changes
    
    bool beybladeMode = (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == 
                        tap::communication::serial::Remote::SwitchState::UP);
    
    if (beybladeMode)
    {
        // On first frame of Beyblade activation, initialize
        if (!wasInBeyblade)
        {
            wasInBeyblade = true;
            // Force PID to be active immediately in beyblade mode
            pidYawActive = true;
            gimbalYawTargetPos = yawMotor->getEncoderUnwrapped();  // Start from current position
            lastGyroHeadingDeg = getCorrectedGyroHeadingDegrees();
            // EXTENDED startup delay: 250 frames (~500ms) to let gyro stabilize before applying feedforward
            // This prevents noisy initial gyro readings from causing 2-3 seconds of unwanted motor spin
            beybladeStartupCounter = 50;
            // Reset gyro bias calibration accumulators
            gyroHeadingDeltaAccumulator = 0.0f;
            gyroCalibrationSampleCount = 0;
            gyroBiasOffset = 0.0f;
            printf("[BEYBLADE] Activated - Position-integrated counter-rotation mode (startup delay: 250 frames)\n");
        }
        
        // Gyro-delta driven counter-rotation: keep turret world-stationary
        float currentHeading = getCorrectedGyroHeadingDegrees();
        
        // === GYRO BIAS CALIBRATION PHASE ===
        // During startup, measure and accumulate gyro drift to calculate bias offset
        if (beybladeStartupCounter > 0)
        {
            float headingDelta = wrapDegrees(currentHeading - lastGyroHeadingDeg);
            gyroHeadingDeltaAccumulator += headingDelta;
            gyroCalibrationSampleCount++;
            
            // Calculate average bias when startup ends (beybladeStartupCounter == 1)
            if (beybladeStartupCounter == 1)
            {
                gyroBiasOffset = gyroHeadingDeltaAccumulator / static_cast<float>(gyroCalibrationSampleCount);
                printf("[BEYBLADE] Gyro bias calibrated: %.4f deg/frame (from %lu samples)\n",
                       (double)gyroBiasOffset, gyroCalibrationSampleCount);
            }
        }
        
        // === POSITION UPDATE WITH BIAS-CORRECTED GYRO ===
        // After startup, use bias-corrected heading delta to eliminate constant drift
        if (beybladeStartupCounter == 0)
        {
            float headingDelta = wrapDegrees(currentHeading - lastGyroHeadingDeg);
            
            // Subtract gyro bias offset to eliminate constant drift
            headingDelta -= gyroBiasOffset;

            // --- Logic Refinement Step 1: Deadband on Gyro Delta ---
            // Prevents drift/noise from causing the gimbal to spin when stationary.
            // Threshold: 0.01 degrees per loop (~5 deg/sec).
            // Reduced further to maximize responsiveness to gyro changes.
            if (std::abs(headingDelta) < 0.01f) 
            {
                headingDelta = 0.0f;
            }

            // Use REAL Sensor Data with restored Deadband:
            // Reduced multiplier to 1.2f to ensure joystick input can override gyro corrections
            float positionIncrement = -headingDelta * 1.2f * ENCODER_COUNTS_PER_DEGREE * YAW_GEAR_RATIO;
            
            gimbalYawTargetPos += positionIncrement;

            // Debug: observe signs to validate counter-rotation
            static uint32_t beybladeGyroDebugCounter = 0;
            if (beybladeGyroDebugCounter++ % 50 == 0)
            {
                printf("[BEY-GYRO] heading=%.2f deg | dHeading=%.2f (bias-corrected) | incr=%.0f cnt | tgt=%.0f | enc=%.0f\n",
                       (double)currentHeading,
                       (double)headingDelta,
                       (double)positionIncrement,
                       (double)gimbalYawTargetPos,
                       (double)yawMotor->getEncoderUnwrapped());
            }
        }
        
        // Preserve driver yaw authority during Beyblade - active ALL the time during beyblade mode
        gimbalYawTargetPos += -yawInput * 200.0f;
        
        // CRITICAL: Always update gyro baseline, even during startup
        // This prevents accumulated heading delta from causing startup spin
        lastGyroHeadingDeg = currentHeading;
        
        // Use PID to chase the moving target position
        float yawError = gimbalYawTargetPos - yawMotor->getEncoderUnwrapped();
        
        // --- HYBRID STABILITY/SPEED CONTROL ---
        // NEW TUNING FOR TIGHT BELT - "ANTI-SEIZURE" MODE
        // Diagnosis: System "gradually overcorrects" = Growing Oscillation.
        // Causes: 
        // 1. P is slightly too high for the stiffness.
        // 2. The "Speed Boost" is kicking in during the overshoot, adding fuel to the fire.
        
        // 1. STABILITY: P gain increased to 25.0 for aggressive counter-rotation response.
        float kp = 12.0f;
        
        // 2. DAMPING: D gain set to 130.0 for aggressive wobble suppression.
        //    Significantly increased to handle loose belt - dampens left/right oscillations.
        //    Higher D resists rapid direction changes and stabilizes the turret.
        float kd = 180.0f;
        
        static float lastYawError = 0;
        
        // Reset error tracking when startup delay ends to prevent D-term kick
        if (beybladeStartupCounter == 1)
        {
            lastYawError = yawError;
        }
        
        float errorDelta = yawError - lastYawError; // Raw delta, no lag.
        lastYawError = yawError;
        
        float baseOutput = (kp * yawError) + (kd * errorDelta);

        // Pure PID control - let PID handle all corrections, no feed-forward
        // Feed-forward was causing startup jerk issues
        
        // Total Output - PID only
        float finalOutput = baseOutput;
        
        // Clamp output
        if (finalOutput > 16384) finalOutput = 16384; 
        if (finalOutput < -16384) finalOutput = -16384;

        int32_t pidOutput = static_cast<int32_t>(finalOutput);
        
        // Suppress motor output during startup delay to prevent jerk
        if (beybladeStartupCounter > 0)
        {
            yawMotor->setDesiredOutput(0);
            beybladeStartupCounter--;
        }
        else
        {
            yawMotor->setDesiredOutput(pidOutput);
        }
        
        // DEBUG: Print Beyblade status
        static uint32_t beybladeDebugCounter = 0;
        if (beybladeDebugCounter++ % 50 == 0)
        {
            printf("[BEY] ChassisVel: %.1f deg/s | TargetPos: %.0f | EncoderPos: %.0f | Error: %.0f\n",
                   (double)chassisAngularVelocity, (double)gimbalYawTargetPos, (double)yawMotor->getEncoderUnwrapped(), (double)yawError);
        }
    }
    else
    {
        // DECELERATION PHASE: Smooth stop instead of instant snap
        if (wasInBeyblade)
        {
            // Start deceleration countdown on transition
            if (beybladeDecelCounter == 0)
            {
                // CRITICAL: Capture current encoder position so we don't unwind
                gimbalYawTargetPos = yawMotor->getEncoderUnwrapped();
                beybladeDecelCounter = BEYBLADE_DECEL_FRAMES;
                printf("[BEYBLADE] Starting smooth deceleration (%.0f ms), freezing at position\n", (float)BEYBLADE_DECEL_TIME_MS);
            }
        }
        
        // Keep integrating position during deceleration with gradually reducing scale
        if (beybladeDecelCounter > 0)
        {
            // Smooth co-ast-down: gradually reduce motor output to zero
            // This avoids sudden jerk and unwanted position changes
            float decelPercent = (float)beybladeDecelCounter / (float)BEYBLADE_DECEL_FRAMES;

            // Allow driver yaw input during decel so control isn't lost
            gimbalYawTargetPos += -yawInput * 12.0f;
            
            // Calculate current error but apply decreasing output
            float yawError = gimbalYawTargetPos - yawMotor->getEncoderUnwrapped();
            pidYaw->runControllerDerivateError(yawError, BEYBLADE_UPDATE_PERIOD);
            
            // Get PID output but scale it down during deceleration
            int32_t pidOutput = static_cast<int32_t>(pidYaw->getOutput());
            int32_t decelOutput = static_cast<int32_t>(pidOutput * decelPercent);
            
            yawMotor->setDesiredOutput(decelOutput);
            
            // Count down decelerator
            beybladeDecelCounter--;
        }
        else
        {
            // Deceleration complete - normal mode resumes
            wasInBeyblade = false;
            beybladeStartupCounter = 0;  // Reset startup delay counter
            // Reset gyro baseline so next Beyblade entry starts clean
            lastGyroHeadingDeg = getCorrectedGyroHeadingDegrees();
            
            // Standard Joystick/Relative Control Logic
            // Wait for 3-second PID activation delay in normal mode
            if (!pidYawActive)
            {
                // During startup (first 3 seconds): direct output control, no jerk
                yawMotor->setDesiredOutput(static_cast<int32_t>(-yawInput * 13300));
            }
            else
            {
                // After 3 seconds: activate PID for position holding
                // Update target position based on joystick input
                gimbalYawTargetPos += -yawInput * 12.0f;  // Faster accumulation for quicker yaw response
                
                // Update PID controller
                float yawError = gimbalYawTargetPos - yawMotor->getEncoderUnwrapped();
                pidYaw->runControllerDerivateError(yawError, 0.002f);
                
                // STICK FEED-FORWARD:
                // Instead of relying on Error to build up (laggy/honey feel),
                // we inject current directly based on stick input.
                // This makes it feel "instant" without needing high P-gain.
                int32_t stickFF = static_cast<int32_t>(-yawInput * 4000.0f);
                
                // Apply PID + Stick FF
                yawMotor->setDesiredOutput(static_cast<int32_t>(pidYaw->getOutput()) + stickFF);
            }
        }
    }
    
    // === PITCH CONTROL (MOTOR8) ===
    // Pitch logic remains untouched (keep your tuned 5.5lb settings!)
    if (!pidPitchActive)
    {
        // During startup (first 3 seconds): direct output control, no jerk
        pitchMotor->setDesiredOutput(static_cast<int32_t>(-pitchInput * 13300));
    }
    else
    {
        // After 3 seconds: activate PID for position holding
        // Update target position based on joystick input
        gimbalPitchTargetPos += -pitchInput * 3.0f;  // Inverted for intuitive up = up
        
        // Update PID controller with error derivative computation
        float pitchError = gimbalPitchTargetPos - pitchMotor->getEncoderUnwrapped();
        pidPitch->runControllerDerivateError(pitchError, 0.002f);  // 0.002s = 500Hz update rate
        
        // Apply PID output
        pitchMotor->setDesiredOutput(static_cast<int32_t>(pidPitch->getOutput()));
    }
}

void Gimbal::sendMotorCommands()
{
    drivers->djiMotorTxHandler.encodeAndSendCanData();
}

float Gimbal::getPitchEncoderPosition() const
{
    return pitchMotor->getEncoderUnwrapped();
}

float Gimbal::getYawAngleDegrees() const
{
    // === PURE ENCODER ANGLE (Relative to Calibration Baseline) ===
    // This returns 0° exactly when called right after calibration
    // All turret-centric math is based on this angle
    // The encoder is the primary source because it never drifts mechanically
    
    float encoderDiff = yawMotor->getEncoderUnwrapped() - initialYawEncoder;
    // Account for 2.5:1 gear ratio: motor spins 2.5x per turret rotation
    return (encoderDiff / ENCODER_COUNTS_PER_DEGREE) / YAW_GEAR_RATIO;
}


