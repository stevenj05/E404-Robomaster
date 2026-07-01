#include "gimbal.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
float wrapDegrees(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}
}  // namespace

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
    
    // Capture the encoder position RIGHT NOW.
    // User presses calibrate with turret pointing forward — that position IS forward = 0°.
    float rawEncoderReading = yawMotor->getEncoderUnwrapped();
    initialYawEncoder = rawEncoderReading;
    
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
        
        printf("[FIRST CALIBRATION] encoder=%.0f | yaw=%.2f deg | gyro=%.2f deg | offset=%.2f deg\n",
               (double)rawEncoderReading, (double)encoderAtCal, (double)gyroAtCal, (double)gyroEncoderMountingOffset);
    }
    else
    {
        printf("[RECALIBRATION]\n");
        printf("  Raw Encoder: %.0f counts -> Corrected: %.0f counts\n", (double)rawEncoderReading, (double)initialYawEncoder);
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
    // Apply 90° correction for turret-centric orientation
    return getYawAngleDegrees() + 90.0f;
}

void Gimbal::update()
{
    // === HYBRID GIMBAL INPUT: Mouse + Joystick ===
    
    // Get mouse input for gimbal control
    constexpr float MOUSE_SENS_YAW = 13.0f;      // Higher for faster left/right response
    constexpr float MOUSE_SENS_PITCH = 6.0f;   // Lower for slower up/down response
    constexpr float DT = 1.0f / 500.0f;  // 500Hz update rate
    
    float mouseX = static_cast<float>(drivers->remote.getMouseX()) * MOUSE_SENS_YAW * DT;
    float mouseY = static_cast<float>(drivers->remote.getMouseY()) * MOUSE_SENS_PITCH * DT;
    
    mouseX = std::clamp(mouseX, -1.0f, 1.0f);
    mouseY = std::clamp(mouseY, -1.0f, 1.0f);
    
    // Get joystick input from remote
    float joystickYaw = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    float joystickPitch = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    
    // Combine mouse and joystick inputs: add them together
    float yawInput = mouseX + joystickYaw;      // Removed negation to fix inversion
    float pitchInput = mouseY + joystickPitch; // Mouse up/down inverted per request
    
    // Clamp combined input
    yawInput = std::clamp(yawInput, -1.0f, 1.0f);
    pitchInput = std::clamp(pitchInput, -1.0f, 1.0f);
    
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
    
    // Beyblade can be activated with Shift key OR right switch on remote
    bool keyboardBeyblade = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT);
    bool remoteBeyblade = (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == 
                          tap::communication::serial::Remote::SwitchState::UP);
    bool beybladeMode = keyboardBeyblade || remoteBeyblade;
    
    if (beybladeMode)
    {
        // On first frame of Beyblade activation, initialize
        if (!wasInBeyblade)
        {
            wasInBeyblade = true;
            pidYawActive = true;
            gimbalYawTargetPos = yawMotor->getEncoderUnwrapped();
            lastGyroHeadingDeg = getCorrectedGyroHeadingDegrees();
            beybladeTargetHeadingDeg = lastGyroHeadingDeg;  // hold this heading from now on
            printf("[BEYBLADE] Activated\n");
        }

        // Gyro-delta driven counter-rotation: keep turret locked to beybladeTargetHeadingDeg
        float currentHeading = getCorrectedGyroHeadingDegrees();
        constexpr float DT = 1.0f / 500.0f;

        // Frame-to-frame heading change → instantaneous spin rate
        float headingDelta = wrapDegrees(currentHeading - lastGyroHeadingDeg);
        lastGyroHeadingDeg = currentHeading;
        float yawRateDegPerSec = headingDelta / DT;

        // D term always active — opposes chassis spin rate for counter-rotation
        // HARDCODED OVER-ROTATION: counter-spin a bit MORE than the gyro reports to
        // compensate for the 19:48 turret gear reduction (turret moves slower than
        // the motor, so the motor must work harder to hold heading).
        //   1.0 = exactly match gyro | >1.0 = spin more | full gear comp ~= 2.53
        // Raise if the turret still drifts WITH the chassis; lower if it over-corrects.
        constexpr float COUNTER_ROTATION_BOOST = 2.3f;
        float dOutput = -yawRateDegPerSec * BEYBLADE_KD_RATE * COUNTER_ROTATION_BOOST;

        float finalOutput;
        bool userInputting = std::abs(yawInput) > BEYBLADE_USER_DEADBAND;

        if (userInputting)
        {
            // VELOCITY MODE: aim via direct stick feedforward layered on the counter-rotation.
            // We DON'T accumulate a position target here: with the 19:48 reduction the encoder
            // counter-spins ~2.5x faster than the turret, so a static target makes the position
            // PID saturate fighting the counter-rotation and steals all aim authority.
            // Keeping the loop neutral (target = encoder, error = 0) frees the output for the
            // stick, so the player can actually slew the turret while beyblading.
            gimbalYawTargetPos = yawMotor->getEncoderUnwrapped();
            pidYaw->runControllerDerivateError(0.0f, BEYBLADE_UPDATE_PERIOD);
            int32_t stickFF = static_cast<int32_t>(-yawInput * BEYBLADE_USER_FF);

            // Record heading so hold mode snaps to here when stick released
            beybladeTargetHeadingDeg = currentHeading;

            // SYMMETRIC AIM: the chassis only beyblades one direction, so dOutput is a
            // large one-sided offset that pins the output to one rail — aiming WITH the
            // spin is then clamped (no authority) while aiming AGAINST it works. Fade the
            // counter-rotation out as the stick is pushed so aim gets the full ±25000 in
            // BOTH directions. Light aim keeps most counter-rotation; hard aim overrides it.
            float aimScale = std::clamp(1.0f - std::abs(yawInput), 0.0f, 1.0f);
            finalOutput = std::clamp(static_cast<float>(stickFF) + dOutput * aimScale, -25000.0f, 25000.0f);
        }
        else
        {
            // HOLD MODE: PD locks to the heading where the stick was released.
            // Motor RPM damping kills oscillation that the gyro can't see
            // (gyro is on chassis, not turret — turret vibration is invisible to it).
            gimbalYawTargetPos = yawMotor->getEncoderUnwrapped();
            pidYaw->runControllerDerivateError(0.0f, BEYBLADE_UPDATE_PERIOD);

            float headingError = wrapDegrees(currentHeading - beybladeTargetHeadingDeg);
            float pOutput  = -headingError * BEYBLADE_KP_HEADING;
            float motorDamp = -(float)yawMotor->getShaftRPM() * BEYBLADE_HOLD_DAMP;
            finalOutput = std::clamp(dOutput + pOutput + motorDamp, -16384.0f, 16384.0f);
        }

        yawMotor->setDesiredOutput(static_cast<int32_t>(finalOutput));

        static uint32_t beybladeDebugCounter = 0;
        if (beybladeDebugCounter++ % 50 == 0)
        {
            printf("[BEY] rate=%.1f d/s | tgt=%.1f | cur=%.1f | input=%s | out=%.0f\n",
                   (double)yawRateDegPerSec, (double)beybladeTargetHeadingDeg,
                   (double)currentHeading, userInputting ? "VEL" : "HOLD", (double)finalOutput);
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
                printf("[BEYBLADE] Starting smooth deceleration (%lu ms), freezing at position\n", (unsigned long)BEYBLADE_DECEL_TIME_MS);
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
            beybladeTargetHeadingDeg = 0.0f;
            lastGyroHeadingDeg = getGyroHeadingDegrees();
            
            // Reset PID state to clear stale beyblade data and restore snappy normal response
            // Set target to current position so error is zero and PID starts fresh
            gimbalYawTargetPos = yawMotor->getEncoderUnwrapped();
            
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
        pitchMotor->setDesiredOutput(static_cast<int32_t>(pitchInput * 13300));
    }
    else
    {
        // After 3 seconds: activate PID for position holding
        // Update target position based on joystick input
        gimbalPitchTargetPos += pitchInput * 12.0f;  // Increased for snappier response
        
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
    
    // Motor shaft degrees × gear ratio (1:1 direct drive) = turret degrees
    float encoderDiff = yawMotor->getEncoderUnwrapped() - initialYawEncoder;
    return (encoderDiff / ENCODER_COUNTS_PER_DEGREE) * YAW_GEAR_RATIO;
}


