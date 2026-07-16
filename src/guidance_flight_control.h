#ifndef GUIDANCE_FLIGHT_CONTROL_H
#define GUIDANCE_FLIGHT_CONTROL_H

#include "globals.h"
#include "Launchsequence.h"
#include "flight_profile.h"
#include <QuickPID.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>


#define LIFTOFF_ACCEL_G      3.0f      
#define BURNOUT_ACCEL_G      0.5f      
#define APOGEE_VEL_MS       -0.5f      
#define APOGEE_BUFFER_SIZE   10


// ============================================================================
// STRUCTURAL LIMITS
// ============================================================================

static constexpr float MAX_DEFLECTION_DEG  = 30.0f;
static constexpr float SERVO_CENTER_DEG     = 90.0f;

// ============================================================================
// GAIN SCHEDULING LOOKUP TABLE
// ============================================================================

struct GainStep {
    float velocity;
    float kp;
    float ki;
    float kd;
};

static constexpr GainStep gainSchedule[5] = {
    { 15.0f, 4.80f, 0.60f, 0.24f },
    { 30.0f, 3.00f, 0.40f, 0.15f },
    { 50.0f, 2.00f, 0.30f, 0.10f },
    { 80.0f, 1.30f, 0.20f, 0.07f },
    {120.0f, 0.90f, 0.12f, 0.04f },
};

static constexpr int GAIN_TABLE_LEN = 5;

// ============================================================================
// ADAFRUIT PCA9685 SERVO DRIVER
// ============================================================================

extern Adafruit_PWMServoDriver pwm;

namespace UserSpace {
    struct ServoConfig {
        uint8_t channel;
        uint16_t minPulse;
        uint16_t maxPulse;
        float centerOffset;
    };

    static constexpr int NUM_SERVOS = 8;
    static const ServoConfig servos[NUM_SERVOS] = {
        {0, 150, 600, 0.0f},
        {1, 150, 600, 0.0f},
        {2, 150, 600, 0.0f},
        {3, 150, 600, 0.0f},
        {4, 150, 600, 0.0f},
        {5, 150, 600, 0.0f},
        {6, 150, 600, 0.0f},
        {7, 150, 600, 0.0f}
    };

    void writeServoAngle(uint8_t index, float targetAngle) {
        if (index >= NUM_SERVOS) return;

        float calibratedAngle = targetAngle + servos[index].centerOffset;
        calibratedAngle = constrain(calibratedAngle, 0.0f, 180.0f);

        float pulseFraction = calibratedAngle / 180.0f;
        uint16_t pulseTicks = servos[index].minPulse +
            (uint16_t)(pulseFraction * (servos[index].maxPulse - servos[index].minPulse));
        pwm.setPWM(servos[index].channel, 0, pulseTicks);
    }

    void initServos() {
        write(LOG_BOTH, LOG_INFO, "[SERVOS] Centering all 8 control surfaces to 90.0 deg");
        for (int i = 0; i < NUM_SERVOS; i++) {
            writeServoAngle(i, SERVO_CENTER_DEG);
        }
    }
}

// ============================================================================
// 8-PID INDEPENDENT SURFACE TOPOLOGY
// ============================================================================
//
// Each servo has its own QuickPID. The PID input is a weighted combination
// of the three axis errors (roll, pitch, yaw). The weights encode:
//   1. Which axis the surface primarily controls
//   2. Geometric sign conventions (opposite surfaces deflect opposite)
//   3. Cross-axis blending (canards also help roll, flaps also help pitch)
//
// Setpoint is always 0.0 (drive error to zero). PID output is the
// deflection from SERVO_CENTER_DEG — directly written to the servo.
// ============================================================================

struct SurfaceWeights {
    float roll;
    float pitch;
    float yaw;
};

//   Index  Surface           Primary Role      Sign Convention
//   0      Canard 0 (Fwd-L)  Pitch+Roll        +pitch +roll
//   1      Canard 1 (Fwd-R)  Pitch+Roll        -pitch +roll
//   2      Canard 2 (Fwd-L)  Yaw               +yaw
//   3      Canard 3 (Fwd-R)  Yaw               -yaw
//   4      Flap 4  (Aft-L)   Roll+Pitch(Neg)   +roll  -pitch
//   5      Flap 5  (Aft-R)   Roll+Pitch(Neg)   -roll  -pitch
//   6      Flap 6  (Aft-L)   Roll+Pitch(Neg)   +roll  -pitch
//   7      Flap 7  (Aft-R)   Roll+Pitch(Neg)   -roll  -pitch

static constexpr SurfaceWeights surfaceWeights[8] = {
    {  0.15f,  1.00f,  0.00f },
    {  0.15f, -1.00f,  0.00f },
    {  0.00f,  0.00f,  1.00f },
    {  0.00f,  0.00f, -1.00f },
    {  1.00f, -0.30f,  0.00f },
    { -1.00f, -0.30f,  0.00f },
    {  1.00f, -0.30f,  0.00f },
    { -1.00f, -0.30f,  0.00f },
};

static constexpr int NUM_PIDS = 8;

float pidInput[NUM_PIDS]       = {0.0f};
float pidOutput[NUM_PIDS]      = {0.0f};
float pidSetpoint[NUM_PIDS]    = {0.0f};

QuickPID pid[NUM_PIDS] = {
    QuickPID(&pidInput[0], &pidOutput[0], &pidSetpoint[0], gainSchedule[0].kp, gainSchedule[0].ki, gainSchedule[0].kd, QuickPID::Action::direct),
    QuickPID(&pidInput[1], &pidOutput[1], &pidSetpoint[1], gainSchedule[0].kp, gainSchedule[0].ki, gainSchedule[0].kd, QuickPID::Action::direct),
    QuickPID(&pidInput[2], &pidOutput[2], &pidSetpoint[2], gainSchedule[0].kp, gainSchedule[0].ki, gainSchedule[0].kd, QuickPID::Action::direct),
    QuickPID(&pidInput[3], &pidOutput[3], &pidSetpoint[3], gainSchedule[0].kp, gainSchedule[0].ki, gainSchedule[0].kd, QuickPID::Action::direct),
    QuickPID(&pidInput[4], &pidOutput[4], &pidSetpoint[4], gainSchedule[0].kp, gainSchedule[0].ki, gainSchedule[0].kd, QuickPID::Action::direct),
    QuickPID(&pidInput[5], &pidOutput[5], &pidSetpoint[5], gainSchedule[0].kp, gainSchedule[0].ki, gainSchedule[0].kd, QuickPID::Action::direct),
    QuickPID(&pidInput[6], &pidOutput[6], &pidSetpoint[6], gainSchedule[0].kp, gainSchedule[0].ki, gainSchedule[0].kd, QuickPID::Action::direct),
    QuickPID(&pidInput[7], &pidOutput[7], &pidSetpoint[7], gainSchedule[0].kp, gainSchedule[0].ki, gainSchedule[0].kd, QuickPID::Action::direct),
};

void initStabilizationPIDs() {
    for (int i = 0; i < NUM_PIDS; i++) {
        pid[i].SetMode(QuickPID::Control::automatic);
        pid[i].SetOutputLimits(-MAX_DEFLECTION_DEG, MAX_DEFLECTION_DEG);
    }
}

// ============================================================================
// GAIN SCHEDULING — linearly interpolate the 5-entry LUT
// ============================================================================

void applyAdvancedGainScheduling(float current_velocity) {
    float v = current_velocity;
    float kp, ki, kd;

    if (v <= gainSchedule[0].velocity) {
        kp = gainSchedule[0].kp;
        ki = gainSchedule[0].ki;
        kd = gainSchedule[0].kd;
    } else if (v >= gainSchedule[GAIN_TABLE_LEN - 1].velocity) {
        kp = gainSchedule[GAIN_TABLE_LEN - 1].kp;
        ki = gainSchedule[GAIN_TABLE_LEN - 1].ki;
        kd = gainSchedule[GAIN_TABLE_LEN - 1].kd;
    } else {
        kp = ki = kd = 0.0f;
        for (int i = 0; i < GAIN_TABLE_LEN - 1; i++) {
            if (v >= gainSchedule[i].velocity && v < gainSchedule[i + 1].velocity) {
                float t = (v - gainSchedule[i].velocity) /
                          (gainSchedule[i + 1].velocity - gainSchedule[i].velocity);
                kp = gainSchedule[i].kp + t * (gainSchedule[i + 1].kp - gainSchedule[i].kp);
                ki = gainSchedule[i].ki + t * (gainSchedule[i + 1].ki - gainSchedule[i].ki);
                kd = gainSchedule[i].kd + t * (gainSchedule[i + 1].kd - gainSchedule[i].kd);
                break;
            }
        }
    }

    for (int i = 0; i < NUM_PIDS; i++) {
        pid[i].SetTunings(kp, ki, kd);
    }
}

// ============================================================================
// KALMAN AIRSPEED ESTIMATOR
// ============================================================================

struct KalmanAirspeed {
    float x;
    float P;
    float Q;
    float R_baro;
    float R_imu;
    float R_gps;
    float baro_alpha;
    bool  gps_fix;

    void init() {
        x = 0.0f;
        P = 10.0f;
        Q = 0.5f;
        R_baro = 1.0f;
        R_imu  = 4.0f;
        R_gps  = 2.0f;
        baro_alpha = 1.0f;
        gps_fix = false;
    }

    void predict(float accel_along, float dt) {
        if (dt <= 0.0f || dt > 0.1f) return;
        x += accel_along * dt;
        if (x < 0.0f) x = 0.0f;
        P += Q;
    }

    void updateBaro(float v_baro) {
        float abs_v = fabsf(x);
        baro_alpha = 1.0f - (abs_v / 120.0f) * 0.9f;
        if (baro_alpha < 0.1f) baro_alpha = 0.1f;
        if (baro_alpha > 1.0f) baro_alpha = 1.0f;

        float R_eff = R_baro / baro_alpha;
        float K = P / (P + R_eff);
        float innovation = fabsf(v_baro) - x;
        x += K * innovation;
        if (x < 0.0f) x = 0.0f;
        P *= (1.0f - K);
    }

    void updateIMU(float accel_along, float dt) {
        if (dt <= 0.0f) return;
        float speed_increment = accel_along * dt;

        float abs_v = fabsf(x);
        float imu_alpha = 0.1f + (abs_v / 120.0f) * 0.9f;
        if (imu_alpha > 1.0f) imu_alpha = 1.0f;

        float R_eff = R_imu / imu_alpha;
        float K = P / (P + R_eff);
        float z = x + speed_increment;
        float innovation = z - x;
        x += K * innovation;
        if (x < 0.0f) x = 0.0f;
        P *= (1.0f - K);
    }

    void updateGPS(float v_gps, bool fix_available) {
        gps_fix = fix_available;
        if (!fix_available || v_gps < 0.0f) return;

        float K = P / (P + R_gps);
        x += K * (v_gps - x);
        if (x < 0.0f) x = 0.0f;
        P *= (1.0f - K);
    }

    float getSpeed() const { return x; }
};

KalmanAirspeed airspeedFilter;

// ============================================================================
// FLIGHT PHASE STATE MACHINE CONFIGURATION
// ============================================================================

// Note: Ensure FlightPhase enum isn't duplicated in globals.h. 
// If it is, delete this enum declaration block to avoid collisions.

// Integrated thread-safe state variable using the shared atomic object from globals.h
// Assuming globals.h names it "currentPhase", we utilize it consistently below.

void flight_core_set_phase(FlightPhase new_phase) {
    currentPhase.store(new_phase, std::memory_order_relaxed);
    if (new_phase == BOOST) {
        write(LOG_BOTH, LOG_INFO, "[PHASE] Core manually forced state into BOOST.");
    }
}

FlightPhase flight_core_get_phase(void) {
    return currentPhase.load(std::memory_order_relaxed);
}

uint32_t landing_start_ms = 0;  // Stores when landing conditions were first met
bool tracking_landing = false;  // Tracks if the timer is actively running


void process_flight_state_machine(float raw_accel_z, float filter_alt) {
    static uint16_t apogee_counter = 0;
    static float max_altitude = 0.0f;

    if (filter_alt > max_altitude) {
        max_altitude = filter_alt;
    }

    FlightPhase current = currentPhase.load(std::memory_order_relaxed);
    bool armed = systemArmed.load(std::memory_order_relaxed);
    SystemMode mode = currentSystemMode.load(std::memory_order_relaxed);

    switch (current) {
        case TRANSPORT:
            // Complete software lockout
            break;

        case PAD:

            break;
        case READY:
            if (raw_accel_z > LIFTOFF_ACCEL_G && armed && mode == MODE_ACTIVE_PAD) {
                currentPhase.store(BOOST, std::memory_order_relaxed);
                ::filter_alt = 0.0f;
                write(LOG_BOTH, LOG_INFO, "[PHASE] READY -> BOOST (Liftoff Accel Trigger)");
            }
        break;
        case BOOST:
            if (raw_accel_z < BURNOUT_ACCEL_G) {
                currentPhase.store(COAST, std::memory_order_relaxed);
                write(LOG_BOTH, LOG_INFO, "[PHASE] BOOST -> COAST (Motor Burnout Trigger)");
            }
            break;

        case COAST:
            if (V_z <= APOGEE_VEL_MS && (max_altitude - filter_alt > 1.5f)) {
                apogee_counter++;
            } else {
                if (apogee_counter > 0) apogee_counter--; 
            }

            if (apogee_counter >= APOGEE_BUFFER_SIZE) {
                currentPhase.store(DESCENT, std::memory_order_relaxed);
                write(LOG_BOTH, LOG_INFO, "[PHASE] COAST -> DESCENT (Apogee Confirmed)");
                fire_apogee_pyro(); 
            }
            break;

        case DESCENT:
        uint32_t landing_start_ms = 0;  // Stores when landing conditions were first met
        bool tracking_landing = false;  // Tracks if the timer is actively running

            if (V_z < APOGEE_VEL_MS) {
                apogee_counter++;
                if (apogee_counter >= APOGEE_BUFFER_SIZE) {
                    currentPhase.store(DESCENT, std::memory_order_relaxed);
                    write(LOG_BOTH, LOG_INFO, "[PHASE] COAST -> DESCENT (Apogee Detected)");
                }
            } else {
                apogee_counter = 0;
            }

            if (filter_alt < 5.0f && !tracking_landing) {
                landing_start_ms = millis();
                tracking_landing = true;
            }

            if (tracking_landing && filter_alt < 5.0f) {
                if (millis() - landing_start_ms >= 3000) {
                    currentPhase.store(TRANSPORT, std::memory_order_relaxed);
                    write(LOG_BOTH, LOG_INFO, "[PHASE] DESCENT -> TRANSPORT (Landing Confirmed)");
                    tracking_landing = false;
                }
            } else {
                tracking_landing = false;
            }
        break;
    }
}

// ============================================================================
// SENSOR FUSION ENGINE
// ============================================================================

void updateRocketFusion(float lin_ax, float lin_ay, float lin_az,
                        float qx, float qy, float qz, float qw,
                        float current_altitude, float &last_altitude,
                        float dt)
{
    if (dt <= 0.0f || dt > 0.1f) return;

    // Isolate absolute vertical earth-frame acceleration using spatial orientation
    float a_earth_z = (2.0f*(qx*qz - qw*qy))*lin_ax +
                      (2.0f*(qy*qz + qw*qx))*lin_ay +
                      (1.0f - 2.0f*(qx*qx + qy*qy))*lin_az;

    float V_baro = (current_altitude - last_altitude) / dt;
    last_altitude = current_altitude;

    // Dynamic Alpha Scaling based on state regime
    float alpha = 0.95f;
    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);

    switch(phase) {
        case TRANSPORT:
        case PAD:
            alpha = 0.80f; // Heavily weight barometer while grounded
            break;
        case BOOST:
            alpha = 0.995f; // Trust internal accelerometer safely over engine pressure spikes
            break;
        case COAST:
            alpha = 0.960f + (fabsf(V_z) * 0.0001f);
            if (alpha > 0.995f) alpha = 0.995f;
            break;
        case DESCENT:
            alpha = 0.900f;
            break;
    }

    // Complementary Filter Update Loops
    float V_predicted = V_z + (a_earth_z * dt);
    V_z = (alpha * V_predicted) + ((1.0f - alpha) * V_baro);
    filter_alt += V_z * dt;

    // Evaluate state thresholds immediately after data refresh
    process_flight_state_machine(a_earth_z, filter_alt);

    // Compute Airspeed Predictions
    float accel_along = a_earth_z;
    airspeedFilter.predict(accel_along, dt);
    airspeedFilter.updateIMU(accel_along, dt);
    airspeedFilter.updateBaro(V_baro);
}

// ============================================================================
// PRE-FLIGHT LOOP PLACEHOLDER
// ============================================================================

void userPreFlightLoop(float dt) {
    (void) dt;
}

// ============================================================================
// 4-PID STABILIZATION + UNROLLED ACTUATION MIXING
// ============================================================================

static constexpr float CANARD_PITCH_BLEND_FROM_FLAP = 0.30f;
static constexpr float CANARD_ROLL_BLEND_FROM_FLAP   = 0.15f;

void userFlightStabilizationLoop(float roll, float pitch, float yaw,
                                  float velocity_z, float dt) {

    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
    if (phase == PAD || phase == TRANSPORT || phase == READY) {
        for (int i = 0; i < NUM_PIDS; i++) {
            pid[i].SetMode(QuickPID::Control::manual);
        }

        for (int i = 0; i < 8; i++) {
            UserSpace::writeServoAngle(i, SERVO_CENTER_DEG);
        }

        for (int i = 0; i < 8; i++) latestServoAngles[i] = SERVO_CENTER_DEG;
        for (int i = 0; i < 8; i++) latestPIDOutputs[i] = 0.0f;
        latestActiveGains[0] = pid[0].GetKp();
        latestActiveGains[1] = pid[0].GetKi();
        latestActiveGains[2] = pid[0].GetKd();
        latestAirspeed   = 0.0f;
        latestKalmanP    = airspeedFilter.P;
        latestBaroAlpha  = airspeedFilter.baro_alpha;
        return;
    }

    for (int i = 0; i < NUM_PIDS; i++) {
        pid[i].SetMode(QuickPID::Control::automatic);
    }

    float estimated_speed = airspeedFilter.getSpeed();
    applyAdvancedGainScheduling(estimated_speed);

    uint32_t dt_us = (uint32_t)(dt * 1000000.0f);
    for (int i = 0; i < NUM_PIDS; i++) {
        pid[i].SetSampleTimeUs(dt_us);
    }

    // Attitude setpoints: from active flight profile, else level (0,0,0)
    float spRoll  = 0.0f;
    float spPitch = 0.0f;
    float spYaw   = 0.0f;
    if (profileEngine.isActive()) {
        spRoll  = profileEngine.getSetpointRoll();
        spPitch = profileEngine.getSetpointPitch();
        spYaw   = profileEngine.getSetpointYaw();
    }

    for (int i = 0; i < NUM_PIDS; i++) {
        // PID input = weighted attitude ERROR (drives error to zero)
        float rollErr  = roll  - spRoll;
        float pitchErr = pitch - spPitch;
        float yawErr   = yaw   - spYaw;
        pidInput[i] = surfaceWeights[i].roll  * rollErr +
                      surfaceWeights[i].pitch * pitchErr +
                      surfaceWeights[i].yaw   * yawErr;
        pidSetpoint[i] = 0.0f;
    }

    for (int i = 0; i < NUM_PIDS; i++) {
        pid[i].Compute();
    }

    for (int i = 0; i < 8; i++) {
        float angle = SERVO_CENTER_DEG +
            constrain(pidOutput[i], -MAX_DEFLECTION_DEG, MAX_DEFLECTION_DEG);
        UserSpace::writeServoAngle(i, angle);
        latestServoAngles[i] = angle;
        latestPIDOutputs[i] = pidOutput[i];
    }

    latestActiveGains[0] = pid[0].GetKp();
    latestActiveGains[1] = pid[0].GetKi();
    latestActiveGains[2] = pid[0].GetKd();

    latestAirspeed = estimated_speed;
    latestKalmanP  = airspeedFilter.P;
    latestBaroAlpha = airspeedFilter.baro_alpha;
}

#endif // GUIDANCE_FLIGHT_CONTROL_H