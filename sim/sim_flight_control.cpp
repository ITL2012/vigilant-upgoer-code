#include "sim_globals.h"
#include <QuickPID.h>
#include <cmath>

int BMP580good = 0;
int BNO080good = 0;
int GPSgood = 0;
int SDgood = 0;
int wifiGood = 0;
int servoGood = 0;
float current_pitch = 0.0f;
float current_roll = 0.0f;
float current_yaw = 0.0f;
bool Enabled5V = false;

std::atomic<SystemMode> currentSystemMode(MODE_TRANSPORT);
std::atomic<FlightPhase> currentPhase(PAD);
std::atomic<bool> systemArmed(false);
std::atomic<bool> wifiActive(true);

float V_z = 0.0f;
float filter_alt = 0.0f;
float baseline_altitude = 0.0f;
float previous_altitude = 0.0f;
float qnh_pressure = 1013.25f;

float latestServoAngles[8] = {90.0f};
float latestPIDOutputs[8]  = {0.0f};
float latestActiveGains[3] = {0.0f};
float latestAirspeed    = 0.0f;
float latestKalmanP     = 0.0f;
float latestBaroAlpha   = 1.0f;
float latestBaroPressure = 0.0f;
float latestBaroTemp     = 0.0f;
float latestDt           = 0.0f;
float latestQx = 0.0f, latestQy = 0.0f, latestQz = 0.0f, latestQw = 1.0f;
float latestAx = 0.0f, latestAy = 0.0f, latestAz = 0.0f;

volatile bool servoOverrideActive = false;
volatile float servoOverrideAngles[8] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

unsigned long lastMicros = 0;
unsigned long lastLogTime = 0;
std::atomic<unsigned long> lastIMUReport_ms(0);
volatile uint32_t logDropCount = 0;

void write(int destination, int severity, const char *format, ...) {
    (void)destination;
    (void)severity;
    (void)format;
}

void userCustomSetup() {}

void fire_apogee_pyro() {}

static constexpr float LIFTOFF_ACCEL_G      = 3.0f;
static constexpr float BURNOUT_ACCEL_G       = 0.5f;
static constexpr float APOGEE_VEL_MS        = -0.5f;
static constexpr int   APOGEE_BUFFER_SIZE   = 10;

static constexpr float MAX_DEFLECTION_DEG  = 30.0f;
static constexpr float SERVO_CENTER_DEG    = 90.0f;

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

struct SurfaceWeights {
    float roll;
    float pitch;
    float yaw;
};

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

static QuickPID pid[NUM_PIDS] = {
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

static KalmanAirspeed airspeedFilter;

static void process_flight_state_machine(float raw_accel_z, float filt_alt) {
    static uint16_t apogee_counter = 0;
    static float max_altitude = 0.0f;

    if (filt_alt > max_altitude) {
        max_altitude = filt_alt;
    }

    FlightPhase current = currentPhase.load(std::memory_order_relaxed);
    bool armed = systemArmed.load(std::memory_order_relaxed);
    SystemMode mode = currentSystemMode.load(std::memory_order_relaxed);

    switch (current) {
        case TRANSPORT:
            break;
        case PAD:
            break;
        case READY:
            if (raw_accel_z > LIFTOFF_ACCEL_G && armed && mode == MODE_ACTIVE_PAD) {
                currentPhase.store(BOOST, std::memory_order_relaxed);
                filter_alt = 0.0f;
            }
            break;
        case BOOST:
            if (raw_accel_z < BURNOUT_ACCEL_G) {
                currentPhase.store(COAST, std::memory_order_relaxed);
            }
            break;
        case COAST:
            if (V_z <= APOGEE_VEL_MS && (max_altitude - filt_alt > 1.5f)) {
                apogee_counter++;
            } else {
                if (apogee_counter > 0) apogee_counter--;
            }
            if (apogee_counter >= APOGEE_BUFFER_SIZE) {
                currentPhase.store(DESCENT, std::memory_order_relaxed);
                fire_apogee_pyro();
            }
            break;
        case DESCENT:
            break;
    }
}

void updateRocketFusion(float lin_ax, float lin_ay, float lin_az,
                        float qx, float qy, float qz, float qw,
                        float current_altitude, float &last_altitude,
                        float dt)
{
    if (dt <= 0.0f || dt > 0.1f) return;

    float a_earth_z = (2.0f*(qx*qz - qw*qy))*lin_ax +
                      (2.0f*(qy*qz + qw*qx))*lin_ay +
                      (1.0f - 2.0f*(qx*qx + qy*qy))*lin_az;

    float V_baro = (current_altitude - last_altitude) / dt;
    last_altitude = current_altitude;

    float alpha = 0.95f;
    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);

    switch(phase) {
        case TRANSPORT:
        case PAD:
            alpha = 0.80f;
            break;
        case BOOST:
            alpha = 0.995f;
            break;
        case COAST:
            alpha = 0.960f + (fabsf(V_z) * 0.0001f);
            if (alpha > 0.995f) alpha = 0.995f;
            break;
        case DESCENT:
            alpha = 0.900f;
            break;
        default:
            break;
    }

    float V_predicted = V_z + (a_earth_z * dt);
    V_z = (alpha * V_predicted) + ((1.0f - alpha) * V_baro);
    filter_alt += V_z * dt;

    process_flight_state_machine(a_earth_z, filter_alt);

    float accel_along = a_earth_z;
    airspeedFilter.predict(accel_along, dt);
    airspeedFilter.updateIMU(accel_along, dt);
    airspeedFilter.updateBaro(V_baro);
}

void userFlightStabilizationLoop(float roll, float pitch, float yaw,
                                  float velocity_z, float dt) {
    (void)velocity_z;

    FlightPhase phase = currentPhase.load(std::memory_order_relaxed);
    if (phase == PAD || phase == TRANSPORT || phase == READY) {
        for (int i = 0; i < NUM_PIDS; i++) {
            pid[i].SetMode(QuickPID::Control::manual);
        }
        for (int i = 0; i < 8; i++) {
            latestServoAngles[i] = SERVO_CENTER_DEG;
        }
        for (int i = 0; i < 8; i++) latestPIDOutputs[i] = 0.0f;
        latestActiveGains[0] = pid[0].GetKp();
        latestActiveGains[1] = pid[0].GetKi();
        latestActiveGains[2] = pid[0].GetKd();
        latestAirspeed = 0.0f;
        latestKalmanP  = airspeedFilter.P;
        latestBaroAlpha = airspeedFilter.baro_alpha;
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

    for (int i = 0; i < NUM_PIDS; i++) {
        pidInput[i] = surfaceWeights[i].roll  * roll +
                      surfaceWeights[i].pitch * pitch +
                      surfaceWeights[i].yaw   * yaw;
        pidSetpoint[i] = 0.0f;
    }

    for (int i = 0; i < NUM_PIDS; i++) {
        pid[i].Compute();
    }

    for (int i = 0; i < 8; i++) {
        float angle = SERVO_CENTER_DEG +
            constrain(pidOutput[i], -MAX_DEFLECTION_DEG, MAX_DEFLECTION_DEG);
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

// ============================================================================
// C API — exported for Python ctypes
// ============================================================================

extern "C" {

void sim_init() {
    currentSystemMode.store(MODE_ACTIVE_PAD);
    currentPhase.store(PAD);
    systemArmed.store(true);
    wifiActive.store(true);
    V_z = 0.0f;
    filter_alt = 0.0f;
    previous_altitude = 0.0f;
    baseline_altitude = 0.0f;
    airspeedFilter.init();
    initStabilizationPIDs();
    for (int i = 0; i < 8; i++) {
        latestServoAngles[i] = SERVO_CENTER_DEG;
        latestPIDOutputs[i] = 0.0f;
    }
}

void sim_set_phase(uint8_t phase) {
    currentPhase.store(static_cast<FlightPhase>(phase));
}

uint8_t sim_get_phase() {
    return static_cast<uint8_t>(currentPhase.load(std::memory_order_relaxed));
}

void sim_set_armed(int armed) {
    systemArmed.store(armed != 0);
}

void sim_set_mode(uint8_t mode) {
    currentSystemMode.store(static_cast<SystemMode>(mode));
}

void sim_step(float roll, float pitch, float yaw,
              float lin_ax, float lin_ay, float lin_az,
              float qx, float qy, float qz, float qw,
              float raw_altitude,
              float dt,
              uint32_t elapsed_us) {
    sim_set_micros(elapsed_us);

    updateRocketFusion(lin_ax, lin_ay, lin_az, qx, qy, qz, qw,
                        raw_altitude, previous_altitude, dt);

    userFlightStabilizationLoop(roll, pitch, yaw, V_z, dt);

    latestQx = qx; latestQy = qy; latestQz = qz; latestQw = qw;
    latestAx = lin_ax; latestAy = lin_ay; latestAz = lin_az;
    latestDt = dt;
}

void sim_get_servos(float *out_angles, float *out_pid_outputs) {
    for (int i = 0; i < 8; i++) {
        out_angles[i] = latestServoAngles[i];
        out_pid_outputs[i] = latestPIDOutputs[i];
    }
}

void sim_get_gains(float *out_gains) {
    out_gains[0] = latestActiveGains[0];
    out_gains[1] = latestActiveGains[1];
    out_gains[2] = latestActiveGains[2];
}

float sim_get_airspeed() { return latestAirspeed; }
float sim_get_kalman_p() { return latestKalmanP; }
float sim_get_baro_alpha() { return latestBaroAlpha; }
float sim_get_vel_z() { return V_z; }
float sim_get_filter_alt() { return filter_alt; }
uint32_t sim_get_log_drops() { return logDropCount; }

}
