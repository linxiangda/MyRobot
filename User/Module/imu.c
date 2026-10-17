#include "imu.h"
#include "bmi088_driver.h"
#include "main.h"

#include <math.h>
#include <float.h>
#include <string.h>

#define IMU_GRAVITY       9.81f
#define IMU_RAD_TO_DEG    57.2957795131f

static uint8_t IsFinite(float value)
{
    return value == value && fabsf(value) <= FLT_MAX;
}

static void ResetFilters(IMU_t *imu)
{
    unsigned int i;

    for (i = 0; i < 3; i++) {
        /* 加速度软件低通：40 Hz */
        LowPass_Init(&imu->accel_filter[i], 40.0f);

        /* 陀螺仪软件低通暂时旁路 */
        LowPass_Init(&imu->gyro_filter[i], 0.0f);
    }
}

/* Handle bypass here without changing lowpass.c. */
static float ApplyLowPass(LowPassFilter_t *filter, float input, float dt)
{
    if (filter->cutoff_hz <= 0.0f) {
        filter->output = input;
        filter->initialized = 1;
        return input;
    }
    return LowPass_Update(filter, input, dt);
}

void IMU_Init(IMU_t *imu)
{
    if (imu == NULL) return;
    memset(imu, 0, sizeof(*imu));

    ResetFilters(imu);

    /* 先使用比例修正，积分修正暂不开启 */
    Mahony_Init(&imu->mahony, 2.0f, 0.0f);

    imu->result.quaternion[0] = 1.0f;
}

void IMU_SetGyroBias(IMU_t *imu, const float bias[3])
{
    unsigned int i;

    if (imu == NULL || bias == NULL) return;
    for (i = 0; i < 3; i++) {
        if (!IsFinite(bias[i])) return;
    }
    for (i = 0; i < 3; i++) {
        imu->gyro_bias[i] = bias[i];

        /* 清除滤波器中旧零偏对应的历史数据 */
        LowPass_Init(&imu->gyro_filter[i],
                     imu->gyro_filter[i].cutoff_hz);
    }

    memset(imu->mahony.integral, 0,
           sizeof(imu->mahony.integral));

    imu->result.valid = 0;
}

uint8_t IMU_Update(IMU_t *imu,
                   const float accel[3],
                   const float gyro[3],
                   float dt)
{
    float accel_norm;
    float accel_weight;
    float euler_rad[3];

    unsigned int i;

    if (imu == NULL) return 0;
    imu->result.valid = 0;
    if (accel == NULL || gyro == NULL) return 0;

    /*
     * 间隔过长时，本版选择重新对齐。
     * 重新对齐后的 yaw 会从 0 开始。
     */
    if (!IsFinite(dt) || dt <= 0.0f || dt > 0.05f) {
        imu->aligned = 0;
        ResetFilters(imu);
        return 0;
    }

    for (i = 0; i < 3; i++) {
        if (!IsFinite(accel[i]) || !IsFinite(gyro[i])) {
            return 0;
        }
    }

    accel_norm = sqrtf(accel[0] * accel[0]
                     + accel[1] * accel[1]
                     + accel[2] * accel[2]);

    if (!IsFinite(accel_norm)) {
        return 0;
    }

    if (!imu->aligned) {
        /*
         * 初次对齐时应保持静止。
         * 模长接近 1g 只是必要检查，不能保证确实静止。
         */
        if (fabsf(accel_norm - IMU_GRAVITY)
            > 0.1f * IMU_GRAVITY) {
            return 0;
        }

        if (!Mahony_AlignGravity(&imu->mahony, accel)) {
            return 0;
        }

        imu->aligned = 1;

        /* 首次只对齐，不积分此前未知的运动 */
        for (i = 0; i < 3; i++) {
            imu->result.accel[i] =
                ApplyLowPass(&imu->accel_filter[i],
                               accel[i], dt);

            imu->result.gyro[i] =
                ApplyLowPass(&imu->gyro_filter[i],
                               gyro[i] - imu->gyro_bias[i], dt);
        }
    } else {
        for (i = 0; i < 3; i++) {
            imu->result.accel[i] =
                ApplyLowPass(&imu->accel_filter[i],
                               accel[i], dt);

            imu->result.gyro[i] =
                ApplyLowPass(&imu->gyro_filter[i],
                               gyro[i] - imu->gyro_bias[i], dt);
        }

        /*
         * 加速度模长越偏离 1g，越少相信它代表重力方向。
         * 偏差达到 0.2g 时，本次只进行陀螺仪积分。
         */
        accel_weight =
            1.0f - fabsf(accel_norm - IMU_GRAVITY)
                 / (0.2f * IMU_GRAVITY);

        if (accel_weight < 0.0f) {
            accel_weight = 0.0f;
        }

        if (!Mahony_Update(&imu->mahony,
                           imu->result.gyro,
                           imu->result.accel,
                           accel_weight,
                           dt)) {
            return 0;
        }
    }

    memcpy(imu->result.quaternion,
           imu->mahony.q,
           sizeof(imu->result.quaternion));

    Mahony_GetEuler(&imu->mahony, euler_rad);

    imu->result.roll  = euler_rad[0] * IMU_RAD_TO_DEG;
    imu->result.pitch = euler_rad[1] * IMU_RAD_TO_DEG;
    imu->result.yaw   = euler_rad[2] * IMU_RAD_TO_DEG;

    imu->result.valid = 1;

    return 1;
}

const IMUResult_t *IMU_GetResult(const IMU_t *imu)
{
    return imu != NULL ? &imu->result : NULL;
}

/* Module-owned state. Raw sensor access and attitude processing stay here. */
static const uint8_t body_axes[3] = IMU_BODY_AXES;
static const int8_t body_signs[3] = IMU_BODY_SIGNS;
typedef struct {
    uint32_t count;
    float gyro_mean[3], gyro_m2[3];
    float accel_mean[3], accel_m2[3];
} Calibration_t;


static IMU_t module_imu;
static Calibration_t calibration;
static IMUData_TypeDef sample;
/* Kept for legacy debuggers. Use IMU_ModuleGetData for coherent reads. */
IMUData_TypeDef IMUdata;
static uint8_t calibration_requested;
static uint8_t module_initialized;
static uint8_t have_previous;
static uint32_t previous_cycles, previous_ms, warmup_ms, retry_ms;
static uint32_t consecutive_errors;

/* Short snapshot copies only; never hold this lock during SPI or math. */
static uint32_t SnapshotLock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return mask;
}

static void SnapshotUnlock(uint32_t mask)
{
    __DMB();
    __set_PRIMASK(mask);
}

static void PublishModuleData(void)
{
    uint32_t mask = SnapshotLock();
    IMUdata = sample;
    if (calibration_requested) {
        IMUdata.valid = IMUdata.calibrated = 0;
        IMUdata.status = IMU_STATUS_CALIBRATING;
    }
    SnapshotUnlock(mask);
}

void IMU_ModuleGetData(IMUData_TypeDef *out)
{
    uint32_t now, mask;
    if (out == NULL) return;
    mask = SnapshotLock();
    *out = IMUdata;
    now = HAL_GetTick();
    SnapshotUnlock(mask);
    if ((uint32_t)(now - out->sample_tick) > IMU_DATA_MAX_AGE_MS)
        out->valid = 0;
}

uint8_t IMU_ReadSnapshot(IMUData_TypeDef *out)
{
    if (out == NULL) return 0;
    IMU_ModuleGetData(out);
    return out->valid && out->calibrated && out->status == IMU_STATUS_RUNNING;
}

void IMU_RequestCalibration(void)
{
    uint32_t mask = SnapshotLock();
    calibration_requested = 1;
    IMUdata.valid = IMUdata.calibrated = 0;
    IMUdata.status = IMU_STATUS_CALIBRATING;
    SnapshotUnlock(mask);
}

static uint8_t MappingValid(void)
{
    unsigned int i, j, inversions = 0;
    int determinant = 1;
    for (i = 0; i < 3; i++) {
        if (body_axes[i] > 2 ||
            (body_signs[i] != 1 && body_signs[i] != -1)) return 0;
        determinant *= body_signs[i];
        for (j = 0; j < i; j++) {
            if (body_axes[i] == body_axes[j]) return 0;
            if (body_axes[j] > body_axes[i]) inversions++;
        }
    }
    if (inversions & 1U) determinant = -determinant;
    return determinant == 1;
}

/* Returns 1 for a stationary window; rejects motion and excessive noise.
 * These are commissioning thresholds, not a proof that the robot is still. */
static uint8_t Calibrate(Calibration_t *cal, const float accel[3],
                         const float gyro[3], float bias[3])
{
    unsigned int i;
    float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (fabsf(norm - 9.81f) > 0.981f) {
        memset(cal, 0, sizeof(*cal));
        return 0;
    }
    for (i = 0; i < 3; i++) {
        if (fabsf(gyro[i]) > 0.15f) {
            memset(cal, 0, sizeof(*cal));
            return 0;
        }
    }
    cal->count++;
    for (i = 0; i < 3; i++) {
        float d = gyro[i] - cal->gyro_mean[i];
        cal->gyro_mean[i] += d / (float)cal->count;
        cal->gyro_m2[i] += d * (gyro[i] - cal->gyro_mean[i]);
        d = accel[i] - cal->accel_mean[i];
        cal->accel_mean[i] += d / (float)cal->count;
        cal->accel_m2[i] += d * (accel[i] - cal->accel_mean[i]);
    }
    if (cal->count < IMU_CALIBRATION_SAMPLES) return 0;
    for (i = 0; i < 3; i++) {
        if (cal->gyro_m2[i] / (float)(cal->count - 1U) > 0.0004f ||
            cal->accel_m2[i] / (float)(cal->count - 1U) > 0.04f) {
            memset(cal, 0, sizeof(*cal));
            return 0;
        }
    }
    memcpy(bias, cal->gyro_mean, sizeof(cal->gyro_mean));
    return 1;
}


static void ResetCalibration(void)
{
    IMU_Init(&module_imu);
    memset(&calibration, 0, sizeof(calibration));
    sample.valid = sample.calibrated = 0;
    sample.calibration_count = 0;
    sample.reset_count++;
    have_previous = 0;
}

int IMU_ModuleInit(void)
{
    module_initialized = 0;
    sample.valid = sample.calibrated = 0;
    sample.status = IMU_STATUS_INITIALIZING;
    sample.quaternion[0] = 1.0f;
    sample.quaternion[1] = sample.quaternion[2] = sample.quaternion[3] = 0.0f;
    PublishModuleData();
    if (!MappingValid()) {
        sample.status = IMU_STATUS_CONFIG_ERROR;
        PublishModuleData();
        return -1;
    }
    if (Drive_BMI088_Init() != BMI088_OK) {
        sample.read_errors++;
        sample.status = IMU_STATUS_SENSOR_ERROR;
        retry_ms = HAL_GetTick();
        PublishModuleData();
        return -1;
    }
    /* Use the shared cycle counter without resetting another user's clock. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    sample.cycle_clock = (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
    ResetCalibration();
    consecutive_errors = 0;
    warmup_ms = HAL_GetTick();
    module_initialized = 1;
    sample.status = IMU_STATUS_WARMUP;
    PublishModuleData();
    return 0;
}

static void ReadFailure(void)
{
    sample.valid = 0;
    sample.read_errors++;
    sample.status = IMU_STATUS_SENSOR_ERROR;
    if (!sample.calibrated) {
        memset(&calibration, 0, sizeof(calibration));
        sample.calibration_count = 0;
    }
    if (++consecutive_errors >= 10U) {
        module_initialized = 0;
        retry_ms = HAL_GetTick();
    }
    PublishModuleData();
}

uint8_t IMU_ModuleUpdate(void)
{
    float sensor[BMI088_DATA_LENGTH];
    float dt;
    uint32_t now_ms, now_cycles, elapsed_ms, elapsed_cycles, mask;
    uint8_t request;
    unsigned int i;
    if (!module_initialized) {
        if (sample.status != IMU_STATUS_CONFIG_ERROR &&
            (uint32_t)(HAL_GetTick() - retry_ms) >= 1000U)
            IMU_ModuleInit();
        return 0;
    }
    mask = SnapshotLock();
    request = calibration_requested;
    calibration_requested = 0;
    SnapshotUnlock(mask);
    if (request) ResetCalibration();

    sample.valid = 0;
    if (Drive_BMI088_Read(sensor) != BMI088_OK) {
        ReadFailure();
        return 0;
    }
    for (i = 0; i < BMI088_DATA_LENGTH; i++) {
        if (!IsFinite(sensor[i])) {
            ReadFailure();
            return 0;
        }
    }
    now_ms = HAL_GetTick();
    now_cycles = DWT->CYCCNT;
    elapsed_ms = now_ms - previous_ms;
    elapsed_cycles = now_cycles - previous_cycles;
    dt = (float)elapsed_ms * 0.001f;
    if (!have_previous) dt = 0.001f;
    else if (sample.cycle_clock && elapsed_cycles != 0U &&
             elapsed_ms <= 50U && SystemCoreClock != 0U)
        dt = (float)elapsed_cycles / (float)SystemCoreClock;
    previous_ms = now_ms;
    previous_cycles = now_cycles;
    have_previous = 1;
    consecutive_errors = 0;
    sample.dt = dt;
    sample.sample_tick = now_ms;
    sample.sample_count++;
    for (i = 0; i < 3; i++) {
        sample.accel[i] = body_signs[i] * sensor[body_axes[i]];
        sample.gyro[i] = body_signs[i] * sensor[3U + body_axes[i]];
    }
    sample.temperature = sensor[6];
    if (dt <= 0.0f || dt > 0.05f) {
        ResetCalibration();
        sample.status = IMU_STATUS_TIMING_ERROR;
    } else if ((uint32_t)(now_ms - warmup_ms) < 500U) {
        sample.status = IMU_STATUS_WARMUP;
    } else {
        if (!sample.calibrated) {
            sample.status = IMU_STATUS_CALIBRATING;
            if (Calibrate(&calibration, sample.accel, sample.gyro,
                          sample.gyro_bias)) {
                IMU_SetGyroBias(&module_imu, sample.gyro_bias);
                sample.calibrated = 1;
            }
        }
        if (sample.calibrated) {
            if (IMU_Update(&module_imu, sample.accel, sample.gyro, dt)) {
                const IMUResult_t *result = IMU_GetResult(&module_imu);
                memcpy(sample.quaternion, result->quaternion, sizeof(sample.quaternion));
                memcpy(sample.angular_velocity, result->gyro, sizeof(sample.angular_velocity));
                sample.roll = result->roll;
                sample.pitch = result->pitch;
                sample.yaw = result->yaw;
                sample.valid = 1;
                sample.status = IMU_STATUS_RUNNING;
            } else sample.status = IMU_STATUS_ALIGNMENT_ERROR;
        }
        sample.calibration_count = calibration.count;
    }
    PublishModuleData();
    return sample.valid;
}
