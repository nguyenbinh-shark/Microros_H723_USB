"""Compile real INS/Mahony sources with host mocks; no hardware access."""
from pathlib import Path
import subprocess
import shutil
import os

root = Path(__file__).resolve().parents[2]
test_dir = root / 'build' / 'ins_host_test'
test_dir.mkdir(parents=True, exist_ok=True)
headers = {
    'BMI088driver.h': '''#pragma once
#include <stdint.h>
typedef struct { float Accel[3], Gyro[3], AccelScale, GyroOffset[3]; } IMU_Data_t;
extern IMU_Data_t BMI088;
void BMI088_Read(IMU_Data_t *data);
uint8_t BMI088_ReadIsValid(void);
uint32_t HAL_GetTick(void);
''',
    'QuaternionEKF.h': '#pragma once\n',
    'bsp_PWM.h': '#pragma once\n',
    'can_bsp.h': '#pragma once\n',
    'bsp_dwt.h': '''#pragma once
#include <stdint.h>
float DWT_GetDeltaT(uint32_t *count);
''',
    'cmsis_os.h': '''#pragma once
#include <stdint.h>
uint32_t osKernelGetTickCount(void);
void osDelayUntil(uint32_t tick);
''',
    'FreeRTOS.h': '#pragma once\n',
    'task.h': '''#pragma once
void mock_enter(void);
void mock_exit(void);
#define taskENTER_CRITICAL() mock_enter()
#define taskEXIT_CRITICAL() mock_exit()
''',
    'stm32h7xx.h': '#pragma once\n#include <stdint.h>\n',
    'arm_math.h': '#pragma once\n',
}
for name, contents in headers.items():
    (test_dir / name).write_text(contents)

harness = r'''
#include "INS_task.h"
#include "mahony_filter.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

IMU_Data_t BMI088;
extern struct MAHONY_FILTER_t mahony;
static jmp_buf done;
static uint32_t tick, stop_tick, first_lock;
static int mode, critical_depth, read_valid;

void mock_enter(void) { assert(critical_depth == 0); ++critical_depth; }
void mock_exit(void) { assert(critical_depth == 1); --critical_depth; }
uint32_t HAL_GetTick(void) { return tick; }
uint32_t osKernelGetTickCount(void) { return tick; }
void osDelayUntil(uint32_t next) {
    INS_Snapshot_t sample;
    if (INS_GetSnapshot(&sample) && sample.calibrated && first_lock == 0)
        first_lock = sample.sample_tick_ms;
    if (next >= stop_tick) longjmp(done, 1);
    tick = next;
}
float DWT_GetDeltaT(uint32_t *count) {
    (void)count;
    if (mode == 8 && tick == 2500) return 0.1f;
    return 0.001f;
}
uint8_t BMI088_ReadIsValid(void) { return (uint8_t)read_valid; }
void BMI088_Read(IMU_Data_t *data) {
    data->Accel[0] = 0; data->Accel[1] = 0; data->Accel[2] = 9.81f;
    data->Gyro[0] = 0.001f; data->Gyro[1] = 0.002f; data->Gyro[2] = -0.003f;
    read_valid = 1;
    if (mode == 1) data->Gyro[2] = 0.2f;
    if (mode == 2 && tick >= 2500 && tick < 2600) data->Gyro[2] = 0.2f;
    if (mode == 3 && tick >= 5000) data->Gyro[2] += 0.004f;
    if (mode == 4 && tick == 5500) read_valid = 0;
    if (mode == 5 && tick == 5500) data->Gyro[0] = NAN;
    if (mode == 6 && tick == 5500) memset(data->Accel, 0, sizeof(data->Accel));
    if (mode == 7) data->Gyro[2] += (tick & 1) ? 0.003f : -0.003f;
    if (mode == 9 && tick == 5500) mahony.q0 = NAN;
    if (mode == 10) {
        float a = (tick & 1) ? 0.2f : -0.2f;
        data->Accel[0] = a;
        data->Accel[2] = sqrtf(9.81f * 9.81f - a * a);
    }
    if (mode == 11) {
        const float roll = 0.3f, pitch = -0.2f;
        data->Accel[0] = -9.81f * sinf(pitch);
        data->Accel[1] = 9.81f * sinf(roll) * cosf(pitch);
        data->Accel[2] = 9.81f * cosf(roll) * cosf(pitch);
    }
}
static INS_Snapshot_t run(int test_mode, uint32_t until) {
    mode = test_mode; tick = 0; stop_tick = until; first_lock = 0;
    BMI088.AccelScale = 1.2345f;
    if (setjmp(done) == 0) INS_task();
    assert(critical_depth == 0);
    assert(BMI088.AccelScale == 1.2345f);
    INS_Snapshot_t sample;
    INS_GetSnapshot(&sample);
    assert(sample.sample_tick_ms == until - 1);
    assert(sample.sequence == until - 1);
    return sample;
}
static void normalized(const INS_Snapshot_t *s) {
    float norm2 = 0;
    for (int i = 0; i < 4; ++i) { assert(isfinite(s->q[i])); norm2 += s->q[i] * s->q[i]; }
    assert(fabsf(norm2 - 1.0f) < 1e-5f);
}
int main(void) {
    assert(INS_GetSnapshot(NULL) == 0);
    INS_Snapshot_t s = run(0, 3999);
    assert(!s.calibrated && s.valid);
    s = run(0, 6000);
    assert(s.calibrated && s.valid && INS.ins_flag);
    assert(first_lock == 4000);
    assert(fabsf(s.gyro_bias[2] + 0.003f) < 1e-6f);
    assert(fabsf(s.gyro[2]) < 1e-6f); normalized(&s);
    s = run(1, 12000); assert(!s.calibrated && !INS.ins_flag);
    s = run(2, 5500); assert(!s.calibrated);
    s = run(2, 6500); assert(s.calibrated && first_lock == 5600);
    s = run(3, 10000);
    assert(s.calibrated && fabsf(s.gyro[2] - 0.004f) < 1e-6f);
    assert(fabsf(s.gyro_bias[2] + 0.003f) < 1e-6f);
    assert(INS.Yaw > 0.019f && INS.Yaw < 0.021f); normalized(&s);
    for (int bad_mode = 4; bad_mode <= 6; ++bad_mode) {
        s = run(bad_mode, 5501); assert(!s.valid && s.calibrated && !INS.ins_flag);
        s = run(bad_mode, 6000); assert(s.valid && s.calibrated && INS.ins_flag); normalized(&s);
    }
    s = run(7, 12000); assert(!s.calibrated); /* Variance, not merely peak rate. */
    s = run(8, 5500); assert(!s.calibrated);
    s = run(8, 6000); assert(s.calibrated && first_lock == 5501);
    s = run(9, 5501); assert(!s.valid && !INS.ins_flag);
    s = run(9, 6000); assert(s.valid && s.calibrated); normalized(&s);
    s = run(10, 12000); assert(!s.calibrated); /* Moving gravity direction. */
    s = run(11, 6000);
    assert(s.valid && s.calibrated); normalized(&s);
    assert(fabsf(INS.Pitch - 0.3f) < 0.0001f); /* Preserve legacy swapped Euler. */
    assert(fabsf(INS.Roll + 0.2f) < 0.0001f);
    assert(fabsf(INS.MotionAccel_b[0]) < 0.0001f);
    assert(fabsf(INS.MotionAccel_b[1]) < 0.0001f);
    assert(fabsf(INS.MotionAccel_b[2]) < 0.0001f);

    struct MAHONY_FILTER_t m;
    memset(&m, 0xFF, sizeof(m));
    mahony_init(&m, 1, 0.001f, 0.001f);
    assert(m.q0 == 1 && m.q1 == 0 && m.exInt == 0 && m.rMat[2][2] == 1);
    mahony_input(&m, (Axis3f){0,0,0}, (Axis3f){0,0,0});
    mahony_update(&m); assert(!m.valid && m.q0 == 1);
    mahony_input(&m, (Axis3f){NAN,0,0}, (Axis3f){0,0,9.81f});
    mahony_update(&m); assert(!m.valid && isfinite(m.q0));
    mahony_input(&m, (Axis3f){0,0,0}, (Axis3f){0,0,9.81f});
    mahony_update(&m); assert(m.valid);
    m.rMat[2][0] = 1.000001f;
    mahony_output(&m); assert(isfinite(m.pitch));
    puts("PASS: real INS/Mahony startup, motion/variance rejection, frozen bias, invalid-data recovery, tilt/axis preservation, quaternion/asin guards");
    return 0;
}
'''
(test_dir / 'test_ins.c').write_text(harness)
exe = test_dir / 'test_ins.exe'
gcc = shutil.which('gcc') or 'C:/msys64/ucrt64/bin/gcc.exe'
cmd = [gcc, '-std=c11', '-Wall', '-Wextra', '-Werror',
       '-O2', '-I' + str(test_dir), '-I' + str(root / 'User/APP'),
       '-I' + str(root / 'User/Algorithm/mahony'),
       str(test_dir / 'test_ins.c'), str(root / 'User/APP/INS_task.c'),
       str(root / 'User/Algorithm/mahony/mahony_filter.c'), '-lm', '-o', str(exe)]
environment = os.environ.copy()
environment['PATH'] = str(Path(gcc).parent) + os.pathsep + environment['PATH']
subprocess.run(cmd, check=True, env=environment)
subprocess.run([str(exe)], check=True, env=environment)
