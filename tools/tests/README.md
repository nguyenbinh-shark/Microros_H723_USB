# Firmware sensor regression checks

These Python harnesses compile the production C sources with a host GCC and mocked HAL/RTOS/USB functions. They require Python 3 and `gcc` on PATH (INS and USB checks also look for MSYS2 UCRT64 GCC on Windows). Run from the repository root:

```bash
python tools/tests/test_bmi088_faults.py
python tools/tests/test_ins_host.py
python tools/tests/test_usb_cdc_host.py
```

- BMI088: inject a timeout at every sample-read byte; verify read validity, complete-sample commit, bounded failure, chip IDs and calibration values.
- INS/Mahony: stationary startup timing, motion/noise rejection, frozen bias during subsequent slow rotation, invalid-read/state recovery, tilted gravity compensation and quaternion guards.
- USB CDC: boundary wrapping, overflow, busy retries, asynchronous transmit-buffer lifetime, stale completion, disconnect and reconnect.

Generated files stay in ignored `build/` or a temporary directory. These checks do not access the board and do not prove physical axis alignment, sensor noise, yaw drift or live micro-ROS session stability. See [IMU interface and board checks](../../docs/topics.md#36-imudata-bmi088-imu).
