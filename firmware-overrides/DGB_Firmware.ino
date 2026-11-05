#include "config.h"
#include "icm45686_spi.h"
#include "touch_sensor.h"
#include "usb_ds4_hid.h"
#include "gyro_drift_filter.h"
#include "motion_pipeline.h"
#include <stdio.h>

static GyroDriftFilter gyroFilter;
static DS4InputReport ds4Report;
static ReportSchedule reportSchedule;
static uint32_t lastMotionUs, lastTouchUs, startedUs;
static uint32_t submittedReports, freshSamples, staleReports, longestSendGapUs, lastSentUs;
static bool touched, calibrated, haveMotion;
static ImuCalibrationState previousCalibration = ImuCalibrationState::Idle;
static char serialReply[512];
static size_t replyOffset, replyLength;

static const char *calibrationName(ImuCalibrationState state) {
  switch (state) {
    case ImuCalibrationState::Idle: return "idle";
    case ImuCalibrationState::Collecting: return "collecting: keep stationary";
    case ImuCalibrationState::Complete: return "complete";
    case ImuCalibrationState::TimedOut: return "failed: motion, missing data, or bad gravity; send c to retry";
  }
  return "unknown";
}
static void queueStatus() {
  const ImuBiasOffsets b1 = imu.sensorBias(0), b2 = imu.sensorBias(1);
  const int n = snprintf(serialReply, sizeof(serialReply),
      "IMU1=%s IMU2=%s calibration=%s\r\n"
      "bias1_counts=%.3f,%.3f,%.3f bias2_counts=%.3f,%.3f,%.3f\r\n"
      "window_samples=%lu,%lu uptime_ms=%lu queued=%lu fresh=%lu stale_reports=%lu max_queue_gap_us=%lu\r\n"
      "c: stationary gyro recalibration; ?: status. Measure host arrival rate with USB capture.\r\n",
      imu.sensorOk(0) ? "OK" : "FAIL", imu.sensorOk(1) ? "OK" : "FAIL",
      calibrationName(imu.calibrationState()),
      double(b1.gx), double(b1.gy), double(b1.gz), double(b2.gx), double(b2.gy), double(b2.gz),
      (unsigned long)imu.calibrationSamples(0), (unsigned long)imu.calibrationSamples(1),
      (unsigned long)((micros() - startedUs) / 1000U), (unsigned long)submittedReports,
      (unsigned long)freshSamples, (unsigned long)staleReports, (unsigned long)longestSendGapUs);
  replyLength = n > 0 ? (size_t(n) < sizeof(serialReply) ? size_t(n) : sizeof(serialReply) - 1) : 0;
  replyOffset = 0;
}
static void serviceSerial() {
  if (Serial.available()) {
    const int command = Serial.read();
    if (command == 'c' && imu.calibrationState() != ImuCalibrationState::Collecting) {
      imu.startCalibration();
      queueStatus();
    } else if (command == '?') queueStatus();
  }
  // Bounded writes: diagnostics and a closed monitor cannot stall HID.
  if (Serial && replyOffset < replyLength) {
    const int room = Serial.availableForWrite();
    if (room > 0) {
      size_t length = replyLength - replyOffset;
      if (length > size_t(room)) length = size_t(room);
      if (length > 32) length = 32;
      replyOffset += Serial.write((const uint8_t *)serialReply + replyOffset, length);
    }
  }
}
void setup() {
  Serial.begin(115200);
  touchSensor.begin();
  usbDs4.begin();
  imu.begin();
  usbDs4.initializeReport(ds4Report);
  startedUs = lastMotionUs = lastTouchUs = micros();
  reportSchedule.reset(startedUs);
  imu.startCalibration();
  queueStatus();
}
void loop() {
  ImuMotionData motion;
  if (imu.readFusedMotion(motion)) {
    const uint32_t now = micros();
    const uint32_t elapsed = now - lastMotionUs;
    lastMotionUs = now;
    ++freshSamples;
    const ImuCalibrationState state = imu.calibrationState();
    if (state == ImuCalibrationState::Complete && previousCalibration != state) {
      gyroFilter.reset();
      calibrated = true;
    }
    const MotionVector gyro = ds4Gyro({motion.gx, motion.gy, motion.gz}, IMU_COUNTS_PER_DPS);
    const MotionVector accel = ds4Accel({motion.ax, motion.ay, motion.az}, IMU_COUNTS_PER_G);
    float gx, gy, gz;
    gyroFilter.process(gyro.x, gyro.y, gyro.z, gx, gy, gz, elapsed ? elapsed : 1U);
    // Before first successful calibration emit neutral gyro. Gravity stays live.
    ds4Report.gyro_x = calibrated ? motionCount(gx) : 0;
    ds4Report.gyro_y = calibrated ? motionCount(gy) : 0;
    ds4Report.gyro_z = calibrated ? motionCount(gz) : 0;
    ds4Report.accel_x = motionCount(accel.x);
    ds4Report.accel_y = motionCount(accel.y);
    ds4Report.accel_z = motionCount(accel.z);
    haveMotion = true;
  }
  const ImuCalibrationState state = imu.calibrationState();
  if (state != previousCalibration) {
    previousCalibration = state;
    queueStatus();
  }
  uint32_t now = micros();
  if (uint32_t(now - lastTouchUs) >= 4000U) {
    touched = touchSensor.readTouch();
    lastTouchUs = now;
  }
  now = micros();
  if (reportSchedule.due(now) && usbDs4.isReady()) {
    if (!haveMotion || uint32_t(now - lastMotionUs) > 10000U) {
      ds4Report.gyro_x = ds4Report.gyro_y = ds4Report.gyro_z = 0;
      ++staleReports;
    }
    usbDs4.prepareReport(ds4Report, now, touched);
    if (usbDs4.sendReport(ds4Report)) {
      reportSchedule.sent(now);
      if (submittedReports && uint32_t(now - lastSentUs) > longestSendGapUs)
        longestSendGapUs = now - lastSentUs;
      lastSentUs = now;
      ++submittedReports;
    }
  }
  serviceSerial();
}
