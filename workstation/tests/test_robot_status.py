from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_status import summarize


SESSION = dict(pid=123, domain=0, mode='navigation')


def healthy_data():
    topics = {name: dict(age_s=0.1, messages=25) for name in
              ('/map', '/scan', '/odom', '/imu/data', '/motor_fb', '/motor_status')}
    return dict(topics=topics, map=dict(width=469, height=285, resolution=0.05),
                tf_map_to_base=True, mapping_ready=True, navigation_available=True,
                motor_status=0x13, velocity=dict(linear=0.123, angular=-0.456))


class RobotStatusTests(unittest.TestCase):
    def test_offline_never_shows_healthy_sensors_or_zero_velocity(self):
        result = summarize(None, None, ssh_failed=True)
        self.assertEqual(result.cards['connection'].level, 'error')
        self.assertTrue(all(sensor.level == 'unknown' for sensor in result.sensors.values()))
        self.assertEqual((result.linear, result.angular), ('—', '—'))

    def test_connected_but_idle_does_not_report_a_ros_failure(self):
        result = summarize(None, None, ssh_ok=True)
        self.assertEqual(result.cards['connection'].level, 'ok')
        self.assertEqual(result.cards['mapping'].level, 'unknown')
        self.assertIn('Bắt đầu Mapping', result.notice)

    def test_action_server_with_missing_pose_does_not_imply_navigation_readiness(self):
        data = healthy_data()
        data['tf_map_to_base'] = data['mapping_ready'] = False
        result = summarize(SESSION, data, ssh_ok=True)
        self.assertEqual(result.cards['navigation'].level, 'warning')
        self.assertIn('2D Pose Estimate', result.notice)
        self.assertEqual(result.notice_level, 'warning')
        self.assertIn('469×285', result.cards['mapping'].detail)

    def test_action_endpoint_alone_is_reported_as_detected_without_claiming_active(self):
        result = summarize(SESSION, healthy_data(), ssh_ok=True)
        self.assertEqual(result.cards['navigation'].level, 'info')
        self.assertIn('active', result.cards['navigation'].detail)

    def test_stale_odometry_removes_numeric_velocity_and_marks_only_affected_sensors(self):
        data = healthy_data()
        data['topics']['/odom']['age_s'] = 5
        result = summarize(SESSION, data, ssh_ok=True)
        self.assertEqual(result.sensors['Odom'].level, 'error')
        self.assertEqual(result.sensors['LiDAR'].level, 'ok')
        self.assertEqual((result.linear, result.angular), ('—', '—'))
        self.assertEqual(result.cards['navigation'].level, 'warning')
        self.assertIn('Odom', result.notice)

    def test_bus_off_is_prominent_even_when_map_and_all_topics_are_fresh(self):
        data = healthy_data()
        data['motor_status'] = 0x17
        result = summarize(SESSION, data, ssh_ok=True)
        self.assertEqual(result.cards['motion'].level, 'error')
        self.assertEqual(result.notice_level, 'error')
        self.assertIn('CAN', result.notice)

    def test_timeout_is_a_warning_not_an_enabled_motor_readiness_claim(self):
        data = healthy_data()
        data['motor_status'] = 0x33
        result = summarize(SESSION, data, ssh_ok=True)
        self.assertEqual(result.cards['motion'].level, 'warning')
        self.assertEqual(result.cards['motion'].badge, 'ROS TIMEOUT')
        self.assertEqual(result.cards['motion'].value, 'MOTOR BẬT')

    def test_lost_ssh_keeps_fresh_ros_values_but_marks_connection_degraded(self):
        result = summarize(SESSION, healthy_data(), ssh_failed=True)
        self.assertEqual(result.cards['connection'].level, 'warning')
        self.assertEqual(result.cards['connection'].badge, 'SSH LỖI')
        self.assertEqual((result.linear, result.angular), ('+0.123', '-0.456'))

    def test_lost_stream_does_not_keep_previous_map_or_motor_as_healthy(self):
        result = summarize(SESSION, None, ssh_ok=True)
        self.assertNotEqual(result.cards['mapping'].level, 'ok')
        self.assertEqual(result.cards['motion'].level, 'unknown')
        self.assertEqual((result.linear, result.angular), ('—', '—'))

    def test_unseen_sensor_is_distinct_from_a_sensor_that_stopped_sending(self):
        data = healthy_data()
        data['topics']['/imu/data'] = dict(age_s=None, messages=0)
        result = summarize(SESSION, data)
        self.assertEqual(result.sensors['IMU'].level, 'warning')
        data['topics']['/imu/data'] = dict(age_s=4, messages=10)
        result = summarize(SESSION, data)
        self.assertEqual(result.sensors['IMU'].level, 'error')

    def test_presenter_does_not_modify_telemetry(self):
        data = healthy_data()
        original = deepcopy(data)
        summarize(SESSION, data)
        self.assertEqual(data, original)

    def test_lost_odometry_is_prioritized_over_initial_pose_guidance(self):
        data = healthy_data()
        data['tf_map_to_base'] = False
        data['topics']['/odom']['age_s'] = 5
        result = summarize(SESSION, data, ssh_ok=True)
        self.assertEqual(result.notice_level, 'error')
        self.assertIn('Odom', result.notice)


if __name__ == '__main__':
    unittest.main()
