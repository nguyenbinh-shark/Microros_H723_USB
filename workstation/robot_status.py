"""Evidence-based display states. These summaries never issue robot commands."""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Indicator:
    level: str
    value: str
    badge: str
    detail: str = ''


@dataclass
class RobotStatus:
    cards: dict = field(default_factory=dict)
    sensors: dict = field(default_factory=dict)
    linear: str = '—'
    angular: str = '—'
    notice: str = ''
    notice_level: str = 'unknown'


def summarize(session, data, *, ssh_ok=False, ssh_failed=False, connecting=False, host=''):
    result = RobotStatus()
    topics = (data or {}).get('topics', {})
    def fresh(name):
        age = topics.get(name, {}).get('age_s')
        return age is not None and age < 2
    receiving = any(fresh(name) for name in ('/scan', '/odom', '/imu/data', '/motor_fb', '/motor_status'))
    if receiving:
        result.cards['connection'] = Indicator('warning' if ssh_failed else 'ok', 'ROS ĐANG NHẬN',
                                               'SSH LỖI' if ssh_failed else 'ONLINE', host)
    elif ssh_ok:
        result.cards['connection'] = Indicator('warning' if session else 'ok', 'ĐÃ KẾT NỐI PI',
                                               'CHỜ ROS' if session else 'SSH OK', host)
    elif ssh_failed:
        result.cards['connection'] = Indicator('error', 'NGOẠI TUYẾN', 'MẤT KẾT NỐI', host)
    else:
        result.cards['connection'] = Indicator('info' if connecting else 'unknown',
                                               'ĐANG KẾT NỐI' if connecting else 'CHƯA KẾT NỐI', 'CHỜ', host)
    mode = session.get('mode') if session else None
    map_info = (data or {}).get('map')
    tf_ok = bool((data or {}).get('tf_map_to_base'))
    if not session:
        result.cards['mapping'] = Indicator('unknown', 'CHƯA CÓ PHIÊN', 'CHƯA CHẠY', 'Chưa chạy Mapping / Nav')
    elif data is None:
        result.cards['mapping'] = Indicator('warning', mode.upper(), 'CHỜ DỮ LIỆU', f"ROS domain {session['domain']}")
    elif not map_info:
        result.cards['mapping'] = Indicator('warning', mode.upper(), 'CHƯA CÓ MAP', 'Đang chờ dữ liệu bản đồ')
    else:
        dimensions = f"{map_info['width']}×{map_info['height']} · {map_info['resolution']:.3f} m/ô"
        ready = tf_ok and fresh('/scan') and fresh('/odom')
        result.cards['mapping'] = Indicator('ok' if ready else 'warning', mode.upper(),
                                            'MAP / TF OK' if ready else 'THIẾU TF' if not tf_ok else 'DỮ LIỆU CŨ', dimensions)
    server = bool((data or {}).get('navigation_available'))
    if mode != 'navigation' and not server:
        result.cards['navigation'] = Indicator('unknown', 'CHƯA BẬT NAV', 'CHƯA CHẠY', 'Chọn map và bấm Bắt đầu Nav')
    elif data is None:
        result.cards['navigation'] = Indicator('warning', 'CHỜ DỮ LIỆU', 'CHƯA XÁC NHẬN', 'Đang kiểm tra phiên Navigation')
    elif not map_info:
        result.cards['navigation'] = Indicator('warning', 'CHỜ BẢN ĐỒ', 'CHỜ', 'Chưa nhận map đã lưu')
    elif not tf_ok:
        result.cards['navigation'] = Indicator('warning', 'CHỜ VỊ TRÍ / TF', 'CẦN XỬ LÝ', 'Đặt vị trí ban đầu trong RViz')
    elif not fresh('/scan') or not fresh('/odom'):
        result.cards['navigation'] = Indicator('warning', 'THIẾU DỮ LIỆU', 'CHƯA SẴN SÀNG', 'Cần scan và odom mới')
    elif server:
        # Seeing an action endpoint does not prove that lifecycle nodes are active.
        result.cards['navigation'] = Indicator('info', 'CÓ ACTION SERVER', 'ĐÃ PHÁT HIỆN', 'Xem trạng thái Nav2 active trong RViz')
    else:
        result.cards['navigation'] = Indicator('warning', 'CHỜ NAV2', 'ĐANG KHỞI ĐỘNG', 'Chưa thấy action server điều hướng')
    for name, topic in (('LiDAR', '/scan'), ('Odom', '/odom'), ('IMU', '/imu/data'), ('Motor FB', '/motor_fb')):
        if data is None:
            item = Indicator('unknown', name, 'CHƯA CÓ')
        elif fresh(topic):
            item = Indicator('ok', name, 'OK')
        elif topics.get(topic, {}).get('messages', 0):
            item = Indicator('error', name, 'DỮ LIỆU CŨ')
        else:
            item = Indicator('warning', name, 'ĐANG CHỜ')
        result.sensors[name] = item
    motor = data.get('motor_status') if data and fresh('/motor_status') else None
    if motor is None:
        result.cards['motion'] = Indicator('unknown', 'CHƯA CÓ', 'CHƯA CÓ', 'Chờ trạng thái STM32')
    elif motor & 0x0c:
        result.cards['motion'] = Indicator('error', 'LỖI CAN BUS-OFF', 'LỖI', 'Kiểm tra kết nối CAN của bánh xe')
    elif motor & 0x03 != 0x03:
        result.cards['motion'] = Indicator('error', 'THIẾU PHẢN HỒI', 'LỖI', 'Chưa đủ phản hồi hai bánh xe')
    else:
        enabled = bool(motor & 0x10)
        timed_out = bool(motor & 0x20)
        result.cards['motion'] = Indicator('warning' if timed_out else 'ok' if enabled else 'info',
                                           'MOTOR BẬT' if enabled else 'MOTOR TẮT',
                                           'ROS TIMEOUT' if timed_out else 'CÓ PHẢN HỒI',
                                           'Chưa có lệnh vận tốc ROS mới' if timed_out else 'Đang nhận trạng thái từ STM32')
    if data and fresh('/odom') and data.get('velocity'):
        result.linear = f"{data['velocity']['linear']:+.3f}"
        result.angular = f"{data['velocity']['angular']:+.3f}"
    if not receiving and not ssh_ok:
        result.notice_level = 'error' if ssh_failed else 'info'
        result.notice = 'Chưa liên lạc được Pi. Studio tự kết nối lại khi robot khả dụng.'
    elif not session:
        result.notice_level = 'info'
        result.notice = 'Pi đã kết nối. Bắt đầu Mapping để tạo map, hoặc chọn map đã lưu và bắt đầu Nav.'
    elif result.cards['motion'].level == 'error':
        result.notice_level = 'error'
        result.notice = result.cards['motion'].value + '. ' + result.cards['motion'].detail + '.'
    elif any(item.level == 'error' for item in result.sensors.values()):
        missing = ', '.join(name for name, item in result.sensors.items() if item.level == 'error')
        result.notice_level = 'error'
        result.notice = 'Dữ liệu đã cũ: ' + missing + '. Xem Debug để kiểm tra bản tin và kết nối cảm biến.'
    elif mode == 'navigation' and map_info and not tf_ok:
        result.notice_level = 'warning'
        result.notice = 'Map đã nạp nhưng thiếu TF. Mở RViz Navigation và đặt vị trí bằng 2D Pose Estimate.'
    elif data is None or not receiving:
        result.notice_level = 'warning'
        result.notice = 'Đã biết phiên robot nhưng chưa nhận dữ liệu ROS mới. Studio đang tự phục hồi kết nối.'
    elif mode == 'mapping':
        result.notice_level = 'info'
        result.notice = 'Mapping đang chạy. Mở RViz Map để xem quá trình quét; lưu map trước khi dừng phiên.'
    else:
        result.notice_level = 'info'
        result.notice = 'Theo dõi trạng thái Nav2 active trong RViz trước khi gửi đích điều hướng.'
    return result
