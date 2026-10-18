"""Author details from the owner's website plus the app's own icon; no runtime site dependency."""
from pathlib import Path

ASSETS = Path(__file__).with_name('assets')
APP_NAME = 'RPBot Studio'
AUTHOR = 'Trần Nguyên Bình'
AUTHOR_EN = 'Nguyen-Binh Tran'
WEBSITE = 'https://nguyenbinh-shark.github.io'
GITHUB = 'https://github.com/nguyenbinh-shark'
EMAIL = 'trannguyenbinh.shark@gmail.com'
BIO = 'Cơ điện tử · Hệ thống điều khiển · Robotics'
LOGO = ASSETS / 'rpbot-studio.png'  # 40 px header mark, drawn by tools/make_app_icons.py
ICON = ASSETS / 'rpbot-studio.ico'
ICON_PNG = ASSETS / 'rpbot-studio-256.png'  # Linux window and launcher icon
