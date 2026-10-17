"""Translation integrity and dialog boundaries without a Tk window or hardware."""
from pathlib import Path
import re
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ui_language import Dialogs, Language, english, normalize
from ui_translations import ENGLISH


class LanguageTests(unittest.TestCase):
    def test_invalid_or_missing_language_defaults_to_vietnamese(self):
        for value in (None, '', 'fr', [], {}):
            self.assertEqual(normalize(value), 'vi')

    def test_catalog_fields_are_retained_and_every_template_renders(self):
        for source, target in ENGLISH.items():
            fields = set(re.findall(r'\{(\d+)\}', source))
            self.assertEqual(fields, set(re.findall(r'\{(\d+)\}', target)), source)
            sample = lambda message: re.sub(r'\{(\d+)\}', lambda match: 'value_' + match[1], message)
            self.assertEqual(english(sample(source)), sample(target), source)

    def test_live_language_and_dynamic_errors_preserve_paths_and_values(self):
        language = Language()
        path = r'P:\Ứng dụng\firmware {new}\test.hex'
        self.assertEqual(language.text('Đã lưu: ' + path), 'Đã lưu: ' + path)
        language.select('en')
        self.assertEqual(language.text('Đã lưu: ' + path), 'Saved: ' + path)
        self.assertEqual(language.text('Lỗi: không thấy file'), 'Error: file not found')
        self.assertEqual(language.text('Sector 2: đang ghi  ·  0x0800 0000'), 'Sector 2: writing  ·  0x0800 0000')
        self.assertEqual(language.text('Map đã chọn: lab_01 (chưa nạp vào navigation)'),
                         'Selected map: lab_01 (not loaded into navigation)')
        for raw in ('/cmd_vel', 'robot.local', 'operator', path, 'Trần Nguyên Bình', 'stm32h7x reset_config none'):
            self.assertEqual(language.text(raw), raw)
        language.select('vi')
        self.assertEqual(language.text('Nạp firmware'), 'Nạp firmware')

    def test_dialog_translation_keeps_native_options_and_file_patterns(self):
        module = Mock()
        module.askyesno.return_value = False
        language = Language('en')
        dialogs = Dialogs(language, module)
        self.assertFalse(dialogs.askyesno('Dừng tác vụ robot?', 'Chọn tác vụ cần dừng trong danh sách.',
                                         default='no', icon='warning'))
        module.askyesno.assert_called_once_with('Stop robot tasks?', 'Select the tasks to stop in the list.',
                                               default='no', icon='warning')
        dialogs.askopenfilename(title='Chọn firmware', initialdir=r'P:\build', filetypes=[('Tất cả', '*.*')])
        module.askopenfilename.assert_called_once_with(title='Select firmware', initialdir=r'P:\build',
                                                       filetypes=[('All files', '*.*')])
        language.select('vi')
        dialogs.showinfo('Bản đồ', 'Chọn một map trong danh sách trước.')
        module.showinfo.assert_called_once_with('Bản đồ', 'Chọn một map trong danh sách trước.')


if __name__ == '__main__':
    unittest.main()
