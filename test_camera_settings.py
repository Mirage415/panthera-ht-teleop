import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import pyrealsense2 as rs
from camera_settings import apply_color_settings

class SettingsTests(unittest.TestCase):
    def test_manual_exposure_survives_mode_and_antiflicker_changes(self):
        values={rs.option.exposure:166.,rs.option.gain:64.,rs.option.enable_auto_exposure:1.,rs.option.enable_auto_white_balance:1.,rs.option.power_line_frequency:3.}
        def set_option(option,value):
            values[option]=value
            if option in (rs.option.enable_auto_exposure,rs.option.power_line_frequency):values[rs.option.exposure]=166.
        sensor=SimpleNamespace(get_info=lambda _: 'RGB Camera',supports=lambda _:True,
            get_option_range=lambda _:SimpleNamespace(min=0,max=10000),set_option=set_option,get_option=lambda o:values[o])
        device=SimpleNamespace(get_info=lambda _:'test-camera',query_sensors=lambda:[sensor])
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'settings.json';options=dict(exposure=200,gain=64,power_line_frequency=3,enable_auto_exposure=0,enable_auto_white_balance=1)
            p.write_text(json.dumps(dict(version=1,devices={'test-camera':{'color_options':options}})))
            self.assertEqual(apply_color_settings(device,p),options)
            device.get_info=lambda _:'another-camera'
            self.assertEqual(apply_color_settings(device,p),{})
    def test_invalid_options_rejected_before_writes(self):
        writes=[]
        sensor=SimpleNamespace(get_info=lambda _:'RGB Camera',supports=lambda _:True,
            get_option_range=lambda _:SimpleNamespace(min=0,max=10000),set_option=lambda *v:writes.append(v))
        device=SimpleNamespace(get_info=lambda _:'test',query_sensors=lambda:[sensor])
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'settings.json'
            p.write_text(json.dumps(dict(version=1,devices={'test':{'color_options':dict(exposure=200,gain=-1)}})))
            with self.assertRaises(ValueError):apply_color_settings(device,p)
            self.assertEqual(writes,[])
