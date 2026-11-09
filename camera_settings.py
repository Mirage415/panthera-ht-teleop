"""Restore per-device RGB settings on every camera pipeline connection."""
import json
import math
from pathlib import Path
import pyrealsense2 as rs

DEFAULT_PATH = Path(__file__).with_name('camera-settings.json')
ALLOWED = ('enable_auto_exposure','exposure','gain','enable_auto_white_balance',
           'white_balance','brightness','contrast','saturation','gamma','hue',
           'backlight_compensation','power_line_frequency','sharpness')

def apply_color_settings(device, path=DEFAULT_PATH):
    path=Path(path)
    if not path.exists():return {}
    config=json.loads(path.read_text())
    if config.get('version')!=1:raise ValueError('Unsupported camera settings version')
    serial=device.get_info(rs.camera_info.serial_number)
    values=config.get('devices',{}).get(serial,{}).get('color_options',{})
    if not values:return {}
    sensor=next((s for s in device.query_sensors() if 'RGB' in s.get_info(rs.camera_info.name)),None)
    if sensor is None:raise ValueError('Configured RGB sensor not found')
    for name,value in values.items():
        if name not in ALLOWED or type(value) not in (int,float) or not math.isfinite(value):
            raise ValueError('Invalid camera option: '+name)
        option=getattr(rs.option,name)
        if not sensor.supports(option):raise ValueError('Unsupported camera option: '+name)
        bounds=sensor.get_option_range(option)
        if not bounds.min<=value<=bounds.max:raise ValueError('Camera option out of range: '+name)
    # Disable automatic controllers first. Set gain/exposure/WB after other
    # image controls: changing anti-flicker or AE can restore default exposure.
    automatic=('enable_auto_exposure','enable_auto_white_balance')
    manual=('gain','exposure','white_balance')
    order=([n for n in automatic if values.get(n)==0]+
           [n for n in values if n not in automatic+manual]+
           [n for n in manual if n in values]+
           [n for n in automatic if values.get(n)==1])
    for name in order:sensor.set_option(getattr(rs.option,name),float(values[name]))
    actual={name:sensor.get_option(getattr(rs.option,name)) for name in values}
    for name in automatic+manual:
        if name in values and actual[name]!=values[name]:raise RuntimeError('Camera setting did not persist: '+name)
    return actual
