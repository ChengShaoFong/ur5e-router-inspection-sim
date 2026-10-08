"""集中讀取 task.ini；各演算法只取得自己使用的設定區段。"""

from configparser import ConfigParser
import json
from pathlib import Path
from types import SimpleNamespace


config = ConfigParser(interpolation=None)
path = Path(__file__).with_name("task.ini")
if not config.read(path, encoding="utf-8"):
    raise FileNotFoundError(path)


def section(name):
    """將 INI 的數值或陣列轉成 Python 值，並以大寫名稱提供給模組。"""
    return SimpleNamespace(**{
        key.upper(): json.loads(value) for key, value in config.items(name)
    })


task = section("task")
calibration = section("calibration")
control = section("control")
port_vision = section("port_vision")
adapter_vision = section("adapter_vision")
port_vision.SAMPLE_INTERVAL = port_vision.CAMERA_PERIOD_MS / 1000
adapter_vision.SAMPLE_INTERVAL = adapter_vision.CAMERA_PERIOD_MS / 1000
