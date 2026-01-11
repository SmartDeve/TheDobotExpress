import platform
import struct

if platform.system() != "Windows":
    raise RuntimeError("Dobot SDK is supported only on Windows.")

if struct.calcsize("P") * 8 != 32:
    raise RuntimeError("32-bit Python required.")

from .DobotExpress import DobotExpress
from .manager import DobotManager
from .proxy import DobotProxy
