import ctypes
import os
import time
import sys
from enum import IntEnum
# 1. DEFINE THE DANGEROUS FOLDER
# We point to it, but we don't run inside it.

dobot_studio_path = os.path.dirname(os.path.abspath(__file__))

try:
    os.add_dll_directory(dobot_studio_path)
except AttributeError:
    # Fallback for older Python versions
    os.environ['PATH'] = dobot_studio_path + ';' + os.environ['PATH']

# 3. LOAD THE MAIN DLL
dll_path = os.path.join(dobot_studio_path, "DobotDll.dll")
print(f"Loading from: {dll_path}")


class DobotAPI:
    def __init__(self):

        try:
            # Load the library
            self.api = ctypes.CDLL(dll_path)
            print("SUCCESS: DLL Loaded! No conflicts.")
        except OSError as e:
            print(f"Error loading: {e}")


BAUD_RATE = 115200
PORT_NAME = ""


class DobotError(Exception):
    pass


def clearCommands(api):
    api.SetQueuedCmdStopExec()
    api.SetQueuedCmdClear()


def disconnectDobot(api):
    api.DisconnectDobot.argtypes = []
    api.DisconnectDobot.restype = None


class DobotConnect(IntEnum):
    DobotConnect_NoError = 0
    DobotConnect_NotFound = 1
    DobotConnect_Occupied = 2


def ConnectDobot(api, portName,  baudrate):
    api.ConnectDobot.argtypes = [
        ctypes.c_char_p,  # portName
        ctypes.c_int,     # baudrate
        ctypes.c_char_p,  # fwType (output buffer)
        ctypes.c_char_p   # version (output buffer)
    ]

    api.ConnectDobot.restype = ctypes.c_int
    szPara = ctypes.create_string_buffer(100)
    szPara.raw = portName.encode("utf-8")
    fwType = ctypes.create_string_buffer(100)
    version = ctypes.create_string_buffer(100)
    result = api.ConnectDobot(szPara,  baudrate,  fwType,  version)

    if result is 0:
        return {"res": result, "frm": fwType.value.decode("utf-8"), "ver":  version.value.decode("utf-8")}
    elif result is DobotConnect.DobotConnect_NotFound:
        raise DobotError("Dobot Not Found")
    elif result is DobotConnect.DobotConnect_Occupied:
        raise DobotError("Dobot is busy, check serial port!")
# --- STEP 3: CONFIGURE MOTION PARAMETERS ---

# 1. DEFINE THE C STRUCTURES
# These tell Python how to format the data so the C DLL understands it.


class PTPJointParams(ctypes.Structure):
    _fields_ = [("velocity", ctypes.c_float * 4),      # Array of 4 floats
                ("acceleration", ctypes.c_float * 4)]   # Array of 4 floats


class PTPCoordinateParams(ctypes.Structure):
    _fields_ = [("xyzVelocity", ctypes.c_float),
                ("rVelocity", ctypes.c_float),
                ("xyzAcceleration", ctypes.c_float),
                ("rAcceleration", ctypes.c_float)]


class PTPJumpParams(ctypes.Structure):
    _fields_ = [("jumpHeight", ctypes.c_float),
                ("zLimit", ctypes.c_float)]


class PTPCommonParams(ctypes.Structure):
    _fields_ = [("velocityRatio", ctypes.c_float),
                ("accelerationRatio", ctypes.c_float)]

# 2. DEFINE FUNCTION SIGNATURES
# Common arguments for all: (StructPointer, isQueued, QueuedIndexPointer)


def setCartisianSpeed(api, velocity, acceleration):
    api.SetPTPCoordinateParams.argtypes = [ctypes.POINTER(
        PTPCoordinateParams), ctypes.c_bool, ctypes.POINTER(ctypes.c_uint64)]
    api.SetPTPCoordinateParams.restype = ctypes.c_int
    queuedCmdIndex = ctypes.c_uint64(0)
    coord_params = PTPCoordinateParams()
    coord_params.xyzVelocity = float(velocity)
    coord_params.rVelocity = float(velocity)
    coord_params.xyzAcceleration = float(acceleration)
    coord_params.rAcceleration = float(acceleration)
    api.SetPTPCoordinateParams(ctypes.byref(
        coord_params), False, ctypes.byref(queuedCmdIndex))
    return queuedCmdIndex.value


class PTPJointParams(ctypes.Structure):
    _fields_ = [
        ("velocity", ctypes.c_float * 4),
        ("acceleration", ctypes.c_float * 4)
    ]


def setJointsSpeed(api, velocity, acc):
    # Define Signature
    api.SetPTPJointParams.argtypes = [
        ctypes.POINTER(PTPJointParams),
        ctypes.c_bool,
        ctypes.POINTER(ctypes.c_uint64)
    ]
    api.SetPTPJointParams.restype = ctypes.c_int

    # Create Structure
    joint_params = PTPJointParams()
    # Fill arrays: [J1, J2, J3, J4]
    joint_params.velocity[:] = [float(velocity)] * 4
    joint_params.acceleration[:] = [float(acc)] * 4

    # Create Index Bucket
    queuedCmdIndex = ctypes.c_uint64(0)

    # Send Command (isQueued = True)
    ret = api.SetPTPJointParams(
        ctypes.byref(joint_params),
        True,
        ctypes.byref(queuedCmdIndex)
    )

    if ret == 0:
        # EXECUTE QUEUE
        api.SetQueuedCmdStartExec()
        print(f">>> Joint Speed Set to {velocity}/{acc}")
        return queuedCmdIndex.value  # Return the integer ID
    else:
        print(f">>> ERROR: Failed to set joint speed (Code {ret})")
        return 0


def setup(api):
    api.SetPTPJointParams.argtypes = [ctypes.POINTER(
        PTPJointParams), ctypes.c_bool, ctypes.POINTER(ctypes.c_uint64)]
    api.SetPTPCoordinateParams.argtypes = [ctypes.POINTER(
        PTPCoordinateParams), ctypes.c_bool, ctypes.POINTER(ctypes.c_uint64)]
    api.SetPTPJumpParams.argtypes = [ctypes.POINTER(
        PTPJumpParams), ctypes.c_bool, ctypes.POINTER(ctypes.c_uint64)]
    api.SetPTPCommonParams.argtypes = [ctypes.POINTER(
        PTPCommonParams), ctypes.c_bool, ctypes.POINTER(ctypes.c_uint64)]

    queuedCmdIndex = ctypes.c_uint64(0)
    joint_params = PTPJointParams()
    joint_params.velocity[:] = [50, 50, 50, 50]
    joint_params.acceleration[:] = [50, 50, 50, 50]

    api.SetPTPJointParams(ctypes.byref(joint_params), False,
                          ctypes.byref(queuedCmdIndex))

    coord_params = PTPCoordinateParams()
    coord_params.xyzVelocity = 50.0
    coord_params.rVelocity = 50.0
    coord_params.xyzAcceleration = 50.0
    coord_params.rAcceleration = 50.0

    api.SetPTPCoordinateParams(ctypes.byref(
        coord_params), False, ctypes.byref(queuedCmdIndex))

    common_params = PTPCommonParams()
    common_params.velocityRatio = 100.0
    common_params.accelerationRatio = 100.0

    api.SetPTPCommonParams(ctypes.byref(common_params),
                           False, ctypes.byref(queuedCmdIndex))


class Pose(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("r", ctypes.c_float),
        # Array of 4 floats for J1, J2, J3, J4
        ("jointAngle", ctypes.c_float * 4)
    ]


def getPose(api):
    api.GetPose.argtypes = [ctypes.POINTER(Pose)]
    api.GetPose.restype = ctypes.c_int

    print("Reading current position...")
    current_pose = Pose()
    # 4. CALL THE FUNCTION
    # We pass 'byref' (pointer) so the DLL can write the data INTO our object.
    result = api.GetPose(ctypes.byref(current_pose))

    if result == 0:  # 0 usually means success for GetPose
        return {
            "x": current_pose.x,
            "y": current_pose.y,
            "z": current_pose.z,
            "r": current_pose.r,
            "j1": current_pose.jointAngle[0],
            "j2": current_pose.jointAngle[1],
            "j3": current_pose.jointAngle[2],
            "j4": current_pose.jointAngle[3]
        }
    else:
        raise DobotError("Failed to get Pose, there is something wrong")


# 2. DEFINE FUNCTION SIGNATURE
# Arguments: (Pointer to Struct, isQueued, Pointer to Index)
class HOMECmd(ctypes.Structure):
    _fields_ = [("temp", ctypes.c_int)]


def home(api):

    temp_val = 0
    home_cmd = HOMECmd()
    home_cmd.temp = temp_val
    queuedCmdIndex = ctypes.c_uint64(0)
    api.SetHOMECmd.argtypes = [ctypes.POINTER(
        HOMECmd), ctypes.c_bool, ctypes.POINTER(ctypes.c_uint64)]
    api.SetHOMECmd.restype = ctypes.c_int
    ret = api.SetHOMECmd(
        ctypes.byref(home_cmd),
        True,                   # isQueued = 1
        ctypes.byref(queuedCmdIndex)
    )
    if ret == 0:
        executeCommands(api)
        return queuedCmdIndex.value
    else:
        raise DobotError("Couldn't Home")


def executeCommands(api):
    api.SetQueuedCmdStartExec.argtypes = []
    api.SetQueuedCmdStartExec.restype = ctypes.c_int
    api.SetQueuedCmdStartExec()


def setGripper(api, enable, state):
    api.SetEndEffectorGripper.argtypes = [
        ctypes.c_bool,                   # enableCtrl (Must be True to work)
        # enable (True=ON/Suck, False=OFF/Release)
        ctypes.c_bool,
        ctypes.c_bool,                   # isQueued
        ctypes.POINTER(ctypes.c_uint64)  # cmdIndex
    ]
    api.SetEndEffectorGripper.restype = ctypes.c_int
    queuedCmdIndex = ctypes.c_uint64(0)
    ret = api.SetEndEffectorGripper(
        enable, state, True, ctypes.byref(queuedCmdIndex))

    if ret == 0:
        executeCommands(api)
        return queuedCmdIndex.value
    else:
        raise DobotError("Failed to set gripper")


def setSuction(api, enable, state):
    api.SetEndEffectorSuctionCup.argtypes = [
        ctypes.c_bool,                   # enableCtrl (Must be True to work)
        # enable (True=ON/Suck, False=OFF/Release)
        ctypes.c_bool,
        ctypes.c_bool,                   # isQueued
        ctypes.POINTER(ctypes.c_uint64)  # cmdIndex
    ]
    api.SetEndEffectorSuctionCup.restype = ctypes.c_int
    queuedCmdIndex = ctypes.c_uint64(0)
    ret = api.SetEndEffectorSuctioCup(
        enable, state, True, ctypes.byref(queuedCmdIndex))
    if ret == 0:
        executeCommands(api)
        return queuedCmdIndex.value
    else:
        raise DobotError("Failed to setSuction")


class PTPMode(IntEnum):
    PTPJUMP_XYZ = 0      # Jump (Lift up, move over, drop down)
    PTPMOVJ_XYZ = 1      # Joint Move (Fastest, curved path)
    PTPMOVL_XYZ = 2      # Linear Move (Straight line, slower)
    PTPJUMP_ANGLE = 3    # Jump but using Joint Angles (J1, J2...)
    PTPMOVJ_ANGLE = 4    # Joint Move using Joint Angles
    PTPMOVL_ANGLE = 5    # Linear Move using Joint Angles
    PTPMOVJ_INC = 6      # Relative Move (Move "Distance" from current)
    PTPMOVL_INC = 7
    PTPJUMPL_XYZ = 9


class PTPCmd(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("ptpMode", ctypes.c_byte),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("rHead", ctypes.c_float),
        ("gripper", ctypes.c_float)
    ]

# --- DEFINE RAIL PARAMETERS CLASS ---


class PTPLParams(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("velocity", ctypes.c_float),      # Rail Speed
        ("acceleration", ctypes.c_float)   # Rail Acceleration
    ]


class PTPWithLCmd(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("ptpMode", ctypes.c_byte),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("rHead", ctypes.c_float),
        ("l", ctypes.c_float)
    ]


def move(api, x, y, z, r, MODE):
    api.SetPTPCmd.argtypes = [
        ctypes.POINTER(PTPCmd),
        ctypes.c_bool,
        ctypes.POINTER(ctypes.c_uint64)
    ]
    api.SetPTPCmd.restype = ctypes.c_int

    queuedCmdIndex = ctypes.c_uint64(0)

    cmd = PTPCmd()
    cmd.ptpMode = MODE
    cmd.x = float(x)
    cmd.y = float(y)
    cmd.z = float(z)
    cmd.rHead = float(r)
    cmd.gripper = 0

    clearCommands(api)
    ret = api.SetPTPCmd(ctypes.byref(cmd), True,
                        ctypes.byref(queuedCmdIndex))

    if ret != 0:
        raise DobotError("Failed to move the dobot")

    executeCommands(api)

    return queuedCmdIndex.value


class EMotorExCmd(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("index", ctypes.c_int),    # 0 = Stepper1, 1 = Stepper2
        ("isEnabled", ctypes.c_bool),  # True/False
        ("speed", ctypes.c_int32)   # Speed in pulses/sec
    ]


class EMotor(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("index", ctypes.c_byte),
        ("isEnabled", ctypes.c_byte),
        ("speed", ctypes.c_int32)
    ]


def moveConveyer(api, is_enabled, speed):
    # Calculate velocity (Keep your existing math)
    STEP_PER_CRICLE = 360.0 / 1.8 * 10.0 * 16.0
    MM_PER_CRICLE = 3.1415926535898 * 36.0
    vel = float(speed) * STEP_PER_CRICLE / MM_PER_CRICLE

    # 2. Create the Structure Object
    emotor = EMotor()
    emotor.index = 0        # 0 for Port E1, 1 for Port E2
    emotor.isEnabled = 1 if is_enabled else 0
    emotor.speed = int(vel)  # Must be Integer

    # 3. Configure the DLL Function Signature
    # It expects: (Pointer to EMotor, Boolean isQueued, Pointer to index)
    api.SetEMotor.argtypes = [
        ctypes.POINTER(EMotor),
        ctypes.c_bool,
        ctypes.POINTER(ctypes.c_uint64)
    ]
    api.SetEMotor.restype = ctypes.c_int

    # 4. Call the function using the Structure
    queuedCmdIndex = ctypes.c_uint64(0)

    # Note: 'byref(emotor)' passes the structure pointer
    ret = api.SetEMotor(ctypes.byref(emotor), False,
                        ctypes.byref(queuedCmdIndex))

    if ret != 0:
        raise DobotError(f"SetEMotor failed with code {ret}")

    return ret


def enableLinearRail(api, state):
    api.SetDeviceWithL.argtypes = [
        ctypes.c_bool,
        ctypes.c_int,    # <--- This is VERSION, not isQueued
        ctypes.POINTER(ctypes.c_uint64)
    ]
    api.SetDeviceWithL.restype = ctypes.c_int
    queuedCmdIndex = ctypes.c_uint64(0)
    ret = api.SetDeviceWithL(state, 0, ctypes.byref(queuedCmdIndex))

    if ret == 0:
        executeCommands(api)
        return queuedCmdIndex.value
    else:
        raise DobotError("Couldn't enable linear rail!")


def setLinearRailSpeed(api, speed, acceleration):
    api.SetPTPLParams.argtypes = [ctypes.POINTER(
        PTPLParams), ctypes.c_bool, ctypes.POINTER(ctypes.c_uint64)]
    api.SetPTPLParams.restype = ctypes.c_int
    queuedCmdIndex = ctypes.c_uint64(0)
    l_params = PTPLParams()
    l_params.velocity = speed     # Speed
    l_params.acceleration = acceleration  # Acceleration
    ret = api.SetPTPLParams(ctypes.byref(l_params), True,
                            ctypes.byref(queuedCmdIndex))

    if ret == 0:
        executeCommands(api)
        return queuedCmdIndex.value
    else:
        raise DobotError("Couldn't set Linear Rail speed!")


def moveLinearRail(api, distance):
    positions = getPose(api)
    print(positions)
    queuedCmdIndex = ctypes.c_uint64(0)
    moveCmd = PTPWithLCmd()
    moveCmd.ptpMode = PTPMode.PTPMOVJ_XYZ
    moveCmd.x = positions['x']
    moveCmd.y = positions['y']
    moveCmd.z = positions['z']
    moveCmd.r = positions['r']
    moveCmd.l = distance
    clearCommands(api)
    ret = api.SetPTPWithLCmd(ctypes.byref(
        moveCmd), True, ctypes.byref(queuedCmdIndex))

    if ret == 0:
        executeCommands(api)
        return queuedCmdIndex.value
    else:
        raise DobotError("Couldn't set Linear Rail speed!")


class HOMEParams(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("r", ctypes.c_float)
    ]


def setHomePositon(api, x, y, z, r):
    api.SetHOMEParams.argtypes = [
        ctypes.POINTER(HOMEParams),
        ctypes.c_bool,
        ctypes.POINTER(ctypes.c_uint64)
    ]
    api.SetHOMEParams.restype = ctypes.c_int
    queuedCmdIndex = ctypes.c_uint64(0)

    # Create the struct and fill it
    home_params = HOMEParams()
    home_params.x = float(x)
    home_params.y = float(y)
    home_params.z = float(z)
    home_params.r = float(r)

    ret = api.SetHOMEParams(
        ctypes.byref(home_params),
        True,
        ctypes.byref(queuedCmdIndex)
    )

    if ret == 0:
        executeCommands(api)
        return queuedCmdIndex.value
    else:
        raise DobotError("Couldn't set Home Coordinates")


def getLinearRailState(api):
    is_enable = ctypes.c_bool(False)
    ret = api.GetDeviceWithL(ctypes.byref(is_enable))
    return ret


def getLinearRailPose(api):
    l = ctypes.c_float(0)
    ret = api.GetPoseL(ctypes.byref(l))
    return l.value


def enableColorSensor(api, state, portNum):
    api.SetColorSensor.argtypes = [
        ctypes.c_bool,
        ctypes.c_int,
        ctypes.c_bool
    ]
    queuedCmdIndex = ctypes.c_uint64(0)
    api.SetColorSensor.restype = ctypes.c_int

    queuedCmdIndex = ctypes.c_uint64(0)
    port_map = {1: 0, 2: 1, 4: 2, 5: 3}
    internal_id = port_map[portNum]

    ret = api.SetColorSensor(state, internal_id, True,
                             ctypes.byref(queuedCmdIndex))

    if ret == 0:
        executeCommands(api)
        return queuedCmdIndex.value
    else:
        raise DobotError("Couldn't set Color Sensor")


def readColorSensor(api):
    api.GetColorSensor.argtypes = [
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.POINTER(ctypes.c_ubyte)
    ]

    api.GetColorSensor.restype = ctypes.c_int

    r = ctypes.c_ubyte()
    g = ctypes.c_ubyte()
    b = ctypes.c_ubyte()

    ret = api.GetColorSensor(
        ctypes.byref(r),
        ctypes.byref(g),
        ctypes.byref(b)
    )

    if ret == 0:
        return {
            "R": r.value,
            "G": g.value,
            "B": b.value
        }
    else:
        return None


def enableIRsensor(isEnable, portNum, version=1):
    port_map = {1: 0, 2: 1, 4: 2, 5: 3}
    if portNum not in port_map:
        print(f">>> ERROR: Invalid Port GP{portNum}. Use 1, 2, 4, or 5.")
        return

    internal_id = port_map[portNum]
    print(f"Enabling IR Sensor on GP{portNum} (Version {version})...")

    # CALL WITH 3 ARGUMENTS
    ret = api.SetInfraredSensor(isEnable, internal_id, version)

    if ret == 0:
        print(">>> SUCCESS: IR Sensor Enabled.")
    else:
        print(f">>> ERROR: Enable failed (Code {ret})")


def readIRSensor(api, portNum):
    port_map = {1: 0, 2: 1, 4: 2, 5: 3}
    internal_id = port_map[portNum]

    val = ctypes.c_ubyte(0)
    ret = api.GetInfraredSensor(internal_id, ctypes.byref(val))

    return ret


def wait(api, target_index):
   # If the command failed to queue (index 0), don't wait forever.
    if target_index == 0:
        print("Warning: Wait called on Index 0 (Command failed?)")
        return

    api.GetQueuedCmdCurrentIndex.argtypes = [ctypes.POINTER(ctypes.c_uint64)]
    api.GetQueuedCmdCurrentIndex.restype = ctypes.c_int

    current_index = ctypes.c_uint64(0)
    print(f"Current index{current_index.value} and target index{target_index}")
    while True:
        ret = api.GetQueuedCmdCurrentIndex(ctypes.byref(current_index))
        if ret != 0:
            print("I am free...pata nahi kyu")
            print(
                f"Current index{current_index.value} and target index{target_index}")
            break  # Error reading

        if current_index.value >= target_index:
            print("I am free...")
            print(
                f"Current index{current_index.value} and target index{target_index}")
            break

        time.sleep(0.1)
