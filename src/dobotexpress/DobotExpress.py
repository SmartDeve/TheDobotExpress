from . import dobot
import time


class DobotExpress:
    def __init__(self, PORT):
        self.api = dobot.DobotAPI().api
        self.PTPMode = dobot.PTPMode
        self.isConnected = False
        self.isLinearEnabled = False
        self.isColorEnabled = False
        self.isIREnabled = False
        connectResult = dobot.ConnectDobot(self.api, PORT, 115200)
        if connectResult['res'] == 0:
            print(
                f"Dobot Connected at {PORT} with Version {connectResult['ver']}")

            self.isConnected = True
            dobot.clearCommands(self.api)
            dobot.executeCommands(self.api)
        elif connectResult['res'] == 1:
            print("Dobot port is busy!")
        else:
            print("Something went wrong")

    def clearCommands(self):
        dobot.clearCommands(self.api)
        dobot.executeCommands(self.api)

    def __exit__(self, exc_type, exc_value, traceback):
        """Runs automatically when the 'with' block ends or crashes."""

        dobot.disconnectDobot(self.api.api)

    def __del__(self):
        """Failsafe: Runs when the variable is deleted or script ends."""

        dobot.disconnectDobot(self.api)

    def moveXYZ(self, x, y, z, r):
        res = dobot.move(self.api, x, y, z, r, dobot.PTPMode.PTPMOVJ_XYZ)
        dobot.wait(self.api, res)
        return res

    def moveJoints(self, j1, j2, j3, j4):
        res = dobot.move(self.api, j1, j2, j3, j4, dobot.PTPMode.PTPMOVJ_ANGLE)
        dobot.wait(self.api, res)
        return res

    def moveLinearRail(self, distance):
        dobot.enableLinearRail(self.api, True)
        res = dobot.moveLinearRail(self.api, distance)
        dobot.wait(self.api, res)
        return res

    def moveConyever(self, isEnabled, speed):
        res = dobot.moveConveyer(self.api, isEnabled, speed)
        dobot.wait(self.api, res)
        return res

    def setCartisianSpeed(self, vel, acc):
        res = dobot.setCartisianSpeed(self.api, vel, acc)
        dobot.wait(self.api, res)
        return res

    def setJointSpeed(self, vel, acc):
        res = dobot.setJointsSpeed(self.api, vel, acc)
        dobot.wait(self.api, res)
        return res

    def setLinearSpeed(self, vel, acc):
        res = dobot.setLinearRailSpeed(self.api, vel, acc)
        return res

    def getPosition(self):
        res = dobot.getPose(self.api)
        return res

    def getLinearRailPosition(self):

        isEnabled = dobot.getLinearRailState(self.api)
        if isEnabled:
            res = dobot.getLinearRailPose(self.api)
            return res
        else:
            return -99999999

    def setGripperState(self, enable, state):
        ret = dobot.setGripper(self.api, enable, state)
        dobot.wait(self.api, ret)
        return ret

    def setSuctionState(self, enable, state):
        ret = dobot.setSuction(self.api, enable, state)
        return ret

    def enableColorSensor(self, state, port):
        ret = dobot.enableColorSensor(self.api, state, port)
        self.isColorEnabled = True
        return ret

    def enableIRSensor(self, state, port):
        self.isIREnabled = True
        ret = dobot.enableIRsensor(self, port, 1)
        return ret

    def readColor(self):

        if not self.isColorEnabled:
            return "ENABLE COLOR"
        else:
            ret = dobot.readColorSensor(self.api)
            return ret

    def moveWithMode(self,PTPMode: dobot.PTPMode,a,b,c,d):
        res = dobot.move(self.api, a,b,c,d, PTPMode)
        dobot.wait(self.api, res)
        return res

    def readIRSensor(port,self):
        if not self.isIREnabled:
            return "ENABLE IR"
        else:
            ret = dobot.readIRSensor(port, self.api)
            return ret

    def setHomeCoordinates(self, x, y, z, r):
        self.home_x = x
        self.home_y = y
        self.home_z = z
        self.home_r = r

        ret = dobot.setHomePositon(self.api, x, y, z, r)
        dobot.wait(self.api, ret)
        return ret

    def goHome(self):

        dobot.clearCommands(self.api)
        ret_index = dobot.home(self.api)
        dobot.wait(self.api, ret_index)
        return ret_index

    def enableLinearRail(self, state):
        ret = dobot.enableLinearRail(self.api, True)

    def makeToWait(self, index):
        dobot.wait(self.api, index)
