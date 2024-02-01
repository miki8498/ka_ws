#! /usr/bin/env python3

from pymodbus.client.sync import ModbusSerialClient # pip3 install pymodbus==1.3.2
from pymodbus.exceptions import ModbusIOException
from math import ceil
import numpy as np
from robotiq_control.GripperCommon import Robotiq

GOAL_DETECTION_THRESHOLD = 0.01 # Max deviation from target goal to consider as goal "reached"




class RobotiqCommunication(ModbusSerialClient, Robotiq):
    def __init__(self, gripper_type, device_id=0, com_port='/dev/ttyUSB0',baud=115200):
        ModbusSerialClient.__init__(self, method='rtu',port = com_port ,stopbits=1, bytesize=8, baudrate=baud, timeout=0.2)
        Robotiq.__init__(self, gripper_type)
        self.debug = False
        self.gripper_type = gripper_type
        self.com_port = com_port
        self.device_id = device_id+9
        
        self._error = []
        #initialize_communication_variables
        # Out
        self.rPR = 0
        self.rSP = 255
        self.rFR = 150
        self.rARD = 1
        self.rATR = 0
        self.rGTO = 0
        self.rACT = 0
        # In
        self.gSTA = 0
        self.gACT = 0
        self.gGTO = 0
        self.gOBJ = 0
        self.gFLT = 0
        self.gPO = 0
        self.gPR = 0
        self.gCU = 0
        self._update_cmd()
        self._max_force = 100.0 # [%]

        self.is_gripper_connected = False

    def gripperConnect(self):
        if self.connect(): #Returns:	True if connection succeeded, False otherwise
            print("Connected to {}".format(self.com_port))
            return True
        else:
            print("Unable to connect to {}".format(self.com_port))
            return False
        

    def checkConnection(self):
        self.is_gripper_connected = self.is_socket_open()

        if not self.is_gripper_connected:
            self._updateError("Connection to port{}Loss".format(self.com_port))
            if self.close():
                print("Socket Closed")
        
        return self.is_gripper_connected


    def disconnectFromDevice(self):
        """Close connection"""
        self.close()

    def _updateError(self, error):
        self._error.append(error)

    def __readGripperRegisters(self, numBytes):
        numRegs = int(ceil(numBytes/2.0))


        try:
            response = self.read_holding_registers(0x07D0, numRegs, unit=0x0009)
        except Exception as e:
            print(e)
            return None


        # When reading failes, response is of type None 
        if isinstance(response,ModbusIOException):
            return None

        status = []

        #Fill the output with the bytes in the appropriate order
        for i in range(0, numRegs):
            status.append((response.getRegister(i) & 0xFF00) >> 8)
            status.append( response.getRegister(i) & 0x00FF)
            

        try:
            self.gACT = (status[0] >> 0) & 0x01        
            self.gGTO = (status[0] >> 3) & 0x01
            self.gSTA = (status[0] >> 4) & 0x03
            self.gOBJ = (status[0] >> 6) & 0x03
            self.gFLT =  status[2]
            self.gPR  =  status[3]
            self.gPO  =  status[4]
            self.gCU  =  status[5]
        except Exception as e:
            print(e)
        
        return True

    def __sendCommand(self):   
        """Send a command to the Gripper - the method takes a list of uint8 as an argument. The meaning of each variable depends on the Gripper model (see support.robotiq.com for more details)"""
        #make sure data has an even number of elements   
        if(len(self.message) % 2 == 1):
            self.message.append(0)

        #Initiate message as an empty list
        pkg_to_send = []
       

        #Fill message by combining two bytes in one register
        for i in range(0, int(len((self.message))/2)):
            pkg_to_send.append((self.message[2*i] << 8) + self.message[2*i+1])
            
        #To do!: Implement try/except 
        try:
            self.write_registers(0x03E8, pkg_to_send, unit=0x0009)
            
        except:
            print("Modbus write operation failure")
            return False
        return True

    def _update_cmd(self):

         #Initiate command as an empty list
        self.message = []
        #Build the command with each output variable
        self.message.append(self.rACT + (self.rGTO << 3) + (self.rATR << 4))
        self.message.append(0)
        self.message.append(0)
        self.message.append(self.rPR)
        self.message.append(self.rSP)
        self.message.append(self.rFR)


    def sendUnmonitoredMotionCmd(self, pos, speed, force):
        self.rACT = 1
        self.rGTO = 1
        self.rPR = super().getPositionRequest(pos)
        self.rSP = int(np.clip(255./(0.1 - 0.013) * speed-0.013, 0, 255))
        self.rFR = int(np.clip(255./(self._max_force) * force, 0, 255))
        self._update_cmd()
        return self.__sendCommand()
    
    def setActivationConfig(self):
        self.rACT = 1
        self.rPR  = 0
        self.rSP  = 255
        self.rFR  = 150

    def activate_gripper(self):
        self.setActivationConfig()
        self._update_cmd()
        return self.__sendCommand()

    def deactivate_gripper(self):
        self.rACT = 0
        self._update_cmd()
        return self.__sendCommand()

    def activate_emergency_release(self,open_gripper=True):
        self.rATR = 1
        self.rARD = 1

        if (open_gripper):
            self.rARD=0
        self._update_cmd()
        self.__sendCommand(self.message)
                
    def deactivate_emergency_release(self):
        self.rATR = 0
        self._update_cmd()

    def stop(self):
        self.rACT = 1
        self.rGTO = 0
        self._update_cmd()
        return self.__sendCommand(self.message)
                
    def is_ready(self):
        self.__readGripperRegisters(6)
        if self.debug:
            print(self.gSTA, self.gACT)
        return self.gSTA == 3 and self.gACT == 1

    def is_reset(self):
        self.__readGripperRegisters(6)
        return self.gSTA == 0 or self.gACT == 0

    def is_moving(self):
        self.__readGripperRegisters(6)
        return self.gGTO == 1 and self.gOBJ == 0

    def is_stopped(self):
        self.__readGripperRegisters(6)
        return self.gOBJ != 0

    def object_detected(self):
        self.__readGripperRegisters(6)
        return self.gOBJ == 1 or self.gOBJ == 2

    def get_fault_status(self):
        self.__readGripperRegisters(6)
        return self.gFLT

    def get_pos(self):
        self.__readGripperRegisters(6)
        po = float(self.gPO)
        return np.clip(self.stroke/(3.-230.)*(po-230.), 0, self.stroke)

    def get_req_pos(self):
        self.__readGripperRegisters(6)
        pr = float(self.gPR)
        return np.clip(self.stroke/(3.-230.)*(pr-230.), 0, self.stroke)

    def get_current(self):
        self.__readGripperRegisters(6)
        return self.gCU * 0.1


if __name__ == "__main__":
    gripperComm = RobotiqCommunication()


    gripperComm.gripperConnect()

    # if gripperComm.is_gripperConnected():
    #     print("daje")
    #     iter = 0 
    #     while not gripperComm.is_ready():
    #         success = gripperComm.deactivate_gripper()
    #         if gripperComm.is_reset():
    #             success = gripperComm.activate_gripper()
    #         iter = iter +1
    #         print(iter)
    #         time.sleep(2)
        
    # gripperComm.goto(pos=0.1, speed=0.01, force=100)

        
        


'''

It is possible to select which gripper to send the command to by using the command sid followed by the gripper id.
ex

sid2 (will send commands to gripper with ID 2

Note that ID1 in polyscope represent ID9 in socket communication(default)

Here's the full list 
drivergripper SET/GET commands SET commands : 

 ACT activateRequest 
 MOD gripperMode 
 GTO goto 
 ATR automaticReleaseRoutine 
 ARD autoreleaseDirection 
 MSC maxPeakSupplyCurrent 
 POS positionRequest 
 SPE speedRequest 
 FOR forceRequest 
 SCN_BLOCK scanBlockRequest 
 SCN scanRequest 
 NID updateGripperSlaveId 
 SID socketSlaveId 

 GET commands : 
 ACT activateRequest 
 MOD gripperMode 
 GTO goto 
 STA status 
 VST vacuumStatus 
 OBJ objectDetected 
 FLT fault 
 MSC maxPeakSupplyCurrent 
 PRE positionRequestEcho 
 POS positionRequest 
 COU motorCurrent 
 SNU serialNumber 
 PYE productionYear 
 NCY numberOfCycles 
 PON numberOfSecondsPumpIsOn 
 NPA numberOfPumpActivations 
 FWV firmwareVersion 
 VER driverVersion 
 SPE speedRequest
 FOR forceRequest 
 DRI printableState
 SID socketSlaveId
 
'''