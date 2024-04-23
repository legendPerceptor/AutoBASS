import os
import time
import json
import logging
import threading
from tkinter import messagebox
from MecademicRobot import RobotController
from rLinePipette import PipetteController

# Creat logging instant
logger = logging.getLogger('PipettingRobot')

PATH = os.path.dirname(__file__)

# IP address of robots
PIPROB_HOST = "192.168.31.236"

# rLine port
PIP_PORT = 'COM5'

# Tool variable
PIPETTE = 1
MAGNET = 2

os.chdir(os.path.join(PATH, 'data'))

# Get latest constant values from config file
with open('config.json') as json_file:
    CONSTANT = json.load(json_file)

# Piprobot constant
POST = CONSTANT['PIPROB']['POST_PO'] # Post position
HOME = CONSTANT['PIPROB']['WAIT_J'] # Home + Standby position
ONE_DROP = CONSTANT['ONE_DROP'] # Volume to form one droplet
ALL = True

class PipRobot(RobotController, PipetteController):

    def __init__(self, robot_address=PIPROB_HOST, pippette_port=PIP_PORT, pause_control:threading.Event=None, abort_control:threading.Event=None):
        RobotController.__init__(self, robot_address)
        PipetteController.__init__(self, pippette_port)
        self._move_on = pause_control
        self._abort = abort_control
        self._prep_done = threading.Event()
        self._add_done = threading.Event()
        self._add_done.set()
        self.status = dict(Progress=dict(Initiate=0), Initiated=False, Aborted=False, Cap_Captured=False, Tip_On=False, Tool=None)

    def piprob_is_online(self):
        try:
            rob_res = self.GetStatusRobot()
        except:
            return False
        else:
            rline_res = self.check_connection()
            check = (rob_res["Activated"], rob_res["Homing"], not rob_res["Error"], rline_res)
            return all(check)

    def initiate_piprobot(self):
        # Initilize status value
        rob_res = dict(Connection=False, Activation=False, Homing=False, PipRobot=False)

        # Activate robot, home, reset joints, apply parameters
        logger.info('Initiating Pipetting Robot...')
        conRes = self.connect()
        time.sleep(0.1)
        if conRes is True:
            # Update status
            self.status["Progress"]["Initiate"] = round(1/4*100)
            rob_res.update(dict(Connection=conRes))
            actRes = self.ActivateRobot()
            time.sleep(0.1)
            if actRes == 'Motors activated.':
                # Update status
                self.status["Progress"]["Initiate"] = round(2/4*100)
                rob_res.update(dict(Activation=True))
                homRes = self.home()
                time.sleep(0.1)
                if homRes == 'Homing done.':
                    # Update status
                    self.status["Progress"]["Initiate"] = round(3/4*100)
                    rob_res.update(dict(Homing=True))
                    # Set robot's parameter
                    self.SetCartLinVel(CONSTANT['L_VEL'])
                    self.SetJointVel(CONSTANT['J_VEL'])
                    self.SetJointAcc(20)
                    self.SetConf(1,1,-1)
                    self.MoveJoints(*HOME)
                    time.sleep(0.01)
                    logger.info('Pipetting Robot initiated.')
                    self.initiate_rline()
                    pip_res = self.check_connection()
                    rob_res.update(dict(PipRobot=pip_res))
                    logger.info('rLinePipette online.')
                    self.status["Progress"]["Initiate"] = round(4/4*100)
                else:
                    logger.warning('Pipetting Robot already homed!')
            else:
                logger.warning('Pipetting Robot already activated!')
        elif conRes == 'Another user is already connected, closing connection':
            logger.error('Pipetting Robot already in connection!')
        else:
            logger.critical('Pipetting Robot is not in connection. Check Power buttom')

        return rob_res

    def set_tool(self, tool):
        if tool == PIPETTE:
            # Set Pipeting tool
            self.SetTRF(107,2,22,0,-90,0)
            self.status['Tool'] = PIPETTE
        if tool == MAGNET:
            # Set magnet tool
            self.SetTRF(35.25,61,4,0,0,60)
            self.status['Tool'] = MAGNET
        time.sleep(0.1)

    def reset_positions(self):
        global CONSTANT, POST, HOME, ONE_DROP
        with open('config.json') as json_file:
            CONSTANT = json.load(json_file)

        # Piprobot constant
        POST = CONSTANT['PIPROB']['POST_PO'] # Post position
        HOME = CONSTANT['PIPROB']['WAIT_J'] # Home + Standby position
        ONE_DROP = CONSTANT['ONE_DROP'] # Volume to form one droplet

    def prepare_electrolyte(self, electrolyte_nr:int, electrolyte_vol:int, autodetect:bool=False, pre_operation:int=-1):
        if pre_operation == 0:
            self._prep_done.set()
            return
        if pre_operation == 1:
            total_volume = electrolyte_vol - ONE_DROP
        if pre_operation == 2:
            total_volume = (electrolyte_vol - ONE_DROP)//2
        if pre_operation == -1:
            total_volume = electrolyte_vol
        # clear threading Event, block dispersing move
        self._prep_done.clear()

        global cap_po, tip_po
        cap_po = CONSTANT['PIPROB']['cap'][str(electrolyte_nr)]
        tip_po = CONSTANT['PIPROB']['tip'][str(electrolyte_nr)]
        asp_pos = (tip_po[0]+10, tip_po[1]+19, 44, tip_po[3], tip_po[4], tip_po[5])

        #Reset status
        self.status.update(dict(Cap_Captured=False, Tip_On=False))

        # Set magnet tool
        self.set_tool(MAGNET)

        # Remove caps
        self.MovePose(cap_po[0], cap_po[1], cap_po[2]+30, cap_po[3], cap_po[4], cap_po[5])
        self.MoveLin(cap_po[0], cap_po[1], 70, cap_po[3], cap_po[4], cap_po[5]) # High enough to get the cap magneticly z = 70
        self.status['Cap_Captured'] = True
        time.sleep(0.5)
        self.MoveLinRelWRF(0,0,30,0,0,0)
        self._move_on.wait()
        if self._abort.isSet():
            self.return_cap()
            self._prep_done.set()
            return

        # Set Pipeting tool
        self.set_tool(PIPETTE)

        # Get the tip
        self.MovePose(tip_po[0], tip_po[1], 70, tip_po[3], tip_po[4], tip_po[5])
        self.MoveLin(*tip_po)
        time.sleep(1)
        self.status['Tip_On'] = True
        
        self._move_on.wait()
        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._prep_done.set()
            return

        # Go into vial
        self.MoveLin(tip_po[0], tip_po[1], 100, tip_po[3], tip_po[4], tip_po[5]) # Tip rise High enough z = 100
        self.MoveLin(asp_pos[0], asp_pos[1], 100, asp_pos[3], asp_pos[4], asp_pos[5])
        self.MoveLin(*asp_pos)
        time.sleep(1)
        self._move_on.wait()
        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._prep_done.set()
            return

        if autodetect == True: # With Level detection:
            while self.tellLevel() <= 299:
                self.MoveLinRelWRF(0,0,70,0,0,0)
                self.MoveJoints(-45,0,0,0,0,0)
                user_input = messagebox.askyesnocancel(title="Error detected!", message= "No liquid detected! Do you want to retry?")
                if user_input == False:
                    break
                if user_input == None:
                    self.return_tip()
                    self.return_cap()
                    self._prep_done.set()
                    return
                self.MovePose(asp_pos[0], asp_pos[1], asp_pos[2]+70, asp_pos[3], asp_pos[4], asp_pos[5])
                self.MoveLinRelWRF(0,0,-70,0,0,0)
                time.sleep(2)
            else:
                self._move_on.wait()
                # Aspirate electrolyte
                self.aspirate(total_volume)
                time.sleep(2)

                # Move out from vial
                self.MoveLinRelWRF(0,0,55,0,0,0)

                self._move_on.wait()
                # Abort Flag
                if self._abort.isSet():
                    self.return_tip()
                    self.return_cap()
                    self._prep_done.set()
                    return
                # Set therading Event release dispersing move
                self._prep_done.set()
                # Aspirate some air in case high acc
                self.aspirate(2)
                time.sleep(1)
        else: # Without Level detection
            # Aspirate electrolyte
            if self._abort.isSet():
                self.MoveLinRelWRF(0,0,55,0,0,0)
                self.return_tip()
                self.return_cap()
                self._prep_done.set()
                return
            self.aspirate(total_volume)
            time.sleep(2)

            # Move out from vial
            self.MoveLinRelWRF(0,0,55,0,0,0)

            # Set therading Event release dispersing move
            self._prep_done.set()

            # Aspirate some air in case high acc
            self._move_on.wait()
            self.aspirate(2)
            time.sleep(1)

        # Pippet ready, waiting for assembly robot to finish 
        self.MoveJoints(*HOME)

        # Abort Flag
        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._prep_done.set()
            return
        
    def add_electrolyte(self, volume, touch:bool=False):
        self._prep_done.wait()

        # ------Pause flag------
        self._move_on.wait()

        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return

        # clear threading Event, block dropping component move
        self._add_done.clear()
        
        # First MoveJ above post to avoid collision
        self.MoveJoints(90,0,0,0,0,0)

        # Go to the post, while keepping the vertical posture
        self.MovePose(POST[0], POST[1], POST[2]+20, POST[3], POST[4], POST[5])

        if touch == True:
            self.MoveLin(*POST)
        else:
            self.MoveLin(POST[0], POST[1], POST[2]+3, POST[3], POST[4], POST[5])

        # ------Pause flag------
        self._move_on.wait()

        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return
        
        # Dispense
        if volume == ALL:
            self.blowout()
            time.sleep(1)
            self.aspirate(20)
            time.sleep(1)
            self.clear_and_reset()
        else:
            self.dispense(volume)
        time.sleep(1)
        # Move up
        self.MovePose(POST[0], POST[1], POST[2]+30, POST[3], POST[4], POST[5])
        
        # MoveJ above post to avoid collision
        self.MoveJoints(90,0,0,0,0,0)
        # To wait position
        self.MoveJoints(*HOME)

        # ------Pause flag------
        self._move_on.wait()

        # Abort Flag
        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return

        # Set therading Event release drop component move
        self._add_done.set()

    def clear_tip(self):
        if self.tellPosition() > 35:
            self.set_tool(PIPETTE)
            temp_po = list(self.GetPose())
            if temp_po[0] >= 0 and temp_po[2] < 110:
                self.MoveLin(temp_po[0], temp_po[1], 110, temp_po[3], temp_po[4], temp_po[5])
            if temp_po[0] < 0 and temp_po[2] < 175:
                self.MoveLin(temp_po[0], temp_po[1], 175, temp_po[3], temp_po[4], temp_po[5])
                self.MoveJoints(90,0,0,0,0,0)
                # self.MoveLin(temp_po[0], temp_po[1], 160, temp_po[3], temp_po[4], temp_po[5])
            waste = CONSTANT['PIPROB']['WASTE_PO']
            self.MovePose(waste[0], waste[1], 110, waste[3], waste[4], waste[5])
            self.MoveLin(*waste)
            self.blowout()
            time.sleep(2)
            self.MovePose(waste[0], waste[1], 110, waste[3], waste[4], waste[5])
            self.MoveJoints(*HOME)
    
    def return_tip(self):
        if self.status['Tip_On'] == True:
            self.clear_tip()
            # Return pipett tip
            self.set_tool(PIPETTE)
            temp_po = list(self.GetPose())
            if temp_po[2] < 110:
                self.MoveLin(temp_po[0], temp_po[1], 110, temp_po[3], temp_po[4], temp_po[5])
                self.MoveLin(tip_po[0], tip_po[1], 110, tip_po[3], tip_po[4], tip_po[5])
                self.MoveLin(tip_po[0], tip_po[1], 60, tip_po[3], tip_po[4], tip_po[5]) # Low enough to insert the tip into position z = 60
            else:
                self.MovePose(tip_po[0], tip_po[1], 110, tip_po[3], tip_po[4], tip_po[5])
                self.MoveLin(tip_po[0], tip_po[1], 60, tip_po[3], tip_po[4], tip_po[5]) # Low enough to insert the tip into position z = 60
            self.eject_and_home()
            time.sleep(1)
            # Rise High enough to avoid collision z = 75
            self.MoveLin(tip_po[0], tip_po[1], 75, tip_po[3], tip_po[4], tip_po[5])
            self.status['Tip_On'] = False

    def return_cap(self):
        if self.status['Cap_Captured'] == True:
            # Return cap
            #self.MoveLinRelWRF(0,0,30,0,0,0)
            self.set_tool(MAGNET)
            temp_po = list(self.GetPose())
            if temp_po[2] < 90:
                self.MoveLin(temp_po[0], temp_po[1], 90, temp_po[3], temp_po[4], temp_po[5])
                self.MoveLin(cap_po[0], cap_po[1], 90, cap_po[3], cap_po[4], cap_po[5])
            else:
                self.MovePose(cap_po[0], cap_po[1], 90, cap_po[3], cap_po[4], cap_po[5])
            self.MoveLin(*cap_po)
            time.sleep(1)
            # Move +y to slide cap y = 10
            self.MoveLinRelWRF(0,10,0,0,0,0)
            self.MoveLinRelWRF(0,0,30,0,0,0)
            self.MoveJoints(*HOME)
            self.status['Cap_Captured'] = False

    def finish_dispensing(self):
        self.return_tip()
        time.sleep(0.5)
        self.return_cap()

    def add_electrolyte_plus_additive(self, electrolyte_num:int, additive_num:int, additive_vol:int):
        # Blow out residual electrolyte

        # clear threading Event, block drop component move
        self._add_done.clear()

        # First MoveJ above post to avoid collision
        self.MoveJoints(90,0,0,0,0,0)

        # Go to the post, while keepping the vertical posture
        self.MovePose(POST[0], POST[1], POST[2]+20, POST[3], POST[4], POST[5])

        self.MoveLin(POST[0], POST[1], POST[2]+3, POST[3], POST[4], POST[5])

        # ------Pause flag------
        self._move_on.wait()

        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return
        
        # Dispense
        self.blowout()
        time.sleep(1)
        self.aspirate(20)
        time.sleep(1)
        self.clear_and_reset()
        time.sleep(1)
        # Move up
        self.MovePose(POST[0], POST[1], POST[2]+30, POST[3], POST[4], POST[5])
        
        # MoveJ above post to avoid collision
        self.MoveJoints(90,0,0,0,0,0)
        # To wait position
        self.MoveJoints(*HOME)

        # ------Pause flag------
        self._move_on.wait()

        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return

        # Return tip
        self.set_tool(PIPETTE)
        elyte_tip = CONSTANT['PIPROB']['tip'][str(electrolyte_num)]

        self.MovePose(elyte_tip[0], elyte_tip[1], 110, elyte_tip[3], elyte_tip[4], elyte_tip[5])
        self.MoveLin(elyte_tip[0], elyte_tip[1], 60, elyte_tip[3], elyte_tip[4], elyte_tip[5])
        self.eject_and_home()
        time.sleep(1)
        # Rise High enough to avoid collision z = 75
        self.MoveLin(elyte_tip[0], elyte_tip[1], 75, elyte_tip[3], elyte_tip[4], elyte_tip[5])
        self.status['Tip_On'] = False
        time.sleep(0.5)

        # Return cap
        self.set_tool(MAGNET)
        elyte_cap = CONSTANT['PIPROB']['cap'][str(electrolyte_num)]
        self.MovePose(elyte_cap[0], elyte_cap[1], 90, elyte_cap[3], elyte_cap[4], elyte_cap[5])
        self.MoveLin(*elyte_cap)
        time.sleep(1)
        # Move +y to slide cap y = 10
        self.MoveLinRelWRF(0,10,0,0,0,0)
        self.MoveLinRelWRF(0,0,30,0,0,0)
        self.MoveJoints(*HOME)
        self.status['Cap_Captured'] = False

        # ------Pause flag------
        self._move_on.wait()

        # Abort Flag
        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            return

        # Start to prepear additive

        global cap_po, tip_po
        cap_po = CONSTANT['PIPROB']['cap'][str(additive_num)]
        tip_po = CONSTANT['PIPROB']['tip'][str(additive_num)]
        asp_pos = (tip_po[0]+10, tip_po[1]+19, 48, tip_po[3], tip_po[4], tip_po[5])

        #Reset status
        self.status.update(dict(Cap_Captured=False, Tip_On=False))

        # Set magnet tool
        self.set_tool(MAGNET)

        # Remove caps
        self.MovePose(cap_po[0], cap_po[1], cap_po[2]+30, cap_po[3], cap_po[4], cap_po[5])
        self.MoveLin(cap_po[0], cap_po[1], 70, cap_po[3], cap_po[4], cap_po[5]) # High enough to get the cap magneticly z = 70
        self.status['Cap_Captured'] = True
        time.sleep(0.5)
        self.MoveLinRelWRF(0,0,30,0,0,0)

        self._move_on.wait()
        if self._abort.isSet():
            self.return_cap()
            self._add_done.set()
            return

        # Set Pipeting tool
        self.set_tool(PIPETTE)

        # Get the tip
        self.MovePose(tip_po[0], tip_po[1], 70, tip_po[3], tip_po[4], tip_po[5])
        self.MoveLin(*tip_po)
        time.sleep(1)
        self.status['Tip_On'] = True
        
        self._move_on.wait()
        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return

        # Go into vial
        self.MoveLin(tip_po[0], tip_po[1], 100, tip_po[3], tip_po[4], tip_po[5]) # Tip rise High enough z = 100
        self.MoveLin(asp_pos[0], asp_pos[1], 100, asp_pos[3], asp_pos[4], asp_pos[5])
        self.MoveLin(*asp_pos)
        time.sleep(1)
        self._move_on.wait()

        # Without Level detection
        # Aspirate electrolyte
        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return
        self.aspirate(additive_vol)
        time.sleep(2)

        # Move out from vial
        self.MoveLinRelWRF(0,0,55,0,0,0)

        # Aspirate some air in case high acc
        self._move_on.wait()
        self.aspirate(2)
        time.sleep(0.5)

        # Pippet ready, waiting for assembly robot to finish 
        self.MoveJoints(*HOME)

        # ------Pause flag------
        self._move_on.wait()

        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return

        # First MoveJ above post to avoid collision
        self.MoveJoints(90,0,0,0,0,0)

        # Go to the post, while keepping the vertical posture
        self.MovePose(POST[0], POST[1], POST[2]+20, POST[3], POST[4], POST[5])

        self.MoveLin(POST[0], POST[1], POST[2]+3, POST[3], POST[4], POST[5])

        # ------Pause flag------
        self._move_on.wait()

        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return
        
        # Dispense
        self.blowout()
        time.sleep(1)
        self.aspirate(20)
        time.sleep(1)
        self.clear_and_reset()
        time.sleep(1)
        # Move up
        self.MovePose(POST[0], POST[1], POST[2]+30, POST[3], POST[4], POST[5])
        
        # MoveJ above post to avoid collision
        self.MoveJoints(90,0,0,0,0,0)
        # To wait position
        self.MoveJoints(*HOME)

        # Set therading Event release dispersing move
        self._add_done.set()

        # ------Pause flag------
        self._move_on.wait()

        # Abort Flag
        if self._abort.isSet():
            self.return_tip()
            self.return_cap()
            self._add_done.set()
            return

    def go_home(self):
        self.auto_repair()
        temp_po = list(self.GetPose())
        if self.status['Tool'] == PIPETTE and temp_po[2] < 120:
            self.MoveLin(temp_po[0], temp_po[1], 120, temp_po[3], temp_po[4], temp_po[5])
        if self.status['Tool'] == MAGNET and temp_po[2] < 80:
            self.MoveLin(temp_po[0], temp_po[1], 80, temp_po[3], temp_po[4], temp_po[5])
        time.sleep(0.5)
        self.MoveJoints(*HOME)

    def auto_repair(self):
    # If there is an error we try to autorepair it. Added an extra resume motion over the
    # mecademic suggested version
        if self.is_in_error():
            self.ResetError()
        elif self.GetStatusRobot()['Paused'] == 1:
            self.ResumeMotion()
        self.ResumeMotion()
        self.ResumeMotion()

    def disconnect_piprobot(self):
    # Deactivate and disconnect the robot
        logger.info('Disconnecting Pipetting Robot...')
        # Clear, return tip, return cap, home
        self.finish_dispensing()
        time.sleep(1.5)
        self.DeactivateRobot()
        time.sleep(0.1)
        self.disconnect()
        time.sleep(0.1)
        self.disconnect_pipette()
        logger.info(f"Pipetting Robot disconnected!")

    def pause_pipette_rob(self):
        self.PauseMotion()
    
    def resume_pipette_rob(self):
        self.ResumeMotion()

    def abort_pipette_rob(self):
        self.status['Aborted'] = True