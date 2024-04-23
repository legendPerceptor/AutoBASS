import os
import time
import json
import threading
import logging
import numpy as np
import serial
from tkinter import messagebox
from MecademicRobot import RobotController
from Rail import Rail
from Offsetcal import AutoCorrection

# Creat logging instant
logger = logging.getLogger('AssemblyRobot')

ROOT = os.path.dirname(__file__)
CONFIG_DIR = os.path.join(ROOT, "data", "config.json")

# IP address of robots
ASSEMBLY_HOST = "192.168.31.231"
RAIL_HOST = "192.168.31.233"

# Vaccum Control port
ARDU_PORT = 'COM7'

# Rail Control port
RAIL_PORT = 10001

# Get latest constant values from config file
with open(CONFIG_DIR, "r") as json_file:
    CONSTANT = json.load(json_file)

GRIPPER = 1
SUCTION = 2

COMPONENTS = ('Anode_Case', 'Anode_Spacer', 'Anode', 'Separator', 'Cathode', 'Cathode_Spacer', 'Washer', 'Cathode_Case')
INITSTEPS = ("Connect", "Activate", "Home", "Rail", "VacPump", "Camera")
STEPS = ("Grab", "Drop", "Press", "Retrieve", "Store")

class AssemblyRobot(Rail, RobotController, AutoCorrection):

    def __init__(self, address=ASSEMBLY_HOST, vacuum_port=ARDU_PORT, pause_control:threading.Event=None, abort_control:threading.Event=None):
        RobotController.__init__(self, address)
        Rail.__init__(self)
        AutoCorrection.__init__(self)
        self.vacuum_port = vacuum_port
        self._move_on = pause_control
        self._abort = abort_control
        self.reset_status()
        global home
        home = (0,0,0,0,0,0)

    def reset_status(self):
        self.status = dict(Tool=None, Progress=dict(Initiate=0, LastStep=None), Initiated=False, Vacuumed=False, Grabed=True, Aborted=False)

    def assembrob_is_online(self):
        try:
            rob_res = self.GetStatusRobot()
        except:
            return False
        else:
            vac_pump_res = self.check_connection()
            check = (rob_res["Activated"], rob_res["Homing"], not rob_res["Error"], vac_pump_res)
            return all(check)
    
    def connect_pump(self):
        logger.info(f'initiating Pump Controller on PORT: {self.vacuum_port}...')
        try:
            self.serialcomm = serial.Serial(port = self.vacuum_port, baudrate = 110, timeout = 1)
        except:
            logger.error(f'initiating Pump Controller on PORT: {self.vacuum_port}...', exc_info=True)
            return False
        else:
            logger.info(f'Pump Controller on PORT: {self.vacuum_port} connected!')
            return True

    def check_connection(self):
        return self.serialcomm.isOpen()
    
    def suction_on(self):
        self.serialcomm.write(b'H')
        self.status['Vacuumed'] = True
    
    def suction_off(self):
        self.serialcomm.write(b'L')
        self.status['Vacuumed'] = False

    def disconnect_pump(self):
        self.serialcomm.close()
        # self.serialcomm = None
        logger.info('Pump Controller disconnected!')
        return True

    def reconnect_pump(self):
        self.serialcomm.close()
        self.serialcomm = None
        time.sleep(0.05)
        self.serialcomm = serial.Serial(port = self.vacuum_port, baudrate = 110, timeout = 1)
    
    def init_assembly_robot(self):
        # Initilize status value
        rob_res = dict(Connection=False, Activation=False, Homing=False, Rail=False, VacuumPump=False)

        # Initiate Rail
        rail_res = self.connect_rail(RAIL_HOST, RAIL_PORT)
        self.status["Progress"]["Initiate"] = round(1/6*100)
        rob_res.update(dict(Rail=rail_res))

        # Activate robot, home, reset joints, apply parameters
        logger.info('Initiating Assembly Robot...')
        conRes = self.connect()
        time.sleep(0.1)
        if conRes is True:
            # Update status
            self.status["Progress"]["Initiate"] = round(2/6*100)
            rob_res.update(dict(Connection=conRes))
            # Activate Robot
            actRes = self.ActivateRobot()
            time.sleep(0.1)
            if actRes == 'Motors activated.':
                # Update status
                self.status["Progress"]["Initiate"] = round(3/6*100)
                rob_res.update(dict(Activation=True))
                # Home robot
                homRes = self.home()
                time.sleep(0.1)
                if homRes == 'Homing done.':
                    # Update status
                    self.status["Progress"]["Initiate"] = round(4/6*100)
                    rob_res.update(dict(Homing=True))
                    # Set robot's parameter
                    self.SetGripperForce(CONSTANT['GRIP_F'])
                    self.SetGripperVel(CONSTANT['GRIP_VEL'])
                    self.SetCartLinVel(CONSTANT['L_VEL'])
                    self.SetJointVel(CONSTANT['J_VEL'])
                    self.SetJointAcc(20)
                    self.MoveJoints(0,0,0,0,0,0)
                    time.sleep(0.01)
                    logger.info('Assembly Robot initiated.')
                    vac_pump_res = self.connect_pump()
                    self.status["Progress"]["Initiate"] = round(6/6*100)
                    rob_res.update(dict(VacuumPump=vac_pump_res))
                else:
                    logger.warning('Assembly Robot already homed!')
            else:
                logger.warning('Assembly Robot already activated!')
        elif conRes == 'Another user is already connected, closing connection':
            logger.error('Assembly Robot already in connection!')
        else:
            logger.critical('Assembly Robot is not in connection. Check Power buttom')
        
        return rob_res

    def GripperOpen(self):
        """Opens the gripper of the end-effector.
        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.
        """
        self.status["Grabed"] = False
        cmd = 'GripperOpen'
        return self.exchange_msg(cmd)

    def GripperClose(self):
        """Closes the gripper of the end-effector.
        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.
        """
        self.status["Grabed"] = True
        cmd = 'GripperClose'
        return self.exchange_msg(cmd)

    def change_tool(self, tool_name:int):
        global home
        if tool_name == GRIPPER:
            self.SetTRF(*CONSTANT['TCP_GP'])
            self.status["Tool"] = GRIPPER
            home = CONSTANT["HOME_GP_J"]
        if tool_name == SUCTION:
            self.SetTRF(*CONSTANT['TCP_SK'])
            self.status["Tool"] = SUCTION
            home = CONSTANT["HOME_SK_J"]

    def smart_grab(self):
        if self.status["Tool"] == GRIPPER:
            self.GripperClose()
            time.sleep(1)
        if self.status["Tool"] == SUCTION:
            self.suction_on()
            time.sleep(0.5)

    def smart_drop(self):
        if self.status["Tool"] == GRIPPER:
            self.GripperOpen()
            time.sleep(1)
        if self.status["Tool"] == SUCTION:
            self.suction_off()
            time.sleep(0.2)

    def grab_component(self, rail_po, grab_po):
        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.move(0)
            return

        # Transport assembly robot to destination
        logger.debug(f"Moving to {rail_po}...")
        self.move(rail_po)

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.move(0)
            return

        # Grabing materials from trays
        logger.debug(f"Gripping on {grab_po}...")

        # Determine which tool to use for grabing
        if abs(grab_po[0]) >= 160:
        # Switch the TCP, Reset the joints' position
            self.change_tool(GRIPPER)
        else:
            self.change_tool(SUCTION)
        time.sleep(0.1)
        self.MoveJoints(*home)

        # Move to the start position
        self.MovePose(grab_po[0], grab_po[1], 40, grab_po[3], grab_po[4], grab_po[5])
        if self.status["Grabed"] == True:
            self.GripperOpen()
        time.sleep(0.5)

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.go_home()
            self.move(0)
            return

        # Gripper goes down
        self.MoveLin(*grab_po)

        # Gripping parts, ready to transport
        self.smart_grab()
        self.MoveLin(grab_po[0], grab_po[1], 40, grab_po[3], grab_po[4], grab_po[5])

        self.MoveJoints(*home)

        # ------Pause flag------
        self._move_on.wait()

        self.status["Progress"]["LastStep"] = STEPS[0]

        if self._abort.isSet():
            self.return_component(rail_po, grab_po)

    def return_component(self, rail_po, grab_po):
        # When aborted, return components back to its tray
        # Reset the joints' position
        self.go_home()

        # Transport assembly robot to destination
        logger.warning(f"Returning component to {rail_po}...")
        self.move(rail_po)

        # Move to the start position
        self.MovePose(grab_po[0], grab_po[1], 40, grab_po[3], grab_po[4], grab_po[5])
        time.sleep(0.5)

        # Gripper goes down
        self.MoveLin(*grab_po)

        self.smart_drop()
        
        # Home
        self.MoveLin(grab_po[0], grab_po[1], 40, grab_po[3], grab_po[4], grab_po[5])
        self.MoveJoints(*home)
        self.move(0)

    def drop_component(self, drop_po, component:str, nr:int, auto_calib:bool=True, grab_check:bool=True, save_img:bool=True, show_image:bool=False):
        # Flag to assign user interrupt
        self.userInterupt = False
        
        # Convert position to numpy array
        drop_po = np.array(drop_po, dtype=np.float32)

        # Exclude spring drop from auto correction
        if component != COMPONENTS[6]:
            drop_po[:2] = CONSTANT['POST_C_SK_PO'][:2]

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.return_component(CONSTANT[component]['railPo'][nr-1], CONSTANT[component]['grabPo'][str(nr)])
            return False

        # Transport assembly robot to destination
        logger.debug(f"Moving to Post...")
        self.move(CONSTANT["post"])

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.return_component(CONSTANT[component]['railPo'][nr-1], CONSTANT[component]['grabPo'][str(nr)])
            return False

        # Snapshot on grap status
        # if component in (COMPONENTS[1],COMPONENTS[5],COMPONENTS[7]): # Electrodes
        #     # self.MoveJoints(*CONSTANT['SNAP_SHOT_GRAB_J'])
        #     self.MovePose(*CONSTANT['SNAP_SHOT_GRAB_PO'])
        #     logger.info(f'Checking alignment on {component}\'s Grab')
        #     correction, _ = self.run_autocorrection(state='Grab', component=component, nr=nr, show=False)
        #     drop_po = drop_po + correction
        # elif component in (COMPONENTS[2],COMPONENTS[4]) and auto_calib:
        if component in (COMPONENTS[1],COMPONENTS[2],COMPONENTS[3],COMPONENTS[4],COMPONENTS[5],COMPONENTS[7]) and auto_calib:
            self.MovePose(*CONSTANT['SNAP_SHOT_GRAB_PO'])
            logger.info(f'Checking alignment on {component}\'s Grab')
            correction, self.grabYes = self.run_autocorrection(state='Grab', component=component, nr=nr, show=show_image, save=save_img)
            if self.grabYes:
                drop_po = drop_po + correction
                logger.info("Implementing auto correction...")
            elif grab_check:
                logger.info(f"Trying to grab {component} for the second time...")
                self.go_home()
                self.reconnect_pump()
                time.sleep(0.5)
                self.grab_component(CONSTANT[component]['railPo'][nr-1], CONSTANT[component]['grabPo'][str(nr)])
                self.move(CONSTANT["post"])
                self.MovePose(*CONSTANT['SNAP_SHOT_GRAB_PO'])
                logger.info(f'Checking alignment on {component}\'s Grab')
                correction, self.grabYes = self.run_autocorrection(state='Grab', component=component, nr=nr, show=show_image, save=save_img)
                if self.grabYes:
                    drop_po = drop_po + correction
                    logger.info("Implementing auto correction...")
                else:
                    logger.info(f"No {component} detected! Manual check on tray {component}_NO.[{nr}] required!")
                    self.go_home()
                    manual = messagebox.askokcancel(title="Manual check required!", message=f"Make sure vacuum pump is on and {component} is on the tray NO.[{nr}], then Click 'OK'")
                    if manual == True:
                        self.grab_component(CONSTANT[component]['railPo'][nr-1], CONSTANT[component]['grabPo'][str(nr)])
                        self.move(CONSTANT["post"])
                        self.MovePose(*CONSTANT['SNAP_SHOT_GRAB_PO'])
                        logger.info(f'Checking alignment on {component}\'s Grab')
                        correction, self.grabYes = self.run_autocorrection(state='Grab', component=component, nr=nr, show=show_image, save=save_img)
                        drop_po = drop_po + correction
                        logger.info("Implementing auto correction...")
                    else:
                        self.suction_off()
                        self.userInterupt = True
                        # self._abort.set()
                        return False

        # Drop the material onto the post
        logger.debug(f"Dropping on {drop_po}...")

        # Robot move to the start position
        self.MovePose(drop_po[0], drop_po[1], 100, drop_po[3], drop_po[4], drop_po[5])
        time.sleep(0.5)

        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.return_component(CONSTANT[component]['railPo'][nr-1], CONSTANT[component]['grabPo'][str(nr)])
            return False

        # Gripper goes down, and drop the parts
        self.MoveLin(*drop_po)

        # From now on the component is irreversable!

        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.smart_drop()
            self.go_home()
            self.move(0)
            return True

        if component == COMPONENTS[7]: # Cathode case
            # Perform tilling movement
            self.SetCartAngVel(5)
            self.MoveLinRelWRF(0,-0.3,0.5,0,0,0)
            self.MoveLinRelWRF(0,0,0,2,0,0)
            self.MoveLinRelWRF(0,0,-1.5,0,0,0)
            self.MoveLinRelWRF(0,0,0,-2,0,0)
            # self.MoveLinRelWRF(0,0.6,-1.5,0,0,0)
            # self.MoveLinRelWRF(0,0,-0.5,-2,0,0)
            
            self.smart_drop()

            self.SetCartAngVel(45)
            # Homing the robot
            self.MoveLinRelWRF(0,0,30,0,0,0)
            with open(CONFIG_DIR) as json_file:
                press = json.load(json_file)['tap_press']
            if press != True:
                self.MoveJoints(*CONSTANT['HOME_POST_J'])
                self.MoveJoints(*CONSTANT['HOME_SK_J'])
        # elif component in COMPONENTS[2:5]: # Electrodes + Separator
        elif component in (COMPONENTS[2], COMPONENTS[3], COMPONENTS[4]): # Electrodes
            self.SetCartAngVel(90)

            self.smart_drop()
            if auto_calib:
                self.MoveLinRelWRF(0,0,10,0,0,0)
                self.SetCartAngVel(45)
                # Taking a snap shot
                self.MovePose(*CONSTANT['SNAP_SHOT_DROP_PO'])
                self.run_autocorrection(state='Drop', component=component, nr=nr, save=save_img)
            self.MoveLinRelWRF(0,0,30,0,0,0)

            # ------Pause flag------
            # In case separators were not properly droped manually interfernce is needed
            if not self._move_on.isSet():
                self.MoveJoints(*CONSTANT['HOME_POST_J'])
                self.MoveJoints(*CONSTANT['HOME_SK_J'])
                self.suction_on()
                messagebox.showinfo(title="Manual check required!", message=f"Is {component} lost? Put it back on suction cup then Click [Resume]")
                # ------Pause flag------
                self._move_on.wait()
                if not self._abort.isSet():
                    self.MovePose(drop_po[0], drop_po[1], 100, drop_po[3], drop_po[4], drop_po[5])
                    time.sleep(0.5)
                    self.MoveLin(drop_po[0], drop_po[1], drop_po[2]-0.5, drop_po[3], drop_po[4], drop_po[5])
                    self.smart_drop()
                    self.MoveLinRelWRF(0,0,30,0,0,0)
                else:
                    self.suction_off()
                    self.move(0)
                    return False
            
            self.MoveJoints(*CONSTANT['HOME_POST_J'])
            self.MoveJoints(*CONSTANT['HOME_SK_J'])
        else:
            # ------Pause flag------
            self._move_on.wait()
            ## ------Abort flag------ ##
            if self._abort.isSet():
                self.return_component(CONSTANT[component]['railPo'][nr-1], CONSTANT[component]['grabPo'][str(nr)])
                return True
            self.smart_drop()
            self.MoveLinRelWRF(0,0,30,0,0,0)

            # ------Pause flag------
            self._move_on.wait()

            self.MoveJoints(*CONSTANT['HOME_POST_J'])
            self.MoveJoints(*CONSTANT['HOME_SK_J'])

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.move(0)
            return True

        self.status["Progress"]["LastStep"] = STEPS[1]
        return True

    def press_cell(self):
        # ------Pause flag------
        self._move_on.wait()

        # Transport assembly robot to destination
        # logger.debug("Prepearing to press cell...")
        # self.move(CONSTANT["post"])

        # ------Pause flag------
        # self._move_on.wait()

        # Press cathode case
        logger.info("Pressing...")
        
        self.change_tool(GRIPPER)
        time.sleep(0.1)

        self.MovePose(*np.add(CONSTANT[COMPONENTS[6]]['dropPo'], (0,0,40,0,0,0))) # Washer drop pos z+ 40
        # self.MoveJoints(*CONSTANT['HOME_GP_J'])
        # self.MovePose(*CONSTANT['PRESS_1_PO'])
        time.sleep(0.5)

        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.go_home()
            self.move(0)
            return
        
        # Performing pressing on 5 points
        self.GripperOpen()
        self.MoveLin(*np.add(CONSTANT[COMPONENTS[6]]['dropPo'], (-5,0,2,0,0,0))) # Washer drop pos z+ 2
        # self.MoveLin(*CONSTANT['PRESS_2_PO'])
        # Change press point to x:+5 and press again
        self.MoveLinRelWRF(0,0,5,0,0,0)
        self.MoveLinRelWRF(5,0,0,0,0,0)
        self.MoveLinRelWRF(0,0,-5,0,0,0)

        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.go_home()
            self.move(0)
            return

        # Change press point to x:+5 and press again
        self.MoveLinRelWRF(0,0,5,0,0,0)
        self.MoveLinRelWRF(5,0,0,0,0,0)
        self.MoveLinRelWRF(0,0,-5,0,0,0)
        time.sleep(0.5)
        
        # Move up
        self.MovePose(*np.add(CONSTANT[COMPONENTS[6]]['dropPo'], (0,0,40,0,0,0))) # Washer drop pos z+ 40

        # ------Pause flag------
        self._move_on.wait()

        # Reset joints' position
        self.MoveJoints(*CONSTANT['HOME_POST_J'])
        self.MoveJoints(*CONSTANT['HOME_GP_J'])
        # Move away to avoid collision
        self.move(600)

        # ------Pause flag------
        self._move_on.wait()

        self.status["Progress"]["LastStep"] = STEPS[2]

    def retrieve_cell(self):
        # ------Pause flag------
        self._move_on.wait()

        # Transport assembly robot to destination
        logger.debug("Moving to Post...")
        self.move(CONSTANT["post"])

        # ------Pause flag------
        self._move_on.wait()

        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.move(0)
            return

        # Grab finished cell on the post
        retrieve_po = CONSTANT['POST_C_PO']
        logger.debug("Retrieving cell on Post...")

        # Robot move above the post
        self.change_tool(SUCTION)
        self.MovePose(retrieve_po[0], retrieve_po[1], 70, retrieve_po[3], retrieve_po[4], retrieve_po[5])
        self.GripperOpen()

        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.go_home()
            self.move(0)
            return

        # Get the cell
        self.MoveLin(*retrieve_po)

        # ------Pause flag------
        self._move_on.wait()
        self.smart_grab()

        # Home robot
        self.MoveLin(retrieve_po[0], retrieve_po[1], 70, retrieve_po[3], retrieve_po[4], retrieve_po[5])
        self.MoveJoints(*home)
        self.status["Progress"]["LastStep"] = STEPS[3]

        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.move(0)
            return

    def store_cell(self, rail_po, store_po):
        # ------Pause flag------
        self._move_on.wait()

        # Transport assembly robot to destination
        logger.debug(f"Moving to Post...")
        self.move(rail_po)

        # ------Pause flag------
        self._move_on.wait()

        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.move(0)
            return

        # Put back finished cell
        logger.debug(f"Storing cell on {store_po}...")

        # Robot move to tray position
        self.MovePose(store_po[0]-2, store_po[1], 40, store_po[3], store_po[4]-5, store_po[5])
        time.sleep(0.5)

        ## ------Abort flag------ ##
        if self._abort.isSet():
            self.go_home()
            self.move(0)
            return

        # Drop cell
        self.MoveLin(store_po[0]-2, store_po[1], store_po[2]+4, store_po[3], store_po[4]-5, store_po[5])
        time.sleep(0.5)

        # ------Pause flag------
        self._move_on.wait()
        self.smart_drop()

        self.SetCartAngVel(5)
        self.MoveLinRelWRF(3,0,-3,0,0,0)
        self.MoveLinRelTRF(0,0,0,-5,0,0)
        time.sleep(1)
        self.SetCartAngVel(45)
        # Empty Gripper goes up using MovePose for fast mode
        self.go_home()
        self.move(0)
        self.status["Progress"]["LastStep"] = STEPS[4]

    def go_home(self):
        logger.debug("Homing AssemblyRobot...")
        self.auto_repair()
        temp_po = list(self.GetPose())
        if temp_po[2] <= 80:
            self.MoveLinRelWRF(0,0,40,0,0,0)
            time.sleep(0.5)
        self.MoveJoints(*home)

    def auto_repair(self):
    # If there is an error we try to autorepair it. Added an extra resume motion over the
    # mecademic suggested version
        if self.is_in_error():
            self.ResetError()
        elif self.GetStatusRobot()['Paused'] == 1:
            self.ResumeMotion()
        self.ResumeMotion()
        self.ResumeMotion()

    def disconnect_assembly_robot(self):
    # Deactivate and disconnect the robot
        logger.info('Disconnecting AssemblyRobot...')
        self.go_home()
        time.sleep(1.5)
        self.DeactivateRobot()
        time.sleep(0.1)
        self.disconnect_rail()
        self.disconnect_pump()
        self.disconnect()
        time.sleep(0.1)
        self.reset_status()
        logger.info(f"AssemblyRobot disconnected!")
    
    def pause_assembly_rob(self):
        self.PauseMotion()
        logger.info("Assembly Robot Motion Paused")
        self.stopMotion()
        logger.info("Rail Motion Paused")

    def resume_assembly_rob(self):
        self.resumeMotion()
        logger.info("Rail Motion Resumed")
        time.sleep(0.1)
        self.ResumeMotion()
        logger.info("Assembly Robot Motion Resumed")

    def abort_assembly_rob(self):
        self.status["Aborted"] = True