import os
import time
import json
import logging
import threading
import serial
from MecademicRobot import RobotController

# Creat logging instant
logger = logging.getLogger('TransportRobot')

PATH = os.path.dirname(__file__)

# IP address of robots
TRANSPORT_HOST = "192.168.31.232"

CRIMPER_PORT = "COM3"

# Robot Constant
os.chdir(f"{PATH}\data")

# Get latest constant values from config file
with open('config.json') as json_file:
    CONSTANT = json.load(json_file)

# Tool constants
NORMAL = 1
FLIPED = 2

HOME = CONSTANT['WAIT_1_J']
INITSTEPS = ("Connect", "Activate", "Home")
STEPS = ("Aligned", "Sended", "Crimped", "PickedUp", "Retrieved", "Stored")


class TransportRobot(RobotController):

    def __init__(self, address=TRANSPORT_HOST, crimper_port=CRIMPER_PORT, pause_control:threading.Event=None, abort_control:threading.Event=None):
        RobotController.__init__(self, address)
        self.address = address
        self.crimper_port = crimper_port
        # self._move_on = threading.Event()
        # self._move_on.set()
        # self._abort = threading.Event()
        # self._abort.clear()
        self._move_on = pause_control
        self._abort = abort_control
        self._crimp_done = threading.Event()
        self._crimp_done.set()
        self.reset_status()

        global dir_name, date_stamp
        
        cwd = os.getcwd()
        date_stamp = time.strftime("%Y_%m_%d", time.localtime())
        dir_name = os.path.join(cwd, 'Alignments', date_stamp)

    def reset_status(self):
        self.status = dict(Tool=None, Progress=dict(Initiate=0, LastStep=None), CrimpTime=0, Initiated=False, Grabed=False, Aborted=False)

    def transrob_is_online(self):
        try:
            rob_res = self.GetStatusRobot()
        except:
            return False
        else:
            crimper_res = self.check_connection()
            check = (rob_res["Activated"], rob_res["Homing"], not rob_res["Error"], crimper_res)
            return all(check)

    def set_tool(self, tool_name:int):
        if tool_name == NORMAL:
            self.SetTRF(*CONSTANT['TCP_CP'])
            self.status["Tool"] = NORMAL
        if tool_name == FLIPED:
            self.SetTRF(*CONSTANT['TCP_CP_180'])
            self.status["Tool"] = FLIPED

    def connect_crimper(self):
        logger.info(f'initiating Crimper Controller on PORT: {self.crimper_port}...')
        try:
            self.serialcomm = serial.Serial(port = self.crimper_port, baudrate = 9600, timeout = 1)
        except:
            logger.error(f'initiating Crimper Controller on PORT: {self.crimper_port}...', exc_info=True)
            return False
        else:
            logger.info(f'Crimper Controller on PORT: {self.crimper_port} connected!')
            return True

    def check_connection(self):
        return self.serialcomm.isOpen()
    
    def crimper_on(self):
        self.serialcomm.write(b'H')
    
    def disconnect_crimper(self):
        self.serialcomm.close()
        # self.serialcomm = None
        logger.info('Crimper Controller disconnected!')
        return True

    def init_transport_robot(self):
        # Initilize status value
        rob_res = dict(Connection=False, Activation=False, Homing=False, Crimper=False)

        # Activate robot, home, reset joints, apply parameters
        logger.info('Initiating Transport Robot...')
        conRes = self.connect()
        time.sleep(0.1)
        if conRes is True:
            # Update status
            self.status["Progress"]["Initiate"] = round(1/4*100)
            rob_res.update(dict(Connection=conRes))
            # Activate Robot
            actRes = self.ActivateRobot()
            time.sleep(0.1)
            if actRes == 'Motors activated.':
                # Update status
                self.status["Progress"]["Initiate"] = round(2/4*100)
                rob_res.update(dict(Activation=True))
                # Home robot
                homRes = self.home()
                time.sleep(0.1)
                if homRes == 'Homing done.':
                    # Update status
                    self.status["Progress"]["Initiate"] = round(3/4*100)
                    rob_res.update(dict(Homing=True))
                    # Set robot's parameter
                    self.SetGripperForce(CONSTANT['GRIP_F'])
                    self.SetGripperVel(CONSTANT['GRIP_VEL'])
                    self.SetCartLinVel(CONSTANT['L_VEL'])
                    self.SetJointVel(CONSTANT['J_VEL'])
                    self.SetJointAcc(20)
                    self.MoveJoints(*HOME)
                    time.sleep(0.01)
                    logger.info('Transport Robot initiated.')
                    # Connect crimper relay
                    crimper_res = self.connect_crimper()
                    rob_res.update(dict(Crimper=crimper_res))
                    # Update status
                    self.status["Progress"]["Initiate"] = round(4/4*100)
                else:
                    logger.warning('Transport Robot already homed!')
            else:
                logger.warning('Transport Robot already activated!')
        elif conRes == 'Another user is already connected, closing connection':
            logger.error('Transport Robot already in connection!')
        else:
            logger.critical('Transport Robot is not in connection. Check Power buttom')

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

    def align_cell(self):
        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return
        # Line up the components on the post
        logger.info("Aligning cell...")

        # To the start posiiton
        self.set_tool(NORMAL)
        self.MoveJoints(*CONSTANT['WAIT_2_J'])
        self.MovePose(*CONSTANT['ALIGN_1_PO'])

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return

        # Gripper goes down to position
        self.GripperOpen()
        time.sleep(0.5)
        self.MoveLin(*CONSTANT['ALIGN_2_PO'])
        self.SetGripperVel(CONSTANT['GRIP_VEL']*0.5)
        self.SetGripperForce(CONSTANT['GRIP_F']*0.5)
        time.sleep(0.5)
        self.GripperClose()
        time.sleep(1)
        self.GripperOpen()
        time.sleep(1)
        self.GripperClose()
        time.sleep(2)
        self.SetGripperForce(CONSTANT['GRIP_F'])
        self.SetGripperVel(CONSTANT['GRIP_VEL'])
        self.GripperOpen()
        time.sleep(1)

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return

        # Gripper goes up
        self.MoveLin(*CONSTANT['ALIGN_1_PO'])

        # Home robot
        self.MoveJoints(*CONSTANT['WAIT_2_J'])
        self.MoveJoints(*CONSTANT['WAIT_1_J'])
        self.status["Progress"]["Step"] = STEPS[0]

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return

    def send_to_crimp(self):
        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return

        # Send the assemblyed cell into crimper, to avoid sigularity, two phases of movement should performed in order
        logger.info("Sending cell into crimper...")

        # To the start posiiton
        self.set_tool(FLIPED)
        self.MoveJoints(*CONSTANT['WAIT_2_J'])
        self.MovePose(*CONSTANT['GRIP_1_PO'])

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return

        # Gripper goes down to position
        self.GripperOpen()
        time.sleep(0.5)
        self.MoveLin(*CONSTANT['GRIP_2_PO'])

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return

        time.sleep(0.5)
        self.GripperClose()
        time.sleep(1)

        # Gripper goes up
        self.MoveLin(*CONSTANT['GRIP_1_PO'])

        # Set TCP back to normal
        self.set_tool(NORMAL)

        # Reduce the rotating radius, rotate to crimper
        self.MoveJoints(*CONSTANT['ROTATE_LEFT_J'])
        self.MoveJoints(*CONSTANT['ROTATE_RIGHT_J'])

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return

        # Drive through sigularity, ready to reach in
        self.MoveJoints(*CONSTANT['TRANS_1_J'])
        time.sleep(0.5)

        # Reaching into crimper next to the die:
        self.MoveLin(*CONSTANT['TRANS_2_PO'])
        time.sleep(0.5)

        # Droping CC into the die:
        self.MoveLin(*CONSTANT['TRANS_3_PO'])
        time.sleep(0.5)
        self.GripperOpen()
        time.sleep(0.5)

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return

        # Move the gripper away from die
        self.MoveLin(*CONSTANT['BACKOFF_1_PO'])
        self.MoveLin(*CONSTANT['BACKOFF_2_PO'])
        self.MoveLin(*CONSTANT['BACKOFF_3_PO'])

        # Move out from crimper, goes to waitng position, ready to use the magenetic part
        self.MoveJoints(*CONSTANT['RETRIVE_1_J'])

        self.status["Progress"]["Step"] = STEPS[1]

        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            return

    def crimp_and_collect(self, nr):
        # ------Pause flag------
        self._move_on.wait()
        ## ------Abort flag------ ##
        if self._abort.is_set():
            self.go_home()
            self._crimp_done.set()
            return

        # Release AssemblyRobot, start next assembly
        self._crimp_done.set()

        # Crimper start to work
        self.start_crimping(nr=nr)
        self.status["Progress"]["Step"] = STEPS[2]

        # Pause the AssemblyRobot
        self._crimp_done.clear()

        # Get cell from crimper-die
        logger.info("Retrieving cell from crimper...")

        # Set TCP to normal:
        self.set_tool(NORMAL)
        
        # Magnet reaching into the crimper next to the die:
        self.MoveLin(*CONSTANT['RETRIVE_2_PO'])

        # Above the die, magnetic Grabing:
        self.MoveLin(*CONSTANT['RETRIVE_3_PO'])
        time.sleep(1)
        self.status["Progress"]["Step"] = STEPS[3]

        # Move up 3 mm:
        self.MoveLin(*CONSTANT['RETRIVE_4_PO'])

        # Move the Magnet with CC away from die: Backwards
        self.MoveLin(*CONSTANT['RETRIVE_5_PO'])

        # Move to the ROTATION_RIGHT:
        self.MoveJoints(*CONSTANT['ROTATE_RIGHT_J'])

        # Move to the ROTATION_LEFT:
        self.MoveJoints(*CONSTANT['ROTATE_LEFT_TO_BACK_J'])

        # Release Assembly Robot
        self._crimp_done.set()

        self.MoveJoints(*CONSTANT['WAIT_1_J'])
        self.status["Progress"]["Step"] = STEPS[4]

        # Ready to drop the CC on Post::
        if nr > 0 and nr <= 32:
            store_post = CONSTANT['MAG_ONE_PO']
        if nr > 32 and nr <= 64:
            store_post = CONSTANT['MAG_TWO_PO']

        self.MovePose(store_post[0], store_post[1], store_post[2]+20, store_post[3], store_post[4], store_post[5])

        # Drop CC on the Post:
        self.MoveLin(*store_post)
        time.sleep(0.5)

        # Perform Sliding with lower speed:
        self.SetCartLinVel(10)
        self.MoveLinRelWRF(0,-30,0,0,0,0)
        self.SetCartLinVel(CONSTANT['L_VEL'])

        # Move up, home the position
        self.MoveLinRelWRF(0,0,20,0,0,0)
        self.MoveJoints(*CONSTANT['WAIT_1_J'])

        #self._crimp_done.set()
        self.status["Progress"]["Step"] = STEPS[5]

    def go_home(self):
        logger.info("Homing TransportRobot...")

        # Home the TransportRobot
        self.auto_repair()
        current_j = list(self.GetJoints())
        current_po = list(self.GetPose())
        if current_j[0] < 120 and current_j[0] > 0:
            if current_po[1] > 195:
                # Now the gripper is inside of crimper
                # Lift up the gripper to safety z
                self.MoveLin(current_po[0], current_po[1], 322.150, current_po[3], current_po[4], current_po[5])
                current_po = self.GetPose()
                time.sleep(0.5)

                # Move the gripper to the left to safty x
                self.MoveLin(-18, current_po[1], current_po[2], current_po[3], current_po[4], current_po[5])
                current_po = self.GetPose()
                time.sleep(0.5)

                # Move the gripper backwards to safty y value
                self.MoveLin(current_po[0], 193, current_po[2], current_po[3], current_po[4], current_po[5])
            self.MoveJoints(*CONSTANT['ROTATE_RIGHT_J'])
            self.MoveJoints(*CONSTANT['ROTATE_LEFT_TO_BACK_J'])
            
        elif current_j[0] < 0 and current_j[0] > -120:
            if current_po[2] < 120:
                # Now the gripper is near the post
                # Lift up the gripper to safety z
                self.MoveLin(current_po[0], current_po[1], 200, current_po[3], current_po[4], current_po[5])
                self.set_tool(NORMAL)
            self.MoveJoints(*CONSTANT['ROTATE_LEFT_J'])
            self.MoveJoints(*CONSTANT['ROTATE_LEFT_TO_BACK_J'])
        self.MoveJoints(*CONSTANT['WAIT_1_J'])

    def auto_repair(self):
    # If there is an error we try to autorepair it. Added an extra resume motion over the
    # mecademic suggested version
        if self.is_in_error():
            self.ResetError()
        elif self.GetStatusRobot()['Paused'] == 1:
            self.ResumeMotion()
        self.ResumeMotion()
        self.ResumeMotion()
        
    def disconnect_transport_robot(self):
    # Deactivate and disconnect the robot
        logger.info('Disconnecting TransportRobot')
        self.go_home()
        time.sleep(1.5)
        self.DeactivateRobot()
        time.sleep(0.1)
        self.disconnect()
        self.disconnect_crimper()
        self.reset_status()
        logger.info("TransportRobot disconnected!")

    def create_sealer_log(self, nr):
        file_name = os.path.join(dir_name, 'Cells_Log.json')
        with open(file_name, "r") as infile:
            cellsLog:dict = json.load(infile)
        cellsLog[str(nr)]["sealing_time"] = time.strftime("%Y_%m_%d-%H:%M:%S", time.localtime())
        cellsLog[str(nr)]["success"] = 1
        with open(file_name, "w") as outfile:
            json.dump(cellsLog, outfile, indent=4)
            
    def start_crimping(self, nr:int=None, wait_time=75):
        logger.info('Start crimping...')
        self.crimper_on()
        # time.sleep(wait_time)
        for i in range(wait_time, -1, -1):
            self.status['CrimpTime'] = i
            time.sleep(1)
        print("                                                              ")
        self.create_sealer_log(nr)
    
    def pause_transport_rob(self):
        if not self._crimp_done.is_set():
            self.PauseMotion()
            logger.warning("Transport Robot Motin Paused")

    def resume_transport_rob(self):
        self.ResumeMotion()
        logger.warning("Transport Robot Motin Resumed")

    def abort_transport_rob(self):
        self.status["Aborted"] = True