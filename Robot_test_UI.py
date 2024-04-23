import json
import yaml
import os
import time
import threading
import logging
import logging.config
import cv2 as cv
import numpy as np
from tkinter import *
from tkinter import font
from tkinter import ttk
from tkinter import messagebox
from PIL import ImageTk, Image
from MecademicRobot import RobotFeedback
from MecademicRobot import RobotController
from Ardurino_relay import Ardurelay
from Rail import Rail
from data.obj_config import CONFIG, OBJECT_LIST

PATH = os.path.dirname(__file__) # ..../config

# IP address of robots
ASSEMBLY_HOST = "192.168.31.231"
TRANSPORT_HOST = "192.168.31.232"
RAIL_HOST = "192.168.31.233"

# Rail Control port
RAIL_PORT = 10001

# Arduino port
ARDU_PORT = 'COM7'

# Camera Ports
CAM_PORT_BTM = 1
CAM_PORT_TOP = 2

# Robot Constant
os.chdir(f"{PATH}\data")
# RefreshPosition Constant
with open('calibration.json') as json_file:
    CONSTANT = json.load(json_file)

# Assembly constant
COMPONENTS = ('Anode_Case', 'Anode_Spacer', 'Anode', 'Separator', 'Cathode', 'Cathode_Spacer', 'Washer', 'Cathode_Case')

# Transport Robot Test list
TESTLIST = ['Start Menu', 'Aligning Test', 'Grabing Test', 'Transporting Test', 'Retrieving Test', 'Picking-up Test', 'Sliding Test', 'Done Test']

# Position Constant(Joints)
CHECK_GRIP = [-42.5332,22.6423,4.7280,-30.9798,45.7080,66.0961]

# Assembly Robot Tool Constants
GRIPPER = 1
SUCTION = 2

# Transport Robot Robot Tool Constants
NORM = 3
FLIPED = 4

class TestAssemblyRobot(RobotController, Rail, Ardurelay):
    def __init__(self, address=ASSEMBLY_HOST, vacuum_port=ARDU_PORT):
        Rail.__init__(self)
        RobotController.__init__(self, address)
        Ardurelay.__init__(self, vacuum_port)
        self.load_parameter()
        self.reset_status()
        self.setup_logging()
        self.feedback = RobotFeedback(ASSEMBLY_HOST, '7.0.6')
        # self.status['Standby'] = (-90,0,0,0,60,0)

#----------------------Config functions----------------------

    def load_parameter(self):
        os.chdir(f"{PATH}\data")
        with open('calibration.json') as json_file:
            self.parameter = json.load(json_file)
        # Location parameter set for robot and rail.
        # Format: {'Component Name': [Holer Position, Grab z value, Drop z value, [Pre-Drop Positoion], {'Component Nr.': [Pre-grab Position]}]}

    def reset_status(self):
        self.status = dict(Tool=None, Testmode=None, Standby=CONSTANT["HOME_SK_J"], Progress=dict(Initiate=0), Initiated=False)

    def setup_logging(self, default_path='config.yaml', default_level=logging.INFO):
        path = os.path.join(os.path.dirname(__file__), 'logs', default_path)
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                config['handlers']['file']['filename'] = os.path.join(os.path.dirname(__file__), 'logs', 'Testing.log')
                config['handlers']['error']['filename'] = os.path.join(os.path.dirname(__file__), 'logs', 'Testing_Error.log')
                logging.config.dictConfig(config)
        else:
            logging.basicConfig(level=default_level)

    def choose_tool(self, component):
        self.load_parameter()
        # Automatically decide which tool to choose
        if component == COMPONENTS[6]:
            self.status["Tool"] = GRIPPER
            self.SetTRF(*self.parameter['TCP_GP'])
            self.status['Standby'] = CONSTANT["HOME_GP_J"]
        elif component in COMPONENTS[:6] or component == COMPONENTS[7]:
            self.status["Tool"] = SUCTION
            self.SetTRF(*self.parameter['TCP_SK'])
            self.status['Standby'] = CONSTANT["HOME_SK_J"]

    def get_positions(self, component, cell_nr, auto_get:bool=False):
        self.load_parameter()
        time.sleep(0.1)
        grab_po = self.parameter[component]['grabPo'][str(cell_nr)]
        tray_pos = self.parameter[component]['railPo'][cell_nr-1]
        drop_po = self.parameter[component]['dropPo']
        post_pos = self.parameter['post']
        if auto_get is True and cell_nr not in (1,9,17,25,33,41,49,57):
            grab_po[0] =self.parameter[component]['grabPo'][str(cell_nr-1)][0]
            grab_po[2] = self.parameter[component]['grabPo'][str(cell_nr-1)][2]
            # Generatate the y coordinate accroding to the first cell in coloum
            grab_po[1] = self.parameter[component]['grabPo'][str((cell_nr-1)//8*8+1)][1] - 23*((cell_nr-1)%8)
        return tray_pos, grab_po, post_pos, drop_po

#----------------------Motion functions----------------------
    def init_assembly_robot(self):
        # Initiate Rail
        rail_res = self.connect_rail(RAIL_HOST, RAIL_PORT)
        self.status["Progress"]["Initiate"] = round(1/6*100)

        # Activate robot, home, reset joints, apply parameters
        logging.debug('Initiating Assembly Robot...')
        conRes = self.connect()
        self.feedback.connect()
        time.sleep(0.1)
        if conRes is True:
            # Update status
            self.status["Progress"]["Initiate"] = round(2/6*100)
            # Activate Robot
            actRes = self.ActivateRobot()
            time.sleep(0.1)
            if actRes == 'Motors activated.':
                # Update status
                self.status["Progress"]["Initiate"] = round(3/6*100)
                # Home robot
                homRes = self.home()
                time.sleep(0.1)
                if homRes == 'Homing done.':
                    # Update status
                    self.status["Progress"]["Initiate"] = round(4/6*100)
                    # Set robot's parameter
                    self.SetGripperForce(CONSTANT['GRIP_F'])
                    self.SetGripperVel(CONSTANT['GRIP_VEL'])
                    self.SetCartLinVel(CONSTANT['L_VEL'])
                    self.SetJointVel(CONSTANT['J_VEL'])
                    self.SetJointAcc(20)
                    self.MoveJoints(*self.status['Standby'])
                    time.sleep(0.01)
                    logging.debug('Assembly Robot initiated.')
                else:
                    logging.error('Assembly Robot already homed!')
            else:
                logging.error('Assembly Robot already activated!')
        elif conRes == 'Another user is already connected, closing connection':
            logging.error('Assembly Robot already in connection!')
        else:
            logging.error('Assembly Robot is not in connection. Check Power buttom')
        
        vac_pump_res = self.connect_relay()
        
        self.status["Progress"]["Initiate"] = round(6/6*100)
        rob_res = self.GetStatusRobot()
        vac_pump_res = self.check_connection()
        check = (conRes, rob_res["Activated"], rob_res["Homing"], rail_res, vac_pump_res)
        self.status["Initiated"] = (check == (1,1,1,1,1))

    def suction_on(self):
        self.on()
        self.status["Vacuumed"] = True

    def suction_off(self):
        self.off()
        self.status["Vacuumed"] = False

    def smart_grip(self):
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

    def get_into_position(self, axis_po, arm_pose):
        rail_po = self.getPosition()
        time.sleep(0.1)
        if rail_po == axis_po:
            self.MovePose(arm_pose[0], arm_pose[1], arm_pose[2]+20, arm_pose[3], arm_pose[4], arm_pose[5])
        else:
            self.move(axis_po)
            self.MovePose(arm_pose[0], arm_pose[1], arm_pose[2]+20, arm_pose[3], arm_pose[4], arm_pose[5])
        # Record Rail location once moved
        self.rail_po = self.getPosition()

    def get_into_position_joints(self, axis_po, target_joints):
        rail_po = self.getPosition()
        time.sleep(0.1)
        if rail_po == axis_po:
            self.MoveJoints(*target_joints)
        else:
            self.move(axis_po)
            self.MoveJoints(*target_joints)
        self.rail_po = self.getPosition()

    def auto_repair(self):
        if self.is_in_error():
            self.ResetError()
        elif self.GetStatusRobot()['Paused'] == 1:
            self.ResumeMotion()
        self.ResumeMotion()
        self.ResumeMotion()

    def grip_test(self):
        ak_p = list(self.GetPose())
        self.smart_grip()
        self.MoveLinRelWRF(0,0,20,0,0,0)
        if self.status["Tool"] == SUCTION:
            self.MoveJoints(*CHECK_GRIP)
            self.MoveLinRelWRF(0,0,0,0,0,90)
            time.sleep(2)
            self.MoveLinRelWRF(0,0,0,0,0,-90)
            self.MovePose(ak_p[0],ak_p[1],ak_p[2]+20,ak_p[3],ak_p[4],ak_p[5])
        if self.status["Tool"] == GRIPPER:
            time.sleep(2)
        self.MoveLin(*ak_p)
        self.smart_drop()

    def go_home(self):
        logging.debug("Homing AssemblyRobot...")

        # Home the arm
        self.auto_repair()
        temp_po = list(self.GetPose())
        if temp_po[2] <= 80:
            self.MoveLinRelWRF(0,0,40,0,0,0)
            time.sleep(0.5)
        self.MoveJoints(*self.status['Standby'])

    def disconnect_assembly_robot(self):
    # Deactivate and disconnect the robot
        logging.debug('Disconnecting AssemblyRobot...')
        self.go_home()
        time.sleep(1.5)
        self.DeactivateRobot()
        time.sleep(0.1)
        self.disconnect_rail()
        self.disconnect_relay()
        self.disconnect()
        self.feedback.disconnect()
        time.sleep(0.1)
        self.reset_status()
        logging.debug(f"AssemblyRobot disconnected!")

    def end_assembly_test(self):
        #Shut down the rail and all robots
        return self.disconnect_assembly_robot()

    def cam_calib(self):
        self.choose_tool(component=COMPONENTS[0])
        self.GripperOpen()
        self.move(920)
        self.MovePose(*self.parameter[self.save_cam])
        self.rail_po = self.getPosition()
        # self.get_into_position_joints(axis_po=920, target_joints=self.parameter[self.save_cam])

    def asemb_test_rob(self):
        # Get every paramerter necessary
        tray_pos, grab_po, post_pos, drop_po = self.get_positions(self.component, self.current_nr, auto_get=auto_gen_var.get())
        
        # Choose the right tool
        self.choose_tool(self.component)

        self.test_asemb_status.set("Robot is now Moving, Please Wait...")
        
        if self.status['Testmode'] == 'grab':
            # Get into pre-set position
            if self.current_nr in (1, 9, 17, 25, 33, 41, 49, 57):
                self.go_home()
            else:
                self.MoveLinRelWRF(0,0,10,0,0,0)
            self.get_into_position(tray_pos, grab_po)
            self.GripperOpen()
            time.sleep(0.5)

            # Get to the actual position
            self.MoveLin(*grab_po)

            self.test_asemb_status.set(f"Ready to test the [{self.current_nr}]th {self.component}'s Grabbing Position")

        elif self.status['Testmode'] == 'drop':
            self.go_home()
            if self.current_nr != self.test_start_number:
                # Get return position
                return_tray, return_po, _, _ = self.get_positions(self.component, self.current_nr-1)
                self.get_into_position(return_tray, return_po)
                self.MoveLin(*return_po)
                self.smart_drop()
                self.go_home()
            # Get into pre-set position
            self.get_into_position(tray_pos, grab_po)
            self.GripperOpen()
            time.sleep(0.5)

            # Get to the actual position
            self.MoveLin(*grab_po)

            # Get the component
            self.smart_grip()
            
            self.go_home()

            # To the predrop position
            self.get_into_position(post_pos, drop_po)

            # To the actual drop position
            self.MoveLin(*drop_po)
            self.test_asemb_status.set(f"Ready to test the {self.component}'s Dropping Position")

        elif self.status['Testmode'] == 'sub_grab':
            # Get into pre-set position
            self.go_home()
            self.get_into_position(tray_pos, grab_po)
            self.GripperOpen()
            time.sleep(0.5)
            self.MoveLin(*grab_po)
            self.test_asemb_status.set(f"Ready to test the [{self.current_nr}]th {self.component}'s Grabbing Position")
        elif self.status['Testmode']  == 'sub_drop':
            self.smart_grip()
            self.go_home()
            self.get_into_position(post_pos, drop_po)
            self.MoveLin(*drop_po)
            self.test_asemb_status.set(f"Ready to test the {self.component}'s Dropping Position")
        elif self.status['Testmode']  == 'sub_done':
            self.go_home()
            self.get_into_position(tray_pos, grab_po)
            self.MoveLin(*grab_po)
            self.smart_drop()
            self.go_home()
            self.test_asemb_status.set(f"Testing Done")

#----------------------GUI functions----------------------

    def initiate_sys(self):
        os.chdir(f"{PATH}\images")
        prog_window = Toplevel()
        prog_window.title("Assembly Robot initializing")
        prog_window.iconbitmap("Robotarm.ico")
        prog_window.geometry('280x150')
        prog_text = StringVar()
        prog_label = Label(prog_window, textvariable=prog_text, font=ft_label, pady=10, anchor=CENTER)
        prog = ttk.Progressbar(prog_window, length=250, mode='determinate', orient=HORIZONTAL)
        prog_label.grid(row=2, column=0, columnspan=2)
        prog.grid(row=1, column=0, columnspan=2, pady=20, sticky=W+E)
        threading.Thread(name='startsystem', target=self.init_assembly_robot, daemon=True).start()
        def update_probar():
            prog['value'] = self.status['Progress']['Initiate']
            prog_text.set(f"Initiating System, Please Wait ({prog['value']}%)")
            if prog['value'] == 100:
                prog_text.set(f"Initiating Finishing (100%)...")
                prog_window.update()
                time.sleep(1)
                prog_window.destroy()
            else:
                prog_window.after(30, update_probar)
        update_probar()
        prog_window.mainloop()
                

    def free_move(self):
        self.exit_fm_flag = False
        os.chdir(f"{PATH}\images")
        try:
            self.test_aseembly_run_window.state(newstate='iconic')
        except AttributeError:
            cam_calib_window.state(newstate='iconic')
        free_move_window = Toplevel()

        # Set the title, icon, size of the initial window
        free_move_window.title("Freemove GUI")
        free_move_window.iconbitmap("Robotarm.ico")
        free_move_window.geometry("830x660")

        # Create status bar
        free_move_status = StringVar()
        status_label = Label(free_move_window, textvariable=free_move_status, pady=10, bd=1, relief=SUNKEN, anchor=W)
        status_label.grid(row=0, column=0, columnspan=2, padx=20, sticky=W+E)

        # Update the status
        def refresh_status():
            while self.exit_fm_flag != True:
                self.feedback.get_data()
                free_move_status.set(f"Robot Pose: {self.feedback.cartesian}, Rail Location: {self.rail_po}")
        # Create the control panel
        free_move_frame = LabelFrame(free_move_window, text="Movement Control Panel",\
            padx=25, pady=30, borderwidth=5)
        free_move_frame.grid(row=1, column=0, padx=20, pady=10)

        # Add input box for increment to functional Panel
        free_move_frame_increm = LabelFrame(free_move_frame, text="Increment: ",\
            font=ft_label, pady=8, borderwidth=3)
        self.increment = Entry(free_move_frame_increm, width=5, borderwidth=5)
        unit = Label(free_move_frame_increm, text="mm", font=ft_label)

        free_move_frame_increm.grid(row=0, column=0, padx=10, pady=5)
        self.increment.grid(row=0, column=0)
        unit.grid(row=0, column=1)

        self.increment.insert(0, '0.2')

        free_move_frame_rxyz = LabelFrame(free_move_frame, text="\u03B1-\u03B2-\u03B3-Axis: ",\
            font=ft_label, padx=35, borderwidth=3)
        free_move_frame_rxyz.grid(row=0, column=1)

        free_move_frame_z = LabelFrame(free_move_frame, text="Z-Axis: ",\
            font=ft_label, padx=10, pady=30, borderwidth=3)
        free_move_frame_z.grid(row=1, column=0, padx=10, pady=10)

        free_move_frame_xy = LabelFrame(free_move_frame, text="XY-Axis: ",\
            font=ft_label, padx=20, pady=30, borderwidth=3)
        free_move_frame_xy.grid(row=1, column=1, padx=10, pady=10)

        free_move_frame_rail = LabelFrame(free_move_frame, text="Rail X-Axis: ",\
            font=ft_label, padx=88, borderwidth=3)
        free_move_frame_rail.grid(row=2, column=0, columnspan=2)

        # Add buttons to the control panel
        up_btn = Button(free_move_frame_z, image=arrow_up, padx=10, pady=40, border=5,\
            borderwidth=4, command=lambda: self.free_move_control('+z'))
        down_btn = Button(free_move_frame_z, image=arrow_down, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('-z'))
        left_btn = Button(free_move_frame_xy, image=arrow_left, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('-x'))
        right_btn = Button(free_move_frame_xy, image=arrow_right, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('+x'))
        forward_btn = Button(free_move_frame_xy, image=arrow_up, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('+y'))
        backward_btn = Button(free_move_frame_xy, image=arrow_down, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('-y'))
        
        centrer_label_1 = Label(free_move_frame_z, image=centrer, padx=10, pady=40,\
            border=5, state=DISABLED)
        centrer_label_2 = Label(free_move_frame_xy, image=centrer, padx=10, pady=40,\
            border=5, state=DISABLED)

        rx_btn = Button(free_move_frame_rxyz, text="\u0394\u03B1", font=ft_button,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('rx'))
        ry_btn = Button(free_move_frame_rxyz, text="\u0394\u03B2", font=ft_button, border=5,\
            borderwidth=4, command=lambda: self.free_move_control('ry'))
        rz_btn = Button(free_move_frame_rxyz, text="\u0394\u03B3", font=ft_button, border=5,\
            borderwidth=4, command=lambda: self.free_move_control('rz'))
        rail_poitive_btn = Button(free_move_frame_rail, image=arrow_right, border=5,\
            borderwidth=4, command=lambda: self.free_move_control('+rail'))
        rail_negative_btn = Button(free_move_frame_rail, image=arrow_left, border=5,\
            borderwidth=4, command=lambda: self.free_move_control('-rail'))

        up_btn.grid(row=0, column=0, padx=10)
        down_btn.grid(row=2, column=0, padx=10)
        centrer_label_1.grid(row=1, column=0)
        left_btn.grid(row=1, column=0)
        right_btn.grid(row=1, column=2)
        forward_btn.grid(row=0, column=1)
        backward_btn.grid(row=2, column=1)
        centrer_label_2.grid(row=1, column=1)
        rx_btn.grid(row=0, column=0, padx=10)
        ry_btn.grid(row=0, column=1, padx=10)
        rz_btn.grid(row=0, column=2, padx=10)
        rail_negative_btn.grid(row=0, column=0, padx=20)
        rail_poitive_btn.grid(row=0, column=1, padx=20)

        # Create Functional Frame
        function_frame = LabelFrame(free_move_window, text="Function Control Panel",\
            padx=25, pady=7, borderwidth=5)
        function_frame.grid(row=1, column=1, padx=5, pady=20)

        gripper_control_frame = LabelFrame(function_frame, text="Gripper Control",\
            padx=30, pady=10, borderwidth=5)
        gripper_control_frame.grid(row=0, column=0, padx=5, pady=5)

        vacuum_control_frame = LabelFrame(function_frame, text="Vacuum Control",\
            padx=30, pady=10, borderwidth=5)
        vacuum_control_frame.grid(row=1, column=0, padx=5, pady=5)

        position_control_frame = LabelFrame(function_frame, text="Position Control",\
            padx=30, pady=10, borderwidth=5)
        position_control_frame.grid(row=2, column=0, padx=5, pady=5)

         # Add functional buttons to the functional panel
        global open_gripper_btn, close_gripper_btn, open_vacuum_btn, close_vacuum_btn, observe_btn
        open_gripper_btn = Button(gripper_control_frame, text="Open Gripper",\
            border=5, padx=24, pady=10, borderwidth=4, command=lambda: self.free_move_control('+gripper'))
        close_gripper_btn = Button(gripper_control_frame, text="Close Gripper",\
            padx=24, pady=10, border=5, borderwidth=4, command=lambda: self.free_move_control('-gripper'))
        open_vacuum_btn = Button(vacuum_control_frame, text="Open Vacuum",\
            padx=20, pady=10, border=5, borderwidth=4, command=lambda: self.free_move_control('+vacuum'))
        close_vacuum_btn = Button(vacuum_control_frame, text="Close Vacuum",\
            padx=20, pady=10, border=5, borderwidth=4, command=lambda: self.free_move_control('-vacuum'))
        test_btn = Button(position_control_frame, text="Test Position",\
            padx=25, pady=5, border=5, borderwidth=4, command=lambda: self.free_move_control('gripper test'))
        observe_btn = Button(position_control_frame, text="Observe Position",\
            padx=15, pady=5, border=5, borderwidth=4, command=lambda: self.free_move_control('observe position'))
        save_btn = Button(position_control_frame, text="Save Position",\
            padx=23, pady=5, border=5, borderwidth=4, command=self.save_position)

        def exit_and_back():
            self.exit_fm_flag = True
            free_move_window.destroy()
            try:
                self.test_aseembly_run_window.state(newstate='normal')
            except AttributeError:
                cam_calib_window.state(newstate='normal')
        
        exit_btn = Button(position_control_frame, text="Exit", padx=48, pady=5, borderwidth=4, command=exit_and_back)

        open_gripper_btn.grid(row=0, column=0)
        close_gripper_btn.grid(row=1, column=0, pady=5)
        open_vacuum_btn.grid(row=0, column=0)
        close_vacuum_btn.grid(row=1, column=0, pady=5)
        test_btn.grid(row=0, column=0)
        observe_btn.grid(row=1, column=0, pady=5)
        save_btn.grid(row=2, column=0)
        exit_btn.grid(row=3, column=0, pady=5)

        if self.status['Testmode'] in ('drop', 'sub_drop'):
            test_btn['state'] = 'disabled'
        else:
            observe_btn['state'] = 'disabled'

        threading.Thread(name='refresh_status', target=refresh_status, daemon=True).start()
        free_move_window.mainloop()

    def free_move_rob(self, axis, step):
        if axis == '+x':
            self.MoveLinRelWRF(0,0,2,0,0,0)
            self.MoveLinRelWRF(abs(step),0,0,0,0,0)
            self.MoveLinRelWRF(0,0,-2,0,0,0)
        elif axis == '-x':
            self.MoveLinRelWRF(0,0,2,0,0,0)
            self.MoveLinRelWRF(-abs(step),0,0,0,0,0)
            self.MoveLinRelWRF(0,0,-2,0,0,0)
        elif axis == '+y':
            self.MoveLinRelWRF(0,0,2,0,0,0)
            self.MoveLinRelWRF(0,abs(step),0,0,0,0)
            self.MoveLinRelWRF(0,0,-2,0,0,0)
        elif axis == '-y':
            self.MoveLinRelWRF(0,0,2,0,0,0)
            self.MoveLinRelWRF(0,-abs(step),0,0,0,0)
            self.MoveLinRelWRF(0,0,-2,0,0,0)
        elif axis == '+z':
            self.MoveLinRelWRF(0,0,abs(step),0,0,0)
        elif axis == '-z':
            self.MoveLinRelWRF(0,0,-abs(step),0,0,0)
        elif axis == 'rx':
            self.MoveLinRelWRF(0,0,0,step,0,0)
        elif axis == 'ry':
            self.MoveLinRelWRF(0,0,0,0,step,0)
        elif axis == 'rz':
            self.MoveLinRelWRF(0,0,0,0,0,step)
        elif axis == '+rail':
            self.rel_move(abs(step))
            self.rail_po = self.getPosition()
        elif axis == '-rail':
            self.rel_move(-abs(step))
            self.rail_po = self.getPosition()
        elif axis == '+gripper':
            self.GripperOpen()
        elif axis == '-gripper':
            self.GripperClose()
        elif axis == '+vacuum':
            self.suction_on()
        elif axis == '-vacuum':
            self.suction_off()
        elif axis == 'gripper test':
            self.grip_test()
        elif axis == 'observe position':
            act_pos = list(self.GetPose())
            self.suction_off()
            time.sleep(4)
            self.MoveLin(act_pos[0],act_pos[1],act_pos[2]+20,act_pos[3],act_pos[4],act_pos[5])
            self.MoveJoints(*self.parameter['SNAP_SHOT_J'])
            self.MoveLinRelWRF(0,0,-20,0,0,0)
        elif axis == 'return position':
            self.MoveLinRelWRF(0,0,40,0,0,0)
            self.MovePose(act_pos[0],act_pos[1],act_pos[2]+20,act_pos[3],act_pos[4],act_pos[5])
            self.MoveLin(*act_pos)
            self.suction_on()

    def free_move_control(self, axis):
        try:
            step = float(self.increment.get())
        except ValueError:
            messagebox.showerror("Input Error!", "Only positive float is accepted")
        else:
            if axis == '+gripper':
                open_gripper_btn['state'] = 'disable'
                close_gripper_btn['state'] = 'normal'
            elif axis == '-gripper':
                open_gripper_btn['state'] = 'normal'
                close_gripper_btn['state'] = 'disable'
            elif axis == '+vacuum':
                open_vacuum_btn['state'] = 'disable'
                close_vacuum_btn['state'] = 'normal'
            elif axis == '-vacuum':
                open_vacuum_btn['state'] = 'normal'
                close_vacuum_btn['state'] = 'disable'
            elif axis == 'observe position':
                observe_btn.config(text='Return position', command=lambda: self.free_move_control('return position'))
            elif axis == 'return position':
                observe_btn.config(text='Observe position', command=lambda: self.free_move_control('observe position'))
            threading.Thread(name='FreeMove', target=self.free_move_rob, args=(axis, step,)).start()

    def set_test_assembly(self, mainLev:Tk=None):
        os.chdir(f"{PATH}\images")
        # Start a new window
        self.test_assembly_config_window = Toplevel()
        self.test_assembly_config_window.title("Assembly Robot Testing Interface")
        self.test_assembly_config_window.iconbitmap("Robotarm.ico")
        # self.test_assembly_config_window.geometry("460x490")

        # Load images
        global arrow_left, arrow_right, arrow_up, arrow_down, centrer, done
        arrow_left = ImageTk.PhotoImage(Image.open("arrow_left.png"))
        arrow_right = ImageTk.PhotoImage(Image.open("arrow_right.png"))
        arrow_up = ImageTk.PhotoImage(Image.open("arrow_up.png"))
        arrow_down = ImageTk.PhotoImage(Image.open("arrow_down.png"))
        centrer = ImageTk.PhotoImage(Image.open("centrer.png"))
        done = ImageTk.PhotoImage(Image.open("done.png"))

        # Specify font of labels and button's text
        global ft_label, ft_button
        ft_label = font.Font(family='Arial', size=10, weight=font.BOLD)
        ft_button = font.Font(size=15)

        # Creat frame
        test_assembly_frame = LabelFrame(self.test_assembly_config_window, padx= 50, pady=30, borderwidth=5)
        test_assembly_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

        # Creat a seris of radiobuttons for mode switching
        mode_switch_frame = LabelFrame(test_assembly_frame, text="Testing Mode:  ", padx=45, pady=10, borderwidth=2)
        mode_switch_frame.grid(row=0, column=0, columnspan=2)

        self.test_mode_input = StringVar()
        self.test_mode_input.set("grab")
 
        grip = Radiobutton(mode_switch_frame, text="Grab", variable=self.test_mode_input, value='grab')
        drop = Radiobutton(mode_switch_frame, text="Drop", variable=self.test_mode_input, value='drop')
        both = Radiobutton(mode_switch_frame, text="Grab+Drop", variable=self.test_mode_input, value='both')
        cam = Radiobutton(mode_switch_frame, text="Cam", variable=self.test_mode_input, value='Cam')

        grip.grid(row=0, column=1)
        drop.grid(row=0, column=2)
        both.grid(row=0, column=3)
        cam.grid(row=0, column=4)

        # Create components menu
        components_label = Label(test_assembly_frame, text="Component to Test: ", padx=15, pady=15, font=ft_label)

        components_label.grid(row=1, column=0)

        components = list(COMPONENTS)

        self.test_component = StringVar()

        self.test_component.set(components[0])

        drop = OptionMenu(test_assembly_frame, self.test_component, *components)
        drop.grid(row=1, column=1)

        # Creat labels
        start_number_label = Label(test_assembly_frame, text="First Cell to test:", pady=15, font=ft_label)
        end_number_label = Label(test_assembly_frame, text="Last Cell to test:", pady=15, font=ft_label)

        start_number_label.grid(row=2, column=0)
        end_number_label.grid(row=3, column=0)
        
        # Create input fields
        self.test_start_number_input = Entry(test_assembly_frame, width=10, borderwidth=5)
        self.test_end_number_input = Entry(test_assembly_frame, width=10, borderwidth=5)

        self.test_start_number_input.grid(row=2, column=1)
        self.test_end_number_input.grid(row=3, column=1)

        # Creat checkbox
        global auto_gen_var
        auto_gen_var = BooleanVar()
        auto_gen = Checkbutton(test_assembly_frame, text="Smart Location", variable=auto_gen_var, onvalue=True, offvalue=False)
        auto_gen.deselect()

        auto_gen.grid(row=4, column=0, columnspan=2)

        def exit_and_back():
            self.test_assembly_config_window.destroy()
            threading.Thread(target=self.go_home, daemon=True).start()
            if mainLev:
                mainLev.state(newstate='normal')

        # Create assembly button
        initiate_btn = Button(test_assembly_frame, text="Initiate System", font=ft_button,\
                            padx=5, pady=5, borderwidth=4, command=self.initiate_sys)
        start_test_btn = Button(test_assembly_frame, text="Start Testing!", font=ft_button,\
                            padx=5, pady=5, borderwidth=4, command=self.config_test_assembly)
        # cam_test_btn = Button(test_assembly_frame, text="Calibrate Camera", font=ft_button,\
        #                     padx=5, pady=5, borderwidth=4, command=self.Camcalib_gui)
        exit = Button(self.test_assembly_config_window, text="Exit", font=ft_button,\
                padx=45, borderwidth=4, command=exit_and_back)
        
        initiate_btn.grid(row=5, column=0, padx=5, pady=15)
        start_test_btn.grid(row=5, column=1, padx=5, pady=15)
        # cam_test_btn.grid(row=6, column=0, columnspan=2, padx=5, pady=5)
        exit.grid(row=3, column=0, columnspan=2, padx=10, pady=25)

        self.test_assembly_config_window.mainloop()

    def Camcalib_gui(self):
        self.exit_flag = False
        self.save_H_mtx = None
        self.save_cam = None
        self.cam_name = None
        self.arucoDict = None
        self.arucoSize = None
        def btmCam_calib():
            self.exit_flag = True
            self.save_cam = 'SNAP_SHOT_GRAB_PO'
            self.cam_name = 'Grab'
            self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
            self.arucoSize = 20
            try:
                self.cap.release()
            except:
                logging.debug("Top camera is NOT online.")
            self.cam_calib()
            threading.Thread(target=self.show_frames, args=(CAM_PORT_BTM,), daemon=True).start()
            # self.showCam(CAM_PORT_BTM)

        def topCam_calib():
            self.exit_flag = True
            self.save_cam = 'SNAP_SHOT_DROP_PO'
            self.cam_name = 'Drop'
            self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
            self.arucoSize = 15
            try:
                self.cap.release()
            except:
                logging.debug("Bottom camera is NOT online.")
            self.cam_calib()
            threading.Thread(target=self.show_frames, args=(CAM_PORT_TOP,), daemon=True).start()
            # self.showCam(CAM_PORT_TOP)

        def save_frame():
            time_stamp = time.strftime("%Y_%m_%d_%Hh_%Mm_%Ss", time.localtime())
            dir_name = os.path.join(PATH, 'Alignments', 'Camera_Alignment', time_stamp[:10])
            filename = f"Calibration_{self.cam_name}.jpg"
            if not os.path.exists(dir_name):
                os.makedirs(dir_name)
            os.chdir(dir_name)
            cv.imwrite(filename, frame)
            messagebox.showinfo("Info", "Image has been saved")

        def save_H_mtx():
            self.save_H_mtx = True

        def exit_show():
            self.exit_flag = True
        
        os.chdir(f"{PATH}\images")
        self.test_assembly_config_window.state(newstate='iconic')
        global cam_calib_window
        cam_calib_window = Toplevel()
        cam_calib_window.title("Running Camera Calibration")
        cam_calib_window.iconbitmap("Robotarm.ico")

        # Create status bar
        self.cam_calib_status = StringVar()
        status_label = Label(cam_calib_window, textvariable=self.cam_calib_status, font=font.Font(family='Arial', size=8, weight=font.BOLD), pady=10, bd=1, relief=SUNKEN)
        status_label.grid(row=0, column=0, columnspan=2, padx=20, pady=10, sticky=W+E)
        self.cam_calib_status.set("Choosing from followin cameras to test: ")

        # Creat frame
        test_asemb_frame = LabelFrame(cam_calib_window, padx=10, pady=10, borderwidth=5)
        test_asemb_frame.grid(row=1, column=0, columnspan=2, padx=20, pady=5)

        # Create stop buttons
        home_btn = Button(test_asemb_frame, text="Home", padx=50, pady=5, borderwidth=4, command=lambda:threading.Thread(name='homing', target=self.go_home).start())
        save_btn = Button(test_asemb_frame, text="Save", padx=55, pady=5, borderwidth=4, command=self.save_position)
        free_move_btn = Button(test_asemb_frame, text="Free-move", font=ft_button, padx=16, pady=20, borderwidth=4, command=self.free_move)
        botCam_btn = Button(test_asemb_frame, text="Btm Camera", font=ft_button, padx=10, pady=20, border=5, borderwidth=4, 
                            command=lambda:threading.Thread(target=btmCam_calib(), daemon=True).start())
        topCam_btn = Button(test_asemb_frame, text="Top Camera", font=ft_button, padx=10, pady=20, border=5, borderwidth=4, 
                            command=lambda:threading.Thread(target=topCam_calib(), daemon=True).start())
        
        botCam_btn.grid(row=0, column=0, padx=20)
        topCam_btn.grid(row=0, column=1, padx=20)
        free_move_btn.grid(row=1, column=0, rowspan=2, pady=10)
        home_btn.grid(row=1, column=1)
        save_btn.grid(row=2, column=1)

        show_control = LabelFrame(cam_calib_window, padx=10, pady=10, borderwidth=5)
        show_control.grid(row=2, column=0, columnspan=2, padx=20, pady=5)

        save_image_btn = Button(show_control, text="Save Image", padx=15, pady=10, borderwidth=4, command=save_frame)
        exit_show_btn = Button(show_control, text="Exit Show", padx=15, pady=10, borderwidth=4, command=exit_show)
        save_H_mtx_btn = Button(show_control, text="Save H_MTX", padx=15, pady=10, borderwidth=4, command=save_H_mtx)
        save_image_btn.grid(row=0, column=0, padx=10, pady=5)
        exit_show_btn.grid(row=0, column=2, padx=10, pady=5)
        save_H_mtx_btn.grid(row=0, column=1, padx=35, pady=5)

        def exit_test_asemb():
            cam_calib_window.destroy()
            self.test_assembly_config_window.state(newstate='normal')

        exit = Button(cam_calib_window, text="Exit", font=ft_button, padx=45, borderwidth=4, command=exit_test_asemb)
        exit.grid(row=3, column=0, columnspan=2, padx=10, pady=10)

    def detect_aruco(self, img):
        image = np.copy(img)
        (h, w) = img.shape[:2]
        imageCenter = (w//2, h//2)

        # detect the ArUco markers in the image
        corners, ids, _ = cv.aruco.detectMarkers(image, self.arucoDict)

        # create a list to store the pixel values of the ArUco markers
        aruco_pts = []

        try:
            detection = len(ids)
        except TypeError:
            cv.putText(image, "None Aruco Marker!", (15,15),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            # convert the corners from floating-point to integer pixel values
            corners = np.int0(corners)

            # get the pixel values of the current marker corners
            x, y = np.transpose(corners[0][0])

            # calculate the center point of the current marker
            center_pt = np.int0([np.mean(x), np.mean(y)])

            aruco_pts = list(corners[0][0])
            aruco_pts.append(center_pt)

            if np.allclose(imageCenter, aruco_pts[-1], atol=1):
                if abs(aruco_pts[0][1]-aruco_pts[1][1]) > 1 or abs(aruco_pts[0][0]-aruco_pts[3][0]) > 1:
                    cv.drawMarker(image,aruco_pts[-1],(255,255,0),cv.MARKER_CROSS,5,1)
                    cv.line(img=image, pt1=aruco_pts[0], pt2=aruco_pts[1], color=(0, 0, 255), thickness=1, lineType=8, shift=0)
                    cv.arrowedLine(img=image, pt1=aruco_pts[0], pt2=(aruco_pts[1][0]+20, aruco_pts[0][1]), color=(0, 255, 0), thickness=1)
                    cv.line(img=image, pt1=aruco_pts[0], pt2=aruco_pts[3], color=(0, 0, 255), thickness=1, lineType=8, shift=0)
                    cv.arrowedLine(img=image, pt1=aruco_pts[0], pt2=(aruco_pts[0][0], aruco_pts[3][1]+20), color=(0, 255, 0), thickness=1)
                    cv.putText(image, "Adjust Angle!", (15,15),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                else:
                    cv.drawMarker(image,aruco_pts[-1],(0,255,0),cv.MARKER_TILTED_CROSS,10,1)
                    cv.putText(image, "Aligned!", (15,15),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            elif np.allclose(imageCenter, aruco_pts[-1], atol=20):
                cv.drawMarker(image,aruco_pts[-1],(255,255,0),cv.MARKER_CROSS,5,1)
                cv.drawMarker(image,imageCenter,(0,0,255),cv.MARKER_CROSS,5,1)
                cv.arrowedLine(img=image, pt1=imageCenter, pt2=aruco_pts[-1], color=(0, 255, 0), thickness=1, shift=0)
                cv.putText(image, "Missaligned!", (15,15),
                    cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            else:
                cv.putText(image, "Aruco Marker Too Far!", (15,15),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # return the list of aruco pixels
        return image, np.array(aruco_pts, dtype=np.float32)
    
    def get_H_mtx(self, aruco_pts):
        size = self.arucoSize
        if self.cam_name == 'Drop':
            dst_pts = np.array([
                        [-size/2, size/2, 0, 1],
                        [size/2, size/2, 0, 1],
                        [size/2, -size/2, 0, 1],
                        [-size/2, size/2, 0, 1],
                        [0, 0, 0, 1],], dtype=np.float32)
        elif self.cam_name == 'Grab':
            dst_pts = np.array([
                        [-size/2, -size/2, 0, 1],
                        [size/2, -size/2, 0, 1],
                        [size/2, size/2, 0, 1],
                        [-size/2, size/2, 0, 1],
                        [0, 0, 0, 1],], dtype=np.float32)

        dst_pts = np.array([[x/w, y/w] for x,y,z,w in dst_pts])

        #find the 3x3 Homography Matrix for transforming image plane to floor plane
        H_mtx, _ = cv.findHomography(aruco_pts, dst_pts)

        # Check the total error
        tot_error = self.compute_error(aruco_pts, dst_pts, H_mtx)
        logging.info(f"H_mtx: {H_mtx} -- Total error (X,Y): {tot_error}")

        # Serialize numpy array for dumping into json file
        H_mtx = np.array(H_mtx, dtype=np.float32)

        self.parameter[f"H_mtx_{self.cam_name}"] = H_mtx.tolist()

        with open(os.path.join(PATH, 'data', 'calibration.json'), 'w') as json_file:
            json.dump(self.parameter, json_file, indent=4)
        messagebox.showinfo("Info", f"Homogenous Matrix [{self.cam_name}] has been saved:\n{H_mtx}\nTotal Error: {tot_error}")
        self.save_H_mtx = False

    def compute_error(self, aruco_pts, dst_pts, H_mtx): 
        err_X = []
        err_Y = []
        for image_coordinate, dst_coordinate in zip(aruco_pts, dst_pts):
            x, y, w = H_mtx @ np.array([[*image_coordinate[:2], 1]]).T
            X, Y = np.around(x/w, decimals=6), np.around(y/w, decimals=6) # Transform homogenous coordinates into cart coordinates
            err_X.append(dst_coordinate[0]-X)
            err_Y.append(dst_coordinate[1]-Y)
        return np.array([np.mean(err_X), np.mean(err_Y)], dtype=np.float32)

    def detect_object_center(self, img, obj_id:int):
        object_config = CONFIG[OBJECT_LIST[obj_id-1]]
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img_gray = cv.medianBlur(img_gray, 5)

        circles = cv.HoughCircles(img_gray, cv.HOUGH_GRADIENT, 1, object_config['minDist'],
                                param1=object_config['param1'], param2=object_config['param2'],
                                minRadius=object_config['minR'], maxRadius=object_config['maxR'])
        (h, w) = img.shape[:2]
        imageCenter = (w//2, h//2)
        cv.line(img=img, pt1=(imageCenter[0]-object_config['minR'], imageCenter[1]), pt2=(imageCenter[0]+object_config['minR'], imageCenter[1]), color=(255, 0, 0), thickness=1, lineType=8, shift=0)
        cv.line(img=img, pt1=(imageCenter[0], imageCenter[1]-object_config['minR']), pt2=(imageCenter[0], imageCenter[1]+object_config['minR']), color=(255, 0, 0), thickness=1, lineType=8, shift=0)
        
        # Mark the center of the inner circle
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle outline
                radius = i[2]
                if np.allclose(imageCenter, center, atol=2):
                    cv.circle(img, center, radius, (0,255,0), 1)
                elif np.allclose(imageCenter, center, atol=20):
                    cv.circle(img, center, radius, (0,0,255), 1)
        return img

    def show_frames(self, cam_id):
        if self.exit_flag:
            self.exit_flag = False
        self.cap= cv.VideoCapture(cam_id, cv.CAP_DSHOW)
        while self.cap.isOpened():
            global frame
            ret, frame = self.cap.read()
            if not ret:
                logging.error("Can't receive frame (stream end?). Exiting ...")
                break
            frame_output, aruco_pts = self.detect_aruco(frame)
            if self.save_H_mtx == True:
                self.get_H_mtx(aruco_pts)
            cv.imshow(f'Camera_{self.cam_name} View', frame_output)
            cv.waitKey(1)
            if self.exit_flag:
                self.cap.release()
                cv.destroyAllWindows()
                break
    
    def config_test_assembly(self):
        # Get and check the passed value
        self.status['Testmode'] = self.test_mode_input.get()
        self.component = self.test_component.get()
        if self.status['Testmode'] == 'Cam':
            if self.status['Initiated'] is True:
                self.Camcalib_gui()
            else:
                messagebox.showerror("Error!", "Initiate System first!")
        else:
            try:
                self.test_start_number = int(self.test_start_number_input.get())
                self.test_end_number = int(self.test_end_number_input.get())
            except ValueError:
                self.test_start_number_input.delete(0, END)
                self.test_end_number_input.delete(0, END)
                messagebox.showerror("Input Error", "Only positive integers are accepted!")
            else:
                if self.test_start_number <= 0 or self.test_start_number > 64 or self.test_end_number <= 0 or self.test_end_number > 64:
                    messagebox.showerror("Input Error", "Input Numbers are out of range (1-64)!")
                    self.test_start_number_input.delete(0, END)
                    self.test_end_number_input.delete(0, END)
                elif self.test_end_number < self.test_start_number:
                    messagebox.showerror("Input Error", "Starting number must be higher than ending number!")
                    self.test_start_number_input.delete(0, END)
                    self.test_end_number_input.delete(0, END)
                elif self.status['Initiated'] is True:
                    self.current_nr = self.test_start_number
                    self.init_asemb_test_gui()
                    threading.Thread(name=self.status['Testmode'], target=self.asemb_test_rob, daemon=True).start()
                else:
                    messagebox.showerror("Error!", "Initiate System first!")
            
    def init_asemb_test_gui(self):
        os.chdir(f"{PATH}\images")
        self.test_assembly_config_window.state(newstate='iconic')
        self.test_aseembly_run_window = Toplevel()
        self.test_aseembly_run_window.title("Running Assembly Test")
        self.test_aseembly_run_window.iconbitmap("Robotarm.ico")
        self.test_aseembly_run_window.geometry("535x350")

        # Create status bar
        self.test_asemb_status = StringVar()
        status_label = Label(self.test_aseembly_run_window, textvariable=self.test_asemb_status, font=ft_label, pady=10, bd=1, relief=SUNKEN)
        status_label.grid(row=0, column=0, columnspan=2, padx=20, pady=10, sticky=W+E)

        # Creat frame
        self.test_asemb_frame = LabelFrame(self.test_aseembly_run_window, padx=10, pady=10, borderwidth=5)
        self.test_asemb_frame.grid(row=1, column=0, padx=20, pady=5)

        global grab_btn, drop_btn, save_btn, left_btn, right_btn

        def asemb_test_gui_sub_grab():
            grab_btn['state'] = 'disabled'
            drop_btn.config(text='Drop', command=asemb_test_gui_sub_drop, state=NORMAL)
            self.status['Testmode'] = 'sub_grab'
            threading.Thread(name=self.status['Testmode'], target=self.asemb_test_rob, daemon=True).start()

        def asemb_test_gui_sub_drop():
            grab_btn['state'] = 'normal'
            drop_btn.config(text='Done', command=asemb_test_gui_sub_done)
            self.status['Testmode'] = 'sub_drop'
            threading.Thread(name=self.status['Testmode'], target=self.asemb_test_rob, daemon=True).start()
            # self.asemb_test_rob_thread()

        def asemb_test_gui_sub_done():
            drop_btn.config(text='Drop', command=asemb_test_gui_sub_drop, state=DISABLED)
            self.status['Testmode'] = 'sub_done'
            threading.Thread(name=self.status['Testmode'], target=self.asemb_test_rob, daemon=True).start()
            # self.asemb_test_rob_thread()

        def asemb_test_gui_next():
            if not self.status['Testmode'] in ['grab', 'drop']:
                grab_btn['state'] = 'disabled'
                drop_btn['state'] = 'normal'
                self.status['Testmode'] = 'sub_grab'
            self.current_nr += 1
            if self.current_nr == self.test_end_number:
                left_btn['state'] = 'normal'
                right_btn['state'] = 'disabled'
            else:
                left_btn['state'] = 'normal'
                right_btn['state'] = 'normal'
            threading.Thread(name=self.status['Testmode'], target=self.asemb_test_rob, daemon=True).start()
            # self.asemb_test_rob_thread()

        def asemb_test_gui_back():
            if not self.status['Testmode'] in ['grab', 'drop']:
                grab_btn['state'] = 'disabled' 
                drop_btn['state'] = 'normal'
                self.status['Testmode'] = 'sub_grab'
            self.current_nr -= 1
            if self.current_nr == self.test_start_number:
                right_btn['state'] = 'normal'
                left_btn['state'] = 'disabled'
            else:
                left_btn['state'] = 'normal'
                right_btn['state'] = 'normal'
            threading.Thread(name=self.status['Testmode'], target=self.asemb_test_rob, daemon=True).start()
            # self.asemb_test_rob_thread()

        # Create stop buttons
        grab_btn = Button(self.test_asemb_frame, text="Grab", padx=18, pady=5, borderwidth=4, command=asemb_test_gui_sub_grab)
        drop_btn = Button(self.test_asemb_frame, text="Drop", padx=17, pady=5, borderwidth=4, command=asemb_test_gui_sub_drop)
        home_btn = Button(self.test_asemb_frame, text="Home", padx=35, pady=5, borderwidth=4, command=lambda:threading.Thread(name='homing', target=self.go_home).start())
        save_btn = Button(self.test_asemb_frame, text="Save", padx=38, pady=5, borderwidth=4, command=self.save_position)
        free_move_btn = Button(self.test_asemb_frame, text="Free-move", font=ft_button, pady=15, borderwidth=4, command=self.free_move)
        left_btn = Button(self.test_asemb_frame, image=arrow_left, padx=10, pady=40, border=5, borderwidth=4, command=asemb_test_gui_back, state=DISABLED)
        right_btn = Button(self.test_asemb_frame, image=arrow_right, padx=10, pady=40, border=5, borderwidth=4, command=asemb_test_gui_next)
        
        left_btn.grid(row=0, column=0, padx=50)
        free_move_btn.grid(row=0, column=1)
        grab_btn.grid(row=1, column=0, pady=20)
        home_btn.grid(row=1, column=1)
        save_btn.grid(row=2, column=1)
        drop_btn.grid(row=1, column=2, pady=20)
        right_btn.grid(row=0, column=2, padx=50)

        def exit_test_asemb():
            self.test_aseembly_run_window.destroy()
            self.test_assembly_config_window.state(newstate='normal')

        exit = Button(self.test_aseembly_run_window, text="Exit", padx=45, borderwidth=4, command=exit_test_asemb)
        exit.grid(row=2, column=0, padx=10, pady=10)

        if self.test_start_number == self.test_end_number:
            right_btn['state'] = 'disabled'
            left_btn['state'] = 'disabled'  
        if self.status['Testmode'] != 'both':
            self.test_asemb_status.set(f"Ready to test the [{self.current_nr}]th {self.component}'s {self.status['Testmode'].capitalize()} Position")
            grab_btn['state'] = 'disabled'
            drop_btn['state'] = 'disabled'
        else:
            self.status['Testmode'] = 'sub_grab'
            self.test_asemb_status.set(f"Ready to test the [{self.current_nr}]th {self.component}'s Grab Position")
            grab_btn['state'] = 'disabled'
        

#----------------------Saving functions----------------------

    def save_position(self):
        os.chdir(f"{PATH}\data")
        if self.status['Testmode'] == 'grab':
            rail_po = self.getPosition()
            grab_po = list(self.GetPose())
            self.parameter[self.component]['grabPo'][str(self.current_nr)] = grab_po
            if self.component == COMPONENTS[1]:
                self.parameter[COMPONENTS[5]]['grabPo'][str(self.current_nr)] = list((grab_po[0], grab_po[1], grab_po[2]-0.5, grab_po[3], grab_po[4], grab_po[5]))
                extra_msg = ' and [Cathode_Spacer]'
            else:
                extra_msg = ''
            with open('calibration.json', 'w') as json_file:
                json.dump(self.parameter, json_file, indent=4)
            logging.warning(f"Component [{self.component}]{extra_msg} Nr.[{self.current_nr}]'s grab position has been updated- RailPO: [{rail_po}]; GrabPO: {grab_po}") 
            self.test_asemb_status.set(f"Grab Position(s) of {self.component}{extra_msg} No.[{self.current_nr}] has been saved: {grab_po}")
        elif self.status['Testmode'] == 'drop':
            drop_po = list(self.GetPose())
            self.parameter[self.component]['dropPo'] = drop_po
            if self.component == COMPONENTS[1]:
                self.parameter[COMPONENTS[5]]['dropPo'] = drop_po
                extra_msg = ' and [Cathode_Spacer]'
            else:
                extra_msg = ''
            with open('calibration.json', 'w') as json_file:
                json.dump(self.parameter, json_file, indent=4)
            logging.warning(f"Component [{self.component}]{extra_msg} Nr.[{self.current_nr}]'s drop position has been updated: {drop_po}")
            self.test_asemb_status.set(f"Drop Position(s) of {self.component}{extra_msg} No.[{self.current_nr}] has been saved: {drop_po}")
        elif self.status['Testmode'] == 'sub_grab':
            rail_po = self.getPosition()
            grab_po = list(self.GetPose())
            self.parameter[self.component]['grabPo'][str(self.current_nr)] = grab_po
            if self.component == COMPONENTS[1]:
                self.parameter[COMPONENTS[5]]['grabPo'][str(self.current_nr)] = list((grab_po[0], grab_po[1], grab_po[2]-0.5, grab_po[3], grab_po[4], grab_po[5]))
                extra_msg = ' and [Cathode_Spacer]'
            else:
                extra_msg = ''
            with open('calibration.json', 'w') as json_file:
                json.dump(self.parameter, json_file, indent=4)
            logging.warning(f"Component [{self.component}]{extra_msg} Nr.[{self.current_nr}]'s grab position has been updated- RailPO: [{rail_po}]; GrabPO: {grab_po}")
            self.test_asemb_status.set(f"Grab Position(s) of {self.component}{extra_msg} No.[{self.current_nr}] has been saved: {grab_po}")
        elif self.status['Testmode'] == 'sub_drop':
            drop_po = list(self.GetPose())
            self.parameter[self.component]['dropPo'] = drop_po
            if self.component == COMPONENTS[1]:
                self.parameter[COMPONENTS[5]]['dropPo'] = drop_po
                extra_msg = ' and [Cathode_Spacer]'
            else:
                extra_msg = ''
            with open('calibration.json', 'w') as json_file:
                json.dump(self.parameter, json_file, indent=4)
            logging.warning(f"Component [{self.component}]{extra_msg} Nr.[{self.current_nr}]'s drop position has been updated: {drop_po}")
            self.test_asemb_status.set(f"Drop Position(s) of {self.component}{extra_msg} No.[{self.current_nr}] has been saved: {drop_po}")
        elif self.status['Testmode'] == 'Cam':
            # jt_to_save = list(self.GetJoints())
            p_to_save = list(self.GetPose())
            if self.save_cam != None and p_to_save[1] > 160:
                self.parameter[self.save_cam] = p_to_save
                with open('calibration.json', 'w') as json_file:
                    json.dump(self.parameter, json_file, indent=4)
                logging.warning(f"Camera Position {self.save_cam} has been updated: {p_to_save}")

class TestTransportRobot(RobotController):
    
    def __init__(self, address=TRANSPORT_HOST):
        RobotController.__init__(self, address)
        self.load_parameter()
        self.reset_status()
        self.feedback = RobotFeedback(TRANSPORT_HOST, '7.0.6')
        # self._trans_rob_online = None
    
#----------------------Config functions----------------------
    def reset_status(self):
        self.status = dict(Tool=None, Testmode=None, Progress=dict(Initiate=0), Initiated=False)

    def load_parameter(self):
        os.chdir(f"{PATH}\data")
        with open('calibration.json', 'r') as json_file:
            self.constant = json.load(json_file)
    
    def write_parameter(self):
        os.chdir(f"{PATH}\data")
        with open('calibration.json', 'w') as json_file:
            json.dump(self.constant, json_file, indent=4)

    def setup_logging(self, default_path='config.yaml', default_level=logging.INFO):
        path = os.path.join(os.path.dirname(__file__), 'logs', default_path)
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                config['handlers']['file']['filename'] = os.path.join(os.path.dirname(__file__), 'logs', 'debug.log')
                config['handlers']['error']['filename'] = os.path.join(os.path.dirname(__file__), 'logs', 'error.log')
                logging.config.dictConfig(config)
        else:
            logging.basicConfig(level=default_level)

    def set_tool(self, tool_name:int):
        if tool_name == NORM:
            self.SetTRF(*self.constant['TCP_CP'])
            self.status["Tool"] = NORM
        if tool_name == FLIPED:
            self.SetTRF(*self.constant['TCP_CP_180'])
            self.status["Tool"] = FLIPED

#----------------------Motion functions----------------------

    def init_transport_robot(self):

        # Activate robot, home, reset joints, apply parameters
        logging.debug('Initiating Transport Robot...')
        conRes = self.connect()
        self.feedback.connect()
        self.status["Progress"]["Initiate"] = round(1/6*100)
        time.sleep(0.1)
        if conRes is True:
            # Update status
            self.status["Progress"]["Initiate"] = round(2/6*100)
            # Activate Robot
            actRes = self.ActivateRobot()
            time.sleep(0.1)
            if actRes == 'Motors activated.':
                # Update status
                self.status["Progress"]["Initiate"] = round(3/6*100)
                # Home robot
                homRes = self.home()
                time.sleep(0.1)
                if homRes == 'Homing done.':
                    # Update status
                    self.status["Progress"]["Initiate"] = round(4/6*100)
                    # Set robot's parameter
                    self.SetGripperForce(CONSTANT['GRIP_F'])
                    self.SetGripperVel(CONSTANT['GRIP_VEL'])
                    self.SetCartLinVel(CONSTANT['L_VEL'])
                    self.SetJointVel(CONSTANT['J_VEL'])
                    self.SetJointAcc(20)
                    self.MoveJoints(*CONSTANT['WAIT_1_J'])
                    time.sleep(0.01)
                    logging.debug('Transport Robot initiated.')
                else:
                    logging.error('Transport Robot already homed!')
            else:
                logging.error('Transport Robot already activated!')
        elif conRes == 'Another user is already connected, closing connection':
            logging.error('Transport Robot already in connection!')
        else:
            logging.error('Transport Robot is not in connection. Check Power buttom')
        
        
        self.status["Progress"]["Initiate"] = round(6/6*100)
        rob_res = self.GetStatusRobot()
        check = (conRes, rob_res["Activated"], rob_res["Homing"])
        self.status["Initiated"] = (check == (1,1,1))

    def auto_repair(self):
    # If there is an error we try to autorepair it. Added an extra resume motion over the
    # mecademic suggested version
        if self.is_in_error():
            self.ResetError()
        elif self.GetStatusRobot()['Paused'] == 1:
            self.ResumeMotion()
        self.ResumeMotion()
        self.ResumeMotion()

    def go_home(self):
        logging.debug("Homing TransportRobot...")

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
                self.set_tool(NORM)
            self.MoveJoints(*CONSTANT['ROTATE_LEFT_J'])
            self.MoveJoints(*CONSTANT['ROTATE_LEFT_TO_BACK_J'])
        self.MoveJoints(*CONSTANT['WAIT_1_J'])

    def disconnect_transport_robot(self):
    # Deactivate and disconnect the robot
        logging.debug('Disconnecting TransportRobot')
        self.go_home()
        time.sleep(1.5)
        self.DeactivateRobot()
        time.sleep(0.1)
        self.disconnect()
        self.reset_status()
        logging.debug("TransportRobot disconnected!")

    def grip_test(self):
        ak_p = list(self.GetPose())
        self.GripperClose()
        time.sleep(1)
        self.MoveLin(ak_p[0], ak_p[1], ak_p[2]+35, ak_p[3], ak_p[4], ak_p[5])
        time.sleep(2)
        self.MoveLin(*ak_p)
        self.GripperOpen()

    def trans_test_move_rob(self, testmode, back:bool):
        if testmode == 'Start Menu':
            self.go_home()
        elif testmode == 'Aligning Test' and back is True:
            self.MoveLin(*self.constant['GRIP_1_PO'])
            self.MoveJoints(*self.constant['WAIT_2_J'])
            self.set_tool(NORM)
            self.MovePose(*self.constant['ALIGN_1_PO'])

            # Gripper goes down to position
            self.GripperOpen()
            time.sleep(0.5)
            self.MoveLin(*self.constant['ALIGN_2_PO'])
        elif testmode == 'Aligning Test' and back is False:
            logging.info("Calibrating aligning procedure...")
            self.go_home()
            time.sleep(1)

            self.set_tool(NORM)
            self.MoveJoints(*self.constant['WAIT_2_J'])
            self.MovePose(*self.constant['ALIGN_1_PO'])

            # Gripper goes down to position
            self.GripperOpen()
            time.sleep(0.5)
            self.MoveLin(*self.constant['ALIGN_2_PO'])
        elif testmode == 'Grabing Test' and back is True:
            self.MoveLinRelWRF(0,-80,0,0,0,0)
            self.MoveJoints(*self.constant['TRANS_1_J'])
            self.MoveJoints(*self.constant['ROTATE_RIGHT_J'])
            self.MoveJoints(*self.constant['ROTATE_LEFT_J'])
            # To the start posiiton
            self.set_tool(FLIPED)
            self.MoveJoints(*self.constant['WAIT_2_J'])
            self.MovePose(*self.constant['GRIP_1_PO'])

            # Gripper goes down to position
            time.sleep(0.5)
            self.MoveLin(*self.constant['GRIP_2_PO'])
            self.GripperOpen()
        elif testmode == 'Grabing Test' and back is False:
            # Gripper goes up
            self.MoveLin(*self.constant['ALIGN_1_PO'])
            # To the start posiiton
            self.set_tool(FLIPED)
            self.MoveJoints(*self.constant['WAIT_2_J'])
            self.MovePose(*self.constant['GRIP_1_PO'])

            # Gripper goes down to position
            self.GripperOpen()
            time.sleep(0.5)
            self.MoveLin(*self.constant['GRIP_2_PO'])
        elif testmode == 'Transporting Test' and back is True:
            self.MoveLin(*self.constant['TRANS_2_PO'])
        elif testmode == 'Transporting Test' and back is False:
            self.GripperClose()
            time.sleep(1)

            # Gripper goes up
            self.MoveLin(*self.constant['GRIP_1_PO'])

            # Set TCP back to normal
            self.set_tool(NORM)

            # Reduce the rotating radius, rotate to crimper
            self.MoveJoints(*self.constant['ROTATE_LEFT_J'])
            self.MoveJoints(*self.constant['ROTATE_RIGHT_J'])

            # Drive through sigularity, ready to reach in
            self.MoveJoints(*self.constant['TRANS_1_J'])
            time.sleep(0.5)

            # Reaching into crimper next to the die:
            self.MoveLin(*self.constant['TRANS_2_PO'])
        elif testmode== 'Retrieving Test' and back is True:
            self.MoveLinRelWRF(0,0,-0.5,0,0,0)
            self.MoveLinRelWRF(-40,0,0,0,0,0)
            self.MoveLin(*self.constant['TRANS_2_PO'])
            self.MoveLin(*self.constant['BACKOFF_2_PO'])
            self.MoveLin(*self.constant['BACKOFF_1_PO'])
            self.MoveLin(*self.constant['TRANS_3_PO'])
        elif testmode== 'Retrieving Test' and back is False:
            self.MoveLin(*self.constant['TRANS_3_PO'])
        elif testmode =='Picking-up Test' and back is True:
            self.MoveLin(*self.constant['RETRIVE_6_PO'])
            self.MoveJoints(*self.constant['ROTATE_LEFT_J'])
            self.MoveJoints(*self.constant['ROTATE_RIGHT_J'])
            self.MovePose(*self.constant['RETRIVE_5_PO'])
            self.MoveLin(*self.constant['RETRIVE_4_PO'])
            self.MoveLin(*self.constant['RETRIVE_3_PO'])
        elif testmode =='Picking-up Test' and back is False:
            self.GripperOpen()
            time.sleep(0.5)

            # Move the gripper away from die
            self.MoveLin(*self.constant['BACKOFF_1_PO'])
            self.MoveLin(*self.constant['BACKOFF_2_PO'])
            self.MoveLin(*self.constant['BACKOFF_3_PO'])

            # Move out from crimper, goes to waitng position, ready to use the magenetic part
            self.MoveJoints(*self.constant['RETRIVE_1_J'])
            time.sleep(1)

            # Set TCP to normal:
            self.set_tool(NORM)
            
            # Magnet reaching into the crimper next to the die:
            self.MoveLin(*self.constant['RETRIVE_2_PO'])

            # Above the die, magnetic Grabing:
            self.MoveLin(*self.constant['RETRIVE_3_PO'])
        elif testmode == 'Sliding Test' and back is True:
            store_post = self.constant['MAG_ONE_PO']
            self.MovePose(store_post[0], store_post[1], store_post[2]+20, store_post[3], store_post[4], store_post[5])
            self.MoveLin(*store_post)
        elif testmode == 'Sliding Test' and back is False:
            # Move up 3 mm:
            self.MoveLin(*self.constant['RETRIVE_4_PO'])

            # Move the Magnet with CC away from die: Backwards
            self.MoveLin(*self.constant['RETRIVE_5_PO'])

            # Move to the ROTATION_RIGHT:
            self.MoveJoints(*self.constant['ROTATE_RIGHT_J'])

            # Move to the ROTATION_LEFT:
            self.MoveJoints(*self.constant['ROTATE_BACK_J'])

            # Ready to drop the CC on Post:
            store_post = self.constant['MAG_ONE_PO']
            self.MovePose(store_post[0], store_post[1], store_post[2]+20, store_post[3], store_post[4], store_post[5])

            # Drop CC on the Post:
            self.MoveLin(*store_post)
        elif testmode == 'Done Test':
            # Perform Sliding:
            self.MoveLin(store_post[0], store_post[1]-20, store_post[2], store_post[3], store_post[4], store_post[5])

            # Move up, home the position
            self.MoveLinRelWRF(0,0,20,0,0,0)
            self.MoveJoints(*self.constant['WAIT_1_J'])
        elif testmode == 'Home':
            self.go_home()
        elif testmode == 'Gripper Test':
            self.grip_test()
        elif testmode == 'Initiate':
            self.init_transport_robot()
            # res = self.GetStatusRobot()
            # self._trans_rob_online = (res['Activated'], res['Homing']) == (1, 1)

    def end_trans_test(self):
        return self.disconnect_transport_robot()

#----------------------GUI functions----------------------

    def free_move(self):
        global exit_flag
        exit_flag = False
        os.chdir(f"{PATH}\images")
        self.test_transport_window.state(newstate='iconic')
        free_move_window = Toplevel()

        # Specify font of labels and button's text
        self.ft_label = font.Font(family='Arial', size=15, weight=font.BOLD)
        self.ft_button = font.Font(size=15)

        # Set the title, icon, size of the initial window
        free_move_window.title("Freemove GUI")
        free_move_window.iconbitmap("Robotarm.ico")
        free_move_window.geometry("830x660")

        # Create status bar
        free_move_status = StringVar()
        status_label = Label(free_move_window, textvariable=free_move_status, pady=10, bd=1, relief=SUNKEN, anchor=W)
        status_label.grid(row=0, column=0, columnspan=2, padx=20, sticky=W+E)

         # Update the status
        def refresh_status():
            while exit_flag != True:
                self.feedback.get_data()
                free_move_status.set(f"Robot Pose: {self.feedback.cartesian}, Robot Joints: {self.feedback.joints}")

            # free_move_window.after(100, refresh_status)

        # Create the control panel
        free_move_frame = LabelFrame(free_move_window, text="Movement Control Panel",\
            padx=25, pady=10, borderwidth=5)
        free_move_frame.grid(row=1, column=0, padx=20, pady=10)

        # Add input box for increment to functional Panel
        free_move_frame_increm = LabelFrame(free_move_frame, text="Increment: ",\
            font=self.ft_label, pady=8, borderwidth=3)
        self.increment = Entry(free_move_frame_increm, width=5, borderwidth=5)
        unit = Label(free_move_frame_increm, text="mm", font=self.ft_label)

        free_move_frame_increm.grid(row=0, column=0, padx=10, pady=5)
        self.increment.grid(row=0, column=0)
        unit.grid(row=0, column=1)

        self.increment.insert(0, '0.2')

        free_move_frame_rxyz = LabelFrame(free_move_frame, text="\u03B1-\u03B2-\u03B3-Axis: ",\
            font=self.ft_label, padx=35, borderwidth=3)
        free_move_frame_rxyz.grid(row=0, column=1)

        free_move_frame_z = LabelFrame(free_move_frame, text="Z-Axis: ",\
            font=self.ft_label, padx=10, pady=30, borderwidth=3)
        free_move_frame_z.grid(row=1, column=0, padx=10, pady=10)

        free_move_frame_xy = LabelFrame(free_move_frame, text="XY-Axis: ",\
            font=self.ft_label, padx=20, pady=30, borderwidth=3)
        free_move_frame_xy.grid(row=1, column=1, padx=10, pady=10)

        free_move_frame_rail = LabelFrame(free_move_frame, text="Rail X-Axis: ",\
            font=self.ft_label, padx=88, borderwidth=3)
        free_move_frame_rail.grid(row=2, column=0, columnspan=2)

        # Add buttons to the control panel
        up_btn = Button(free_move_frame_z, image=arrow_up, padx=10, pady=40, border=5,\
            borderwidth=4, command=lambda: self.free_move_control('+z'))
        down_btn = Button(free_move_frame_z, image=arrow_down, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('-z'))
        left_btn = Button(free_move_frame_xy, image=arrow_left, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('-y'))
        right_btn = Button(free_move_frame_xy, image=arrow_right, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('+y'))
        forward_btn = Button(free_move_frame_xy, image=arrow_up, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('-x'))
        backward_btn = Button(free_move_frame_xy, image=arrow_down, padx=10, pady=40,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('+x'))
        
        centrer_label_1 = Label(free_move_frame_z, image=centrer, padx=10, pady=40,\
            border=5, state=DISABLED)
        centrer_label_2 = Label(free_move_frame_xy, image=centrer, padx=10, pady=40,\
            border=5, state=DISABLED)

        rx_btn = Button(free_move_frame_rxyz, text="\u0394\u03B1", font=self.ft_button,\
            border=5, borderwidth=4, command=lambda: self.free_move_control('rx'))
        ry_btn = Button(free_move_frame_rxyz, text="\u0394\u03B2", font=self.ft_button, border=5,\
            borderwidth=4, command=lambda: self.free_move_control('ry'))
        rz_btn = Button(free_move_frame_rxyz, text="\u0394\u03B3", font=self.ft_button, border=5,\
            borderwidth=4, command=lambda: self.free_move_control('rz'))
        rail_poitive_btn = Button(free_move_frame_rail, image=arrow_right, border=5,\
            borderwidth=4, state=DISABLED)
        rail_negative_btn = Button(free_move_frame_rail, image=arrow_left, border=5,\
            borderwidth=4, state=DISABLED)

        up_btn.grid(row=0, column=0, padx=10)
        down_btn.grid(row=2, column=0, padx=10)
        centrer_label_1.grid(row=1, column=0)
        left_btn.grid(row=1, column=0)
        right_btn.grid(row=1, column=2)
        forward_btn.grid(row=0, column=1)
        backward_btn.grid(row=2, column=1)
        centrer_label_2.grid(row=1, column=1)
        rx_btn.grid(row=0, column=0, padx=10)
        ry_btn.grid(row=0, column=1, padx=10)
        rz_btn.grid(row=0, column=2, padx=10)
        rail_negative_btn.grid(row=0, column=0, padx=20)
        rail_poitive_btn.grid(row=0, column=1, padx=20)

        # Create Functional Frame
        function_frame = LabelFrame(free_move_window, text="Function Control Panel",\
            padx=25, pady=7, borderwidth=5)
        function_frame.grid(row=1, column=1, padx=5, pady=20)

        gripper_control_frame = LabelFrame(function_frame, text="Gripper Control",\
            padx=30, pady=10, borderwidth=5)
        gripper_control_frame.grid(row=0, column=0, padx=5, pady=5)

        vacuum_control_frame = LabelFrame(function_frame, text="Vacuum Control",\
            padx=30, pady=10, borderwidth=5)
        vacuum_control_frame.grid(row=1, column=0, padx=5, pady=5)

        position_control_frame = LabelFrame(function_frame, text="Position Control",\
            padx=30, pady=10, borderwidth=5)
        position_control_frame.grid(row=2, column=0, padx=5, pady=5)

        # Add functional buttons to the functional panel
        global open_gripper_btn, close_gripper_btn, open_vacuum_btn, close_vacuum_btn
        open_gripper_btn = Button(gripper_control_frame, text="Open Gripper",\
            border=5, padx=24, pady=10, borderwidth=4, command=lambda: self.free_move_control('+gripper'))
        close_gripper_btn = Button(gripper_control_frame, text="Close Gripper",\
            padx=24, pady=10, border=5, borderwidth=4, command=lambda: self.free_move_control('-gripper'))
        open_vacuum_btn = Button(vacuum_control_frame, text="Open Vacuum",\
            padx=20, pady=10, border=5, borderwidth=4, state=DISABLED)
        close_vacuum_btn = Button(vacuum_control_frame, text="Close Vacuum",\
            padx=20, pady=10, border=5, borderwidth=4, state=DISABLED)
        test_btn = Button(position_control_frame, text="Test Position",\
            padx=25, pady=10, border=5, borderwidth=4, state=DISABLED)
        save_btn = Button(position_control_frame, text="Save Position",\
            padx=23, pady=10, border=5, borderwidth=4, state=DISABLED)

        def exit_and_back():
            global exit_flag
            exit_flag = False
            free_move_window.destroy()
            self.test_transport_window.state(newstate='normal')

        exit_btn = Button(position_control_frame, text="Exit",\
            padx=48, pady=10, borderwidth=4, command=exit_and_back)

        open_gripper_btn.grid(row=0, column=0)
        close_gripper_btn.grid(row=1, column=0, pady=5)
        open_vacuum_btn.grid(row=0, column=0)
        close_vacuum_btn.grid(row=1, column=0, pady=5)
        test_btn.grid(row=0, column=0)
        save_btn.grid(row=1, column=0, pady=5)
        exit_btn.grid(row=2, column=0)

        threading.Thread(target=refresh_status, daemon=True).start()
        free_move_window.mainloop()

    def free_move_rob(self, axis, step):
        if axis == '+x':
            self.MoveLinRelWRF(abs(step),0,0,0,0,0)
        elif axis == '-x':
            self.MoveLinRelWRF(-abs(step),0,0,0,0,0)
        elif axis == '+y':
            self.MoveLinRelWRF(0,abs(step),0,0,0,0)
        elif axis == '-y':
            self.MoveLinRelWRF(0,-abs(step),0,0,0,0)
        elif axis == '+z':
            self.MoveLinRelWRF(0,0,abs(step),0,0,0)
        elif axis == '-z':
            self.MoveLinRelWRF(0,0,-abs(step),0,0,0)
        elif axis == 'rx':
            self.MoveLinRelWRF(0,0,0,step,0,0)
        elif axis == 'ry':
            self.MoveLinRelWRF(0,0,0,0,step,0)
        elif axis == 'rz':
            self.MoveLinRelWRF(0,0,0,0,0,step)
        elif axis == '+gripper':
            self.GripperOpen()
        elif axis == '-gripper':
            self.GripperClose()

    def free_move_control(self, axis):
        try:
            step = float(self.increment.get())
        except ValueError:
            messagebox.showerror("Input Error!", "Only positive float is accepted")
        else:
            if axis == '+gripper':
                open_gripper_btn['state'] = 'disable'
                close_gripper_btn['state'] = 'normal'
            elif axis == '-gripper':
                open_gripper_btn['state'] = 'normal'
                close_gripper_btn['state'] = 'disable'
            else:
                free_move_exe = threading.Thread(name='FreeMove', target=self.free_move_rob, daemon=True, args=(axis, step,))
                free_move_exe.start()

    def init_trans_test_gui(self, mainLev:Tk=None):
        self.status['Testmode'] = 'Start Menu'

        os.chdir(f"{PATH}\images")
        # Start a new window
        self.test_transport_window = Toplevel()
        self.test_transport_window.title("Transport Robot Testing Interface")
        self.test_transport_window.iconbitmap("Robotarm.ico")
        self.test_transport_window.geometry("420x320")

        # Load images
        global arrow_left, arrow_right, arrow_up, arrow_down, centrer, done
        arrow_left = ImageTk.PhotoImage(Image.open("arrow_left.png"))
        arrow_right = ImageTk.PhotoImage(Image.open("arrow_right.png"))
        arrow_up = ImageTk.PhotoImage(Image.open("arrow_up.png"))
        arrow_down = ImageTk.PhotoImage(Image.open("arrow_down.png"))
        centrer = ImageTk.PhotoImage(Image.open("centrer.png"))
        done = ImageTk.PhotoImage(Image.open("done.png"))

        # Specify font of labels and button's text
        global ft_label, ft_button
        ft_label = font.Font(family='Arial', size=10, weight=font.BOLD)
        ft_button = font.Font(size=15)

        # Create status bar
        self.status_str = StringVar()
        status_label = Label(self.test_transport_window, textvariable=self.status_str, font=ft_label, pady=10, bd=1, relief=SUNKEN)
        status_label.grid(row=0, column=0, columnspan=2, padx=20, pady=10, sticky=W+E)
        self.status_str.set("Start Menu")

        # Creat frame
        self.test_transport_frame = LabelFrame(self.test_transport_window, padx=10, pady=10, borderwidth=5)
        self.test_transport_frame.grid(row=1, column=0, padx=20, pady=5)

        # Creat labels
        intro_1 = Label(self.test_transport_frame,\
                    text="Place the testing cell on Post,\nThen Press [Start] when ready: ")

        intro_1.grid(row=0, column=0, columnspan=3)

        # Create start buttons
        global start_test_btn, home_btn, load_btn
        start_test_btn = Button(self.test_transport_frame, text="Start", font=ft_button,\
                        padx=15, pady=15, borderwidth=4, command=self.start_trans_test, state=DISABLED)
        start_test_btn.grid(row=1, column=0, padx=10, pady=10)
        home_btn = Button(self.test_transport_frame, text="Home", font=ft_button,\
                    padx=15, pady=15, borderwidth=4, command=lambda: self.trans_test_rob_thread('Home'), state=DISABLED)
        home_btn.grid(row=1, column=1, padx=10, pady=10)
        load_btn = Button(self.test_transport_frame, text="Initiate", font=ft_button,\
                    padx=15, pady=15, borderwidth=4, command=self.initiate_test)
        load_btn.grid(row=1, column=2, padx=10, pady=10)

        def exit_and_back():
            self.test_transport_window.destroy()
            if mainLev:
                mainLev.state(newstate='normal')

        # Greate exit button
        exit = Button(self.test_transport_window, text="Exit", font=ft_button,\
                padx=45, pady=5, borderwidth=4, command=exit_and_back)
        exit.grid(row=2, column=0, padx=10, pady=10)

        self.test_transport_window.mainloop()
    
    def initiate_test(self):

        os.chdir(f"{PATH}\images")
        prog_window = Toplevel()
        prog_window.title("Transport Robot initializing")
        prog_window.iconbitmap("Robotarm.ico")
        prog_window.geometry('280x150')
        prog_text = StringVar()
        prog_label = Label(prog_window, textvariable=prog_text, font=ft_label, pady=10, anchor=CENTER)
        prog = ttk.Progressbar(prog_window, length=250, mode='determinate', orient=HORIZONTAL)
        prog_label.grid(row=2, column=0, columnspan=2)
        prog.grid(row=1, column=0, columnspan=2, pady=20, sticky=W+E)
        # self.trans_test_rob_thread('Initiate')
        threading.Thread(name='startsystem', target=self.init_transport_robot, daemon=True).start()
        def update_probar():
            prog['value'] = self.status['Progress']['Initiate']
            prog_text.set(f"Initiating System, Please Wait ({prog['value']}%)")
            if prog['value'] == 100:
                prog_text.set(f"Initiating Finishing (100%)...")
                prog_window.update_idletasks()
                time.sleep(1)
                prog_window.destroy()
                start_test_btn['state'] = 'normal'
                home_btn['state'] = 'normal'
            else:
                prog_window.after(30, update_probar)
        update_probar()
        prog_window.mainloop()

    def start_trans_test(self):
        # if self._trans_rob_online is True:
        if self.status['Initiated'] is True:
            self.trans_test_gui('Aligning Test')
        else:
            messagebox.showerror('Connection Error', "Please Initiate System first!")
    
    def trans_test_gui(self, testmode:str, back:bool=False):
        run_txt = {
        'Start Menu': "Returning to the Start Position",
        'Aligning Test': "Changing to Aligning Position",
        'Grabing Test': "Changing to Grabing Position",
        'Transporting Test': "Changing to Transporting Position",
        'Retrieving Test': "Changing Retrieving Position",
        'Picking-up Test': "Changing to Picking-Up Position",
        'Sliding Test': "Changing to Sliding Position",
        'Done Test': "Finishing Test, Returning to Homeing Position",
        'Home': "Homing Position",
        'Gripper Test': "Implementing Gripper Test",
        'Initiate': "Initiating"
        }

        # test = ['Start Menu', 'Aligning Test', 'Grabing Test', 'Transporting Test', 'Retrieving Test', 'Picking-up Test', 'Sliding Test', 'Done Test']
        if testmode in TESTLIST:
            self.status['Testmode'] = testmode
            test_index = TESTLIST.index(testmode)
            if testmode == 'Start Menu':
                # Clear pervious widget
                self.test_transport_window.geometry("420x320")
                for widget in self.test_transport_frame.winfo_children():
                    widget.destroy()
                
                # Creat labels
                intro_1 = Label(self.test_transport_frame,\
                            text="Place the testing cell on Assembly Post\nwith Cathode_Case facing upwards.\nClick [Start] when ready: ")
                intro_1.grid(row=0, column=0, columnspan=3)

                # Create start buttons
                start_test_btn = Button(self.test_transport_frame, text="Start", font=ft_button,\
                                padx=15, pady=15, borderwidth=4, command=lambda: self.trans_test_gui('Aligning Test'))
                start_test_btn.grid(row=1, column=0, padx=10, pady=10)
                home_btn = Button(self.test_transport_frame, text="Home", font=ft_button,\
                            padx=15, pady=15, borderwidth=4, command=lambda: self.trans_test_rob_thread('Home'))
                home_btn.grid(row=1, column=1, padx=10, pady=10)
                load_btn = Button(self.test_transport_frame, text="Initiate", font=ft_button,\
                            padx=15, pady=15, borderwidth=4, command=lambda: self.trans_test_rob_thread('Initiate'))
                load_btn.grid(row=1, column=2, padx=10, pady=10)
                if self.status['Initiated'] is True:
                    start_test_btn['state'] = 'normal'
                    home_btn['state'] = 'normal'
                else:
                    start_test_btn['state'] = 'disabled'
                    home_btn['state'] = 'disabled'
            elif testmode == 'Aligning Test':
                # Clear pervious widget
                self.test_transport_window.geometry("500x340")
                for widget in self.test_transport_frame.winfo_children():
                    widget.destroy()

                # update status bar
                status_label = Label(self.test_transport_window, textvariable=self.status_str, font=ft_label,\
                                    pady=10, bd=1, relief=SUNKEN)
                status_label.grid(row=0, column=0, columnspan=2, padx=20, pady=10, sticky=W+E)

                # Greate other functional buttons
                global left_btn, right_btn, save_btn
                home_btn = Button(self.test_transport_frame, text="Home",\
                            padx=10, pady=5, borderwidth=4, command=lambda: self.trans_test_gui('Start Menu'))
                test_btn = Button(self.test_transport_frame, text="Test", padx=20,\
                            pady=5, borderwidth=4, command=lambda: self.trans_test_rob_thread('Gripper Test'))
                free_move_btn = Button(self.test_transport_frame, text="Free-move",\
                                padx=5, pady=5, borderwidth=4, command=self.free_move)
                save_btn = Button(self.test_transport_frame, text="Save", font=ft_button,\
                            padx=10, pady=15, borderwidth=4, command=lambda: self.save_position(testmode))
                left_btn = Button(self.test_transport_frame, image=arrow_left, padx=10, pady=40, border=5, borderwidth=4, command=lambda: self.trans_test_gui('Start Menu'), state=DISABLED)
                right_btn = Button(self.test_transport_frame, image=arrow_right, padx=10, pady=40, border=5, borderwidth=4, command=lambda: self.trans_test_gui(TESTLIST[test_index+1]), state=DISABLED)
                
                left_btn.grid(row=0, column=0, padx=50)
                save_btn.grid(row=0, column=1)
                home_btn.grid(row=1, column=0, pady=20)
                free_move_btn.grid(row=1, column=1, pady=20)
                test_btn.grid(row=1, column=2, pady=20)
                right_btn.grid(row=0, column=2, padx=50)

                self.status_str.set(f"Robot is now {run_txt[testmode]}, Please Wait...")
            elif testmode == 'Done Test':
                save_btn.config(state=DISABLED)
                left_btn.config(image=arrow_left, command=lambda: self.trans_test_gui(testmode='Sliding Test', back=True), state=DISABLED)
                right_btn.config(image=done, command=lambda: self.trans_test_gui('Start Menu'), state=DISABLED)
            else:
                self.status_str.set(f"Robot is now {run_txt[testmode]}, Please Wait...")
                save_btn.config(command=lambda: self.save_position(testmode))
                left_btn.config(image=arrow_left, command=lambda: self.trans_test_gui(testmode=TESTLIST[test_index-1], back=True), state=DISABLED)
                right_btn.config(image=arrow_right, command=lambda: self.trans_test_gui(TESTLIST[test_index+1]), state=DISABLED)
        
        # Robot execute simultanous movement    
        self.trans_test_rob_thread(testmode=testmode, back=back)

    def check_rob_thread(self, testmode):
        done_txt = {
        'Start Menu': "[Start Menu]",
        'Aligning Test': "[Aligning Test]",
        'Grabing Test': "[Grabing Test]",
        'Transporting Test': "[Transporting Test]",
        'Retrieving Test': "[Retrieving Test]",
        'Picking-up Test': "[Picking-Up Test]",
        'Sliding Test': "[Sliding Test]",
        'Done Test': "All Tests has been Done, Click [✓] to Return the testing cell",
        'Home': f"[{self.status['Testmode']}]: Homing Done",
        'Gripper Test': f"[{self.status['Testmode']}]: Gripper Test has been Done",
        'Initiate': f"[{self.status['Testmode']}]: System has been initiated"
        }
        if not self.rob_thread.is_alive():
            if self.status['Testmode'] != 'Start Menu':
                left_btn['state'] = 'normal'
                right_btn['state'] = 'normal'
            self.status_str.set(done_txt[testmode])
        else:
            self.test_transport_window.after(100, lambda:self.check_rob_thread(testmode))
        
    def trans_test_rob_thread(self, testmode, back:bool=False):
        self.rob_thread = threading.Thread(target=self.trans_test_move_rob, args=(testmode, back,), daemon=True)
        self.rob_thread.start()
        self.check_rob_thread(testmode=testmode)

#----------------------Saving functions----------------------

    def save_position(self, testmode):
        os.chdir(f"{PATH}\data")
        if testmode == 'Aligning Test':
            ak_po = list(self.GetPose())
            self.constant['ALIGN_2_PO'] = ak_po
            self.constant['ALIGN_1_PO'] = [ak_po[0], ak_po[1], ak_po[2]+35, ak_po[3], ak_po[4], ak_po[5]]
            self.write_parameter()
            logging.warning(f"Positions have been updated:  ['ALIGN_1_PO']: {self.constant['ALIGN_1_PO']}")
            logging.warning(f"Positions have been updated:  ['ALIGN_2_PO']: {self.constant['ALIGN_2_PO']}")
            messagebox.showinfo("Information", f"Positions have been updated:\
            \n['ALIGN_1_PO']: {self.constant['ALIGN_1_PO']}\
            \n['ALIGN_2_PO']: {self.constant['ALIGN_2_PO']}")
        elif testmode == 'Grabing Test':
            ak_po = list(self.GetPose())
            self.constant['GRIP_2_PO'] = ak_po
            self.constant['GRIP_1_PO'] = [ak_po[0], ak_po[1], ak_po[2]+85, ak_po[3], ak_po[4], ak_po[5]]
            self.write_parameter()
            logging.warning(f"Positions have been updated:  ['GRIP_1_PO']: {self.constant['GRIP_1_PO']}")
            logging.warning(f"Positions have been updated:  ['GRIP_2_PO']: {self.constant['GRIP_2_PO']}")
            messagebox.showinfo("Information", f"Positions have been updated:\
            \n['GRIP_1_PO']: {self.constant['GRIP_1_PO']}\
            \n['GRIP_2_PO']: {self.constant['GRIP_2_PO']}")
        elif testmode == 'Transporting Test':
            ak_po = list(self.GetPose())
            self.constant['TRANS_2_PO'] = ak_po
            self.write_parameter()
            logging.warning(f"Position has been updated: ['TRANS_2_PO']: {self.constant['TRANS_2_PO']}")
            messagebox.showinfo("Information", f"Positions have been updated:\
            \n['TRANS_2_PO']: {self.constant['TRANS_2_PO']}")
        elif testmode == 'Retrieving Test':
            ak_po = list(self.GetPose())
            self.constant['TRANS_3_PO'] = ak_po
            self.constant['BACKOFF_1_PO'] = [ak_po[0], ak_po[1]+18, ak_po[2], ak_po[3], ak_po[4], ak_po[5]]
            self.constant['BACKOFF_2_PO'] = [ak_po[0]-35, ak_po[1]+18, ak_po[2], ak_po[3], ak_po[4], ak_po[5]]
            self.constant['BACKOFF_3_PO'] = [ak_po[0]-35, ak_po[1]-79, ak_po[2], ak_po[3], ak_po[4], ak_po[5]]
            self.write_parameter()
            logging.warning(f"Position has been updated: ['TRANS_3_PO']: {self.constant['TRANS_3_PO']}")
            logging.warning(f"Position has been updated: ['BACKOFF_1_PO']: {self.constant['BACKOFF_1_PO']}")
            logging.warning(f"Position has been updated: ['BACKOFF_2_PO']: {self.constant['BACKOFF_2_PO']}")
            logging.warning(f"Position has been updated: ['BACKOFF_3_PO']: {self.constant['BACKOFF_3_PO']}")
            messagebox.showinfo("Information", f"Positions have been updated:\
            \n['TRANS_3_PO']: {self.constant['TRANS_3_PO']},\
            \n['BACKOFF_1_PO']: {self.constant['BACKOFF_1_PO']},\
            \n['BACKOFF_2_PO']: {self.constant['BACKOFF_2_PO']}:\
            \n['BACKOFF_3_PO']: {self.constant['BACKOFF_3_PO']}")
        elif testmode == 'Picking-up Test':
            ak_po = list(self.GetPose())
            self.constant['RETRIVE_3_PO'] = ak_po
            self.constant['RETRIVE_2_PO'] = [ak_po[0]-25, ak_po[1], ak_po[2], ak_po[3], ak_po[4], ak_po[5]]
            self.constant['RETRIVE_4_PO'] = [ak_po[0], ak_po[1], ak_po[2]+3, ak_po[3], ak_po[4], ak_po[5]]
            self.constant['RETRIVE_5_PO'] = [ak_po[0], ak_po[1]-60, ak_po[2]+3, ak_po[3], ak_po[4], ak_po[5]]
            self.write_parameter()
            logging.warning(f"Position has been updated: ['RETRIVE_2_PO']: {self.constant['RETRIVE_2_PO']}")
            logging.warning(f"Position has been updated: ['RETRIVE_3_PO']: {self.constant['RETRIVE_3_PO']}")
            logging.warning(f"Position has been updated: ['RETRIVE_4_PO']: {self.constant['RETRIVE_4_PO']}")
            logging.warning(f"Position has been updated: ['RETRIVE_5_PO']: {self.constant['RETRIVE_5_PO']}")
            messagebox.showinfo("Information", f"Positions have been updated:\
            \n['RETRIVE_2_PO']: {self.constant['RETRIVE_2_PO']},\
            \n['RETRIVE_3_PO']: {self.constant['RETRIVE_3_PO']},\
            \n['RETRIVE_4_PO']: {self.constant['RETRIVE_4_PO']}:\
            \n['RETRIVE_5_PO']: {self.constant['RETRIVE_5_PO']}")
        elif testmode == 'Sliding Test':
            ak_po = list(self.GetPose())
            self.constant['MAG_ONE_PO'] = ak_po
            self.constant['MAG_TWO_PO'] = [ak_po[0]-1, ak_po[1]-75, ak_po[2]+5.5, ak_po[3], ak_po[4], ak_po[5]]
            self.write_parameter()
            logging.warning(f"Positions have been updated: ['MAG_ONE_PO']: {self.constant['MAG_ONE_PO']}")
            logging.warning(f"Positions have been updated: ['MAG_TWO_PO']: {self.constant['MAG_TWO_PO']}")
            messagebox.showinfo("Information", f"Positions have been updated:\
            \n['MAG_ONE_PO']: {self.constant['MAG_ONE_PO']}\
            \n['MAG_TWO_PO']: {self.constant['MAG_TWO_PO']}")

if __name__ == '__main__':
    os.chdir(PATH)
    # ui = TestTransportRobot()
    # ui.init_trans_test_gui()
    ui = TestAssemblyRobot()
    ui.set_test_assembly()
