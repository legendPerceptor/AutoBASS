import time
import json
import yaml
import os
import threading
import logging
import logging.config
import PipRobot_v2 as Dispensor
import AssemblyRobot
import TransportRobot

# Get the folder path
ROOT = os.path.dirname(__file__)

# Assembly constant
COMPONENTS = ('Anode_Case', 'Anode_Spacer', 'Anode', 'Separator', 'Cathode', 'Cathode_Spacer', 'Washer', 'Cathode_Case')
STEPS = dict(   
                Start=dict(step=0, component=0, pre_operation=-1, step_mark=1),
                Anode_Case=dict(
                            Grab=dict(step=4, component=0, pre_operation=-1, step_mark=1), 
                            Drop=dict(step=8, component=1, pre_operation=-1, step_mark=1)), 
                Anode_Spacer=dict(
                            Grab=dict(step=12, component=1, pre_operation=-1, step_mark=1), 
                            Drop=dict(step=16, component=2, pre_operation=-1, step_mark=1),
                            Dispense=dict(step=20, component=2, pre_operation=1, step_mark=1)),
                Anode=dict(
                            Grab=dict(step=24, component=2, pre_operation=1, step_mark=1), 
                            Drop=dict(step=28, component=3, pre_operation=1, step_mark=1),
                            Dispense=dict(step=32, component=3, pre_operation=2, step_mark=1)), 
                Separator=dict(
                            Grab=dict(step=36, component=3, pre_operation=2, step_mark=1),
                            Drop=dict(step=40, component=4, pre_operation=2, step_mark=1),
                            Dispense=dict(step=44, component=4, pre_operation=0, step_mark=1)), 
                Cathode=dict(
                            Grab=dict(step=48, component=4, pre_operation=0, step_mark=1),
                            Drop=dict(step=52, component=5, pre_operation=0, step_mark=1)), 
                Cathode_Spacer = dict(
                            Grab=dict(step=56, component=5, pre_operation=0, step_mark=1),
                            Drop=dict(step=60, component=6, pre_operation=0, step_mark=1)),
                Washer=dict(
                            Grab=dict(step=64, component=6, pre_operation=0, step_mark=1),
                            Drop=dict(step=68, component=7, pre_operation=0, step_mark=1)), 
                Cathode_Case=dict(
                            Grab=dict(step=72, component=7, pre_operation=0, step_mark=1),
                            Drop=dict(step=76, component=0, pre_operation=0, step_mark=2)),
                Press=dict(step=80, component=0, pre_operation=0, step_mark=3),
                Crimp=dict(step=90, component=0, pre_operation=0, step_mark=4),
                Done=dict(step=100, component=0, pre_operation=-1, step_mark=1),
            )

class Assembly():
    def __init__(self):
        self._move_on = threading.Event()
        self._abort = threading.Event()
        self.rubi = AssemblyRobot.AssemblyRobot(pause_control=self._move_on, abort_control=self._abort)
        self.pangpang = TransportRobot.TransportRobot(pause_control=self._move_on, abort_control=self._abort)
        self.dispensor = Dispensor.PipRobot(pause_control=self._move_on, abort_control=self._abort)
        self.reset_status()

    def setup_logging(self, default_path='config.yaml', default_level=logging.INFO):
        path = os.path.join(os.path.dirname(__file__), 'logs', default_path)
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                config['handlers']['file']['filename'] = os.path.join(os.path.dirname(__file__), 'logs', 'AutoBASS_OP.log')
                config['handlers']['error']['filename'] = os.path.join(os.path.dirname(__file__), 'logs', 'System_Error.log')
                logging.config.dictConfig(config)
        else:
            logging.basicConfig(level=default_level)

    def reset_status(self):
        # self.status = dict(Progress=dict(Cell_nr=None, Component=None, Step=None), Initiated=False, Pause=False, Aborted=False)
        self.status = dict(Progress=dict(cell_nr=0, electrolyte_nr=0, electrolyte_vol=35, 
                                         additive_nr=0, additive_vol=35, step=0, component=0,
                                         pre_operation=-1, step_mark=1), Initiated=False, initResult=(False,False,False), Pause=False, Aborted=False)

    def sys_is_online(self):
        rob_1_res = self.rubi.assembrob_is_online()
        rob_2_res = self.pangpang.transrob_is_online()
        rob_3_res = self.dispensor.piprob_is_online()
        res = [rob_1_res, rob_2_res, rob_3_res]
        return all(res)

    def get_initiate_prog(self):
        assemblyInit = self.rubi.status["Progress"]["Initiate"]
        transportInit = self.pangpang.status["Progress"]["Initiate"]
        dispensorInit = self.dispensor.status["Progress"]["Initiate"]
        return round((assemblyInit+transportInit+dispensorInit)/3)

    def get_step_val(self):
        return self.status["Progress"]["step"]

    def load_parameter(self):
        with open(os.path.join(ROOT, 'data', 'config.json')) as json_file:
            self.parameter = json.load(json_file)

    def _record_consuming_time(self, cell_number:int, consuming_time:float):
        today = time.strftime("%Y_%m_%d", time.localtime())
        cellLogDir = os.path.join(ROOT, 'Alignments', today, 'Cells_Log.json')
        with open(cellLogDir, "r") as infile:
            cellsLog:dict = json.load(infile)
        cellsLog[str(cell_number)]['time_consume'] = consuming_time
        with open(cellLogDir, "w") as outfile:
            json.dump(cellsLog, outfile, indent=4)

    def apply_set_up(self):
        self.load_parameter()
        self.rubi.SetJointVel(self.parameter['J_VEL'])
        self.rubi.SetCartLinVel(self.parameter['L_VEL'])
        self.rubi.SetGripperVel(self.parameter['GRIP_VEL'])
        self.rubi.SetGripperForce(self.parameter['GRIP_F'])
        self.pangpang.SetJointVel(self.parameter['J_VEL'])
        self.pangpang.SetCartLinVel(self.parameter['L_VEL'])
        self.pangpang.SetGripperVel(self.parameter['GRIP_VEL'])
        self.pangpang.SetGripperForce(self.parameter['GRIP_F'])
        self.dispensor.SetJointVel(self.parameter['J_VEL'])
        self.dispensor.SetCartLinVel(self.parameter['L_VEL'])
        self.dispensor.reset_positions()
        self.rubi.set_vel(self.parameter['AX_VEL'])
        self.press = self.parameter['tap_press']
        self.auto_calib = self.parameter['auto_calib']
        self.save_img = self.parameter['save_img']
        self.grab_check = self.parameter['grab_check']
        self.show_image = self.parameter['show_image']
        self.anode_electrolyte = self.parameter['add_electrolyte']

    def initiate_all(self):
        initRes = dict(AssemblyRobot=False, TransportRobot=False, PipRoobot=False)
        try:
            assemblyRes:dict = self.rubi.init_assembly_robot()
        except:
            logging.error(f"AssemblyRobot initiating failed, ErrorCode: {assemblyRes}", exc_info=True)
        else:
            initRes.update(dict(AssemblyRobot=all(list(assemblyRes.values()))))
            try:
                transportRes:dict = self.pangpang.init_transport_robot()
            except:
                logging.error(f"Initiating failed, ErrorCode: {transportRes}", exc_info=True)
            else:
                initRes.update(dict(TransportRobot=all(list(transportRes.values()))))
                try:
                    dispensorRes:dict = self.dispensor.initiate_piprobot()
                except:
                    logging.error(f"Initiating failed, ErrorCode: {dispensorRes}", exc_info=True)
                else:
                    initRes.update(dict(PipRoobot=all(list(dispensorRes.values()))))
        self.status["initResult"] = tuple(initRes.values())
        self.status["Initiated"] = all(list(initRes.values()))

    def one_cell(self, component_Nr:int=1, electrolyte_Nr:int=1, electrolyte_Vol:int=35,
                 additive_Nr:int=0, additive_Vol:int=10, bk_config:dict=None):

        self.status['Progress'].update(STEPS['Start'])
        
        # Load config file
        self.load_parameter()
        time.sleep(0.1)

        # If broken point is present, overwrite component and electrolyte num
        if bk_config['cell_nr'] > 0:
            component_Nr = bk_config['cell_nr']
            electrolyte_Nr = bk_config['electrolyte_nr']
            logging.info(f"-------------------------------Resume from Last Assembly-----------------------------------------")

        # Creat Piprobot threads
        self.pre_thread = threading.Thread(target=self.dispensor.prepare_electrolyte, args=(electrolyte_Nr, electrolyte_Vol, False, bk_config['pre_operation']), name='Electrolyte_pre')

        # Creat Crimp thread
        self.crimp_thread = threading.Thread(target=self.pangpang.crimp_and_collect, name='Crimp_collect', args=(component_Nr,))

        start_time = time.time()
        logging.info(f"-------------------------------Cell No.[{component_Nr}]-----------------------------------------")

        # update the status
        self.status['Progress'].update(cell_nr=component_Nr, electrolyte_nr=electrolyte_Nr, electrolyte_vol=electrolyte_Vol, additive_Nr=additive_Nr, additive_Vol=additive_Vol, step=0)

        # Prepare electrolyte simutanously
        self.pre_thread.start()

        if bk_config['step_mark'] <= 1:

            for component in COMPONENTS[bk_config['component']:]:

                # Calculate the position for individual cell-Nr.
                grab_po = self.parameter[component]['grabPo'][str(component_Nr)]
                drop_po = self.parameter[component]['dropPo']
                holder_pos =  self.parameter[component]['railPo'][component_Nr-1]

                # Creat Piprobot adding thread
                self.add_thread = threading.Thread(target=self.electrolyte_additing, name='Electrolyte_add', args=(component, electrolyte_Nr, electrolyte_Vol, additive_Nr, additive_Vol, bk_config['pre_operation']))
                
                #Abort flag
                if self._abort.is_set():
                    # Join unfinished thread from piprobot
                    if self.pre_thread.is_alive():
                        self.pre_thread.join()
                    if self.add_thread.is_alive():
                        self.add_thread.join()
                    # Perform finishing again in case no active thread but piprobot is idle
                    self.dispensor.finish_dispensing()
                    return

                # Set wait Event, block the sequence while transrobot working and assembly robot will work on separator, cathode, cathode case
                if not self.pangpang._crimp_done.is_set() and component in (COMPONENTS[3], COMPONENTS[4], COMPONENTS[7]):
                    self.rubi.move(self.parameter['RAIL_STANDBY'])
                    self.pangpang._crimp_done.wait()

                logging.info(f'Grabing {component}')
                self.rubi.grab_component(holder_pos, grab_po)
                self.status['Progress'].update(STEPS[component]['Grab'])

                # Drive the robot to drop position
                
                #Abort flag
                if self._abort.is_set():
                    # Join unfinished thread from piprobot
                    if self.pre_thread.is_alive():
                        self.pre_thread.join()
                    if self.add_thread.is_alive():
                        self.add_thread.join()
                    # Perform finishing again in case no active thread but piprobot is idle
                    self.dispensor.finish_dispensing()
                    return

                # Set wait Event, block the sequence when transrobot working
                if not self.pangpang._crimp_done.is_set():
                    self.rubi.move(self.parameter['RAIL_STANDBY'])
                self.pangpang._crimp_done.wait()

                # Block the sequence when no electrolyte is prepared
                # if component in COMPONENTS[2:4]:
                #     self.dispensor._prep_done.wait()

                # To avoid collision with the transrobot, drive the assembly robot to standby posiiton, this is particular the case when dispersing additives
                if not self.dispensor._add_done.is_set() and additive_Nr != 0 and component in (COMPONENTS[3], COMPONENTS[4], COMPONENTS[7]):
                    self.rubi.move(self.parameter['RAIL_STANDBY'])

                # Set wait Event, block the sequence when piprobot working
                self.dispensor._add_done.wait()

                #Abort flag
                if self._abort.is_set():
                    # Join unfinished thread from piprobot
                    if self.pre_thread.is_alive():
                        self.pre_thread.join()
                    if self.add_thread.is_alive():
                        self.add_thread.join()
                    # Perform finishing again in case no active thread but piprobot is idle
                    self.dispensor.finish_dispensing()
                    return

                logging.info(f'Dropping {component} on post')
                drop_res = self.rubi.drop_component(drop_po, component, component_Nr, self.auto_calib, self.grab_check, self.save_img, self.show_image)
                if drop_res:
                    self.status['Progress'].update(STEPS[component]['Drop'])
                elif self.rubi.userInterupt == True:
                    logging.info(f"Failed finding {component}, interrupting process manually...")
                    self.abort()
                time.sleep(0.5)

                #Abort flag
                if self._abort.is_set():
                    # Join unfinished thread from piprobot
                    if self.pre_thread.is_alive():
                        self.pre_thread.join()
                    if self.add_thread.is_alive():
                        self.add_thread.join()
                    # Perform finishing again in case no active thread but piprobot is idle
                    self.dispensor.finish_dispensing()
                    return

                # Set wait Event, block the sequence when transisrobot working
                self.pangpang._crimp_done.wait()

                # Drop electrolyt onto the anode spacer, (anode) and separator, Simultanously grab next component
                self.add_thread.start()

        if bk_config['step_mark'] <= 2:

            #Abort flag
            if self._abort.is_set():
                return

            if self.press == True:
                logging.info('Pressing COMPONENTS on the post')

                #Abort flag
                if self._abort.is_set():
                    return

                self.rubi.press_cell()
                self.status['Progress'].update(STEPS['Press'])
                time.sleep(0.5)

                #Abort flag
                if self._abort.is_set():
                    return
            else:
                self.rubi.move(self.parameter['RAIL_STANDBY'])

                #Abort flag
                if self._abort.is_set():
                    self.rubi.move(0)
                    return

        if bk_config['step_mark'] <= 3:

            # Send cell to the crimper
            logging.info("Sending cell to the crimper")
            self.pangpang.send_to_crimp()
            self.status['Progress'].update(STEPS['Crimp'])

            # Crimp and collect using threading
            self.crimp_thread.start()

            # Retrieve cell from post
            # Abort flag
            if self._abort.is_set():
                return

            # Assembly done, robot sent back to zero
            self.rubi.move(0)

            self.status['Progress'].update(STEPS['Done'])

            timeConsumed = round((time.time()-start_time)/60, 2)

            logging.info(f"Cell complete, Cell [{component_Nr}] Time consume: {timeConsumed} min(s).\n")

            self._record_consuming_time(cell_number=component_Nr, consuming_time=timeConsumed)

            time.sleep(1)

    def electrolyte_additing(self, component:str, electrolyte_num:int=1, electrolyte_vol:int=35,
                             additive_num:int=0, additive_vol:int=0, add_check:int=-1):
        
        if component == COMPONENTS[1] and add_check != 1: # Anode spacer
                self.dispensor.add_electrolyte(volume=self.parameter['ONE_DROP'], touch=True)
                self.status['Progress'].update(STEPS[component]['Dispense'])
        if component == COMPONENTS[2] and add_check != 2 and self.anode_electrolyte == True: # Anode
            logging.info(f'Dispersing 1/2 Electrolyte from [{electrolyte_num}] on {component}')
            self.dispensor.add_electrolyte((electrolyte_vol-self.parameter['ONE_DROP'])//2)
            self.status['Progress'].update(STEPS[component]['Dispense'])
        if component == COMPONENTS[3] and add_check != 0: # Separator
            if additive_num != 0:
                logging.info(f'Dispersing additive from [{additive_num}] on {component}...')
                self.dispensor.add_electrolyte_plus_additive(electrolyte_num=electrolyte_num, additive_num=additive_num, additive_vol=additive_vol)
            else:
                logging.info(f'Dispersing Electrolyte on {component}')
                self.dispensor.add_electrolyte(Dispensor.ALL)
            self.status['Progress'].update(STEPS[component]['Dispense'])
            # Finish up simutanously
            self.dispensor.finish_dispensing()

    def pause(self):
        self.status["Pause"] = True
        # self.rubi.pause_assembly_rob()
        self._move_on.clear()
        logging.info(f"Assembly Paused: Cell [{self.status['Progress']['cell_nr']}]")
    
    def resume(self):
        self.status["Pause"] = False
        self._move_on.set()
        # time.sleep(0.5)
        # self.rubi.resume_assembly_rob()
        logging.info(f"Assembly Resumed: Cell [{self.status['Progress']['cell_nr']}]")
    
    def abort(self):
        self.status["Aborted"] = True
        self._abort.set()
        self.rubi.abort_assembly_rob()
        self.pangpang.abort_transport_rob()
        self.dispensor.abort_pipette_rob()
        if self.status["Pause"] == True:
            self.status["Pause"] = False
            self._move_on.set()
            time.sleep(0.5)
            # self.rubi.resume_assembly_rob()
        logging.info(f"Assembly manually aborted: Cell [{self.status['Progress']['cell_nr']}]")
    
    def home_all(self):
        self.rubi.go_home()
        self.rubi.reconnect_pump()
        self.pangpang.go_home()
        self.dispensor.go_home()
        self.dispensor.finish_dispensing()
                    
    def power_off(self):
        #Shut down the rail and all robots
        fail_re = []
        try:
            self.rubi.disconnect_assembly_robot()
        except:
            logging.error("Assembly Robot has lost connection!", exc_info=True)
            self.rubi.disconnect_pump()
            fail_re.append("Assembly Robot")
        try:
            self.pangpang.disconnect_transport_robot()
        except:
            logging.error("Transport Robot has lost connection!", exc_info=True)
            fail_re.append("Transport Robot")
        try:
            self.dispensor.disconnect_piprobot()
        except:
            logging.error("Pipetting Robot has lost connection!", exc_info=True)
            fail_re.append("Pipetting Robot")
        self.reset_status()
        return tuple(fail_re)

if __name__ == '__main__':
    os.chdir(ROOT)
    workflow = Assembly()