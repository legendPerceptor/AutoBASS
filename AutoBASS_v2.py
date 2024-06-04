from tkinter import *
from tkinter import font
from tkinter import messagebox
from tkinter import ttk
import os
import time
import json
import threading
import Assembly_v2 as asb
import Robot_test_UI as testrob
from Position_generator import ResetParameter

from pathlib import Path

cur_dir = os.path.dirname(__file__)
AUTOBASS_CELL_CONFIG = os.path.join(cur_dir, 'data', 'Cell_to_assemble.json')
FINALES_CELL_CONFIG = "C:/Users/Operator/Desktop/Bojing/AutoBASS_Tenant/AutoBASS_Tenant/src/AutoBASS_Tenant/Cell_to_assemble_finales.json"
AUTOBASS_LOCAL_SIGNAL = "C:/Users/Operator/Desktop/Bojing/AutoBASS_Tenant/AutoBASS_Tenant/src/AutoBASS_Tenant/AutoBASS_local_signal.json"
 
class AutobassGUI(ResetParameter):
    def __init__(self):
        ResetParameter.__init__(self)
        self.workflow = asb.Assembly()
        self.test_rubi = testrob.TestAssemblyRobot()
        self.test_pangpang = testrob.TestTransportRobot()
        self.init_window = Tk()

        # Specify font of labels and button's text
        global ft_label, ft_button
        ft_label = font.Font(family='Arial', size=12)
        ft_button = font.Font(size=15)

        # Specify Assembly Look-up notification
        self.assembly_win_str = "Initiate System and Setup your Cells"

        # Specify variable for gui status
        self.gui_state = None
        self.shutdown_click = False

        # Specify flag for finales control
        self.finalesInControl = False

    def centralize_window(self, window:Tk, window_width:int, window_height:int, offset:tuple=(0,0)):
        """
        This function takes window from Tk module and centralize it
            according to your screen's resolution
        
        Parameters
        ----------
        window: The Tk() object you want to resize and centralize
        window_width: width of the window you want to resize
        window_height: height of the window you want to resize
        """
        # Retrieve the screen width and height
        screen_width = window.winfo_screenwidth()
        screen_height = window.winfo_screenheight()

        # Set the window size and position
        window_x = int((screen_width - window_width) / 2)
        window_y = int((screen_height - window_height) / 2)

        window.geometry(f"{window_width}x{window_height}+{window_x+offset[0]}+{window_y+offset[1]}")

    def set_init_window(self):
        image_folder = Path(cur_dir) / "images"
        # Set the title, icon, size of the initial window
        self.init_window.title("AutoBASS GUI")
        self.init_window.iconbitmap(str(image_folder / "Robotarm.ico"))

        self.centralize_window(self.init_window, 820, 450)

        for widget in self.init_window.winfo_children():
            widget.destroy()

        init_frame = LabelFrame(self.init_window, padx=50, pady=20, borderwidth=5)
        init_frame.grid(row=0, column=0, padx=20, pady=25)

        init_label_1 = Label(init_frame, text="Select from following operations:", pady=5, font=('Arial', 14))
        init_label_1.grid(row=0, column=0, columnspan=3, pady=30)

        global shutdown_btn
        init_btn_1 = Button(init_frame, text="Assembly Coin Cell", font=ft_button,\
                    padx=10, pady=40, border=5, borderwidth=4, command=self.set_assembly_window)
        init_btn_2 = Button(init_frame, text="Calibrate Robots", font=ft_button,\
                    padx=20, pady=38, border=5, borderwidth=4, command=self.set_calibration_window)
        shutdown_btn = Button(init_frame, text="Shut Down System", font=ft_button,\
                        padx=10, pady=38, border=5, borderwidth=4, command=self.shutdown_system)
        exit_btn = Button(self.init_window, text="Exit", font=ft_button,\
                    padx=52, borderwidth=4, command=self.init_window.destroy)

        init_btn_1.grid(row=1, column=0, padx=10)
        init_btn_2.grid(row=1, column=1, padx=10)
        shutdown_btn.grid(row=1, column=2, padx=10)
        exit_btn.grid(row=2, column=0)

        self.init_window.mainloop()

    def set_assembly_window(self):
        
        self.init_window.title("Cell Assembly Interface")

        self.centralize_window(self.init_window, 425, 650)

        for widget in self.init_window.winfo_children():
            widget.destroy()

        # Create status bar
        self.assembly_status = StringVar()
        status_label = Label(self.init_window, textvariable=self.assembly_status, font=('Arial', 10), height=4, bd=2, relief=SUNKEN)
        status_label.grid(row=0, column=0, sticky=W+E)
        self.assembly_status.set(self.assembly_win_str)

        # Creat input frame
        assembly_frame = LabelFrame(self.init_window, padx=20, pady=10)
        assembly_frame.grid(row=1, column=0, padx=20, pady=10)

        # Creat input frame
        Cell_ArrangeFrame = LabelFrame(assembly_frame, padx=50, pady=20)
        Cell_ArrangeFrame.grid(row=0, column=0, columnspan=2, pady=10)

        prime_pump_btn = Button(Cell_ArrangeFrame, text="Setup Cells", padx=15, pady=24, borderwidth=4, font=ft_button, command=self.setup_cell)
        prime_pump_btn.grid(row=0, column=0, padx=5)

        # Create assembly button
        global assembly_btn, initiate_btn, config_btn#, abort_btn
        config_btn = Button(assembly_frame, text="Config System",  padx=3, pady=24, borderwidth=4, font=ft_button, command=self.start_config_gui)
        initiate_btn = Button(assembly_frame, text="Initiate System", padx=3, pady=24, borderwidth=4, font=ft_button, command=self.init_assembly_system)
        
        assembly_btn = Button(assembly_frame, text="Start Assembly", padx=85, pady=25, borderwidth=4, font=ft_button, command=self.verify_assembly_input)
        home_btn = Button(assembly_frame, text="Home System", padx=98, borderwidth=4, font=ft_button, command=self.workflow.home_all)
        back_btn = Button(self.init_window, text="Back", borderwidth=4, padx=34, font=ft_button, command=self.set_init_window)
        # exit_btn = Button(self.init_window, text="Exit", borderwidth=4, padx=35, font=ft_button, command=self.init_window.destroy)

        
        config_btn.grid(row=2, column=0, rowspan=2, padx=5)
        initiate_btn.grid(row=2, column=1, padx=5)
        
        assembly_btn.grid(row=4, column=0, columnspan=2, pady=10)
        home_btn.grid(row=5, column=0, columnspan=2)

        back_btn.grid(row=3, column=0, pady=5)
        # exit_btn.grid(row=4, column=0, pady=5)

        if self.workflow.status["Initiated"] != True:
            assembly_btn['state'] = 'disabled'

    def setup_cell(self):
        image_folder = Path(cur_dir) / "images"
        global dispense_window
        dispense_window = Toplevel()
        dispense_window.title("Select Dispense Mode")
        dispense_window.iconbitmap(str(image_folder / "Robotarm.ico"))

        self.centralize_window(dispense_window, 480, 280)

        SetupFrame = LabelFrame(dispense_window, padx=40, pady=20, borderwidth=5)
        SetupFrame.grid(row=0, column=0, columnspan=3)

        Button(SetupFrame, text="Manually Setup", background='Orange', font=ft_button, padx=10, pady=40, borderwidth=4, command=self.manual_setup).grid(row=0, column=0, padx=10, pady=20)
        Button(SetupFrame, text="FINALES Control",background='Green3', font=ft_button, padx=10, pady=40, borderwidth=4, command=self._finales_control).grid(row=0, column=2, padx=10, pady=20)
        Button(dispense_window, text="Back", borderwidth=4, padx=35, font=ft_button, command=dispense_window.destroy).grid(row=1, column=0, columnspan=3, pady=20)

    def manual_setup(self):

        for widget in dispense_window.winfo_children():
            widget.destroy()

        image_folder = Path(cur_dir) / "images"
        dispense_window.title("Setup Dispense")
        dispense_window.iconbitmap(str(image_folder / "Robotarm.ico"))

        self.centralize_window(dispense_window, 1080, 800)

        # Create A Main Frame
        main_frame = Frame(dispense_window)
        main_frame.pack(fill=BOTH, expand=1)

        # Create A Canvas
        canvas = Canvas(main_frame)
        canvas.pack(side=LEFT, fill=BOTH, expand=1)

        # Add A Scrollbar to the Canvas
        scrollbar = ttk.Scrollbar(main_frame, orient=VERTICAL, command=canvas.yview)
        scrollbar.pack(side=RIGHT, fill=Y)

        # Configure the Canvas
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.bind("<Configure>", lambda e: canvas.config(scrollregion=canvas.bbox("all")))

        # Create ANOTHER Frame INSIDE the Canvas
        second_frame = Frame(canvas)

        # Add that New Frame to a window in the Canvas
        canvas.create_window((0,0), window=second_frame, anchor=NW)

        def update_color():
            for Nr in range(64):
                if Cell_Check_Vals[Nr].get() == 1:
                    Cell_SetupFrames[Nr]['bg'] = "Green3"
                    Cell_Checkbuttons[Nr]['bg'] = "Green3"
                    Cell_Checkbuttons[Nr]['selectcolor'] = "Red1"
                else:
                    Cell_SetupFrames[Nr]['bg'] = "Grey"
                    Cell_Checkbuttons[Nr]['bg'] = "Grey"
                    Cell_Checkbuttons[Nr]['selectcolor'] = "White"

        def update_menu(_:int=1):
            for Nr in range(64):
                if Electrolyte_Nr2_Vals[Nr].get() == 0:
                    Electrolyte_Nr2_Vol_OptionMenus[Nr].config(state='disabled')
                else:
                    Electrolyte_Nr2_Vol_OptionMenus[Nr].config(state='normal')

        def check_all():
            if all_val.get() == 0:
                for Nr in range(64):
                    Cell_Check_Vals[Nr].set(0)
            if all_val.get() == 1:
                for Nr in range(64):
                    Cell_Check_Vals[Nr].set(1)
            update_color()

        data_folder = Path(cur_dir) / "data"
        with open(str(data_folder / 'Cell_to_assemble.json')) as infile:
            setings = json.load(infile)

        # Specify font of labels and button's text
        ft_label = font.Font(family='Arial', size=14, weight=font.BOLD)

        all_val = IntVar()
        All_Checkbuttons = Checkbutton(second_frame, text="Check All", font=ft_label, anchor="w", variable=all_val, onvalue=1, offvalue=0, command=check_all)
        All_Checkbuttons.grid(row=2, column=0, columnspan=8)

        Label(second_frame, text="Setup your Cells And Electrolytes to Assemble: ", padx=15, pady=15, font=ft_label).grid(row=0, column=0, columnspan=8)

        ExampleFrame = LabelFrame(second_frame, text="Example:  ", padx= 5, pady=5, borderwidth=5)
        ExampleFrame.grid(row=1, column=0, columnspan=8)

        Label(ExampleFrame, text='Click on this number to add the Cell', font=font.Font(family='Arial', size=7)).grid(row=0,column=0,columnspan=2)
        Checkbutton(ExampleFrame, text="#1: ", font=ft_label, state='disabled').grid(row=1,column=0,columnspan=2)
        Label(ExampleFrame, text='Elyectrolyte Nr', font=font.Font(family='Arial', size=7)).grid(row=2,column=0)
        Label(ExampleFrame, text='Volume µL', font=font.Font(family='Arial', size=7)).grid(row=2,column=1)
        exampleNr = IntVar()
        exampleVolume = IntVar()
        exampleNrMenu = OptionMenu(ExampleFrame, exampleNr, *(1,))
        exampleVolMenu = OptionMenu(ExampleFrame, exampleVolume, *(35,))
        exampleNrMenu.config(state='disabled')
        exampleVolMenu.config(state='disabled')
        exampleNrMenu.grid(row=3,column=0)
        exampleVolMenu.grid(row=3,column=1)
        exampleNr.set(1)
        exampleVolume.set(35)

        SetupFrame = LabelFrame(second_frame, padx= 5, pady=5, borderwidth=5)
        SetupFrame.grid(row=3, column=0, columnspan=8)

        Cell_SetupFrames = [LabelFrame(SetupFrame, padx=5, pady=5, borderwidth=5) for _ in range(64)]
        Cell_Check_Vals = [IntVar() for _ in range(64)]
        Cell_Checkbuttons = [Checkbutton(Cell_SetupFrames[Nr], text=f"#{Nr+1}: ", activebackground="Green", font=ft_label, variable=Cell_Check_Vals[Nr], onvalue=1, offvalue=0, command=update_color) for Nr in range(64)]
        
        Electrolyte_Nr1_Vals = [IntVar() for _ in range(64)]
        Electrolyte_Nr1_OptionMenus = [OptionMenu(Cell_SetupFrames[Nr], Electrolyte_Nr1_Vals[Nr], *(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)) for Nr in range(64)]

        Electrolyte_Nr1_Vols = [IntVar() for _ in range(64)]
        Electrolyte_Nr1_Vol_OptionMenus = [OptionMenu(Cell_SetupFrames[Nr], Electrolyte_Nr1_Vols[Nr], *(10,15,20,25,30,35,40,45,50,55,60,65,70,75,80)) for Nr in range(64)]

        Electrolyte_Nr2_Vals = [IntVar() for _ in range(64)]
        Electrolyte_Nr2_OptionMenus = [OptionMenu(Cell_SetupFrames[Nr], Electrolyte_Nr2_Vals[Nr], *(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16), command=update_menu) for Nr in range(64)]

        Electrolyte_Nr2_Vols = [IntVar() for _ in range(64)]
        Electrolyte_Nr2_Vol_OptionMenus = [OptionMenu(Cell_SetupFrames[Nr], Electrolyte_Nr2_Vols[Nr], *(10,15,20,25,30,35,40,45,50,55,60,65,70,75,80)) for Nr in range(64)]

        for x in range(8):
            for y in range(8):
                Nr = x*8+(y+1)

                Cell_Check_Vals[Nr-1].set(setings['New'][str(Nr)][0])
                Electrolyte_Nr1_Vals[Nr-1].set(setings['New'][str(Nr)][1])
                Electrolyte_Nr1_Vols[Nr-1].set(setings['New'][str(Nr)][2])
                Electrolyte_Nr2_Vals[Nr-1].set(setings['New'][str(Nr)][3])
                Electrolyte_Nr2_Vols[Nr-1].set(setings['New'][str(Nr)][4])

                Electrolyte_Nr1_OptionMenus[Nr-1].grid(row=1, column=0)
                Electrolyte_Nr1_Vol_OptionMenus[Nr-1].grid(row=1, column=1)
                Electrolyte_Nr2_OptionMenus[Nr-1].grid(row=2, column=0)
                Electrolyte_Nr2_Vol_OptionMenus[Nr-1].grid(row=2, column=1)
                Cell_Checkbuttons[Nr-1].grid(row=0, column=0, columnspan=2)
                
                Cell_SetupFrames[Nr-1].grid(row=y, column=x)
        update_color()
        update_menu()
        
        def save_setup():
            cells = []
            electrolytes = []
            # total_number = 0
            for count, cell_val in enumerate(Cell_Check_Vals):
                cell = cell_val.get()
                elyc_1 = Electrolyte_Nr1_Vals[count].get()
                vol_1 = Electrolyte_Nr1_Vols[count].get()
                elyc_2 = Electrolyte_Nr2_Vals[count].get()
                vol_2 = Electrolyte_Nr2_Vols[count].get()
                setings['New'][str(count+1)] = [cell, elyc_1, vol_1, elyc_2, vol_2]
                if cell == 1:
                    cells.append(count+1)
                    electrolytes.append(elyc_1)
                    # total_number += 1
            with open('Cell_to_assemble.json', 'w') as outfile:
                json.dump(setings, outfile, indent=4)
            if len(cells) == 1:
                self.assembly_win_str = f"\nAssmebly Task: #{cells[0]}\nElectrolyte: {electrolytes[0]}"
            else:
                task = ""
                conList = []
                for x in cells:
                    conList.append(x)
                    if x+1 not in cells:
                        if len(conList) == 1:
                            task += f"#{x}, "
                        else:
                            task += f"#{conList[0]}-{conList[-1]}, "
                        conList.clear()
                self.assembly_win_str = f"\nAssmebly Task: {task}\nTotal Number: {len(cells)} pcs\nElectrolyte Used: {set(electrolytes)}"
            self.assembly_status.set(self.assembly_win_str)
            messagebox.showinfo(title="Setup saved", message=f"Setup has been confirmed!")
            dispense_window.destroy()
            
        Button(second_frame, text="Save", bg='Green3', font=ft_button, padx=15, pady=5, borderwidth=4, command=save_setup).grid(row=4, column=3, pady=10)
        Button(second_frame, text="Exit",bg='Red', font=ft_button, padx=15, pady=5, borderwidth=4, command=dispense_window.destroy).grid(row=4, column=4, pady=10)

    def set_calibration_window(self):
        # Start a new window
        #self.init_window = Toplevel()
        self.init_window.title("Robot Calibration Interface")
        #self.init_window.iconbitmap("Robotarm.ico")
        self.init_window.geometry("570x450")

        for widget in self.init_window.winfo_children():
            widget.destroy()

        # Creat input frame
        cali_frame = LabelFrame(self.init_window, padx=50, pady=50, bd=5)
        cali_frame.grid(row=0, column=0, padx=20, pady=25)

        # Creat labels
        cali_label_1 = Label(cali_frame, text="Select The Robot to test:", pady=5, font=ft_label)
        cali_label_1.grid(row=0, column=0, columnspan=3, pady=10)

        # Create choices of testing robots
        cali_btn_1 = Button(cali_frame, text="Assembly Robot", font=ft_button,\
                        padx=10, pady=40, border=5, borderwidth=4, command=self.test_assembly)
        cali_btn_2 = Button(cali_frame, text="Transport Robot", font=ft_button, padx=15,\
                        pady=40, border=5, borderwidth=4, command=self.test_transport)
        exit_btn = Button(self.init_window, text="Exit", font=ft_button,\
                    padx=40, borderwidth=4, command=self.init_window.destroy)
        back_btn = Button(self.init_window, text="Back", font=ft_button, padx=34,\
                borderwidth=4, command=self.set_init_window)

        cali_btn_1.grid(row=1, column=0, padx=10)
        cali_btn_2.grid(row=1, column=1, padx=10)
        back_btn.grid(row=2, column=0, pady=5)
        exit_btn.grid(row=3, column=0, padx=10, pady=5)
        
    def test_assembly(self):
        self.gui_state = 'TestAssembly'
        self.init_window.state(newstate='iconic')
        self.test_rubi.set_test_assembly(self.init_window)

    def test_transport(self):
        self.gui_state = 'TestTransport'
        self.init_window.state(newstate='iconic')
        self.test_pangpang.init_trans_test_gui(self.init_window)

    def verify_assembly_input(self):
        """
        Phrase the saved jason data into a tuple which is compatible to one_cell function
        The tuple is a serial of single cell infomations, which is in the form: (cell_number, electrolyte_number, electrolyte_volume, breakinfo_dict)
        """
        data_folder = Path(cur_dir) / "data"
        with open(str(data_folder / 'Cell_to_assemble.json')) as infile:
            self.settings = json.load(infile)
        cell_infos = []
        bk_config = self.settings['Broken']
        if bk_config['cell_nr'] > 0 and bk_config['date'] == time.strftime("%Y_%m_%d_%Hh_%Mm_%Ss", time.localtime())[:10]:
            user = messagebox.askyesno("Broken Point detected!", f"You seem to have unfinished cell [{bk_config['cell_nr']}] from previous assembly, Do you want to continue from last step?")
            if user == True:
                cell_infos=[(bk_config['cell_nr'], bk_config['electrolyte_nr'], bk_config['electrolyte_vol'],
                             bk_config['additive_nr'], bk_config['additive_vol'], bk_config)]
            else:
                self.settings['Broken'].update(dict(cell_nr=-1, step=0, component=0, pre_operation=-1, step_mark=1))
                with open(str(data_folder / 'Cell_to_assemble.json'), 'w') as outfile:
                        json.dump(self.settings, outfile, indent=4)
        for cell_count, cell_info in self.settings['New'].items():
            if cell_info[0] == 1:
                prased_info = (int(cell_count), cell_info[1], cell_info[2], cell_info[3], cell_info[4],
                                dict(cell_nr=0, electrolyte_nr=0, step=0, component=0, pre_operation=-1, step_mark=1))
                cell_infos.append(prased_info)
        cell_infos = tuple(cell_infos)
        
        if len(cell_infos) == 0:
            messagebox.showerror("Error!", "No task has been assigned, Please Setup Your Cells First!")
            # Switch off finales connection to avoid trouble
            if self.finalesInControl:
                connectionSwitch.config(image = offImage)
                statusLabel.config(text = "FINALES is Disconnected!", fg = "grey")
                self.finalesIsInConnection = False
                self._kill_finales_autobass_tenant()
        else:
            if self.workflow.status["Initiated"] is True:
                # Reset pause and abort flag to enable assembly
                self.workflow._move_on.set()
                self.workflow._abort.clear()
                # Disable Assembly btn
                assembly_btn["state"] = 'disabled'
                time.sleep(0.1)
                self.assembly_gui(cell_infos)
            else:
                ErrorCode = tuple(zip(("Rubi--", "Pangpang--", "Dispensor--"), self.workflow.status["initResult"]))
                messagebox.showerror("Error!", message=f"System needs to be initiated first! Following Error has been discovered: {ErrorCode}")
                # Switch off finales connection to avoid trouble
                if self.finalesInControl:
                    connectionSwitch.config(image = offImage)
                    statusLabel.config(text = "FINALES is Disconnected!", fg = "grey")
                    self.finalesIsInConnection = False
                    self._kill_finales_autobass_tenant()
    
    def assembly_gui(self, cell_infos):
        self.init_window.state(newstate='iconic')
        
        global asebl_prog_window, prog_text, progbar
        asebl_prog_window = Toplevel()
        asebl_prog_window.title("Assembly Progress")
        asebl_prog_window.geometry('320x220')
        prog_text = StringVar()
        prog_label = Label(asebl_prog_window, textvariable=prog_text, font=ft_label)
        progbar = ttk.Progressbar(asebl_prog_window, length=200, mode='determinate', orient=HORIZONTAL)

        global abort_btn, pause_btn
        abort_btn = Button(asebl_prog_window, text="Abort", padx=35, pady=25, borderwidth=4, font=ft_button, command=self.abort)
        pause_btn = Button(asebl_prog_window, text="Pause", padx=32, pady=25, borderwidth=4, font=ft_button, command=self.pause)
        
        progbar.grid(row=1, column=0, columnspan=2, pady=10)
        prog_label.grid(row=2, column=0, columnspan=2, pady=10)
        pause_btn.grid(row=3, column=0, padx=5, pady=10)
        abort_btn.grid(row=3, column=1, padx=5, pady=10)

        global cell_nr
        cell_nr = cell_infos[0][0]
        # Reset Abort Flag to start new batch
        self.workflow.status["Aborted"] = False

        # Refresh the UI when first time started
        prog_text.set(f"Assembly Cell {cell_nr} in progress: (1%)")
        asebl_prog_window.update_idletasks()

        self.asbl_thread = threading.Thread(target=self.start_assembly_rob, args=(cell_infos,), daemon=True)
        self.asbl_thread.start()
        time.sleep(0.1)
        def update_probar():
            if self.workflow.status["Pause"] == True:
                prog_text.set(f"Assembly Paused! Cell: {cell_nr} ({progbar['value']}%)")
                asebl_prog_window.after(500, update_probar)
            if self.workflow.status["Aborted"] == True:
                if self.workflow.crimp_thread.is_alive():
                    prog_text.set(f"Aborting Cell {cell_nr}, Waiting for Crimping to be finished: ({self.workflow.pangpang.status['CrimpTime']}s)")
                    asebl_prog_window.update_idletasks()
                    asebl_prog_window.after(500, update_probar)
                elif self.asbl_thread.is_alive():
                    prog_text.set(f"Aborting Cell {cell_nr}, Waiting for Restoring...")
                    asebl_prog_window.update_idletasks()
                    asebl_prog_window.after(500, update_probar)
                else:
                    progbar['value'] = 100
                    time.sleep(1)
                    assembly_btn['state'] = 'normal'
                    self.assembly_status.set(f"Cell [{cell_nr}] has been aborted! \nB-Point:{self.workflow.get_step_val()}")
                    self.init_window.state(newstate='normal')
                    asebl_prog_window.destroy()
            if self.workflow.status["Pause"] == False and self.workflow.status["Aborted"] == False:
                if self.workflow.get_step_val() == 100:
                    progbar['value'] = 100
                    if not self.asbl_thread.is_alive():
                        if not self.workflow.crimp_thread.is_alive():
                            assembly_btn['state'] = 'normal'
                            self.init_window.state(newstate='normal')
                            asebl_prog_window.destroy()
                        else:
                            prog_text.set(f"Waiting for Cell {cell_nr} to finish Crimping: ({self.workflow.pangpang.status['CrimpTime']}s)")
                            asebl_prog_window.update_idletasks()
                            asebl_prog_window.after(500, update_probar)
                    else:
                        prog_text.set(f"Finishing Cell Assembly {cell_nr} (100%)...")
                        self.assembly_status.set(f"Assembly {cell_nr} completed!")
                        asebl_prog_window.update_idletasks()
                        progbar['value'] = 0
                        asebl_prog_window.after(500, update_probar)
                elif 0 <= progbar['value'] < self.workflow.get_step_val():
                    progbar['value'] += 1
                    prog_text.set(f"Assembly Cell {cell_nr} in progress: ({progbar['value']}%)")
                    asebl_prog_window.update_idletasks()
                    asebl_prog_window.after(1000, update_probar)
                else:
                    asebl_prog_window.after(500, update_probar)

        update_probar()
        asebl_prog_window.mainloop()
    
    def init_assembly_system(self):
        self.gui_state = 'Assembly'
        image_folder = Path(cur_dir) / "images"
        if self.workflow.status["Initiated"] != True:
            initiate_btn['state'] = 'disabled'
            prog_window = Toplevel()
            prog_window.title("Assembly Robot initializing")
            prog_window.iconbitmap(str(image_folder / "Robotarm.ico"))
            prog_window.geometry('280x150')
            prog_text = StringVar()
            prog_label = Label(prog_window, textvariable=prog_text, font=ft_label, pady=10, anchor=CENTER)
            prog = ttk.Progressbar(prog_window, length=250, mode='determinate', orient=HORIZONTAL)
            prog_label.grid(row=2, column=0, columnspan=2)
            prog.grid(row=1, column=0)
            
            init_thread = threading.Thread(target=self.workflow.initiate_all, daemon=True)
            init_thread.start()

            # Initialize progress bar value first time start
            prog['value'] = 1
            prog_window.update_idletasks()

            def update_probar():
                if init_thread.is_alive():
                    prog_text.set(f"Initiating System, Please Wait ({prog['value']}%)")
                    if prog['value'] < self.workflow.get_initiate_prog():
                        prog['value'] += 1
                    prog_window.update_idletasks()
                    prog_window.after(500, update_probar)
                else:
                    if self.workflow.status['Initiated'] == True:
                        prog['value'] = 100
                        prog_text.set(f"Initiating Finishing (100%)...")
                        initiate_btn['state'] = 'normal'
                        assembly_btn['state'] = 'normal'
                        prog_window.update_idletasks()
                        time.sleep(1)
                        prog_window.destroy()
                    else:
                        prog_text.set(f"Initiating system failed, exiting interface...")
                        initiate_btn['state'] = 'normal'
                        prog_window.update_idletasks()
                        time.sleep(1)
                        prog_window.destroy()
            update_probar()
            prog_window.mainloop()
        else:
            messagebox.showinfo("Attention!", "System is already initialized!")

    def _create_cells_log(self, cell_infos:tuple):
        if len(cell_infos) > 0:
            today = time.strftime("%Y_%m_%d", time.localtime())
            cellLogDir = str(Path(cur_dir) / 'Alignments' / today)
            if not os.path.exists(cellLogDir):
                os.makedirs(cellLogDir)
            file_name = os.path.join(cellLogDir, 'Cells_Log.json')
            if not os.path.exists(file_name):
                cellsLog:dict = {
                    "Date": today,
                }
            else:
                with open(file_name, "r") as readFile:
                    cellsLog:dict = json.load(readFile)
            for cellInfo in cell_infos:
                    cellsLog[cellInfo[0]] = {
                        "sealing_time": "",
                        "success": 0,
                        "time_consume": 0
                    }
            with open(file_name, "w") as outfile:
                json.dump(cellsLog, outfile, indent=4)

    def start_assembly_rob(self, cell_infos:tuple):
        global cell_nr
        self.workflow.apply_set_up()
        self._create_cells_log(cell_infos=cell_infos)
        for cell_info in cell_infos:
            cell_nr = cell_info[0]
            # electrolyte_nr = cell_info[1]
            self.assembly_status.set(f"Assembly Cell {cell_nr}")
            # self.workflow.one_cell(cell_nr, electrolyte_nr)
            self.workflow.one_cell(*cell_info)
            if self.workflow._abort.isSet():
                self.settings['New'][str(cell_nr)][0] = 0
                self.settings['Broken'] = self.workflow.status['Progress']
                self.settings['Broken']['date'] = time.strftime("%Y_%m_%d_%Hh_%Mm_%Ss", time.localtime())[:10]
                self._write_autobass_config(configToWrite=self.settings)
                # os.chdir(f"{PATH}\data")
                # with open('Cell_to_assemble.json', 'w') as outfile:
                #     json.dump(self.settings, outfile, indent=4)
                break
            else:
                self.settings['New'][str(cell_nr)][0] = 0
                self._write_autobass_config(configToWrite=self.settings)
                # os.chdir(f"{PATH}\data")
                # with open('Cell_to_assemble.json', 'w') as outfile:
                #     json.dump(self.settings, outfile, indent=4)
            # time.sleep(0.5)

    def pause(self):
        prog_text.set(f"Assembly Paused! Cell: {cell_nr} ({progbar['value']}%)")
        self.workflow.pause()
        pause_btn.config(text='Resume', padx=35, command=self.resume)

    def resume(self):
        prog_text.set(f"Assembly Cell {cell_nr}, Progress: ({progbar['value']}%)")
        self.workflow.resume()
        pause_btn.config(text='Pause', padx=42, command=self.pause)

    def abort(self):
        self.workflow.pause()
        userConfirm = messagebox.askyesno(title="Abort Assembly", message="Are you sure to abort the assembly?")
        if userConfirm == True:
            self.workflow.abort()
            self.workflow.resume()
        else:
            self.workflow.resume()

    def shutdown_system(self):
        self.shutdown_click = True
        threading.Thread(name='Shutting down', target=self.shutdown_system_rob).start()
            
    def shutdown_system_rob(self):
        try:
            shutdown_btn['state'] = 'disabled'
        except:
            pass
        if self.gui_state == 'Assembly':
            off_res = self.workflow.power_off()
            if len(off_res) == 0:
                messagebox.showinfo("Information", "AuoBASS is offline!")
            else:
                messagebox.showinfo("Information", f"Following devices have lost their connection: {off_res}")
        elif self.gui_state == 'TestAssembly':
            try:
                self.test_rubi.end_assembly_test()
            except:
                print("Assembly Test is not online!")
            else:
                messagebox.showinfo("Information", "Assembly Robot is offline!")
        elif self.gui_state == 'TestTransport':
            try:
                self.test_pangpang.end_trans_test()
            except:
                print("Transport Test is not online!")
            else:
                messagebox.showinfo("Information", "Transport Robot is offline!")
        else:
            pass
        try:
            shutdown_btn['state'] = 'normal'
        except:
            pass

    # ------------------------------------FINALES Functions------------------------------------
    def _read_finales_config(self) -> dict:
        with open(FINALES_CELL_CONFIG, "r") as fileToRead:
            finalesConfig = json.load(fileToRead)
        return finalesConfig
    
    def _write_finales_config(self, configToWrite:dict) -> None:
        with open(FINALES_CELL_CONFIG, "w") as fileToWrite:
            json.dump(configToWrite, fileToWrite, indent=4)

    def _read_autobass_config(self) -> dict:
        with open(AUTOBASS_CELL_CONFIG, "r") as configToRead:
            autobassConfig = json.load(configToRead)
        return autobassConfig
    
    def _write_autobass_config(self, configToWrite:dict) -> None:
        with open(AUTOBASS_CELL_CONFIG, "w") as fileToWrite:
            json.dump(configToWrite, fileToWrite, indent=4)

    def _read_autobass_local_signal(self) -> dict:
        with open(AUTOBASS_LOCAL_SIGNAL, "r") as infile:
            autoBASSLocalSignal = json.load(infile)
        return autoBASSLocalSignal

    def _write_autobass_local_signal(self, signal_to_write:dict) -> None:
        with open(AUTOBASS_LOCAL_SIGNAL, "w") as fileToWrite:
            json.dump(signal_to_write, fileToWrite, indent=4)

    def _request_is_in_position(self) -> bool:
        autoBASSLocalSignal = self._read_autobass_local_signal()
        return autoBASSLocalSignal['request_in_position'] == 1

    def _read_last_cell_position(self) -> int:
        return self._read_autobass_local_signal()['last_cell_position']

    def _remove_local_request(self) -> None:
        autoBASSLocalSignal = self._read_autobass_local_signal()
        autoBASSLocalSignal['request_in_position'] = 0
        autoBASSLocalSignal['request_id'] = ""
        self._write_autobass_local_signal(autoBASSLocalSignal)

    def _kill_finales_autobass_tenant(self) -> None:
        finalesConfig:dict = self._read_finales_config()
        finalesConfig['kill_tenant'] = 1
        self._write_finales_config(configToWrite=finalesConfig)

    # Define our switch function
    def _switch_finales_connection(self):
        # Determine is on or off
        if self.finalesIsInConnection:
            connectionSwitch.config(image = offImage)
            statusLabel.config(text = "FINALES is Disconnected!", fg = "grey")
            self.finalesIsInConnection = False
            self._kill_finales_autobass_tenant()
        else:
            connectionSwitch.config(image = onImage)
            statusLabel.config(text = "FINALES is Connected!", fg = "green")
            self.finalesIsInConnection = True

    def _pack_task_for_finales(self) -> tuple:
        """
        Phrase the saved jason data into a tuple which is compatible to one_cell function
        The tuple is a serial of single cell infomations, which is in the form: (cell_number, electrolyte_number, electrolyte_volume, breakinfo_dict)
        """
        data_folder = Path(cur_dir) / "data"
        with open(str(data_folder / 'Cell_to_assemble.json')) as infile:
            self.settings = json.load(infile)
        cell_infos = []
        bk_config = self.settings['Broken']
        if bk_config['cell_nr'] > 0 and bk_config['date'] == time.strftime("%Y_%m_%d_%Hh_%Mm_%Ss", time.localtime())[:10]:
            user = messagebox.askyesno("Broken Point detected!", f"You seem to have unfinished cell [{bk_config['cell_nr']}] from previous assembly, Do you want to continue from last step?")
            if user == True:
                cell_infos=[(bk_config['cell_nr'], bk_config['electrolyte_nr'], bk_config['electrolyte_vol'],
                             bk_config['additive_nr'], bk_config['additive_vol'], bk_config)]
            else:
                self.settings['Broken'].update(dict(cell_nr=-1, step=0, component=0, pre_operation=-1, step_mark=1))
                with open(str(data_folder / 'Cell_to_assemble.json'), 'w') as outfile:
                        json.dump(self.settings, outfile, indent=4)
        for cell_count, cell_info in self.settings['New'].items():
            if cell_info[0] == 1:
                prased_info = (int(cell_count), cell_info[1], cell_info[2], cell_info[3], cell_info[4],
                                dict(cell_nr=0, electrolyte_nr=0, step=0, component=0, pre_operation=-1, step_mark=1))
                cell_infos.append(prased_info)
        return tuple(cell_infos)

    def _finales_execute_assembly(self):
        while self.finalesInControl:
            if self.finalesIsInConnection:
                if self._request_is_in_position():
                    info = self.file_setup()
                    userConfirm = messagebox.askyesno(title="Information", message=info)
                    if userConfirm == True:
                        print("Processing request...                 ")
                        time.sleep(1)
                        cellInfo = self._pack_task_for_finales()
                        if len(cellInfo) == 0:
                            messagebox.showerror("Error!", "No task has been assigned, Please Setup Your Cells First!")
                            # Switch off finales connection to avoid trouble
                            connectionSwitch.config(image = offImage)
                            statusLabel.config(text = "FINALES is Disconnected!", fg = "grey")
                            self.finalesIsInConnection = False
                        else:
                            if self.workflow.status["Initiated"] is True:
                                # Reset pause and abort flag to enable assembly
                                self.workflow._move_on.set()
                                self.workflow._abort.clear()
                                time.sleep(0.1)
                                self.start_assembly_rob(cell_infos=cellInfo)
                                if cell_nr == self._read_last_cell_position() and self.workflow.status["Aborted"] == False:
                                    timeToTransfer = self.workflow.pangpang.status['CrimpTime']+5
                                    print(f"Finishing last cell, waiting for transfering results in {timeToTransfer} s")
                                    time.sleep(timeToTransfer)
                                    self._remove_local_request()
                                    self._tranfer_results()
                            else:
                                ErrorCode = tuple(zip(("Rubi--", "Pangpang--", "Dispensor--"), self.workflow.status["initResult"]))
                                messagebox.showerror("Error!", message=f"System needs to be initiated first! Following Error has been discovered: {ErrorCode}")
                                # Switch off finales connection to avoid trouble
                                if self.finalesInControl:
                                    connectionSwitch.config(image = offImage)
                                    statusLabel.config(text = "FINALES is Disconnected!", fg = "grey")
                                    self.finalesIsInConnection = False
                    else:
                        self._remove_local_request()
                        connectionSwitch.config(image = offImage)
                        statusLabel.config(text = "FINALES is Disconnected!", fg = "grey")
                        self.finalesIsInConnection = False
                else:
                    for i in range(10,-1,-1):
                        print(f"No request in position, Next check {i} s ", end="\r", flush=True)
                        time.sleep(1)
        else:
            print("\nFinales connection terminated!")

    def _check_avaliable_cell_positions(self, batch_volume:int) -> list:
        cellPositionsToAssign = []
        startPosition = self._read_last_cell_position()
        for _ in range(batch_volume):
            startPosition = startPosition + 1
            if startPosition > 64:
                startPosition = 1
            cellPositionsToAssign.append(str(startPosition))
        return cellPositionsToAssign
    
    def _assign_task_to_avaliable_cell_positions(self, cell_positions_to_assign:list) -> None:
        if len(cell_positions_to_assign) > 0:
            cellToAssemble: dict = self._read_autobass_config()
            for cellKey in cell_positions_to_assign:
                cellToAssemble['New'][cellKey][0] = 1
            self._write_autobass_config(configToWrite=cellToAssemble)
            autobassLocalSignal = self._read_autobass_local_signal()
            autobassLocalSignal['last_cell_position'] = int(cell_positions_to_assign[-1])
            self._write_autobass_local_signal(signal_to_write=autobassLocalSignal)
    
    def _check_avaliable_electrolyte_position(self, formula:list) -> int:
        finalesCellConfig: dict = self._read_finales_config()
        avaliableFormulas:list = finalesCellConfig['avaliable_formulas']
        if formula in avaliableFormulas:
            electrolytePositionToAssign = avaliableFormulas.index(formula)+1
        elif [{}] in avaliableFormulas:
            electrolytePositionToAssign = avaliableFormulas.index([{}])+1
        else:
            electrolytePositionToAssign = 1
        return electrolytePositionToAssign

    def _assgin_avaliable_electrolyte_position(self, cell_positions_to_assign:list, electrolyte_position_to_assign:int) -> None:
        cellToAssemble: dict = self._read_autobass_config()
        for i in cell_positions_to_assign:
            cellToAssemble['New'][i][1] = electrolyte_position_to_assign
        self._write_autobass_config(configToWrite=cellToAssemble)

    def file_setup(self) -> str:
        finalesCellConfig: dict = self._read_finales_config()
        taskData: dict = finalesCellConfig.get('tasks')
        autobassLocalSignal: dict = self._read_autobass_local_signal()
        requestId = autobassLocalSignal['request_id']
        taskToAssign = taskData[requestId]
        if len(taskToAssign['batch_positions']) == 0:
            batchVolume = taskToAssign['batch_volume']
            batchFormula = taskToAssign['electrolyte_formula']
            cellPositionsToAssign = self._check_avaliable_cell_positions(batch_volume=batchVolume)
            self._assign_task_to_avaliable_cell_positions(cell_positions_to_assign=cellPositionsToAssign)
            electrolytePositionToAssign = self._check_avaliable_electrolyte_position(formula=batchFormula)
            self._assgin_avaliable_electrolyte_position(cell_positions_to_assign=cellPositionsToAssign, electrolyte_position_to_assign=electrolytePositionToAssign)
            
            finalesCellConfig['tasks'][requestId]['electrolyte_number'] = electrolytePositionToAssign
            finalesCellConfig['tasks'][requestId]['batch_positions'] = cellPositionsToAssign
            finalesCellConfig['avaliable_formulas'][electrolytePositionToAssign-1] = batchFormula
            self._write_finales_config(configToWrite=finalesCellConfig)
            
            info = f"-------- Incomming task --------\n[{requestId}]\n \
                  \n Position: {cellPositionsToAssign} \
                  \n Anode: Gr ------ Mass loading: {taskToAssign['cell_info']['anode']['mass_loading']} \
                  \n Cathode: LNO ------ Mass loading: {taskToAssign['cell_info']['cathode']['mass_loading']}\
                  \n Separator: {taskToAssign['cell_info']['separator']} \
                  \n Electrolyte Number: {electrolytePositionToAssign} \
                  \n Electrolyte Formula: {taskToAssign['cell_info']['electrolyte']} \
                  \n ----------------------------------\
                  \n Are you sure to proceed? \
                  "
        else:
            info = f"-------- Incomming task --------\n[{requestId}]\n \
                  \n Position: {taskToAssign['batch_positions']} \
                  \n Anode: Gr ------ Mass loading: {taskToAssign['cell_info']['anode']['mass_loading']} mAh/cm2\
                  \n Cathode: LNO ------ Mass loading: {taskToAssign['cell_info']['cathode']['mass_loading']} mAh/cm2\
                  \n Separator: {taskToAssign['cell_info']['separator']['material']} \
                  \n Electrolyte Number: {taskToAssign['electrolyte_number']} \
                  \n Electrolyte Formula: {taskToAssign['cell_info']['electrolyte']} \
                  \n ----------------------------------\
                  \n Are you sure to proceed? \
                  "
        return info

    def _tranfer_results(self):
        finalesConfig: dict = self._read_finales_config()
        taskData: dict = finalesConfig.get('tasks')
        # Basically we are only dealing with the latest request
        result_id:str = list(taskData.keys())[-1]
        date = time.strftime("%Y_%m_%d", time.localtime())
        resultDir = os.path.join(cur_dir, 'Alignments', date, 'Origin', 'Cathode_Drop')
        with open(os.path.join(cur_dir, 'Alignments', date, 'Cells_Log.json')) as sealingRecord:
            cellsLog:dict = json.load(sealingRecord)
        if not os.path.exists(resultDir):
            messagebox.showerror("Error!", message="No Results is available!")
            return
        results = os.listdir(resultDir)
        for i in taskData[result_id]['batch_positions']:
            pseudoResult = f"[No{i}]_Cathode_Drop_{date}.jpg"
            if pseudoResult in results:
                phrasedResult = {
                    i:{
                        "image":pseudoResult,
                        "sealing_time": cellsLog[i]['sealing_time'],
                        "success": cellsLog[i]['success'],
                        "time_consume": cellsLog[i]['time_consume']
                    }
                }
                taskData[result_id]['result'].update(phrasedResult)
        finalesConfig['tasks'] = taskData
        finalesConfig['result_ready'] = 1
        self._write_finales_config(configToWrite=finalesConfig)

    def _finales_control(self):
        self.init_window.state(newstate='iconic')
        self.finalesInControl = True
        self.finalesIsInConnection = False
        for widget in dispense_window.winfo_children():
            widget.destroy()
        dispense_window.title("FINALES Control Mode")
        dispense_window.iconbitmap("Robotarm.ico")

        self.centralize_window(dispense_window, 620, 400)

        global statusLabel, connectionSwitch, onImage, offImage

        statusLabel = Label(
            dispense_window,
            text = "FINALES is Disconnected!",
            fg = "grey",
            font = ("Helvetica", 32)
        )
        statusLabel.pack(pady=20)

        # Define Our Images
        image_folder = Path(cur_dir) / "images"
        onImage = PhotoImage(file = str(image_folder / "on.png"))
        offImage = PhotoImage(file = str(image_folder / "off.png"))

        connectFrame = LabelFrame(dispense_window, borderwidth=5)
        connectFrame.pack(pady=20)

        connectionSwitch = Button(connectFrame, image=offImage, bd=0, padx=10, pady=50, borderwidth=4, command=self._switch_finales_connection)
        connectionSwitch.pack()

        controlFrame = LabelFrame(dispense_window, borderwidth=5)
        controlFrame.pack(pady=30)

        def pause():
            self.workflow.pause()
            finalesPauseBtn.config(text='Resume', padx=35, command=resume)

        def resume():
            self.workflow.resume()
            finalesPauseBtn.config(text='Pause', padx=42, command=pause)

        finalesAbortBtn = Button(controlFrame, text="Abort", padx=35, borderwidth=4, font=ft_button, command=self.abort)
        finalesPauseBtn = Button(controlFrame, text="Pause", padx=32, borderwidth=4, font=ft_button, command=pause)

        finalesAbortBtn.pack(padx=50, pady=10, side='left')
        finalesPauseBtn.pack(padx=50, pady=10, side='right')

        Button(dispense_window, text="Back", borderwidth=4, padx=35, font=ft_button, command=self._back_to_setup_cell).pack(pady=10)

        self.finalesAssembly = threading.Thread(target=self._finales_execute_assembly, daemon=True)
        self.finalesAssembly.start()

    def _back_to_setup_cell(self):
        self.finalesInControl = False

        def _back_to_assembly_gui():
            dispense_window.destroy()
            self.init_window.state(newstate='normal')

        for widget in dispense_window.winfo_children():
            widget.destroy()
        image_folder = Path(cur_dir) / "images"
        dispense_window.title("Select Dispense Mode")
        dispense_window.iconbitmap(image_folder / "Robotarm.ico")

        self.centralize_window(dispense_window, 480, 280)

        SetupFrame = LabelFrame(dispense_window, padx=40, pady=20, borderwidth=5)
        SetupFrame.grid(row=0, column=0, columnspan=3)

        Button(SetupFrame, text="Manually Setup", background='Orange', font=ft_button, padx=10, pady=40, borderwidth=4, command=self.manual_setup).grid(row=0, column=0, padx=10, pady=20)
        Button(SetupFrame, text="FINALES Control",background='Green3', font=ft_button, padx=10, pady=40, borderwidth=4, command=self._finales_control).grid(row=0, column=2, padx=10, pady=20)
        Button(dispense_window, text="Back", borderwidth=4, padx=35, font=ft_button, command=_back_to_assembly_gui).grid(row=1, column=0, columnspan=3, pady=20)

if __name__ == "__main__":
    ao = AutobassGUI()
    ao.set_init_window()
    if ao.shutdown_click == False:
        ao.shutdown_system_rob()