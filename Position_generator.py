import json
import os
from tkinter import *
from tkinter import font
from tkinter import messagebox
import numpy as np
import math as ma
import logging

# Creat logging instant
logger = logging.getLogger('SystemConfig')

ROOT = os.path.dirname(__file__) # ..../config

CONFIG_DIR = os.path.join(ROOT, 'data', 'config.json')
CALIBRATION_DIR = os.path.join(ROOT, 'data', 'calibration.json')

COMPONENTS = ('Anode_Case', 'Anode_Spacer', 'Anode', 'Separator', 'Cathode', 'Cathode_Spacer', 'Washer', 'Cathode_Case')

class ResetParameter():

    def __init__(self):
        pass

    def start_config_gui(self):
        self.pos_config_window = Toplevel()
        # Set the title, icon, size of the initial window
        self.pos_config_window.title("AutoBASS Config")
        self.pos_config_window.iconbitmap(os.path.join(ROOT, "images", "Location.ico"))

        generate_frame = LabelFrame(self.pos_config_window, text="Adjust Position:", padx=105, pady=18, borderwidth=5)
        generate_frame.grid(row=0, column=0, columnspan=2, padx=5, pady=10)

        procedure_frame = LabelFrame(self.pos_config_window, text="Procedure Setup:", padx=10, pady=20, borderwidth=5)
        procedure_frame.grid(row=1, columnspan=2, padx=5, pady=10)

        setup_frame = LabelFrame(self.pos_config_window, text="General Setup:", padx=10, pady=20, borderwidth=5)
        setup_frame.grid(row=2, columnspan=2, padx=5)

        def corelated_check():
            if auto_correction.get() == 0:
                show_image_bx.config(state=DISABLED)
                grab_check_bx.config(state=DISABLED)
            else:
                show_image_bx.config(state=NORMAL)
                grab_check_bx.config(state=NORMAL)

        # Creat checkbox
        global tap_press, electrolyte, auto_correction, save_img, grab_check, show_image
        tap_press = IntVar()
        pressing_bx = Checkbutton(procedure_frame, text="Tap Pressing", variable=tap_press, onvalue=1, offvalue=0)
        electrolyte = IntVar()
        add_electrolyte_bx = Checkbutton(procedure_frame, text="Electrolyte on Anode", variable=electrolyte, onvalue=1, offvalue=0)
        auto_correction = IntVar()
        auto_correction_bx = Checkbutton(procedure_frame, text="Auto Correction", variable=auto_correction, onvalue=1, offvalue=0, command=corelated_check)
        save_img = IntVar()
        save_img_bx = Checkbutton(procedure_frame, text="Save Image", variable=save_img, onvalue=1, offvalue=0)
        grab_check = IntVar()
        grab_check_bx = Checkbutton(procedure_frame, text="Grabe Check", variable=grab_check, onvalue=1, offvalue=0)
        show_image = IntVar()
        show_image_bx = Checkbutton(procedure_frame, text="Show Image", variable=show_image, onvalue=1, offvalue=0)

        pressing_bx.grid(row=0, column=0, sticky=W, padx=12, pady=5)
        add_electrolyte_bx.grid(row=0, column=1, sticky=W, padx=12, pady=5)
        auto_correction_bx.grid(row=1, column=0, sticky=W, padx=12, pady=5)
        save_img_bx.grid(row=1, column=1, sticky=W, padx=12, pady=5)
        grab_check_bx.grid(row=2, column=0, sticky=W, padx=12, pady=5)
        show_image_bx.grid(row=2, column=1, sticky=W, padx=12, pady=5)

        reset_grabz_btn = Button(generate_frame, text="Reset GrabZ", font=('Arial', 10), padx=5, pady=3, border=5, borderwidth=4, command=self.adjust_grabZ)

        exit_btn = Button(self.pos_config_window, text="OK", font=('Arial', 14), padx=40, borderwidth=4, command=self.confirm_setup)

        reset_grabz_btn.pack(padx=10, pady=3)
        exit_btn.grid(row=3, column=0, pady=10, columnspan=2)

        global joint_vel_slider, cartlin_vel_slider, gripper_vel_slider, gripper_force_slider, axis_vel_slider
        joint_vel_slider = Scale(setup_frame, from_=30, to=80, length=300, label='Joint Velocity mm/s', orient=HORIZONTAL)
        cartlin_vel_slider = Scale(setup_frame, from_=10, to=50, length=300, label='CarLin Velocity mm/s', orient=HORIZONTAL)
        axis_vel_slider = Scale(setup_frame, from_=1, to=10, length=300, label='Axis Velocity m/s', orient=HORIZONTAL)
        gripper_vel_slider = Scale(setup_frame, from_=10, to=100, length=300, label='Gripper Velocity %', orient=HORIZONTAL)
        gripper_force_slider = Scale(setup_frame, from_=10, to=100, length=300, label='Gripper Force %', orient=HORIZONTAL)
        
        joint_vel_slider.grid(row=0, column=0)
        cartlin_vel_slider.grid(row=1, column=0)
        axis_vel_slider.grid(row=2, column=0)
        gripper_vel_slider.grid(row=3, column=0)
        gripper_force_slider.grid(row=4, column=0)

        with open(CONFIG_DIR) as json_file:
            readings = json.load(json_file)
        joint_vel_slider.set(readings['J_VEL'])
        cartlin_vel_slider.set(readings['L_VEL'])
        gripper_vel_slider.set(readings['GRIP_VEL'])
        gripper_force_slider.set(readings['GRIP_F'])
        axis_vel_slider.set(readings['AX_VEL'])
        tap_press.set(readings['tap_press'])
        electrolyte.set(readings['add_electrolyte'])
        auto_correction.set(readings['auto_calib'])
        save_img.set(readings['save_img'])
        grab_check.set(readings['grab_check'])
        show_image.set(readings['show_image'])
        
        self.pos_config_window.mainloop()

    def adjust_grabZ(self):
        adjust_grabZ_window = Toplevel()
        # Set the title, icon, size of the initial window
        adjust_grabZ_window.title("Z Axis Config")
        adjust_grabZ_window.iconbitmap(os.path.join(ROOT, "images", "Location.ico"))

        ft_label = font.Font(family='Arial', size=12)
        ft_btn = font.Font(family='Arial', size=5, weight='bold')

        config_frame = LabelFrame(adjust_grabZ_window, text="Configure Z Axis Value (mm):", padx=15, pady=20, borderwidth=5)
        config_frame.grid(row=0, column=0, columnspan=2, padx=5, pady=5)

        min_z_vals = []
        with open(CALIBRATION_DIR) as json_file:
            readings_calib = json.load(json_file)
        for component in COMPONENTS:
            z_vals = []
            for nr in range(1, 65):
                z_vals.append(readings_calib[component]['grabPo'][str(nr)][2])
            min_z_vals.append(min(z_vals))

        # Creat Unit Anode Case
        ###########################################################################################
        def AnodeCaseZ_plus():
            newZ = float(AnodeCase_input.get())+0.1
            AnodeCase_input.delete(0, END)
            AnodeCase_input.insert(0, round(newZ, 3))
        def AnodeCaseZ_minus():
            newZ = float(AnodeCase_input.get())-0.1
            AnodeCase_input.delete(0, END)
            AnodeCase_input.insert(0, round(newZ, 3))

        AnodeCase_label = Label(config_frame, text="Anode Case:", padx=10, pady=15, font=ft_label)
        AnodeCase_input = Entry(config_frame, width=8, bd=5)
        AnodeCase_input.insert(0, min_z_vals[0])
        AnodeCase_plusminusFrame = LabelFrame(config_frame)
        AnodeCase_plus = Button(AnodeCase_plusminusFrame, text="+", font=ft_btn, padx=8, command=AnodeCaseZ_plus)
        AnodeCase_minus = Button(AnodeCase_plusminusFrame, text="-", font=ft_btn, padx=9, command=AnodeCaseZ_minus)

        AnodeCase_label.grid(row=0, column=0)
        AnodeCase_input.grid(row=0, column=1)
        AnodeCase_plusminusFrame.grid(row=0, column=2, padx=5)
        AnodeCase_plus.grid(row=0, column=0)
        AnodeCase_minus.grid(row=1, column=0)

        # Creat Unit Anode Spacer
        ###########################################################################################
        def AnodeSpacerZ_plus():
            newZ = float(AnodeSpacer_input.get())+0.1
            AnodeSpacer_input.delete(0, END)
            AnodeSpacer_input.insert(0, round(newZ, 3))
        def AnodeSpacerZ_minus():
            newZ = float(AnodeSpacer_input.get())-0.1
            AnodeSpacer_input.delete(0, END)
            AnodeSpacer_input.insert(0, round(newZ, 3))

        AnodeSpacer_label = Label(config_frame, text="Anode Spacer:", padx=10, pady=15, font=ft_label)
        AnodeSpacer_input = Entry(config_frame, width=8, bd=5)
        AnodeSpacer_input.insert(0, min_z_vals[1])
        AnodeSpacer_plusminusFrame = LabelFrame(config_frame)
        AnodeSpacer_plus = Button(AnodeSpacer_plusminusFrame, text="+", font=ft_btn, padx=8, command=AnodeSpacerZ_plus)
        AnodeSpacer_minus = Button(AnodeSpacer_plusminusFrame, text="-", font=ft_btn, padx=9, command=AnodeSpacerZ_minus)

        AnodeSpacer_label.grid(row=1, column=0)
        AnodeSpacer_input.grid(row=1, column=1)
        AnodeSpacer_plusminusFrame.grid(row=1, column=2, padx=5)
        AnodeSpacer_plus.grid(row=0, column=0)
        AnodeSpacer_minus.grid(row=1, column=0)

        # Creat Unit Anode
        ###########################################################################################
        def AnodeZ_plus():
            newZ = float(Anode_input.get())+0.1
            Anode_input.delete(0, END)
            Anode_input.insert(0, round(newZ, 3))
        def AnodeZ_minus():
            newZ = float(Anode_input.get())-0.1
            Anode_input.delete(0, END)
            Anode_input.insert(0, round(newZ, 3))

        Anode_label = Label(config_frame, text="Anode:", padx=10, pady=15, font=ft_label)
        Anode_input = Entry(config_frame, width=8, bd=5)
        Anode_input.insert(0, min_z_vals[2])
        Anode_plusminusFrame = LabelFrame(config_frame)
        Anode_plus = Button(Anode_plusminusFrame, text="+", font=ft_btn, padx=8, command=AnodeZ_plus)
        Anode_minus = Button(Anode_plusminusFrame, text="-", font=ft_btn, padx=9, command=AnodeZ_minus)

        Anode_label.grid(row=2, column=0)
        Anode_input.grid(row=2, column=1)
        Anode_plusminusFrame.grid(row=2, column=2, padx=5)
        Anode_plus.grid(row=0, column=0)
        Anode_minus.grid(row=1, column=0)

        # Creat Unit Separator
        ###########################################################################################
        def SeparatorZ_plus():
            newZ = float(Separator_input.get())+0.1
            Separator_input.delete(0, END)
            Separator_input.insert(0, round(newZ, 3))
        def SeparatorZ_minus():
            newZ = float(Separator_input.get())-0.1
            Separator_input.delete(0, END)
            Separator_input.insert(0, round(newZ, 3))

        Separator_label = Label(config_frame, text="Separator:", padx=10, pady=15, font=ft_label)
        Separator_input = Entry(config_frame, width=8, bd=5)
        Separator_input.insert(0, min_z_vals[3])
        Separator_plusminusFrame = LabelFrame(config_frame)
        Separator_plus = Button(Separator_plusminusFrame, text="+", font=ft_btn, padx=8, command=SeparatorZ_plus)
        Separator_minus = Button(Separator_plusminusFrame, text="-", font=ft_btn, padx=9, command=SeparatorZ_minus)

        Separator_label.grid(row=3, column=0)
        Separator_input.grid(row=3, column=1)
        Separator_plusminusFrame.grid(row=3, column=2, padx=5)
        Separator_plus.grid(row=0, column=0)
        Separator_minus.grid(row=1, column=0)

        # Creat Unit Cathode
        ###########################################################################################
        def CathodeZ_plus():
            newZ = float(Cathode_input.get())+0.1
            Cathode_input.delete(0, END)
            Cathode_input.insert(0, round(newZ, 3))
        def CathodeZ_minus():
            newZ = float(Cathode_input.get())-0.1
            Cathode_input.delete(0, END)
            Cathode_input.insert(0, round(newZ, 3))

        Cathode_label = Label(config_frame, text="Cathode:", padx=10, pady=15, font=ft_label)
        Cathode_input = Entry(config_frame, width=8, bd=5)
        Cathode_input.insert(0, min_z_vals[4])
        Cathode_plusminusFrame = LabelFrame(config_frame)
        Cathode_plus = Button(Cathode_plusminusFrame, text="+", font=ft_btn, padx=8, command=CathodeZ_plus)
        Cathode_minus = Button(Cathode_plusminusFrame, text="-", font=ft_btn, padx=9, command=CathodeZ_minus)

        Cathode_label.grid(row=4, column=0)
        Cathode_input.grid(row=4, column=1)
        Cathode_plusminusFrame.grid(row=4, column=2, padx=5)
        Cathode_plus.grid(row=0, column=0)
        Cathode_minus.grid(row=1, column=0)

        # Creat Unit Cathode Spacer
        ###########################################################################################
        def CathodeSpacerZ_plus():
            newZ = float(CathodeSpacer_input.get())+0.1
            CathodeSpacer_input.delete(0, END)
            CathodeSpacer_input.insert(0, round(newZ, 3))
        def CathodeSpacerZ_minus():
            newZ = float(CathodeSpacer_input.get())-0.1
            CathodeSpacer_input.delete(0, END)
            CathodeSpacer_input.insert(0, round(newZ, 3))

        CathodeSpacer_label = Label(config_frame, text="Cathode Spacer:", padx=10, pady=15, font=ft_label)
        CathodeSpacer_input = Entry(config_frame, width=8, bd=5)
        CathodeSpacer_input.insert(0, min_z_vals[5])
        CathodeSpacer_plusminusFrame = LabelFrame(config_frame)
        CathodeSpacer_plus = Button(CathodeSpacer_plusminusFrame, text="+", font=ft_btn, padx=8, command=CathodeSpacerZ_plus)
        CathodeSpacer_minus = Button(CathodeSpacer_plusminusFrame, text="-", font=ft_btn, padx=9, command=CathodeSpacerZ_minus)

        CathodeSpacer_label.grid(row=5, column=0)
        CathodeSpacer_input.grid(row=5, column=1)
        CathodeSpacer_plusminusFrame.grid(row=5, column=2, padx=5)
        CathodeSpacer_plus.grid(row=0, column=0)
        CathodeSpacer_minus.grid(row=1, column=0)

        # Creat Unit Washer
        ###########################################################################################
        def WasherZ_plus():
            newZ = float(Washer_input.get())+0.1
            Washer_input.delete(0, END)
            Washer_input.insert(0, round(newZ, 3))
        def WasherZ_minus():
            newZ = float(Washer_input.get())-0.1
            Washer_input.delete(0, END)
            Washer_input.insert(0, round(newZ, 3))

        Washer_label = Label(config_frame, text="Washer:", padx=10, pady=15, font=ft_label)
        Washer_input = Entry(config_frame, width=8, bd=5)
        Washer_input.insert(0, min_z_vals[6])
        Washer_plusminusFrame = LabelFrame(config_frame)
        Washer_plus = Button(Washer_plusminusFrame, text="+", font=ft_btn, padx=8, command=WasherZ_plus)
        Washer_minus = Button(Washer_plusminusFrame, text="-", font=ft_btn, padx=9, command=WasherZ_minus)

        Washer_label.grid(row=6, column=0)
        Washer_input.grid(row=6, column=1)
        Washer_plusminusFrame.grid(row=6, column=2, padx=5)
        Washer_plus.grid(row=0, column=0)
        Washer_minus.grid(row=1, column=0)

        # Creat Unit Cathode Case
        ###########################################################################################
        def CathodeCaseZ_plus():
            newZ = float(CathodeCase_input.get())+0.1
            CathodeCase_input.delete(0, END)
            CathodeCase_input.insert(0, round(newZ, 3))
        def CathodeCaseZ_minus():
            newZ = float(CathodeCase_input.get())-0.1
            CathodeCase_input.delete(0, END)
            CathodeCase_input.insert(0, round(newZ, 3))

        CathodeCase_label = Label(config_frame, text="Cathode Case:", padx=10, pady=15, font=ft_label)
        CathodeCase_input = Entry(config_frame, width=8, bd=5)
        CathodeCase_input.insert(0, min_z_vals[7])
        CathodeCase_plusminusFrame = LabelFrame(config_frame)
        CathodeCase_plus = Button(CathodeCase_plusminusFrame, text="+", font=ft_btn, padx=8, command=CathodeCaseZ_plus)
        CathodeCase_minus = Button(CathodeCase_plusminusFrame, text="-", font=ft_btn, padx=9, command=CathodeCaseZ_minus)

        CathodeCase_label.grid(row=7, column=0)
        CathodeCase_input.grid(row=7, column=1)
        CathodeCase_plusminusFrame.grid(row=7, column=2, padx=5)
        CathodeCase_plus.grid(row=0, column=0)
        CathodeCase_minus.grid(row=1, column=0)

        inputSum = (AnodeCase_input, AnodeSpacer_input, Anode_input, Separator_input,
        Cathode_input, CathodeSpacer_input, Washer_input, CathodeCase_input)

        def save_z_vals():
            user = messagebox.askyesnocancel(title="Config is about to change!", message= "Are you sure you want to save the Z Axis values?")
            if user == True:
                with open(CALIBRATION_DIR) as json_file:
                    readings_calib = json.load(json_file)
                for compNr, component in enumerate(COMPONENTS):
                    finalZ_val = float(inputSum[compNr].get())
                    for nr in range(1,65):
                        readings_calib[component]['grabPo'][str(nr)][2] = finalZ_val
                with open(CONFIG_DIR, 'w') as outfile:
                    json.dump(readings_calib, outfile, indent=4)
                user2 = messagebox.askyesnocancel(title="Config is about to change!", message= "Are you also going to change calibrated data as well?")
                if user2 == True:
                    with open(CALIBRATION_DIR, 'w') as outfile:
                        json.dump(readings_calib, outfile, indent=4)
                    logger.warning("New grabZ values has been gennerated on calibtated data!")
                adjust_grabZ_window.destroy()

        savebtn = Button(adjust_grabZ_window, text="Save", font=font.Font(family="Arial", weight="bold", size=10), padx=15, pady=10, borderwidth=5, command=save_z_vals)
        savebtn.grid(row=1, column=0, padx=5, pady=5)
        exitbtn = Button(adjust_grabZ_window, text="Exit", font=font.Font(family="Arial", weight="bold", size=10), padx=17, pady=10, borderwidth=5, command=adjust_grabZ_window.destroy)
        exitbtn.grid(row=1, column=1, padx=5, pady=5)

    def confirm_setup(self):
        with open(CALIBRATION_DIR, 'r') as json_file:
            readings = json.load(json_file)
        readings['tap_press'] = tap_press.get()
        readings['add_electrolyte'] = electrolyte.get()
        readings['J_VEL'] = joint_vel_slider.get()
        readings['L_VEL'] = cartlin_vel_slider.get()
        readings['GRIP_VEL'] = gripper_vel_slider.get()
        readings['GRIP_F'] = gripper_force_slider.get()
        readings['AX_VEL'] = axis_vel_slider.get()
        readings['auto_calib'] = auto_correction.get()
        readings['save_img'] = save_img.get()
        readings['grab_check'] = grab_check.get()
        readings['show_image'] = show_image.get()
        with open(CONFIG_DIR, 'w') as outfile:
                json.dump(readings, outfile, indent=4)
        self.pos_config_window.destroy()
    
if __name__ == '__main__':
    os.chdir(ROOT)
    pos_gen = ResetParameter()
    pos_gen.start_config_gui()
