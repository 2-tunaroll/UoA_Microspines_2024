## UNIVERSITY OF ADELAIDE - 2024
## Engineering Honours Project
## Microspine Anchoring Mechanisms for Robotic Exploration of Small Celestial Bodies
## Amber Pegoli a1799418, Callie Hopwood a1801146, 
## Fida Matin a1798239, Georgia Dallimore a1794409, Grace Gunner a1750264
## 2024s1-EME.Me-EHa-UG-13948

# Graphical User Interface (GUI) Class File:
# Creates user dashboard creating buttons for engagement and disengagement controls
# Connect to motors and run diagnostics
# Display data visualisation through load values of each motor
# Implement state machine diagram software logic
# Record data to csv files

import tkinter as tk                # import tkinter as a class for 
from tkinter import *               # import all tkinter functions
from types import NoneType          # import NoneType for checking when registers are empty
from tkmacosx import Button         # import MacOS compatible tkinter Button
from PIL import ImageTk,Image       # import Image functionality which works with Tkinter

import dynamixel                    # import dynamixel driver to get specific calibrated values
from port import Port               # import Port class to establish port connection to microcontroller 
from servo import Servo             # import Servo class file to connect to servo motors
from config import mac as config    # import mac config for setting mac specific configurations

import datetime                     # import datetime for recording timestamps
import csv                          # import csv for saving data to csv

# Get calibrated engagement (engage) and disengagement (home) positions from driver file
ENGAGE_ANGLE = dynamixel.DXL_MINIMUM_POSITION_VALUE
HOME_ANGLE = dynamixel.DXL_MAXIMUM_POSITION_VALUE       

# Get calibrated PID position values
PID_GAINS = [
    dynamixel.DXL_POSITION_P_GAIN, 
    dynamixel.DXL_POSITION_I_GAIN, 
    dynamixel.DXL_POSITION_D_GAIN
]

# Current (torque) values from motor calibration
MAX_TORQUE = 221
GRIP_TORQUE = 221

# Motor ID Values
ID_vals = [1, 2, 3, 4, 5, 6]

class Gui(tk.Tk):

    # Set initial values for state and data
    STATE = ''
    currState = ''
    loadData = ''

    # motors, port are assigned to class
    motors = []
    port = None

    # buttons and timestamps are setup
    engage_button = None
    disengage_button = None
    estop_botton = None
    time = None
    load_dict = []
   
    ######################
    ## INITIALISE GUI ##
    ######################

    def __init__(self):
        ## Use tkinter 
        super().__init__() 

        ## State-Machine init
        self.port = Port(config.DEVICENAME)
        self.port.init()
        self.time = datetime.datetime.now()

        ## GUI Window setup
        self.STATE = 'IDLE'
        self.currState = StringVar()
        self.currState.set('IDLE')
        self.loadData = StringVar()
        self.engage_button = Button(
            self, 
            text="Engage", 
            command=lambda :self.engage_function(), 
            font=("Arial 25 bold"), 
            cursor="spider", 
            activeforeground="orange"
        )
        self.engage_button.config(bg='orange')

        self.disengage_button = Button(
            self, 
            text="Disengage", 
            command=lambda :self.disengage_function(), 
            font=("Arial 25 bold"), 
            cursor="spider",
            activeforeground="orange"
        )
        self.disengage_button.config(bg='orange')
        self.disengage_button.config(state=tk.DISABLED)

        self.estop_botton = Button(
            self, 
            text="E-STOP", 
            command=lambda :self.disengage_function(), 
            font=("Arial 25 bold"), 
            cursor="spider",
            activeforeground="red",
            fg="red"
        )

    ##################################
    ## ENGAGEMENT AND DISENGAGEMENT ##
    ##################################
        
    def engage_function(self):
        # State Updates
        self.STATE = 'ATTACH'
        self.currState.set(self.get_state())
        self.after(200, self.update())

        # Motor Updates
        for motor in self.motors:
            motor.enable()
            motor.setCurrLim(GRIP_TORQUE)
            motor.setPos(ENGAGE_ANGLE)

        # Button Updates
        self.disengage_button.config(state=tk.NORMAL)
        self.engage_button.config(state=tk.DISABLED)

    def disengage_function(self):
        # State Updates
        self.STATE = 'DETTACH'
        self.currState.set(self.get_state())
        self.after(200, self.update())

        # Motor Updates
        for motor in self.motors:
            motor.enable()
            motor.setCurrLim(MAX_TORQUE)
            motor.setPos(HOME_ANGLE)

        # Button Updates
        self.engage_button.config(state=tk.NORMAL)
        self.disengage_button.config(state=tk.DISABLED)

    ###########################
    ## DASHBOARD ENVIRONMENT ##
    ###########################

    def generate_env(self):
        # Establish window
        self.title("Microspines: Hooked on Space!")
        self.geometry("1900x1000")
        
        # Import background image
        image1 = Image.open("microspine_img.png")
        img=image1.resize((450, 350))
        background = ImageTk.PhotoImage(img)
        label1 = Label(image=background)
        label1.image = background

        # create all buttons
        self.engage_button.place(x=50,y=250)
        self.disengage_button.place(x=50,y=500)
        self.estop_botton.place(x=10, y=10)

        # set text for getting current Status of motors
        state_label = Label(self, textvariable=self.currState, font='Arial 50 bold')
        state_label.pack(pady=20)

        # set text for getting current Status of load
        load_label = Label(self, textvariable=self.loadData, font='Arial 25')
        load_label.place(x=500, y=250)

        # Position image
        label1.place(x=0,y=700)

    #################
    ## MOTOR SETUP ##
    #################

    def setup(self):
        # Open Port
        self.port.open()

        # set up motors
        for ID in ID_vals:
            # add each class instance of a motor
            self.motors.append(Servo(ID, self.port.handler))

        # set current-based position mode for each motor
        for motor in self.motors:
            motor.reboot()

            # disable torque
            motor.disable()

            # Set operating mode
            motor.setCurrPosMode()
            op_mode = motor.getOpMode()
            while op_mode != dynamixel.DXL_CURRENT_POS_CONTROL:
                print("Waiting for operating mode")
                op_mode = motor.getOpMode()

            assert op_mode == dynamixel.DXL_CURRENT_POS_CONTROL
            self.time = datetime.datetime.now()
            motor.setPos_PID(PID_GAINS[0],PID_GAINS[1], PID_GAINS[2])

    #####################
    ## DATA COLLECTION ##
    #####################

    def update_load(self):
        # Get time as a difference from when class was initialised
        timeStamp = str(datetime.datetime.now() - self.time)
        message = timeStamp + "\n"
        
        # Get all load data from each motor
        for motor in self.motors:
            load = motor.getLoad()
            pos = motor.getPos()
            ID = motor.ID
            message += "[ID: " + str(ID) + "] Load: " + str(round(load, 4)) + "kg" + "    Position: " + str(pos) + "\n"
            # Add load data to dictionary array
            presentLoad_data = {'Time': timeStamp, 'ID':ID, 'Position':pos, 'Load':load}
            self.load_dict.append(presentLoad_data)
        
        # Show data on GUI
        self.loadData.set(message)
        self.after(200, self.update())

    #########################
    ## STATE MACHINE LOGIC ##
    ######################### 

    def state_machine(self):
        # Check if any motors need a reboot
        for motor in self.motors:
            message = motor.getShutdown()
            if message != 53:
                motor.reboot()

        self.update_load()          # Update data for loads
        state = self.get_state()    # Get latest state

        # Run State Machine cases
        # Follows a structure with 4 states 
        match state:
            #--------------#
            ## IDLE STATE ##
            #--------------#
            case 'IDLE':

                # Ensure all motors are turned off
                for motor in self.motors:
                    # Turning motors off
                    for motor in self.motors:
                        motor.disable()
                        motor.setCurrLim(0)

            #----------------#
            ## ATTACH STATE ##
            #----------------#
            case 'ATTACH':
                # If at attach state then 
                # motors are already being driven to engagement position
                # Update state to 'HOLD'
                self.set_state('HOLD')
                for motor in self.motors:
                    motor.setCurrLim(GRIP_TORQUE)

            #--------------#
            ## HOLD STATE ##
            #--------------#
            case 'HOLD':
                # Check if any motors have shutdown
                for motor in self.motors:
                    curr_pos = motor.getPos()
                    curr_load = motor.getLoad()

                    # if no load then wait until reboot has occured
                    # then activate motors to engage
                    if (curr_load == 0.0):
                        if isinstance(curr_pos, NoneType):
                            break
                        
                        if curr_pos:
                            motor.enable()
                            motor.setCurrLim(GRIP_TORQUE)
                            motor.setPos(curr_pos - 200)

            #-----------------#
            ## DETTACH STATE ##
            #-----------------#
            case 'DETTACH':
                # Check if home angle has been reached
                # if not then keep driving motors
                total_status = False
                status = [False, False, False, False, False, False]
                for motor in self.motors:
                    curr_pos = motor.getPos()
                    diff = abs(HOME_ANGLE - curr_pos)
                    if (diff <= dynamixel.DXL_MOVING_STATUS_THRESHOLD):
                        status[motor.ID - 1] = True
                    else:
                        motor.enable()
                        motor.setCurrLim(MAX_TORQUE)
                        motor.setPos(HOME_ANGLE)

                for val in status:
                    if val == False:
                        total_status = False
                        break
                    else:
                        total_status = True

                # If home angle has been reached
                # Disengagement is complete and motors can be set to idle
                if total_status == True:
                    self.set_state('IDLE')
                    for motor in self.motors:
                        motor.setCurrLim(0)
                else:
                    self.disengage_function()

        # Rerun state-machine every 100 milliseconds (10 Hz)
        self.after(100, self.state_machine)

    #####################
    ## STATUS CONTROLS ## 
    #####################

    def get_state(self):
       return self.STATE
    
    def set_state(self, state):
        self.STATE = state
        self.currState.set(self.get_state())
        self.after(200, self.update())

    ###########
    ## CLOSE ##
    ###########
        
    def close(self):
        # disable torque for each motor
        for motor in self.motors:
            # Close servo instance
            motor.disable()

        # Store all data into csv file
        with open('load_data.csv', 'w', newline='') as csvfile:
            fieldnames = ['Time', 'ID', 'Position', 'Load']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.load_dict)

        # Close Port
        self.port.close()