import sys
import serial
import serial.tools.list_ports
import os
import time
import numpy as np
import pandas as pd

from PySide6 import QtGui
from PySide6.QtCore import Qt, QThreadPool, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QFileDialog,
    QAbstractItemView,
    QMessageBox
)

from ControlPanel import Ui_ControlPanel
from Table import PandasModel
from Thread import Worker
from DialogBoxes import InformationDialog, QuestionDialog, WarningDialog, CriticalDialog


class MainWindow(QMainWindow, Ui_ControlPanel):
    
    def __init__(self) -> None:
        super(MainWindow, self).__init__()
        self.setupUi(self) # create UI from ControlPanel


        ### ARDUINO CONTROLS
        self.arduino = None # Arduino board object
        self.is_connected = False # is the program connected to an Arduino?
        self.packet = None # serial data from Arduino
        self.combo_box_USB_connection.addItem("None")
        self.button_create_connection.clicked.connect(self.create_connection)
        self.button_scan_disconnect.clicked.connect(self.scan_disconnect)
        self.data_threadpool = QThreadPool() # CPU thread for reading serial data


        ### TAB PROCESSES
        self.tab_widget_controller.tabBarClicked.connect(self.check_tab_index)


        ### PID CONTROLS
        ## --Manual--
        self.button_send_target_load.clicked.connect(self.send_target_load)

        ## --Automatic--
        # program table setup
        data = np.zeros((10,2), dtype=int) # create data of zeros
        empty_df = pd.DataFrame(data, index=range(1, len(data) + 1), columns=['Load', 'Time']) # create Pandas dataframe from data
        self.program_model = PandasModel(empty_df) # display dataframe as program table
        self.table_program.setModel(self.program_model)
        self.table_program.setColumnWidth(0, 40)
        self.table_program.setColumnWidth(1, 40)
        self.table_program.setSelectionMode(QAbstractItemView.SingleSelection)

        # editing table
        self.tool_button_add_row.clicked.connect(self.add_row)
        self.tool_button_delete_row.clicked.connect(self.delete_row)

        # program commands
        self.button_import_program.clicked.connect(self.import_program)
        self.button_execute_program.clicked.connect(self.execute_program)
        self.button_pause_program.clicked.connect(self.pause_program)
        self.button_skip_entry.clicked.connect(self.skip_entry)
        self.button_terminate_program.clicked.connect(self.terminate_program)
        self.button_clear_program.clicked.connect(self.clear_program)

        # process controls
        self.program_threadpool = QThreadPool() # CPU thread for running the PID program
        self.thread_process = False # is the PID program currently running?
        self.terminate = False # has the terminate button been clicked?
        self.skip = False # has the skip button been clicked?
        self.pause = False # has the pause button been clicked?
        self.time_start = False # should the timer start counting?

        # timer
        self.timer = QTimer() # timer object
        self.timer.setInterval(100) # timer interval
        self.timer.timeout.connect(self.run_count)
        self.timer.start()
        self.counter = 0


        ### SPEED CONTROLLER
        self.dial_velocity.valueChanged.connect(self.velocity_value)
        self.slider_velocity_multiplier.valueChanged.connect(self.slider_multiplier_value)
        self.button_set_multiplier.clicked.connect(self.multiplier_value)
        self.radio_button_default_off.clicked.connect(self.default_off_mode)
        self.radio_button_default_on.clicked.connect(self.default_on_mode)
        self.button_pulse_motor.pressed.connect(self.pulse_activate)
        self.button_pulse_motor.released.connect(self.pulse_deactivate)
        self.dial_velocity_position = 0
        self.off_mode = True # is the speed controller in default off mode?


        ### STARTUP
        self.scan_disconnect() # run method at startup
        self.close_action = False # no closing action queued
        self.intentional_disconnect = False


    ### ARDUINO FUNCTIONS
    # this function creates a connection to an arduino at a COM port if not already connected
    def create_connection(self) -> None:
        if not self.is_connected:
            self.is_connected = True
            self.terminate = False
            self.intentional_disconnect = False
            connection = self.combo_box_USB_connection.currentText().split()[0]
            self.arduino = serial.Serial(connection, 9600) # stores Arduino board connection in self.arduino object
            self.label_connection_status.setText("Status: connected to " + connection)
            self.button_scan_disconnect.setText("Disconnect")
            self.label_slider_multiplier_push_value.setText("Pushed value: 50")
            self.slider_velocity_multiplier.setValue(50)
            self.data_worker = Worker(self.serial_event) # begin CPU thread for reading serial data
            self.data_threadpool.start(self.data_worker)
            self.data_worker.signals.finished.connect(self.serial_alert)

    # this function encodes a string and sends it to the connected arduino via serial coms
    def push_to_arduino(self, value: str) -> None:
        value = value + 'e'
        self.arduino.write(value.encode())

    # this function either scans for new ports or disconnects the Arduino from the software
    def scan_disconnect(self) -> None:
        self.intentional_disconnect = True
        print('trigger')
        if self.is_connected:
            if self.thread_process:
                dlg = QuestionDialog() # open dialog if attempting to disconnect while program is running
                dlg.setWindowTitle("Program in progress")
                dlg.setText("Are you sure you want to disconnect? The\nprogram will terminate automatically.")
                button = dlg.exec()
            if not self.thread_process or button == QMessageBox.Yes: # disconnect Arduino
                self.terminate_program()
                self.arduino = None
                self.is_connected = False
                self.label_connection_status.setText("Status: no connection")
                self.button_scan_disconnect.setText("Scan")
            else:
                self.intentional_disconnect = False
        else: # scan comports
            ports = serial.tools.list_ports.comports()
            self.combo_box_USB_connection.clear()
            for port in ports:
                self.combo_box_USB_connection.addItem(str(port))
            self.combo_box_USB_connection.addItem("None")

    
    ### SERIAL DATA PROCESSING
    # this function runs in the data_threadpool CPU thread and listens to Arduino serial data
    # the function ends when a message is received
    def serial_event(self):
        while self.is_connected:
            try:
                if self.arduino.in_waiting:
                    self.packet = self.arduino.readline()
                    self.packet = int(self.packet.decode('utf').rstrip('\n')) + 0
                    break
            except:
                print('run0')
                self.is_connected = False

    # this function alerts the user when the Arduino passes an error code
    # the function is called upon the serial_event function ends (end of data_threadpool CPU thread)
    def serial_alert(self):
        self.terminate = True
        self.thread_process = False

        if not self.is_connected and not self.close_action and not self.intentional_disconnect:
            print('trigger2')
            self.arduino = None
            self.is_connected = False
            self.label_connection_status.setText("Status: no connection")
            self.button_scan_disconnect.setText("Scan")
            dlg = CriticalDialog() # open dialog if attempting to disconnect while program is running
            dlg.setWindowTitle("Attention")
            dlg.setText("Arduino disconnected. All processes\nterminated. Please check your connection.")
            dlg.exec()
        else:
            self.velocity_push(0)
            if self.packet == 0:
                dlg = InformationDialog()
                dlg.setWindowTitle("Attention")
                dlg.setText("Material failure detected.\nAll processes terminated.")
                dlg.exec()
            elif self.packet == 1:
                dlg = CriticalDialog()
                dlg.setWindowTitle("Warning")
                dlg.setText("Positive feedback detected. All processes terminated. This\nsystem is only compatible with cells calibrated in tension.\nPlease check your load cell calibration.")
                dlg.exec()
            elif self.packet == 2:
                dlg = CriticalDialog()
                dlg.setWindowTitle("Warning")
                dlg.setText("No sensor data detected. All\nprocesses terminated. Please\ncheck if SensorVue is running.")
                dlg.exec()
            elif self.packet == 3:
                self.radio_button_default_off.setChecked(True)
                self.default_off_mode()

            self.data_worker = Worker(self.serial_event) # restarts listening via serial_event thread
            self.data_threadpool.start(self.data_worker)
            self.data_worker.signals.finished.connect(self.serial_alert)


    ### TAB PROCESS FUNCTIONS
    # this function ensures that switching controllers occurs safely
    # the function is called whenever attempts to switch controllers
    def check_tab_index(self, index):
        if index == 0:
            if self.thread_process: # checks for program in progress when leaving PID Control and prompts user to resolve conflict
                dlg = QuestionDialog()
                dlg.setWindowTitle("Program in progress")
                dlg.setText("Are you sure you want to switch to Speed Control?\nThe PID program will terminate automatically.")
                button = dlg.exec()
                if button == QMessageBox.Cancel:
                    QTimer.singleShot(0, lambda: self.change_tab()) # cancel tab change
                else:
                    self.terminate_program()
            if not self.thread_process or button == QMessageBox.Yes:
                self.radio_button_default_off.setChecked(True)
                self.default_off_mode()
        elif index == 1: # switches to default off when leaving Speed Controller
            self.radio_button_default_off.setChecked(True)
            self.default_off_mode()
            self.label_current_target_load.setText('-- lbs')
    
    # this function changes tabs automatically
    def change_tab(self):
        if self.tab_widget_controller.currentIndex() == 0:
            self.tab_widget_controller.setCurrentIndex(1)
        else:
            self.tab_widget_controller.setCurrentIndex(0)


    ### PID CONTROL FUNCTIONS
    # this function encodes the PID command into the proper format and pushes the command to the Arduino
    def pid_push(self, value: int) -> None:
        if value >= 0:
            self.push_to_arduino(str(10000 + value))
        else:
            self.push_to_arduino(str(20000 - value))

    ## --Manual--
    # this function takes the value in the lineedit and sends it to the arduino
    def send_target_load(self) -> None:
        if self.is_connected:
            if not self.thread_process:
                target_load = self.lineedit_manual_load.text()
                if abs(int(target_load)) <= 1600:
                    self.pid_push(int(target_load))
                    self.label_current_target_load.setText(target_load + " lbs")
                else:
                    dlg = CriticalDialog()
                    dlg.setWindowTitle("Load limit reached")
                    dlg.setText("Cannot exceed 1600 lbs.")
                    dlg.exec()
            else:
                dlg = WarningDialog()
                dlg.setWindowTitle("Program in progress")
                dlg.setText("Cannot send a target load\nwhile program is running.")
                dlg.exec()
        else:
            dlg = WarningDialog()
            dlg.setWindowTitle("No connection")
            dlg.setText("Please check your Arduino connection.")
            dlg.exec()

    ## --Automatic--
    # this function takes in data to create a PandasModel object and sets it as the table program model
    def create_frame(self, data, data_frame=False):
        if data_frame is False:
            data = pd.DataFrame(data, index=range(1, len(data) + 1), columns=['Load', 'Time'])
        self.program_model = PandasModel(data)
        self.table_program.setModel(self.program_model)

    # this function adds a row to either above the highlighted box or to the bottom if no selection
    def add_row(self):
        if not self.thread_process:
            indices = self.table_program.selectionModel().selectedIndexes()
            df = self.program_model.getData()
            
            if indices:
                row = indices[0].row()
                data = np.insert(df.values, row, [0, 0], axis=0)
            else:
                data = np.append(df.values, [[0, 0]], axis=0)

            self.create_frame(data)
        else:
            dlg = WarningDialog()
            dlg.setWindowTitle("Program in progress")
            dlg.setText("Cannot edit a program while\nit is currently running.")
            dlg.exec()

    # this function deletes either the highlighted row or the last row if no selection
    def delete_row(self):
        if not self.thread_process:
            indices = self.table_program.selectionModel().selectedIndexes()
            df = self.program_model.getData()

            if indices:
                row = indices[0].row()
                data = np.delete(df.values, row, axis=0)
            else:
                data = np.delete(df.values, -1, axis=0)

            self.create_frame(data)
        else:
            dlg = WarningDialog()
            dlg.setWindowTitle("Program in progress")
            dlg.setText("Cannot edit a program while\nit is currently running.")
            dlg.exec()

    # this function imports a program from excel and populates the PandasModel table
    def import_program(self) -> None:
        if not self.thread_process:
            filename = QFileDialog.getOpenFileName(
                parent=self,
                caption="Select a file",
                dir=os.getcwd(),
                filter="Data File (*.xlsx *.csv *.dat);; Excel File (*.xlsx *.xls)" # get only excel or csv files
            )
            df = pd.read_excel(filename[0])
            df.index += 1
            self.create_frame(df, data_frame=True)
        else:
            dlg = WarningDialog()
            dlg.setWindowTitle("Program in progress")
            dlg.setText("Please terminate the program\nbefore importing another.")
            dlg.exec()

    # this creates and begins a CPU thread for the execution of a loaded program
    def execute_program(self):
        if not self.thread_process and self.is_connected:
            self.thread_process = True
            self.program_worker = Worker(self.run_program_thread)
            self.program_threadpool.start(self.program_worker)
        elif self.thread_process and self.is_connected:
            dlg = WarningDialog()
            dlg.setWindowTitle("Program in progress")
            dlg.setText("Cannot execute a program while\nanother is currently running.")
            dlg.exec()
        else:
            dlg = WarningDialog()
            dlg.setWindowTitle("No connection")
            dlg.setText("Please check your Arduino connection.")
            dlg.exec()


    # this function runs the loaded program in the table
    def run_program_thread(self):
        # get data from the table
        df = self.program_model.getData()
        control_values = df.values[:, 0]
        time_step = df.values[:, 1]

        # iterate over each row of table
        for i, command in enumerate(control_values):
            if time_step[i] == 0: # skip when time is zero
                continue
            
            # highlight current row
            self.label_current_target_load.setText(str(command) + ' lbs')
            self.program_model.changeColor(i, 0, QtGui.QBrush(Qt.yellow))
            self.program_model.changeColor(i, 1, QtGui.QBrush(Qt.yellow))
            self.pid_push(command)

            # check user commands every hundreth of a second
            for _ in range(time_step[i] * 100):
                if self.pause: # pause program if pause is pressed
                    self.time_start = False # freeze the time
                    self.label_program_status.setText('Paused at ' + str(int(self.counter + 1.01)) + 's')
                while self.pause: # freeze program if paused
                    if self.terminate or self.skip: # break pause if terminate or skip is pressed
                        break
                    pass
                if self.terminate or self.skip: # move onto next row if terminate or skip is pressed
                    break
                self.time_start = True # start the time
                time.sleep(0.01)
            
            self.skip = False # toggle skip
            self.counter = 0
            self.program_model.changeColor(i, 0, QtGui.QBrush(Qt.white)) # recolor past row as white
            self.program_model.changeColor(i, 1, QtGui.QBrush(Qt.white))
            if self.terminate: # break program loop if terminate is pressed
                break

        if self.terminate:
            self.label_program_status.setText('Terminated')
            self.terminate = False # reset terminate status
        else:
            self.label_program_status.setText('Complete!')

        self.thread_process = False # no program thread running
        self.time_start = False # stop timer count
    
    # this function keep tracks of the running time for each cell in the program
    # the function is called by the QTimer() object (declared in the instatiation) every 0.1s
    def run_count(self):
        if self.time_start:
            self.counter += 0.1
            self.label_program_status.setText('Running for ' + str(int(self.counter + 1.01)) + 's')

    # this function pauses or unpauses the running program
    def pause_program(self):
        if self.pause:
            self.pause = False
            self.button_pause_program.setText("Pause")
        else:
            self.pause = True
            self.button_pause_program.setText("Unpause")

    # this function skips an entry in the program
    def skip_entry(self):
        if self.thread_process:
            self.skip = True

    # this function terminates the running program
    def terminate_program(self):
        if self.thread_process:
            self.terminate = True
            self.velocity_push(0)

    # this function clears the program table of its entries
    def clear_program(self):
        if not self.thread_process:
            df = self.program_model.getData() * 0 # multiply existing data by 0
            self.create_frame(df, data_frame=True) # set model as new data
        else:
            dlg = WarningDialog()
            dlg.setWindowTitle("Program in progress")
            dlg.setText("Cannot clear a program while\nit is currently running.")
            dlg.exec()
    

    ### SPEED CONTROLLER FUNCTIONS
    # this function encodes the speed command into the proper format and pushes the command to the Arduino
    def velocity_push(self, value: int) -> None:
        if value >= 0:
            self.push_to_arduino(str(30000 + value))
        else:
            self.push_to_arduino(str(40000 - value))
    
    # this function encodes the multiplier command into the proper format and pushes the command to the Arduino
    def multiplier_push(self, value: int) -> None:
        self.push_to_arduino(str(50000 + value))

    # this function interprets the dial position as speed
    def velocity_value(self, position):
        self.dial_velocity_position = position
        self.label_dial_value.setText("Dial value: " + str(self.dial_velocity_position))
        if self.off_mode:
            self.label_pulse_value.setText(str(self.dial_velocity_position))
        else:
            self.velocity_push(self.dial_velocity_position)
            self.label_current_velocity.setText(str(self.dial_velocity_position))
    
    # this function interprets the slider as the velocity multiplier
    def slider_multiplier_value(self, position):
        self.slider_multiplier_position = position
        self.label_slider_multiplier_value.setText("Slider value: " + str(position))

    # this function pushes the velocity multiplier to the Arduino
    def multiplier_value(self):
        self.multiplier_push(self.slider_multiplier_position)
        self.label_slider_multiplier_push_value.setText("Pushed value: " + str(self.slider_multiplier_position))
    
    # this function sets the speed controller to default off mode
    def default_off_mode(self):
        self.off_mode = True
        self.velocity_push(0)
        self.button_pulse_motor.setText('Pulse on')
        self.label_current_velocity.setText('0')
        self.label_pulse_value.setText(str(self.dial_velocity_position))

    # this function sets the speed controller to default on mode
    def default_on_mode(self):
        self.off_mode = False
        self.velocity_push(self.dial_velocity_position)
        self.button_pulse_motor.setText('Pulse off')
        self.label_current_velocity.setText(str(self.dial_velocity_position))
        self.label_pulse_value.setText('0')
    
    # this function sends the proper commands when holding pulse
    def pulse_activate(self):
        if self.off_mode:
            self.velocity_push(self.dial_velocity_position)
            self.label_current_velocity.setText(str(self.dial_velocity_position))
        else:
            self.velocity_push(0)
            self.label_current_velocity.setText('0')
    
    # this function sends the proper commands when releasing pulse
    def pulse_deactivate(self):
        if self.off_mode:
            self.velocity_push(0)
            self.label_current_velocity.setText('0')
        else:
            self.velocity_push(self.dial_velocity_position)
            self.label_current_velocity.setText(str(self.dial_velocity_position))
    

    ### CLOSE FUNCTIONS
    # this function runs prior to exiting and ensures safe exit of the program
    def closeEvent(self, event):
        dlg = QuestionDialog()
        dlg.setWindowTitle("Attention")
        dlg.setText("Are you sure you want to exit? All processes\nwill terminate automatically.")
        button = dlg.exec()
        if button == QMessageBox.Yes:
            self.velocity_push(0)
            self.close_action = True
            self.terminate = True
            self.arduino = None
            self.is_connected = False
            event.accept()
        else:
            event.ignore()


app = QApplication(sys.argv)

window = MainWindow()
window.show()
app.exec()