import sys
import os

from PyQt5 import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtCore import QDir
from PyQt5.uic import loadUiType
from PyQt5.uic import loadUi
import main 
from psychopy import core

import pandas as pd
import numpy as np
import rospy
from math import cos, sin, atan, asin, pi

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import StartExperiment_amcl_v5_GUI 
import UserInput_v3 
import StartExperiment_JOY_v6 
import utils
import json
import cv2
import time
import datetime

Ui_MainWindow, QMainWindow = loadUiType("gui.ui")


class Main(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(Main, self).__init__()


        self.setupUi(self)
        self.start_button.clicked.connect(self.start)
        #self.maze_startButton.clicked.connect(self.maze_start)
        self.load_button.clicked.connect(self.load)
        
        RewardLocationCSV = '/home/sinapse/Desktop/RewardData/rewardlocations.csv'
        Rewardlocation = pd.read_csv(RewardLocationCSV, index_col=0)
        RewardLocation = len(Rewardlocation)
        #print 'Number of saved reward locations loaded : ', RewardLocation

        self.subject.setText("r")
        self.session.setText("1")
        self.juicer_height.setText("100")
        self.juicer_height.textChanged.connect(self.calculate_flowrate)
        self.calculate_flowrate()
        self.fixation_criterion.setText("1.0")
        self.fixation_grace.setText("0.3")
        self.eccentricity.setText("5.0")
        self.autocycle.setChecked(False)
        self.intertrial_interval.setText("1.0")

        self.trial_timeout.setText("100.0")
        self.fixation_size.setText("1.0")
        self.fixation_window.setText("1.0")
        self.fixation_buffer.setText("0.0")
        self.eyespot.setChecked(False)
        self.use_dummy.setChecked(True)
        self.calibrate.setChecked(True)
        self.play_sound.setChecked(True)
        self.block_report.setChecked(False)

        # monitor related stuff
        self.screen_distance.setText("57")
        self.screen_size.setText("22")
        self.screen_height.setText("1050")
        self.screen_width.setText("1680")

        self.mask_params = {}
        self.fixation_mask.addItem("square")
        self.fixation_mask.addItem("gauss")
        self.fixation_mask.addItem("circle")
        self.fixation_mask.addItem("bullseye")
        self.fixation_mask.activated.connect(self.setMaskParams)
        self._combination_chooser = None

        # calibration stuff
        self.calibration_reward_duration.setText("0.5")
        self.calibration_target_size.setText("1.0")
        self.calibration_type.addItem("3 points")
        self.calibration_type.addItem("5 points")
        self.calibration_type.addItem("9 points")
        self.calibration_type.addItem("13 points")
        self.calibration_target_color.addItem("white")
        self.calibration_target_color.addItem("yellow")
        self.calibration_target_color.addItem("blue")
        self.calibration_stimulus.addItem("Image...")
        self.calibration_stimulus.addItem("Gabor patch")
        self.calibration_stimulus.addItem("Circle")
        self.calibration_stimulus.activated.connect(self.set_calibration_image)
        self.manual_calibration.setChecked(False)
        
        #Maze stuff
        #self.maze_SelectDirectory.addItem("Directory Selection")
        #self.maze_SelectDirectory.activated.connect(self.set_maze_Directory)
        #self.maze_directoryDisplay.setText(QDir.home().dirName())
        #self.maze_directoryDisplay.setReadOnly(True)

        self.maze_Map.addItem("Select Directory")
        self.maze_Map.activated.connect(self.set_maze_Map)

        self.maze_rewardLocation.setText(str(RewardLocation))
        self.maze_rewardLocation.setReadOnly(True)
        self.maze_numberOfTrials.setText("3")
        self.maze_angleTolerance.setText("90")
        self.maze_positionTolerance.setText("20") #in CM
        self.maze_destinationDuration.setText("2")
        self.maze_trialDuration.setText("80") #in Seconds
        self.maze_timeToStart.setText("5") #in Seconds
        self.maze_ITI_Min.setText("1")    #in Seconds
        self.maze_ITI_Max.setText("3")   #in Seconds

        # platform stuff
        self.platform_clear_dist.setText("1.6") #need convert to metres default 1.2 [1.6 = 80cm slow]
        self.platform_stop_dist.setText("1.0")  #need convert to metres default 0.7 [1.0 = 60cm stop]
        self.platform_slowDownSpeed.setText("0.1") 
        self.platform_normalSpeed.setText("0.2")

        # target stuff
        self.target_onset_ref.addItem("Fixation")
        self.target_onset_ref.addItem("Trial start")
        self.num_targets.setText("0")
        self.target_size.setText("1.0")
        self.target_onset.setText("0.0")
        self.target_duration.setText("0.0")
        self.inter_target_interval.setText("0.0")
        self.preview_targets.clicked.connect(self.show_target_preview)
        self.set_combinations.clicked.connect(self.choose_combinations)

        # response stuff
        self.response_cue_onset_min.setText("0.0")
        self.response_cue_onset_max.setText("0.0")
        self.response_cue_ref.addItem("Target onset")
        self.response_cue_ref.addItem("Target offset")
        self.response_cue_ref.addItem("Fixation")
        self.max_rtime.setText("0.0")
        self.max_saccade_time.setText("0.0")
        self.target_fix_time.setText("0.0")
        self.target_window_size.setText(self.target_size.text())
        self.repeat_failed_targets.setText("0")

        # saveas
        self.saveas_button.clicked.connect(self.saveas)

        # reward stuff
        self.reward_duration.setText("0.5")
        self.reward_condition.addItem("Maintain fixation")
        self.reward_condition.addItem("Saccade to target")
        self.manual_reward_duration.setText("0.5")
        self.manual_reward_duration_2.setText("0.5")
        self.reward_color.addItem("None")
        self.reward_color.addItem("Green")
        self.failure_color.addItem("None")
        self.failure_color.addItem("Red")
        self.failure_color.addItem("Black")
        self.scale_reward.setChecked(False)
        self.scale_reward.stateChanged.connect(self.scale_reward_changed)
        self.base_reward_duration.setText("0.5")
        self.base_reward_duration.setEnabled(False)

        # results
        self.results = {}
        self.combos = []

    def scale_reward_changed(self, int):
        if self.scale_reward.isChecked():
            self.base_reward_duration.setEnabled(True)
        else:
            self.base_reward_duration.setEnabled(False)

    def calculate_flowrate(self):
        juicer_height = self.juicer_height.text()
        if juicer_height:
            y = float(juicer_height)
            self.droprate.setText("%.3f" % (utils.calc_flowrate(y), ))


    def setMaskParams(self):
        if self.fixation_mask.currentText() == "gauss":
            text, ok = QtGui.QInputDialog.getText(self, "Mask parameter", "Standard deviation: ")
            self.mask_params = {"sd": float(text)}
        else:
            self.mask_params = {}

    def choose_combinations(self):
        if self._combination_chooser is None:
            num_targets = int(self.num_targets.text())
            self._combination_chooser = QtGui.QDialog(self)
            self._combination_chooser.resize(200, 200)
            main_layout = QtGui.QVBoxLayout()
            _textfield = QtGui.QListWidget()
            _line_edit = QtGui.QLineEdit()
            # populate the box
            for cc in self.combos:
                _textfield.addItem(cc)

            def _copy_text():
                _ss = str(_line_edit.text())
                if len(_ss) > num_targets:
                    # this is not an allowed combination
                    _line_edit.setText("")
                else:
                    _items = _textfield.findItems(_line_edit.text(), QtCore.Qt.MatchExactly)
                    if len(_items) == 0:
                        _textfield.addItem(_line_edit.text())
                    else:
                        # update the item at _idx
                        _items[0].setText(_line_edit.text())
                    if _ss not in self.combos:
                        self.combos.append(_ss)
                    _line_edit.setText("")


            def _retrieve_text(item):
                _line_edit.setText(item.text())

            def _delete_item():
                listItems = _textfield.selectedItems()
                if not listItems: return
                for item in listItems:
                    _textfield.takeItem(_textfield.row(item))
                    self.combos.remove(str(item.text()))

            _textfield.connect(QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Delete), _textfield),
                             QtCore.SIGNAL('activated()'), _delete_item)
            _textfield.connect(QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Backspace), _textfield),
                               QtCore.SIGNAL('activated()'), _delete_item)
            _textfield.itemActivated.connect(_retrieve_text)

            _line_edit.returnPressed.connect(_copy_text)
            main_layout.addWidget(_line_edit)
            main_layout.addWidget(_textfield)
            self._combination_chooser.setLayout(main_layout)

        self._combination_chooser.exec_()

    def set_calibration_image(self):
        if self.calibration_stimulus.currentText() == "Image...":
            filters = "Image and movie files (*.jpg *.tiff *.png *.mp4)"
            
            filename = QtGui.QFileDialog.getOpenFileName(self, "Set calibration image", os.getcwd(),
                                                         filters)
            if filename:
                idx = self.calibration_stimulus.findText(filename, QtCore.Qt.MatchFixedString)
                if idx < 0:  # not found, so add it
                    self.calibration_stimulus.addItem(filename)
                    idx = self.calibration_stimulus.findText(filename, QtCore.Qt.MatchFixedString)
                self.calibration_stimulus.setCurrentIndex(idx)

    def set_maze_Directory(self):
        self.maze_SelectDirectory = QtWidgets.QFileDialog.getExistingDirectory(None, 'Select Directory', QDir.homePath(), QtWidgets.QFileDialog.ShowDirsOnly | QtWidgets.QFileDialog.DontResolveSymlinks)

    def set_maze_Map(self):
        filters = "Image only (*.jpg *.tiff *.png *.pgm *.jpeg)"
        filename, _filter = QtWidgets.QFileDialog.getOpenFileName(self, "set maze map" , os.getcwd(), filters)
        #filename = QtGui.QFileDialog.getOpenFileName(self, 'set maze map', os.getcwd(), 'All Files(*.*)', filters)



        if filename:
            idx = self.maze_Map.findText(filename, QtCore.Qt.MatchFixedString)
            if idx < 0: # if not found, add the map
                self.maze_Map.addItem(filename)
                idx = self.maze_Map.findText(filename, QtCore.Qt.MatchFixedString)
            self.maze_Map.setCurrentIndex(idx)



        #filename = QtWidgets.QFileDialog.getOpenFileName(self, "set maze map" , os.getcwd(), filters)
        #filename = QtGui.QFileDialog.getOpenFileName(self, "set_maze_map", os.getcwd(), filters)
        """
        if filename:
            idx = self.maze_Map.findText(filename, QtCore.Qt.MatchFixedString)
            if idx < 0: # if not found, add the map
                self.maze_Map.addItem(filename)
                idx = self.maze_Map.findText(filename, QtCore.Qt.MatchFixedString)
            self.maze_Map.setCurrentIndex(idx)"""
        
        """idx = self.maze_Map.findText(filename, QtCore.Qt.MatchFixedString)
        files = filter(os.path.isfile, os.listdir(os.curdir))
        images = len(os.listdir(os.getcwd()))
        print (filename)"""




    def load(self):
        filters = "Settings files (*_settings.txt)"
        # TODO: Why is this not resolving symoblic links?
        #filename = QtGui.QFileDialog.getOpenFileName(self, "Load settings", os.getcwd(),filters)
        filename, _filer = QtWidgets.QFileDialog.getOpenFileName(self, "Load settings", os.getcwd(),filters)

        if filename:
            exp_info = json.load(open(str(filename), "r"))
            self.subject.setText(exp_info.get("subject", "r"))
            self.juicer_height.setText(str(exp_info.get("juicer_height", 100.0)))
            self.calculate_flowrate()
            self.exp_plot.setChecked(exp_info.get("exp_plot", False))
            self.use_dummy.setChecked(exp_info.get("use_dummy", True))
            self.eyespot.setChecked(exp_info.get("draw_eyespot", True))
            self.screen_width.setText(str(exp_info.get("screen_width", 1680)))
            self.screen_height.setText(str(exp_info.get("screen_height", 1080)))
            self.screen_size.setText(str(exp_info.get("screen_size", 25)))
            self.screen_distance.setText(str(exp_info.get("screen_distance", 57)))
            self.fixation_criterion.setText(str(exp_info.get("fixation_criterion", 1.0)))
            self.fixation_grace.setText(str(exp_info.get("fixation_grace", 0.3)))
            self.fixation_buffer.setText(str(exp_info.get("fixation_buffer", "0.0")))
            self.eccentricity.setText(str(exp_info.get("eccentricity", 5.0)))
            self.autocycle.setChecked(exp_info.get("autocycle", False))
            self.intertrial_interval.setText(str(exp_info.get("intertrial_duration", 1.0)))
            self.reward_duration.setText(str(exp_info.get("reward_duration", 0.5)))
            self.scale_reward.setChecked(exp_info.get("scale_reward", False))
            self.base_reward_duration.setEnabled(exp_info.get("scale_reward", False))
            self.base_reward_duration.setText(str(exp_info.get("base_reward_duration", 0.5)))
            make_response = exp_info.get("make_response", False)
            if make_response:
                idx = self.reward_condition.findText("Saccade to target", QtCore.Qt.MatchFixedString)
            else:
                idx = self.reward_condition.findText("Maintain fixation", QtCore.Qt.MatchFixedString)
            self.reward_condition.setCurrentIndex(idx)

            self.manual_reward_duration.setText(str(exp_info.get("manual_reward_duration", 0.5)))
            self.manual_reward_duration_2.setText(str(exp_info.get("manual_reward_duration_2", 0.5)))
            self.serial_path.setText(str(exp_info.get("serial_port", "")))
            self.trial_timeout.setText(str(exp_info.get("trial_timeout", 100.0)))
            self.fixation_size.setText(str(exp_info.get("fixation_size", 1.0)))
            self.fixation_window.setText(str(exp_info.get("fixation_window", 1.0)))

            self.play_sound.setChecked(bool(exp_info.get("pay_sound", True)))
            self.block_report.setChecked(bool(exp_info.get("block_report", False)))

            # combobox
            fixmask = exp_info.get("fixation_mask", "circle")
            idx = self.fixation_mask.findText(fixmask, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.fixation_mask.setCurrentIndex(idx)
            rcolor = str(exp_info.get("reward_color", None))
            idx = self.reward_color.findText(rcolor, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.reward_color.setCurrentIndex(idx)
            fcolor = str(exp_info.get("failure_color", None))
            idx = self.failure_color.findText(fcolor, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.failure_color.setCurrentIndex(idx)

            #  calibration
            self.calibration_reward_duration.setText(str(exp_info.get("calibration_reward_duration", 0.5)))
            self.calibration_target_size.setText(str(exp_info.get("calibration_target_size", 1.0)))
            calib_type = exp_info.get("calibration_type", "9 points")
            idx = self.calibration_type.findText(calib_type, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.calibration_type.setCurrentIndex(idx)
            calib_color = exp_info.get("calibration_target_color", "white")
            idx = self.calibration_target_color.findText(calib_color, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.calibration_target_color.setCurrentIndex(idx)

            calibration_stim = exp_info.get("calibration_stimulus", "Circle")
            idx = self.calibration_stimulus.findText(calibration_stim)
            if idx >= 0:
                self.calibration_stimulus.setCurrentIndex(idx)
            else:
                self.calibration_stimulus.addItem(calibration_stim)
            self.manual_calibration.setChecked(exp_info.get("manual_calibration", False))

            # target
            _target_onset_ref = exp_info.get("target_onset_ref", "Fixation")
            idx = self.target_onset_ref.findText(_target_onset_ref, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.target_onset_ref.setCurrentIndex(idx)
            self.num_targets.setText(str(exp_info.get("num_targets", 0)))
            self.target_onset.setText(str(exp_info.get("target_onset", 0.0)))
            self.target_size.setText(str(exp_info.get("target_size", 2.0)))
            self.target_duration.setText(str(exp_info.get("target_duration", 1.0)))
            self.inter_target_interval.setText(str(exp_info.get("inter_target_interval", 0.0)))
            target_locations = exp_info.get("target_locations", [])
            self.target_locations.setPlainText("\n".join([str(t).strip("[]") for t in target_locations]))
            self.target_anchors.setChecked(bool(exp_info.get("show_anchors", False)))
            self.repeat_locations.setChecked(bool(exp_info.get("repeat_locations", False)))

            if "response_cue_onset" in exp_info.keys():
                self.response_cue_onset_min.setText(str(exp_info.get("response_cue_onset", 0.0)))
                self.response_cue_onset_max.setText(str(exp_info.get("response_cue_onset", 0.0)))
            else:
                self.response_cue_onset_min.setText(str(exp_info.get("response_cue_onset_min", 0.0)))
                self.response_cue_onset_max.setText(str(exp_info.get("response_cue_onset_max", 0.0)))
            response_cue_ref = exp_info.get("response_cue_ref", "Target offset")
            idx = self.response_cue_ref.findText(response_cue_ref)
            if idx >= 0:
                self.response_cue_ref.setCurrentIndex(idx)

            self.max_rtime.setText(str(exp_info.get("max_rtime", 0.0)))
            self.max_saccade_time.setText(str(exp_info.get("max_saccade_time", 0.0)))
            self.target_fix_time.setText(str(exp_info.get("target_fix_time", 0.0)))
            self.target_window_size.setText(str(exp_info.get("target_window_size", self.target_size.text())))
            self.repeat_failed_targets.setText(str(exp_info.get("repeat_failed_targets", 0)))
            self.combos = exp_info.get("allowed_combos", [])

            #Maze
            self.maze_rewardLocation.setText(str(exp_info.get("maze_rewardlocation", 3.0)))
            self.maze_numberOfTrials.setText(str(exp_info.get("maze_numberOfTrials", 3.0)))
            self.maze_angleTolerance.setText(str(exp_info.get("maze_angleTolerance", 90)))
            self.maze_positionTolerance.setText(str(exp_info.get("maze_positionTolerance", 15)))
            self.maze_destinationDuration.setText(str(exp_info.get("maze_destinationDuration", 2.0)))
            self.maze_trialDuration.setText(str(exp_info.get("maze_trialDuration", 60.0)))
            self.maze_timeToStart.setText(str(exp_info.get("maze_timeToStart", 5.0)))
            self.maze_ITI_Min.setText(str(exp_info.get("maze_ITI_Min", 1.0)))
            self.maze_ITI_Max.setText(str(exp_info.get("maze_ITI_Max", 5.0)))

            #Platform
            self.platform_clear_dist.setText(str(exp_info.get("platform_clear_dist", 1.2)))
            self.platform_stop_dist.setText(str(exp_info.get("platform_stop_dist", 0.7)))
            self.platform_normalSpeed.setText(str(exp_info.get("platform_normalSpeed", 0.2)))
            self.platform_slowDownSpeed.setText(str(exp_info.get("platform_slowDownSpeed", 0.1)))


    def saveas(self):
        #filename = QtGui.QFileDialog.getSaveFileName(self, "Save settings", os.getcwd())
        filename = QtWidgets.QFileDialog.getSaveFileName(self, "Save settings", os.getcwd())
        
        if filename:
            exp_info = self.get_settings()
            json.dump(exp_info, open('/home/sinapse/Desktop/MonkeyGUI-master/GUIsaved_settings.txt',"w"))
            #json.dump(exp_info, open(filename, "w"))

    def show_target_preview(self):
        exp_info = self.get_settings()
        main.target_preview(exp_info)

    def get_settings(self):
        subject = str(self.subject.text())
        session = int(self.session.text())
        droprate = float(self.droprate.text())
        screen_width = float(self.screen_width.text())
        screen_height = float(self.screen_height.text())
        screen_distance = float(self.screen_distance.text())
        screen_size = float(self.screen_size.text())
        serialpath = str(self.serial_path.text())
        eccentricity = float(self.eccentricity.text())
        autocycle = self.autocycle.isChecked()
        fixcrit = float(self.fixation_criterion.text())
        fixation_grace = float(self.fixation_grace.text())
        fixation_buffer = float(self.fixation_buffer.text())
        iti = float(self.intertrial_interval.text())
        reward_duration = float(self.reward_duration.text())
        scale_reward = self.scale_reward.isChecked()
        base_reward_duration = float(self.base_reward_duration.text())
        manual_reward_duration = float(self.manual_reward_duration.text())
        manual_reward_duration_2 = float(self.manual_reward_duration_2.text())
        trial_timeout = float(self.trial_timeout.text())
        fixsize = float(self.fixation_size.text())
        fix_window = float(self.fixation_window.text())
        use_dummy = self.use_dummy.isChecked()
        play_sound = self.play_sound.isChecked()
        block_report = self.block_report.isChecked()
        use_eyespot = self.eyespot.isChecked()
        exp_plot = self.exp_plot.isChecked()
        fixation_mask = str(self.fixation_mask.currentText())
        reward_color = str(self.reward_color.currentText())
        if reward_color == 'None':
            reward_color = None
        if str(self.reward_condition.currentText()) == "Maintain fixation":
            make_response = False
        else:
            make_response = True
        failure_color = str(self.failure_color.currentText())
        if failure_color == 'None':
            failure_color = None
        calibrate = self.calibrate.isChecked()
        # calibration stuff
        calibration_reward_duration = float(self.calibration_reward_duration.text())
        calibration_target_size = float(self.calibration_target_size.text())
        calibration_target_color = str(self.calibration_target_color.currentText())
        calibration_type = str(self.calibration_type.currentText())
        calibration_stimulus = str(self.calibration_stimulus.currentText())
        manual_calibration = self.manual_calibration.isChecked()

        # Maze stuff
        maze_Map = str(self.maze_Map.currentText())
        maze_rewardLocation = float(self.maze_rewardLocation.text())
        maze_numberOfTrials = float(self.maze_numberOfTrials.text())
        maze_angleTolerance = float(self.maze_angleTolerance.text())
        maze_positionTolerance = float(self.maze_positionTolerance.text())
        maze_destinationDuration = float(self.maze_destinationDuration.text())
        maze_trialDuration = float(self.maze_trialDuration.text())
        maze_timeToStart = float(self.maze_timeToStart.text())
        maze_ITI_Min = float(self.maze_ITI_Min.text())
        maze_ITI_Max = float(self.maze_ITI_Max.text())

        # Platform stuff
        platform_clear_dist = float(self.platform_clear_dist.text())
        platform_stop_dist = float(self.platform_stop_dist.text())
        platform_normalSpeed = float(self.platform_normalSpeed.text())
        platform_slowDownSpeed = float(self.platform_slowDownSpeed.text())

        # target
        _target_onset_ref = str(self.target_onset_ref.currentText())
        num_targets = int(self.num_targets.text())
        _targets = str(self.target_locations.toPlainText()).split('\n')
        target_locations = []
        for tt in _targets:
            vv = tt.split(",")
            if len(vv) >= 2:
                try:
                    target_locations.append((float(vv[0]), float(vv[1])))
                except ValueError:
                    pass
        # sort target locations by row, then column
        target_locations.sort(key=lambda x: x[::-1])
        target_size = float(self.target_size.text())
        target_duration = float(self.target_duration.text())
        target_onset = float(self.target_onset.text())
        inter_target_interval = float(self.inter_target_interval.text())
        repeat_locations = self.repeat_locations.isChecked()

        response_cue_onset_min = float(self.response_cue_onset_min.text())
        response_cue_onset_max = float(self.response_cue_onset_max.text())
        response_cue_ref = str(self.response_cue_ref.currentText())
        max_rtime = float(self.max_rtime.text())
        max_saccade_time = float(self.max_saccade_time.text())
        target_fix_time = float(self.target_fix_time.text())
        target_window_size = float(self.target_window_size.text())
        target_anchors = self.target_anchors.isChecked()
        repeat_failed_targets = int(self.repeat_failed_targets.text())
        #combo selection
        allowed_combos = self.combos
        # increment the session counter for the next session
        self.session.setText(str(session+1))
        exp_info = {"subject": subject,
                    "session": session,
                    "droprate": droprate,
                    "screen_width": screen_width,
                    "screen_height": screen_height,
                    "screen_size": screen_size,
                    "screen_distance": screen_distance,
                    "serial_port": serialpath,
                    "fixation_criterion": fixcrit,
                    "fixation_grace": fixation_grace,
                    "eccentricity": eccentricity,
                    "autocycle": autocycle,
                    "intertrial_duration": iti,
                    "reward_duration": reward_duration,
                    "scale_reward": scale_reward,
                    "base_reward_duration": base_reward_duration,
                    "manual_reward_duration": manual_reward_duration,
                    "manual_reward_duration_2": manual_reward_duration_2,
                    "trial_timeout": trial_timeout,
                    "fixation_size": fixsize,
                    "fixation_window": fix_window,
                    "fixation_buffer": fixation_buffer,
                    "fixation_mask": fixation_mask,
                    "mask_params": self.mask_params,
                    "draw_eyespot": use_eyespot,
                    "use_dummy": use_dummy,
                    "play_sound": play_sound,
                    "block_report": block_report,
                    "calibrate": calibrate,
                    "exp_plot": exp_plot,
                    "target_onset_ref": _target_onset_ref,
                    "target_locations": target_locations,
                    "target_onset": target_onset,
                    "target_duration": target_duration,
                    "target_size": target_size,
                    "num_targets": num_targets,
                    "inter_target_interval": inter_target_interval,
                    "show_anchors": target_anchors,
                    "reward_color": reward_color,
                    "make_response": make_response,
                    "failure_color": failure_color,
                    "response_cue_onset_min": response_cue_onset_min,
                    "response_cue_onset_max": response_cue_onset_max,
                    "response_cue_ref": response_cue_ref,
                    "max_rtime": max_rtime,
                    "max_saccade_time": max_saccade_time,
                    "target_fix_time": target_fix_time,
                    "target_window_size": target_window_size,
                    "calibration_reward_duration": calibration_reward_duration,
                    "calibration_target_size": calibration_target_size,
                    "calibration_type": calibration_type,
                    "calibration_target_color": calibration_target_color,
                    "calibration_stimulus": calibration_stimulus,
                    "manual_calibration": manual_calibration,
                    "repeat_failed_targets": repeat_failed_targets,
                    "allowed_combos": allowed_combos,
                    "repeat_locations": repeat_locations,
                    "maze_Map": maze_Map,
                    "maze_rewardLocation": maze_rewardLocation,
                    "maze_numberOfTrials": maze_numberOfTrials,
                    "maze_angleTolerance": maze_angleTolerance,
                    "maze_positionTolerance" : maze_positionTolerance,
                    "maze_destinationDuration": maze_destinationDuration,
                    "maze_trialDuration": maze_trialDuration,
                    "maze_timeToStart": maze_timeToStart,
                    "maze_ITI_Min": maze_ITI_Min,
                    "maze_ITI_Max": maze_ITI_Max,
                    "platform_stop_dist": platform_stop_dist,
                    "platform_clear_dist": platform_clear_dist,
                    "platform_normalSpeed": platform_normalSpeed,
                    "platform_slowDownSpeed": platform_slowDownSpeed}
        return exp_info

    def AMCL(self):
        #Initialize function initialparam at AMCL_v5_GUI start experiment results in the location being gone.
        def get_xy_loc(msg):
                #declare them as global variable
                global loc_x
                global loc_y
                global orien_z
                global orien_w
                global loc_x_rotate
                global loc_y_rotate
                global angle_current
                 
                #assign the subsribed position value x&y to loc_x and loc_y
                loc_x=msg.pose.pose.position.x
                loc_y=msg.pose.pose.position.y
                orien_z=msg.pose.pose.orientation.z
                orien_w=msg.pose.pose.orientation.w
                    
                print(loc_x,loc_y,orien_z)
                   
                #convert the coordinate system from generated map into the reward location map coordinate system
                angle_rotate = -0 #in radiance, anticlockwise is positive
                size_x_map = 1088 #in pixel 1728 no difference when changed 3328
                size_y_map = 1088 #in pixel 1156 no difference when changed 1952
                locs_x_map_origin = -12.2 #in meter   //map7.yaml parameters (-31.4)
                locs_y_map_origin = -13.8 #in meter  //map7.yaml parameters (-23.4)
                 
                loc_x=(loc_x - locs_x_map_origin)/0.025
                loc_y=(-locs_y_map_origin - loc_y)/0.025
                    
                loc_x_new=loc_x-(size_x_map/2)
                loc_y_new=loc_y-(size_y_map/2)
                    
                loc_x_rotate=loc_x_new*(cos(angle_rotate))-loc_y_new*(sin(angle_rotate))+(size_x_map/2)
                loc_y_rotate=loc_y_new*(cos(angle_rotate))+loc_x_new*(sin(angle_rotate))+(size_y_map/2)
                    
                if(orien_z>=0) & (orien_w>=0):
                    angle_current=asin(orien_z)*2
                    
                elif (orien_z>0) & (orien_w<0):
                    angle_current=2*pi-asin(orien_z)*2
                    
                elif (orien_z<0) & (orien_w>0):
                    angle_current=2*pi+asin(orien_z)*2
                          

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, get_xy_loc)

        DegToRad = pi/180
        loc_x=0
        loc_y=0
        LocationMap = '/home/sinapse/catkin_ws/realworldmap_final3.pgm'  #change map  
        RewardLocationCSV = '/home/sinapse/Desktop/RewardData/rewardlocations.csv'
        orien_z=0
        orien_w=0
        loc_x_rotate=0
        loc_y_rotate=0
        current_orientation_w=0
        current_orientation_z=0
        angle_current=0
        angle_desired=0
      
        frequency = 1100 # Hertz
        # Define Experiment Parameters here
        ExperimentNumber =2
        ExperimentDate = '12032019'
        MonkeyName = 'Chimpian'
        Orientation=0
        ExperimentFolder = '/home/sinapse/Desktop/RewardData/'  # Main folder for saving related data
        
        # Create a folder to save Experiment related parameters
        SaveDataFolder = os.path.join(ExperimentFolder, ExperimentDate, str(ExperimentNumber) + '_' + MonkeyName)
        if not os.path.exists(SaveDataFolder):
            os.makedirs(SaveDataFolder)

        # Save Experiment parameters
        saveAMCL = StartExperiment_amcl_v5_GUI.SaveExpParameters(SaveDataFolder, ExperimentNumber=ExperimentNumber, ExperimentDate=ExperimentDate, MonkeyName=MonkeyName, NumberOfTrials=self.maze_numberOfTrials, RewardTimeOut=self.maze_trialDuration)

        rospy.init_node('robot_pose', anonymous=True)

        initAMCL = StartExperiment_amcl_v5_GUI.Experiment(self.maze_numberOfTrials, self.maze_trialDuration, Orientation, self.maze_angleTolerance, self.maze_positionTolerance, self.maze_destinationDuration, LocationMap, RewardLocationCSV, SaveDataFolder)

    def maze_start(self):
        startAMCL = self.AMCL()

    def start(self):
        exp_info = self.get_settings()
        print "Starting experiment"
        #self.results, status = main.run_experiment(exp_info, self.results)
        startAMCL = self.AMCL()
        if len(status) > 1:
            # Something happened
            dlg = QtGui.QErrorMessage(self)
            dlg.showMessage(status)
        else:
            if exp_info["calibrate"]:
                # Safety check to avoid accidentally having to rel-calibrate
                self.calibrate.setChecked(False)


if __name__ == "__main__":
    import sys 

    app = QtWidgets.QApplication(sys.argv)
    myapp = Main()
    myapp.show()
    sys.exit(app.exec_())
