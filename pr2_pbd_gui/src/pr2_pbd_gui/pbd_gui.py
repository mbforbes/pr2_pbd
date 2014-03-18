#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_pbd_gui')


import os
import time
import glob
from subprocess import call
import rospy, yaml
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox, QIcon, QTableView
from python_qt_binding.QtCore import Slot, qDebug, QSignalMapper, QTimer, qWarning, Signal
from pr2_pbd_speech_recognition.msg import Command
from pr2_pbd_interaction.msg import GuiCommand, ExperimentState
from pr2_pbd_interaction.msg import ScoreResult, ScoreResultList
from sound_play.msg import SoundRequest
from pr2_pbd_interaction.srv import GetExperimentState


class ClickableLabel(QtGui.QLabel):
    def __init__(self, parent, index, clickCallback):
        QtGui.QLabel.__init__(self, parent)
        self.index = index
        self.clickCallback = clickCallback
    
    def mousePressEvent(self, event):
        self.emit(QtCore.SIGNAL('clicked()'), "Label pressed")
        self.clickCallback(self.index)


class ActionIcon(QtGui.QGridLayout):
    def __init__(self, parent, index, clickCallback):
        QtGui.QGridLayout.__init__(self)
        self.setSpacing(0)
        path = os.popen('rospack find pr2_pbd_gui').read()
        path = path[0:len(path)-1]
        self.notSelectedIconPath = path + '/icons/actions0.png'
        self.selectedIconPath = path + '/icons/actions1.png'
        self.notSelectedDoneIconPath = path + '/icons/actions0_done.png'
        self.selectedDoneIconPath = path + '/icons/actions1_done.png'
        self.selected = True
        self.all_reachable = False
        self.actionIconWidth = 40
        self.index = index
        self.icon = ClickableLabel(parent, index, clickCallback)
        self.text = QtGui.QLabel(parent)
        self.text.setText(self.getName())
        self.updateView()
        self.addWidget(self.icon, 0, 0, QtCore.Qt.AlignCenter)
        self.addWidget(self.text, 1, 0, QtCore.Qt.AlignCenter)
    
    def getName(self):
        return 'Action' + str(self.index + 1)
    
    def updateView(self):
        if self.selected:
            if self.all_reachable:
                pixmap = QtGui.QPixmap(self.selectedDoneIconPath)
            else:
                pixmap = QtGui.QPixmap(self.selectedIconPath)
        else:
            if self.all_reachable:
                pixmap = QtGui.QPixmap(self.notSelectedDoneIconPath)
            else:
                pixmap = QtGui.QPixmap(self.notSelectedIconPath)
        self.icon.setPixmap(pixmap.scaledToWidth(self.actionIconWidth,
            QtCore.Qt.SmoothTransformation))


class ScoreIcon(QtGui.QGridLayout):
    def __init__(self, parent, index, clickCallback):
        QtGui.QGridLayout.__init__(self)
        self.setSpacing(0)
        path = os.popen('rospack find pr2_pbd_gui').read()
        path = path[0:len(path)-1]
        self.notSelectedIconPath = path + '/icons/score_deselected.png'
        self.selectedIconPath = path + '/icons/score_selected.png'
        self.selected = True
        self.actionIconWidth = 40
        self.index = index
        self.icon = ClickableLabel(parent, index, clickCallback)
        self.text = QtGui.QLabel(parent)
        self.text.setText(self.getName())
        self.updateView()
        self.addWidget(self.icon, 0, 0, QtCore.Qt.AlignCenter)
        self.addWidget(self.text, 1, 0, QtCore.Qt.AlignCenter)
    
    def getName(self):
        return 'Score func. ' + str(self.index + 1)
    
    def updateView(self):
        if self.selected:
            pixmap = QtGui.QPixmap(self.selectedIconPath)
        else:
            pixmap = QtGui.QPixmap(self.notSelectedIconPath)
        self.icon.setPixmap(pixmap.scaledToWidth(self.actionIconWidth,
            QtCore.Qt.SmoothTransformation))


class PbDGUI(Plugin):

    exp_state_sig = Signal(ExperimentState)
    score_result_list_sig = Signal(ScoreResultList)

    def __init__(self, context):
        super(PbDGUI, self).__init__(context)
        self.setObjectName('PbDGUI')
        self._widget = QWidget()
        
        self.speech_cmd_publisher = rospy.Publisher('recognized_command', Command)
        self.gui_cmd_publisher = rospy.Publisher('gui_command', GuiCommand)
        
        rospy.Subscriber('experiment_state', ExperimentState, self.exp_state_cb)
        rospy.Subscriber('score_result_list', ScoreResultList,
            self.score_result_list_cb)
        
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.exp_state_sig.connect(self.update_state)
        self.score_result_list_sig.connect(self.update_score_result_list)
        
        self.commands = dict()
        self.commands[Command.CREATE_NEW_ACTION] = 'New action'
        self.commands[Command.TEST_MICROPHONE] = 'Test microphone'
        self.commands[Command.NEXT_ACTION] = 'Next action'
        self.commands[Command.PREV_ACTION] = 'Previous action'
        self.commands[Command.SAVE_POSE] = 'Save pose'
        self.commands[Command.RELAX_RIGHT_ARM] = 'Relax right arm'
        self.commands[Command.RELAX_LEFT_ARM] = 'Relax left arm'
        self.commands[Command.FREEZE_RIGHT_ARM] = 'Freeze right arm'
        self.commands[Command.FREEZE_LEFT_ARM] = 'Freeze left arm'
        self.commands[Command.OPEN_RIGHT_HAND] = 'Open right hand'
        self.commands[Command.OPEN_LEFT_HAND] = 'Open left hand'
        self.commands[Command.CLOSE_RIGHT_HAND] = 'Close right hand'
        self.commands[Command.CLOSE_LEFT_HAND] = 'Close left hand'
        self.commands[Command.CLOSE_LEFT_HAND] = 'Close left hand'
        self.commands[Command.EXECUTE_ACTION] = 'Execute action'
        self.commands[Command.STOP_EXECUTION] = 'Stop execution'
        self.commands[Command.DELETE_ALL_STEPS] = 'Delete all'
        self.commands[Command.DELETE_LAST_STEP] = 'Delete last'
        self.commands[Command.RECORD_OBJECT_POSE] = 'Record object poses'
        
        self.currentAction = -1
        self.currentStep = -1
        # TODO(max): Make GUI area to select this / report back to Interaction
        self.currentTask = 3
        self.currentScoreFunc = -1
        self.results = [] # for score result list
        self.selectedResult = -1 # start invalid

        # Settings (hardcoded for now; can just filter bad requests on node...)
        n_score_funcs = 3

        # Settings
        #self.n_tasks = len(glob.glob(rospy.get_param(
        #    '/pr2_pbd_interaction/dataRoot') + '/data/experimentTesting/task*'))
        #self.n_tests = len(glob.glob(rospy.get_param(
        #    '/pr2_pbd_interaction/dataRoot') + '/data/experimentTesting/task' + 
        #    str(self.currentTask) + '/*.bag'))

        # Creating components
        # ======================================================================
        # Main box
        allWidgetsBox = QtGui.QVBoxLayout()

        # Create task buttons to swtich between tasks & show active one
        # TODO

        # Instructions area
        instructionsGroupBox = QGroupBox('Information', self._widget)
        instructionsGroupBox.setObjectName('InstructionsGroup')
        instructionsBox = QtGui.QVBoxLayout()
        self.i4 = QtGui.QLabel('(Waiting to load action.)')

        self.palette = QtGui.QPalette()
        self.palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.red)
        self.i4.setPalette(self.palette)
        instructionsBox.addWidget(self.i4)
        instructionsGroupBox.setLayout(instructionsBox)

        # Create one action icon per test
        unreachable_nums = PbDGUI._get_n_tests_for_task(self.currentTask)
        n_unreachable_boxes = []
        self.action_grids = []
        self.action_icon_sets = []
        for i in range(1, len(unreachable_nums)):
            n_unreachable_boxes += [QGroupBox(str(i) + ' unreachable',
                self._widget)]
            grid = QtGui.QGridLayout()
            grid.setContentsMargins(0,0,0,0)
            self.action_grids += [QtGui.QGridLayout()]
            self.action_grids[-1].setHorizontalSpacing(0)
            for j in range(1, unreachable_nums[i] + 1):
                self.action_grids[-1].addItem(
                    QtGui.QSpacerItem(50, 0), 0, j - 1, QtCore.Qt.AlignCenter)
                self.action_grids[-1].setColumnStretch(j - 1, 0)
            self.action_icon_sets += [dict()]
            actionBoxLayout = QtGui.QHBoxLayout()
            actionBoxLayout.setContentsMargins(0,0,0,0)
            actionBoxLayout.addLayout(self.action_grids[-1])
            n_unreachable_boxes[-1].setLayout(actionBoxLayout)
            n_unreachable_boxes[-1].setFlat(True)
            n_unreachable_boxes[-1].setContentsMargins(0,0,0,0)

        # Create buttons to switch between score functions
        self.score_funcs = []
        score_funcs_box = QGroupBox('Score functions', self._widget)
        score_funcs_grid = QtGui.QGridLayout()
        score_funcs_grid.setHorizontalSpacing(0)
        for i in range(n_score_funcs):
            # Add placeholders (looks like)
            score_funcs_grid.addItem(
                QtGui.QSpacerItem(90, 20), 0, i, QtCore.Qt.AlignCenter)
            score_funcs_grid.setColumnStretch(i, 0)
            # We're adding the real icons here because we don't need to create
            # them dynamically.
            icon = ScoreIcon(self._widget, i, self.score_pressed)
            icon.selected = False
            icon.updateView()
            self.score_funcs += [icon]
            score_funcs_grid.addLayout(icon, 0, i)

        # Create selectable text area things to show top n from score function
        self.resultsGrid = QtGui.QGridLayout()
        self.resultsModel = QtGui.QStandardItemModel(self)
        self.resultsView = self._create_table_view(self.resultsModel,
            self.result_clicked_cb)
        self.resultsGrid.addItem(QtGui.QSpacerItem(560, 10), #item
            0, # row
            0, # col
            2, # rowSpan
            1) # colSpan
        self.resultsGrid.addWidget(
            QtGui.QLabel('Top results for selected score function'), # widget
            0, # row
            0) # col
        self.resultsGrid.addWidget(self.resultsView, # widget
            1, # row
            0) # col

        # Let us actually try it...
        runButtonGrid = QtGui.QHBoxLayout()
        runButtonGrid.addWidget(self.create_button(Command.RECORD_OBJECT_POSE))
        runButtonGrid.addWidget(self.create_button(Command.EXECUTE_ACTION))
        runButtonGrid.addWidget(self.create_button(Command.STOP_EXECUTION))

        # helpful commands to have when manipulating the real robot
        misc_grid = QtGui.QHBoxLayout()
        misc_grid.addWidget(self.create_button(Command.FREEZE_LEFT_ARM))        
        misc_grid.addWidget(self.create_button(Command.FREEZE_RIGHT_ARM))
        misc_grid.addWidget(self.create_button(Command.RELAX_LEFT_ARM))
        misc_grid.addWidget(self.create_button(Command.RELAX_RIGHT_ARM))

        # Add all to overall score box...
        score_funcs_layout = QtGui.QVBoxLayout()
        score_funcs_layout.addLayout(score_funcs_grid)
        score_funcs_layout.addLayout(self.resultsGrid)
        score_funcs_layout.addLayout(runButtonGrid)
        score_funcs_layout.addLayout(misc_grid)
        score_funcs_box.setLayout(score_funcs_layout)

        # Adding components
        # ======================================================================
        # add instructions to main area
        allWidgetsBox.addWidget(instructionsGroupBox)
        allWidgetsBox.addStretch(1)
        
        # add the three action sections
        for box in n_unreachable_boxes:
            allWidgetsBox.addWidget(box)

        # add the score function scetion
        allWidgetsBox.addWidget(score_funcs_box)

        # Fix layout and add main widget to the user interface
        # ======================================================================
        QtGui.QApplication.setStyle(QtGui.QStyleFactory.create('plastique'))
        vAllBox = QtGui.QVBoxLayout()
        vAllBox.addLayout(allWidgetsBox)
        vAllBox.addStretch(1)
        hAllBox = QtGui.QHBoxLayout()
        hAllBox.addLayout(vAllBox)
        hAllBox.addStretch(1)

        self._widget.setObjectName('PbDGUI')
        self._widget.setLayout(hAllBox)
        context.add_widget(self._widget)

        rospy.loginfo('Will wait for the experiment state service...')
        rospy.wait_for_service('get_experiment_state')
        exp_state_srv = rospy.ServiceProxy('get_experiment_state',
                                                 GetExperimentState)
        rospy.loginfo('Got response from the experiment state service...')

        response = exp_state_srv()
        self.update_state(response.state)
        
    # STATIC METHODS
    # ==========================================================================

    @staticmethod
    def _get_n_tests_for_task(task_no):
        '''Returns an array of the number of tests for each no. unreachable
        (start configs) based on the task.

        For example, task 1 has
        (5) 1 unreachable,
        (3) 2 unreachable, and
        (2) 3 unreachable,

        so it would return the array [0, 5, 3, 2]
        '''
        if task_no == 1:
            return [0, 5, 3, 2]
        else:
            return [0, 2, 2, 2, 2, 2]


    # OBJECT METHODS
    # ==========================================================================
    def create_button(self, command):
        btn = QtGui.QPushButton(self.commands[command], self._widget)
        btn.clicked.connect(self.command_cb)
        return btn

    def _get_row_col_idxes_for_action(self, action_no):
        '''Returns row_idx, col_idx to get action action_no from the sets.
        Assumes that passed param (action_no) is ONE-BASED INDEXING.'''
        testarr = PbDGUI._get_n_tests_for_task(self.currentTask)
        idx = 1
        togo = action_no
        while True:
            n_in_cur = testarr[idx]
            if togo - n_in_cur <= 0:
                return idx - 1, togo - 1 # convert to 0-based indexing
            else:
                togo -= n_in_cur
                idx += 1

    def _create_table_view(self, model, row_click_cb):
        proxy = QtGui.QSortFilterProxyModel(self)
        proxy.setSourceModel(model)
        view = QtGui.QTableView(self._widget)
        horizontalHeader = view.horizontalHeader()
        horizontalHeader.setStretchLastSection(True)
        horizontalHeader.setClickable(False)
        # TODO(max): These aren't doing anything...
        model.setHeaderData(0, QtCore.Qt.Horizontal, QtGui.QLabel('user dir'))
        model.setHeaderData(1, QtCore.Qt.Horizontal, QtGui.QLabel('user action'))
        model.setHeaderData(2, QtCore.Qt.Horizontal, QtGui.QLabel('score'))
        verticalHeader = view.verticalHeader()
        verticalHeader.sectionClicked.connect(row_click_cb)
        view.setModel(proxy)
        view.setMaximumWidth(560)
        view.setMinimumHeight(200)
        view.setSortingEnabled(False)
        view.setCornerButtonEnabled(False)
        return view
    
    def get_uid(self, arm_index, index):
        '''Returns a unique id of the marker'''
        return (2 * (index + 1) + arm_index)
    
    def get_arm_and_index(self, uid):
        '''Returns a unique id of the marker'''
        arm_index = uid % 2
        index = (uid - arm_index) / 2
        return (arm_index, (index - 1))

    def result_clicked_cb(self, idx):
        '''Fired when one of the topn score results clicked.'''
        # debug
        #print 'RESULT CLICKED:', idx, self.results[idx]
        # first update the visuals
        self.selectedResult = idx
        self.resultsView.selectRow(idx)

        # then trigger a gui command creation / broadcast
        res = self.results[idx]
        userdir, useract = res[0], res[1]
        # NOTE(max): With uint16 (2^16 = 65536), this only works through
        # - max user no. 655
        # and
        # - max action 36 for that user
        # or
        # - action 99 for all other users
        uid = userdir * 100 + useract
        self.result_pressed(uid)

    def r_row_clicked_cb(self, logicalIndex):
        self.step_pressed(self.get_uid(0, logicalIndex))

    def l_row_clicked_cb(self, logicalIndex):
        self.step_pressed(self.get_uid(1, logicalIndex))

    def update_score_result_list(self, scoreResultList):
        # NOTE(max): This is getting called twice... is it because we have two
        # instances of PbD GUI?

        # clear
        self.resultsModel.invisibleRootItem().removeRows(0, len(self.results))
        self.results = []

        # get new results
        for res in scoreResultList.results:
            row = [QtGui.QStandardItem(str(res.userdir)),
                QtGui.QStandardItem(str(res.useract)), 
                QtGui.QStandardItem(str(res.score))]
            self.resultsModel.invisibleRootItem().appendRow(row)
            self.results += [[res.userdir, res.useract, res.score]]
        self.update_results_view()
        if len(scoreResultList.results) > 0:
            self.selectedResult = 0
            self.resultsView.selectRow(self.selectedResult)
        else:
            # invalid
            self.selectedResult = -1

    def update_results_view(self):
        col_widths = [150, 150, 150]
        for idx, width in enumerate(col_widths):
            self.resultsView.setColumnWidth(idx, width)

    def update_state(self, state):
        # NOTE(max): Too spammy...
        # qWarning('Received new state')
        n_actions = self.n_actions()
        if n_actions < state.n_actions:
            for i in range(n_actions, state.n_actions):
                self.new_action()

        # Go from 1-based to 0-based indexing
        new_act_idx = state.i_current_action - 1
        if self.currentAction != new_act_idx:
            self.delete_all_steps()
            self.action_pressed(new_act_idx, False)

        # Get icon (_get_row_col_idxes_for_action uses 1-based)
        row, col = self._get_row_col_idxes_for_action(new_act_idx + 1)
        # NOTE: each action icon set maps from action index (0-based) to
        # action, not starting at 0, but from the real action number.
        # e.g. action icon set 2 might start at action 5 (6 in 1-baed)
        # rather than 0.
        icon = self.action_icon_sets[row][new_act_idx]
        icon.updateView()

        # Extract no. unreachable markers
        nu = state.n_unreachable_markers
        self.i4.setText('Number of unreachable markers: ' + str(nu))
        if nu > 0:
            self.palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.red)
            icon.all_reachable = False
        else:
            self.palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.green)
            icon.all_reachable = True
        self.i4.setPalette(self.palette)

        # TODO: Deal with the following arrays:
        # state.r_gripper_states
        # state.l_gripper_states
        # state.r_ref_frames
        # state.l_ref_frames
        # state.objects
            
    def delete_all_steps(self, actionIndex=None):
        if actionIndex is None:
            actionIndex = self.currentAction

    def n_actions(self):
        return sum([len(a.keys()) for a in self.action_icon_sets])

    def new_action(self):
        # Deselect the rest (automatically selects the new one)
        for s in self.action_icon_sets:
            for key, icon in s.iteritems():
               icon.selected = False
               icon.updateView()

        # n_actions is 1-based, so good for insertion into 0-based indexing
        actionIndex = self.n_actions()
        # _get_row_col_idxes_for_action takes one-based indexing
        row, col = self._get_row_col_idxes_for_action(actionIndex + 1)
        # The new action icon is automatically selected upon construction.
        actIcon = ActionIcon(self._widget, actionIndex, self.action_pressed)

        #taskIdx = actionIndex / nColumns # row idx
        #nColumns = self.n_tests

        actionGrid = self.action_grids[row]
        actionGrid.addLayout(actIcon,
            0, # row idx; was actionIndex / nColumns
            col) # col idx; was actionIndex % nColumns
        self.action_icon_sets[row][actionIndex] = actIcon

    def step_pressed(self, step_index):
        gui_cmd = GuiCommand(GuiCommand.SELECT_ACTION_STEP, step_index)
        self.gui_cmd_publisher.publish(gui_cmd)

    def result_pressed(self, uid):
        gui_cmd = GuiCommand(GuiCommand.SELECT_RESULT, uid)
        self.gui_cmd_publisher.publish(gui_cmd)

    def score_pressed(self, score_index):
        # debug
        if self.currentScoreFunc == score_index:
            return
        self.currentScoreFunc = score_index
        for idx, icon in enumerate(self.score_funcs):
            icon.selected = (idx == score_index)
            icon.updateView()
        # NOTE: keeping 0-based indexing here as sending to code...
        self.gui_cmd_publisher.publish(
            GuiCommand(GuiCommand.SELECT_SCORE_FUNC, score_index))

    def action_pressed(self, actionIndex, isPublish=True):
        for i, s in enumerate(self.action_icon_sets):
            for key, icon in s.iteritems():
                icon.selected = (key == actionIndex)
                icon.updateView()
        self.currentAction = actionIndex
        if isPublish:
            gui_cmd = GuiCommand(GuiCommand.SWITCH_TO_ACTION, (actionIndex+1))
            self.gui_cmd_publisher.publish(gui_cmd)
        
    def command_cb(self):
        clickedButtonName = self._widget.sender().text()
        for key in self.commands.keys():
            if (self.commands[key] == clickedButtonName):
                qWarning('Sending speech command: '+ key)
                command = Command()
                command.command = key
                self.speech_cmd_publisher.publish(command)
        
    def score_result_list_cb(self, scoreResultList):
        # happens twice ...
        self.score_result_list_sig.emit(scoreResultList)

    def exp_state_cb(self, state):
        # NOTE(max): Too spammy...
        #qWarning('Received new experiment state.')
        self.exp_state_sig.emit(state)
        
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.speech_cmd_publisher.unregister()
        self.gui_cmd_publisher.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

