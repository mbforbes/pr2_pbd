#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_pbd_gui')


import os
import time
from subprocess import call
import rospy, yaml
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox, QIcon, QTableView
from python_qt_binding.QtCore import Slot, qDebug, QSignalMapper, QTimer, qWarning, Signal
from pr2_pbd_speech_recognition.msg import Command
from pr2_pbd_interaction.msg import GuiCommand
from sound_play.msg import SoundRequest
from pr2_pbd_interaction.msg import ExperimentState
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
        self.actionIconWidth = 50
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
            pixmap = QtGui.QPixmap(self.selectedIconPath)
        else:
            pixmap = QtGui.QPixmap(self.notSelectedIconPath)
        self.icon.setPixmap(pixmap.scaledToWidth(self.actionIconWidth, QtCore.Qt.SmoothTransformation))


class PbDGUI(Plugin):

    exp_state_sig = Signal(ExperimentState)

    def __init__(self, context):
        super(PbDGUI, self).__init__(context)
        self.setObjectName('PbDGUI')
        self._widget = QWidget()
        
        self.speech_cmd_publisher = rospy.Publisher('recognized_command', Command)
        self.gui_cmd_publisher = rospy.Publisher('gui_command', GuiCommand)
        
        rospy.Subscriber('experiment_state', ExperimentState, self.exp_state_cb)
        rospy.Subscriber('robotsound', SoundRequest, self.robotSoundReceived)
        
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.exp_state_sig.connect(self.update_state)
        
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

        # Settings
        self.n_tasks = 3 # How many different action groups we'll have
        task_names = ['Pick and Place', 'Constrained Pick and Place',
            'Multi-Object Move']
        self.n_tests = 10 # How many 'actions' we'll have per task

        # Main box
        allWidgetsBox = QtGui.QVBoxLayout()

        # TODO(max): Fix all references to:
        # - actionBox (has become task_boxes)
        # - self.actionGrid (has become self.action_grids)
        # - self.actionIcons (has become self.action_icon_sets)

        # Create one box per task
        task_boxes = []
        self.action_grids = []
        self.action_icon_sets = []
        for i in range(self.n_tasks):
            task_boxes.append(QGroupBox(task_names[i], self._widget))
            grid = QtGui.QGridLayout()
            self.action_grids.append(QtGui.QGridLayout())
            self.action_grids[i].setHorizontalSpacing(0)
            for j in range(self.n_tests):
                self.action_grids[i].addItem(QtGui.QSpacerItem(90, 90), 0, j, QtCore.Qt.AlignCenter)
                self.action_grids[i].setColumnStretch(j, 0)
            self.action_icon_sets.append(dict())
            actionBoxLayout = QtGui.QHBoxLayout()
            actionBoxLayout.addLayout(self.action_grids[i])
            task_boxes[i].setLayout(actionBoxLayout)
        
        actionButtonGrid = QtGui.QHBoxLayout()
        actionButtonGrid.addWidget(self.create_button(
                                        Command.CREATE_NEW_ACTION))
        self.stepsBox = QGroupBox('No actions created yet', self._widget)
        self.stepsGrid = QtGui.QGridLayout()
        
        self.l_model = QtGui.QStandardItemModel(self)
        self.l_view = self._create_table_view(self.l_model,
                                              self.l_row_clicked_cb)
        self.r_model = QtGui.QStandardItemModel(self)
        self.r_view = self._create_table_view(self.r_model,
                                              self.r_row_clicked_cb)

        self.stepsGrid.addItem(QtGui.QSpacerItem(280, 10), 0, 0, 2, 3)
        self.stepsGrid.addItem(QtGui.QSpacerItem(10, 10), 0, 1, 2, 3)
        self.stepsGrid.addItem(QtGui.QSpacerItem(280, 10), 0, 2, 2, 3)
        
        self.stepsGrid.addWidget(QtGui.QLabel('Left Arm'), 0, 0)
        self.stepsGrid.addWidget(QtGui.QLabel('Right Arm'), 0, 2)

        self.stepsGrid.addWidget(self.l_view, 1, 0)
        self.stepsGrid.addWidget(self.r_view, 1, 2)
        
        stepsBoxLayout = QtGui.QHBoxLayout()
        stepsBoxLayout.addLayout(self.stepsGrid)
        self.stepsBox.setLayout(stepsBoxLayout)

        stepsButtonGrid = QtGui.QHBoxLayout()
        stepsButtonGrid.addWidget(self.create_button(Command.SAVE_POSE))
        stepsButtonGrid.addWidget(self.create_button(Command.EXECUTE_ACTION))
        stepsButtonGrid.addWidget(self.create_button(Command.STOP_EXECUTION))
        stepsButtonGrid.addWidget(self.create_button(Command.DELETE_ALL_STEPS))
        stepsButtonGrid.addWidget(self.create_button(Command.DELETE_LAST_STEP))

        misc_grid = QtGui.QHBoxLayout()
        misc_grid.addWidget(self.create_button(Command.TEST_MICROPHONE))
        misc_grid.addWidget(self.create_button(Command.RECORD_OBJECT_POSE))
        misc_grid.addStretch(1)
        
        misc_grid2 = QtGui.QHBoxLayout()
        misc_grid2.addWidget(self.create_button(Command.RELAX_RIGHT_ARM))
        misc_grid2.addWidget(self.create_button(Command.RELAX_LEFT_ARM))
        misc_grid2.addWidget(self.create_button(Command.FREEZE_RIGHT_ARM))
        misc_grid2.addWidget(self.create_button(Command.FREEZE_LEFT_ARM))
        misc_grid2.addStretch(1)

        misc_grid3 = QtGui.QHBoxLayout()
        misc_grid3.addWidget(self.create_button(Command.OPEN_RIGHT_HAND))
        misc_grid3.addWidget(self.create_button(Command.OPEN_LEFT_HAND))
        misc_grid3.addWidget(self.create_button(Command.CLOSE_RIGHT_HAND))
        misc_grid3.addWidget(self.create_button(Command.CLOSE_LEFT_HAND))
        misc_grid3.addStretch(1)
        
        misc_grid4 = QtGui.QHBoxLayout()
        misc_grid4.addWidget(self.create_button(Command.PREV_ACTION))
        misc_grid4.addWidget(self.create_button(Command.NEXT_ACTION))
        misc_grid4.addStretch(1)

        speechGroupBox = QGroupBox('Robot Speech', self._widget)
        speechGroupBox.setObjectName('RobotSpeechGroup')
        speechBox = QtGui.QHBoxLayout()
        self.speechLabel = QtGui.QLabel('Robot has not spoken yet')
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        self.speechLabel.setPalette(palette)
        speechBox.addWidget(self.speechLabel)
        speechGroupBox.setLayout(speechBox)

        # add the three action sections
        for i in range(self.n_tasks):
            allWidgetsBox.addWidget(task_boxes[i])

        # action button
        allWidgetsBox.addLayout(actionButtonGrid)
        
        allWidgetsBox.addWidget(self.stepsBox)
        allWidgetsBox.addLayout(stepsButtonGrid)
        
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid2)
        allWidgetsBox.addLayout(misc_grid3)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid4)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        
        allWidgetsBox.addWidget(speechGroupBox)
        allWidgetsBox.addStretch(1)
        
        # Fix layout and add main widget to the user interface
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
        
    def _create_table_view(self, model, row_click_cb):
        proxy = QtGui.QSortFilterProxyModel(self)
        proxy.setSourceModel(model)
        view = QtGui.QTableView(self._widget)
        verticalHeader = view.verticalHeader()
        verticalHeader.sectionClicked.connect(row_click_cb)
        view.setModel(proxy)
        view.setMaximumWidth(250)
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

    def r_row_clicked_cb(self, logicalIndex):
        self.step_pressed(self.get_uid(0, logicalIndex))

    def l_row_clicked_cb(self, logicalIndex):
        self.step_pressed(self.get_uid(1, logicalIndex))

    def create_button(self, command):
        btn = QtGui.QPushButton(self.commands[command], self._widget)
        btn.clicked.connect(self.command_cb)
        return btn

    def update_state(self, state):
        qWarning('Received new state')
        n_actions = self.n_actions()
        if n_actions < state.n_actions:
            for i in range(n_actions, state.n_actions):
                self.new_action()

        if (self.currentAction != (state.i_current_action - 1)):
            self.delete_all_steps()
            self.action_pressed(state.i_current_action - 1, False)

        n_steps = self.n_steps()
        if (n_steps < state.n_steps):
            for i in range(n_steps, state.n_steps):
                self.save_pose()
        elif (n_steps > state.n_steps):
            n_to_remove = n_steps - state.n_steps
            self.r_model.invisibleRootItem().removeRows(state.n_steps,
                                                      n_to_remove)
            self.l_model.invisibleRootItem().removeRows(state.n_steps,
                                                      n_to_remove)
        
        ## TODO: DEAL with the following arrays!!!
        state.r_gripper_states
        state.l_gripper_states
        state.r_ref_frames
        state.l_ref_frames
        state.objects
            
        if (self.currentStep != state.i_current_step):
            if (self.n_steps() > 0):
                self.currentStep = state.i_current_step
                arm_index, index = self.get_arm_and_index(self.currentStep)
                if (arm_index == 0):
                    self.r_view.selectRow(index)
                else:
                    self.l_view.selectRow(index)

    def save_pose(self, actionIndex=None):
        nColumns = 9
        if actionIndex is None:
            actionIndex = self.currentAction
        stepIndex = self.n_steps(actionIndex)
        r_step = [QtGui.QStandardItem('Step' + str(stepIndex + 1)),
                    QtGui.QStandardItem('Go to pose'), 
                    QtGui.QStandardItem('Absolute')]
        l_step = [QtGui.QStandardItem('Step' + str(stepIndex + 1)),
                    QtGui.QStandardItem('Go to pose'), 
                    QtGui.QStandardItem('Absolute')]
        self.r_model.invisibleRootItem().appendRow(r_step)
        self.l_model.invisibleRootItem().appendRow(l_step)
        self.update_table_view()
        self.currentStep = stepIndex
        
    def update_table_view(self):
        self.l_view.setColumnWidth(0, 50)
        self.l_view.setColumnWidth(1, 100)
        self.l_view.setColumnWidth(2, 70)
        self.r_view.setColumnWidth(0, 50)
        self.r_view.setColumnWidth(1, 100)
        self.r_view.setColumnWidth(2, 70)
        
    def n_steps(self, actionIndex=None):
        return self.l_model.invisibleRootItem().rowCount()
        
    def delete_all_steps(self, actionIndex=None):
        if actionIndex is None:
            actionIndex = self.currentAction
        n_steps = self.n_steps()
        if (n_steps > 0):
            self.l_model.invisibleRootItem().removeRows(0, n_steps)
            self.r_model.invisibleRootItem().removeRows(0, n_steps)

    def n_actions(self):
        return sum([len(a.keys()) for a in self.action_icon_sets])

    def new_action(self):
        nColumns = self.n_tests
        actionIndex = self.n_actions()
        taskIdx = actionIndex / self.n_tests
        if taskIdx >= self.n_tasks:
            rospy.logwarn('Cannot add more actions; this study is fixed!')
            return
        for s in self.action_icon_sets:
            for key, icon in s.iteritems():
               icon.selected = False
               icon.updateView()
        actIcon = ActionIcon(self._widget, actionIndex, self.action_pressed)
        actionGrid = self.action_grids[taskIdx]
        actionGrid.addLayout(actIcon, actionIndex / nColumns,
            actionIndex % nColumns)
        self.action_icon_sets[taskIdx][actionIndex] = actIcon

    def step_pressed(self, step_index):
        gui_cmd = GuiCommand(GuiCommand.SELECT_ACTION_STEP, step_index)
        self.gui_cmd_publisher.publish(gui_cmd)

    def action_pressed(self, actionIndex, isPublish=True):
        for s in self.action_icon_sets:
            for key, icon in s.iteritems():
                icon.selected = (key == actionIndex)
                icon.updateView()
        self.currentAction = actionIndex
        self.stepsBox.setTitle('Steps for Action ' + str(self.currentAction+1))
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
        
    def robotSoundReceived(self, soundReq):
        if (soundReq.command == SoundRequest.SAY):
            qWarning('Robot said: ' + soundReq.arg)
            self.speechLabel.setText('Robot sound: ' + soundReq.arg)
    
    def exp_state_cb(self, state):
        qWarning('Received new experiment state.')
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

