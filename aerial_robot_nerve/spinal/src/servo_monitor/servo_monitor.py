from __future__ import print_function
import os
import rospy
import rospkg
from spinal.srv import *
from spinal.msg import ServoTorqueCmd, ServoStates

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import python_qt_binding as pyqt
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
import distutils.util
from functools import partial
from operator import add

#def _servo

class ServoMonitor(Plugin):

    def __init__(self, context):
        super(ServoMonitor, self).__init__(context)

        self.setObjectName('ServoMonitor')

        self.get_board_info_client_ = rospy.ServiceProxy('/get_board_info', GetBoardInfo)
        self.set_board_config_client_ = rospy.ServiceProxy('/set_board_config', SetBoardConfig)
        self.servo_torque_pub_ = rospy.Publisher('/servo/torque_enable', ServoTorqueCmd, queue_size = 1)

        from argparse import ArgumentParser
        parser = ArgumentParser()

        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ServoMonitor.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('ServoMonitor')

        self._widget.boardInfoUpdateButton.clicked.connect(self.updateButtonCallback)
        self._widget.allServoOnButton.clicked.connect(self.allServoOnButtonCallback)
        self._widget.allServoOffButton.clicked.connect(self.allServoOffButtonCallback)
        self._widget.servoTableWidget.clicked.connect(self.tableClickedCallback)

        self._widget.servoTableWidget.setContextMenuPolicy(Qt.ActionsContextMenu)
        servoOnAction = QAction("servo on", self._widget.servoTableWidget)
        servoOnAction.triggered.connect(self.servoOn)
        self._widget.servoTableWidget.addAction(servoOnAction)
        servoOffAction = QAction("servo off", self._widget.servoTableWidget)
        servoOffAction.triggered.connect(self.servoOff)
        self._widget.servoTableWidget.addAction(servoOffAction)
        jointCalibAction = QAction("joint calib", self._widget.servoTableWidget)
        jointCalibAction.triggered.connect(self.jointCalib)
        self._widget.servoTableWidget.addAction(jointCalibAction)

        self._table_data = []
        self._board_id = None
        self._servo_id = None
        self._command = None

        self._current_servo_serial_index = None

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._headers = ["board", "index", "id", "angle", "temperature", "load", "error", "pid_gains", "profile_velocity", "current_limit", "send_data_flag", "servo on", "servo off", "calibration"]

        self._widget.setLayout(self._widget.gridLayout)
        self.updateButtonCallback()
        self.servo_state_sub_ = rospy.Subscriber('/servo/states', ServoStates, self.servoStateCallback)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(1000)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def servoTorqueControl(self, enable):
        servo_index = self._widget.servoTableWidget.currentIndex().row()
        msg = ServoTorqueCmd()
        msg.index = chr(servo_index)
        msg.torque_enable = chr(enable)
        self.servo_torque_pub_.publish(msg)

    def servoOn(self):
        self.servoTorqueControl(1)

    def servoOff(self):
        self.servoTorqueControl(0)

    def allServoTorqueControl(self, enable):
        servo_num = self._widget.servoTableWidget.rowCount()
        msg = ServoTorqueCmd()
        msg.index = reduce(add, [chr(i) for i in range(servo_num)])
        msg.torque_enable = reduce(add, [chr(enable)] * servo_num)
        self.servo_torque_pub_.publish(msg)

    def allServoOnButtonCallback(self):
        self.allServoTorqueControl(1)

    def allServoOffButtonCallback(self):
        self.allServoTorqueControl(0)

    def jointCalib(self):
        servo_index = self._widget.servoTableWidget.currentIndex().row()
        req = SetBoardConfigRequest()
        req.data.append(int(self._widget.servoTableWidget.item(servo_index, 0).text())) #board id
        req.data.append(int(self._widget.servoTableWidget.item(servo_index, 1).text())) #servo index

        try:
            req.data.append(int(self._widget.homingOffsetLineEdit.text()))
        except ValueError as e:
            print(e)
            return
        req.command = req.SET_SERVO_HOMING_OFFSET

        #need to disable servo torque
        servo_trq_msg = ServoTorqueCmd()
        servo_trq_msg.index = chr(servo_index)
        servo_trq_msg.torque_enable = chr(0)
        self.servo_torque_pub_.publish(servo_trq_msg)
        rospy.sleep(0.5)

        rospy.loginfo('published message')
        rospy.loginfo('command: ' + str(req.command))
        rospy.loginfo('data: ' + str(req.data))
        rospy.wait_for_service('/set_board_config')
        try:
            res = self.set_board_config_client_(req)
            rospy.loginfo(bool(res.success))
        except rospy.ServiceException, e:
            print("/set_board_config service call failed: %s"%e)


    def servoStateCallback(self, msg):
        for s in msg.servos:
            self._table_data[s.index][3] = s.angle
            self._table_data[s.index][4] = s.temp
            self._table_data[s.index][5] = s.load
            self._table_data[s.index][6] = s.error

    def update(self):
        self._widget.servoTableWidget.setRowCount(len(self._table_data))
        self._widget.servoTableWidget.setColumnCount(len(self._table_data[0]))
        for i in range(len(self._table_data)):
            for j in range(len(self._table_data[0])):
                item = QTableWidgetItem(str(self._table_data[i][j]))

                self._widget.servoTableWidget.setItem(i, j, item)

        #self._widget.servoTableWidget.setHorizontalHeaderLabels(self._headers)
        #self._widget.servoTableWidget.horizontalHeaderItem(3).setBackground(Qt.blue)

        for i, h in enumerate(self._headers):
            item = QTableWidgetItem(str(h))
            self._widget.servoTableWidget.setHorizontalHeaderItem(i, item)

        for i in range(len(self._table_data)):
            item = QTableWidgetItem('servo' + str(i))
            self._widget.servoTableWidget.setVerticalHeaderItem(i, item)

        self._widget.servoTableWidget.show()

    def updateButtonCallback(self):
        rospy.wait_for_service('/get_board_info')
        try:
            res = self.get_board_info_client_()

            servo_index = 0
            self._table_data = []
            for b in res.boards:
                for i, s in enumerate(b.servos):
                    rowData = []
                    rowData.append(b.slave_id)
                    rowData.append(i)
                    rowData.append(s.id)
                    rowData.extend([None] * 4)
                    rowData.append(str(s.p_gain) + ', ' + str(s.i_gain) + ', ' + str(s.d_gain))
                    rowData.append(s.profile_velocity)
                    rowData.append(s.current_limit)
                    rowData.append(str(bool(s.send_data_flag)))

                    self._table_data.append(rowData)

        except rospy.ServiceException, e:
            print("/get_board_info service call failed: %s"%e)

    def tableClickedCallback(self):
        col = self._widget.servoTableWidget.currentIndex().column()
        row = self._widget.servoTableWidget.currentIndex().row()
        print(col)
        print(row)

        '''
        param = self._widget.boardInfoTreeView.currentIndex().sibling(row, 0).data()
        value = self._widget.boardInfoTreeView.currentIndex().sibling(row, 1).data()

        board_param_list = ['board_id', 'imu_send_data_flag']
        servo_param_list = ['pid_gain', 'profile_velocity', 'current_limit', 'send_data_flag']

        if value and (param in servo_param_list):
            servo_index = self._widget.boardInfoTreeView.currentIndex().parent().data()
            board_id = self._widget.boardInfoTreeView.currentIndex().parent().parent().parent().data()
            self._widget.paramLabel.setText('board_id: ' + board_id + ' servo_index: ' + servo_index + ' ' + param)
            self._board_id = board_id
            self._servo_index = servo_index
            self._command = param
            self._current_servo_serial_index = int(self._widget.boardInfoTreeView.currentIndex().parent().child(1, 1).data())
        elif value and (param in board_param_list):
            board_id = self._widget.boardInfoTreeView.currentIndex().parent().data()
            self._widget.paramLabel.setText('board_id: ' + board_id + ' ' + param)
            self._board_id = board_id
            self._command = param
        else:
            self._board_id = None
            self._command = None
            self._servo_index = None
            self._widget.paramLabel.setText('')
        '''
    def configureButtonCallback(self):
        req = SetBoardConfigRequest()
        req.data.append(int(self._board_id))

        if self._command == 'board_id':
            try:
                req.data.append(int(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SLAVE_ID
        elif self._command == 'imu_send_data_flag':
            try:
                req.data.append(distutils.util.strtobool(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_IMU_SEND_FLAG
        elif self._command == 'pid_gain':
            try:
                req.data.append(int(self._servo_index))
                pid_gains = map(lambda x: int(x), self._widget.lineEdit.text().split(','))
                if len(pid_gains) != 3:
                    raise ValueError('Input 3 gains(int)')
                req.data.extend(pid_gains)
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_PID_GAIN
        elif self._command == 'profile_velocity':
            try:
                req.data.append(int(self._servo_index))
                req.data.append(int(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_PROFILE_VEL
        elif self._command == 'send_data_flag':
            try:
                req.data.append(int(self._servo_index))
                req.data.append(distutils.util.strtobool(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_SEND_DATA_FLAG
        elif self._command == 'current_limit':
            try:
                req.data.append(int(self._servo_index))
                req.data.append(int(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_CURRENT_LIMIT
            #need to disable servo torque
            servo_trq_msg = ServoTorqueCmd()
            servo_trq_msg.index = chr(self._current_servo_serial_index)
            servo_trq_msg.torque_enable = chr(0)
            self.servo_torque_pub_.publish(servo_trq_msg)
            rospy.sleep(0.5)

        rospy.loginfo('published message')
        rospy.loginfo('command: ' + str(req.command))
        rospy.loginfo('data: ' + str(req.data))
        rospy.wait_for_service('/set_board_config')
        try:
            res = self.set_board_config_client_(req)
            rospy.loginfo(bool(res.success))
        except rospy.ServiceException, e:
            print("/set_board_config service call failed: %s"%e)
