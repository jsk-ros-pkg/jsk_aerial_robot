from __future__ import print_function
import os
import rospy
import rospkg
from spinal.srv import *
from spinal.msg import ServoTorqueCmd, ServoStates, ServoTorqueStates

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import python_qt_binding as pyqt
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
import distutils.util
import rosgraph
from functools import partial
from operator import add
import xml.etree.ElementTree as ET


class ServoMonitor(Plugin):

    def __init__(self, context):
        super(ServoMonitor, self).__init__(context)

        self.setObjectName('ServoMonitor')

        # search the robot prefix for topics
        master = rosgraph.Master('/rostopic')
        try:
            _, _, srvs = master.getSystemState()
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")
        board_info_srvs = [srv[0] for srv in srvs if '/get_board_info' in srv[0]]

        # choose the first robot name
        robot_ns = board_info_srvs[0].split('/get_board_info')[0]

        self.get_board_info_client_ = rospy.ServiceProxy(robot_ns + '/get_board_info', GetBoardInfo)
        self.set_board_config_client_ = rospy.ServiceProxy(robot_ns + '/set_board_config', SetBoardConfig)
        self.servo_torque_pub_ = rospy.Publisher(robot_ns + '/servo/torque_enable', ServoTorqueCmd, queue_size = 1)

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
        boardRebootAction = QAction("board reboot", self._widget.servoTableWidget)
        boardRebootAction.triggered.connect(self.boardReboot)
        self._widget.servoTableWidget.addAction(boardRebootAction)

        self._table_data = []
        self._board_id = None
        self._servo_id = None
        self._command = None

        self._current_servo_serial_index = None

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._headers = ["torque", "joint name", "board", "index", "id", "angle", "temperature", "load", "error", "pid_gains", "profile_velocity", "current_limit", "send_data_flag"]

        self._widget.setLayout(self._widget.gridLayout)
        self.joint_id_name_map = {}
        try:
            param_tree = rospy.get_param(robot_ns + "/servo_controller")
            ctrl_pub_topic = 'servo/target_states'

            for key in param_tree.keys():
                if param_tree[key]['ctrl_pub_topic'] == ctrl_pub_topic:
                    for elem in [l for l in param_tree[key].keys() if 'controller' in l]:
                        self.joint_id_name_map[param_tree[key][elem]['id']] = param_tree[key][elem]['name']
        except:
            rospy.loginfo("robot info not found")

        self.updateButtonCallback()
        self.servo_state_sub_ = rospy.Subscriber(robot_ns + '/servo/states', ServoStates, self.servoStateCallback)
        self.servo_torque_state_sub_ = rospy.Subscriber(robot_ns + '/servo/torque_states', ServoTorqueStates, self.servoTorqueStatesCallback)

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
        if servo_index == -1:
            rospy.logerr("No servo exists")
            return
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
        if servo_num == 0:
            rospy.logerr("No servo exists")
            return
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
        if servo_index == -1:
            rospy.logerr("No servo exists")
            return

        req = SetBoardConfigRequest()
        req.data.append(int(self._widget.servoTableWidget.item(servo_index, self._headers.index("board")).text())) #board id
        req.data.append(int(self._widget.servoTableWidget.item(servo_index, self._headers.index("index")).text())) #servo index

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
        try:
            res = self.set_board_config_client_(req)
            rospy.loginfo(bool(res.success))
        except rospy.ServiceException, e:
            print("/set_board_config service call failed: %s"%e)

    def boardReboot(self):
        servo_index = self._widget.servoTableWidget.currentIndex().row()
        if servo_index == -1:
            rospy.logerr("No servo exists")
            return

        req = SetBoardConfigRequest()
        req.data.append(int(self._widget.servoTableWidget.item(servo_index, self._headers.index("board")).text())) #board id
        req.command = req.REBOOT

        rospy.loginfo('published message')
        rospy.loginfo('command: ' + str(req.command))
        rospy.loginfo('data: ' + str(req.data))
        try:
            res = self.set_board_config_client_(req)
            rospy.loginfo(bool(res.success))
        except rospy.ServiceException, e:
            print("/set_board_config service call failed: %s"%e)

    def error2string(self, error):
        error_list = []
        if error & 0b10000000:
            error_list.append('Encoder Connection Error')
        if error & 0b1000000:
            error_list.append('Resoluation Ratio Error')
        if error & 0b100000:
            error_list.append('Overload Error')
        if error & 0b10000:
            error_list.append('Electrical Shock Error')
        if error & 0b1000:
            error_list.append('Motor Encoder Error')
        if error & 0b100:
            error_list.append('Overheating Error')
        if error & 0b1:
            error_list.append('Input Voltage Error')

        if error_list:
            return reduce(lambda a, b: a + ', ' + b, error_list)
        else:
            return 'No Error'

    def servoStateCallback(self, msg):
        for s in msg.servos:
            self._table_data[s.index][self._headers.index("angle")] = s.angle
            self._table_data[s.index][self._headers.index("temperature")] = s.temp
            self._table_data[s.index][self._headers.index("load")] = s.load
            self._table_data[s.index][self._headers.index("error")] = self.error2string(int(s.error))

    def servoTorqueStatesCallback(self, msg):
        for i, s in enumerate(msg.torque_enable):
            self._table_data[i][self._headers.index("torque")] = "on" if bool(ord(s)) else "off"

    def update(self):
        if not self._table_data:
            return
        self._widget.servoTableWidget.setRowCount(len(self._table_data))
        self._widget.servoTableWidget.setColumnCount(len(self._table_data[0]))
        for i in range(len(self._table_data)):
            for j in range(len(self._table_data[0])):
                item = QTableWidgetItem(str(self._table_data[i][j]))

                self._widget.servoTableWidget.setItem(i, j, item)
                if j == 0 and item.text() == "on":
                    item.setBackground(Qt.cyan)
                elif j == 0 and item.text() == "off":
                    item.setBackground(Qt.gray)

        for i, h in enumerate(self._headers):
            item = QTableWidgetItem(str(h))
            self._widget.servoTableWidget.setHorizontalHeaderItem(i, item)

        for i in range(len(self._table_data)):
            item = QTableWidgetItem('servo' + str(i))
            self._widget.servoTableWidget.setVerticalHeaderItem(i, item)

        self._widget.servoTableWidget.resizeColumnsToContents()
        self._widget.servoTableWidget.show()

    def updateButtonCallback(self):
        try:
            res = self.get_board_info_client_()

            servo_index = 0
            self._table_data = []
            for b in res.boards:
                for i, s in enumerate(b.servos):
                    rowData = []
                    rowData.append(None)
                    rowData.append(self.joint_id_name_map.get(servo_index))
                    servo_index += 1
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
            rospy.logerr("/get_board_info service call failed: %s"%e)
