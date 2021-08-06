import os
import rospy
import rospkg
from spinal.srv import *
from spinal.msg import ServoTorqueCmd

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import python_qt_binding as pyqt
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
import rosgraph
import distutils.util
import xml.etree.ElementTree as ET

class BoardConfigurator(Plugin):

    def __init__(self, context):
        super(BoardConfigurator, self).__init__(context)

        self.setObjectName('BoardConfigurator')

        # search the robot prefix for topics
        master = rosgraph.Master('/rostopic')
        try:
            _, _, srvs = master.getSystemState()
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")
        board_info_srvs = [srv[0] for srv in srvs if '/get_board_info' in srv[0]]

        # choose the first robot name
        robot_ns = board_info_srvs[0].split('/get_board_info')[0]

        print robot_ns
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
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'BoardConfigurator.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('BoardConfigurator')

        self._widget.boardInfoUpdateButton.clicked.connect(self.updateButtonCallback)
        self._widget.boardInfoTreeView.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._widget.boardInfoTreeView.clicked.connect(self.treeClickedCallback)
        self._widget.configureButton.clicked.connect(self.configureButtonCallback)

        self.model = QStandardItemModel()
        self._widget.boardInfoTreeView.setModel(self.model)
        self._widget.boardInfoTreeView.setUniformRowHeights(True)
        self.model.setHorizontalHeaderLabels(['board ID', 'param'])
        self._widget.boardInfoTreeView.show()
        self._widget.setLayout(self._widget.gridLayout)
        self._widget.boardInfoTreeView.setEditTriggers(QAbstractItemView.NoEditTriggers)

        self._board_id = None
        self._servo_id = None
        self._command = None

        self._current_servo_serial_index = None

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

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

    def updateButtonCallback(self):
        try:
            res = self.get_board_info_client_()

            self.model = QStandardItemModel()
            self._widget.boardInfoTreeView.setModel(self.model)
            self._widget.boardInfoTreeView.setUniformRowHeights(True)

            self.model.setHorizontalHeaderLabels(['param', 'value'])
            servo_serial_index = 0
            for i, b in enumerate(res.boards):
                board = QStandardItem(str(b.slave_id))
                board.setFlags(board.flags() ^ Qt.ItemIsEditable)
                board.appendRow([QStandardItem('board_id'), QStandardItem(str(b.slave_id))])
                board.appendRow([QStandardItem('imu_send_data_flag'), QStandardItem(str(bool(b.imu_send_data_flag)))])
                board.appendRow([QStandardItem('dynamixel_ttl_rs485_mixed'), QStandardItem(str(bool(b.dynamixel_ttl_rs485_mixed)))])
                servos = QStandardItem('servo (' + str(len(b.servos)) + ')')
                for j, s in enumerate(b.servos):
                    servo = QStandardItem(str(j))
                    servo.appendRow([QStandardItem('servo_id'), QStandardItem(str(s.id))])
                    servo.appendRow([QStandardItem('servo_serial_index'), QStandardItem(str(servo_serial_index))])
                    servo.appendRow([QStandardItem('joint_name'), QStandardItem(str(self.joint_id_name_map.get(servo_serial_index)))])
                    servo_serial_index += 1
                    servo.appendRow([QStandardItem('pid_gain'), QStandardItem(str(s.p_gain) + ', ' + str(s.i_gain) + ', ' + str(s.d_gain))])
                    servo.appendRow([QStandardItem('profile_velocity'), QStandardItem(str(s.profile_velocity))])
                    servo.appendRow([QStandardItem('send_data_flag'), QStandardItem(str(bool(s.send_data_flag)))])
                    servo.appendRow([QStandardItem('current_limit'), QStandardItem(str(s.current_limit))])
                    servo.appendRow([QStandardItem('external_encoder_flag'), QStandardItem(str(bool(s.external_encoder_flag)))])
                    servo.appendRow([QStandardItem('resolution[joint:servo]'), QStandardItem(str(s.joint_resolution) + ' : ' + str(s.servo_resolution))])
                    servos.appendRow(servo)
                board.appendRow(servos)
                self.model.appendRow(board)
                # span container columns
                self._widget.boardInfoTreeView.setFirstColumnSpanned(i, self._widget.boardInfoTreeView.rootIndex(), True)

        except rospy.ServiceException, e:
            rospy.logerr("/get_board_info service call failed: %s"%e)

        self._widget.boardInfoTreeView.setColumnWidth(0, 250)
        self._widget.boardInfoTreeView.setColumnWidth(1, 70)
        self._widget.boardInfoTreeView.expandAll()
        self._widget.boardInfoTreeView.show()

    def treeClickedCallback(self):
        col = self._widget.boardInfoTreeView.currentIndex().column()
        row = self._widget.boardInfoTreeView.currentIndex().row()
        param = self._widget.boardInfoTreeView.currentIndex().sibling(row, 0).data()
        value = self._widget.boardInfoTreeView.currentIndex().sibling(row, 1).data()

        board_param_list = ['board_id', 'imu_send_data_flag', 'dynamixel_ttl_rs485_mixed']
        servo_param_list = ['pid_gain', 'profile_velocity', 'current_limit', 'send_data_flag', 'external_encoder_flag', 'resolution[joint:servo]']

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

    def configureButtonCallback(self):
        req = SetBoardConfigRequest()
        if self._board_id:
            req.data.append(int(self._board_id))
        else:
            rospy.logerr("board id is not registered")
            return

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
        elif self._command == 'dynamixel_ttl_rs485_mixed':
            try:
                req.data.append(distutils.util.strtobool(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_DYNAMIXEL_TTL_RS485_MIXED
        elif self._command == 'external_encoder_flag':
            try:
                req.data.append(int(self._servo_index))
                req.data.append(distutils.util.strtobool(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_EXTERNAL_ENCODER_FLAG
        elif self._command == 'resolution[joint:servo]':
            try:
                req.data.append(int(self._servo_index))
                resolutions = map(lambda x: int(x), self._widget.lineEdit.text().split(':'))
                if len(resolutions) != 2:
                    raise ValueError('Input 2 resoultion(int)')
                req.data.extend(resolutions)
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_RESOLUTION_RATIO


        rospy.loginfo('published message')
        rospy.loginfo('command: ' + str(req.command))
        rospy.loginfo('data: ' + str(req.data))
        try:
            res = self.set_board_config_client_(req)
            rospy.loginfo(bool(res.success))
            rospy.sleep(1)
            self.updateButtonCallback()
        except rospy.ServiceException, e:
            print "/set_board_config service call failed: %s"%e
