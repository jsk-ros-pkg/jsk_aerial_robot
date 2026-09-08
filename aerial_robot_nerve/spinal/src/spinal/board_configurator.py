import os
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP

import rospy
import rospkg
from spinal.srv import *
from spinal.msg import ServoInfo, ServoTorqueCmd

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import python_qt_binding as pyqt
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
import rosgraph
import distutils.util
import xml.etree.ElementTree as ET

SERVO_MODE_NAMES = {
    ServoInfo.MODE_DP: 'DP',
    ServoInfo.MODE_DV: 'DV',
    ServoInfo.MODE_DC: 'DC',
    ServoInfo.MODE_MIT: 'MIT',
    ServoInfo.MODE_EM: 'EM',
    ServoInfo.MODE_DEP: 'DEP',
    ServoInfo.MODE_CBP: 'CbP',
}
SERVO_INTERNAL_MODE_NAMES = {
    ServoInfo.SERVO_MODE_CURRENT: 'Current',
    ServoInfo.SERVO_MODE_VELOCITY: 'Velocity',
    ServoInfo.SERVO_MODE_POSITION: 'Position',
    ServoInfo.SERVO_MODE_EXTENDED_POSITION: 'Extended Position',
    ServoInfo.SERVO_MODE_CURRENT_BASED_POSITION: 'Current-based Position',
    ServoInfo.SERVO_MODE_UNKNOWN: 'Unknown',
}
SERVO_BUS_NAMES = {
    ServoInfo.BUS_RS485: 'R',
    ServoInfo.BUS_TTL: 'T',
    ServoInfo.BUS_UNKNOWN: 'Unknown',
}

# Parameters supported by SetDirectServoConfig.  board_id 0 is DirectServo;
# 1..14 select Neurons, and servo_index is local to that board.
ADDRESSED_SERVO_CONFIG_PARAMS = {
    'servo_feedback_frequency',
    'servo_target_watchdog',
    'bus_type',
    'mode',
    'mit_pd_si',
    'pid_gain',
    'profile_velocity',
    'send_data_flag',
    'current_limit',
    'dynamixel_ttl_rs485_mixed',
    'external_encoder_flag',
    'resolution[joint:servo]',
}

TORQUE_OFF_SERVO_CONFIG_PARAMS = {
    'bus_type',
    'mode',
    'mit_pd_si',
    'pid_gain',
    'current_limit',
    'external_encoder_flag',
    'resolution[joint:servo]',
}


def parse_servo_mode(text):
    mode = text.strip().upper()
    values = {name.upper(): value
              for value, name in SERVO_MODE_NAMES.items()
              if value != ServoInfo.MODE_EM}
    if mode not in values:
        raise ValueError('Input one of DP, DEP, DV, DC, MIT or CbP')
    return values[mode]


def parse_servo_bus(text):
    bus = text.strip().upper()
    if bus in ('R', 'RS485'):
        return ServoInfo.BUS_RS485
    if bus in ('T', 'TTL'):
        return ServoInfo.BUS_TTL
    raise ValueError('Input R/RS485 or T/TTL')


def parse_non_negative_decimal(text, name):
    try:
        value = Decimal(str(text).strip())
    except (InvalidOperation, ValueError):
        raise ValueError('{} must be a number'.format(name))
    if not value.is_finite() or value < 0:
        raise ValueError('{} must be non-negative and finite'.format(name))
    return value


def encode_mit_raw_gain(raw_gain, name):
    raw_gain = parse_non_negative_decimal(raw_gain, name)
    scale = ServoInfo.GAIN_DECIMAL_SCALE
    scaled = int((raw_gain * scale).to_integral_value(
        rounding=ROUND_HALF_UP))
    maximum = 32767 * scale + scale - 1
    if scaled > maximum:
        raise ValueError('{} exceeds the MCU MIT gain range'.format(name))
    integer = scaled // ServoInfo.GAIN_DECIMAL_SCALE
    decimal = scaled % ServoInfo.GAIN_DECIMAL_SCALE
    return integer, decimal


def controller_mit_scales(controller, group):
    def inherited_value(name, default=None):
        if name in controller:
            return controller[name]
        return group.get(name, default)

    angle_scale = inherited_value('angle_scale')
    velocity_scale = inherited_value('velocity_scale')
    torque_scale = inherited_value('torque_scale', 1.0)
    if angle_scale is None or velocity_scale is None:
        return None
    try:
        scales = {
            'angle': Decimal(str(angle_scale)),
            'velocity': Decimal(str(velocity_scale)),
            'torque': Decimal(str(torque_scale)),
        }
    except (InvalidOperation, ValueError):
        return None
    if any(not value.is_finite() or value <= 0
           for value in scales.values()):
        return None
    return scales


def mit_pd_raw_to_si(kp_integer, kp_decimal, kd_integer, kd_decimal,
                     scales):
    # BoardInfo transports the raw current/position and current/velocity
    # gains used by the MCU.  Board Configurator presents the equivalent
    # joint-side SI gains using the same scales as servo_bridge.
    gain_scale = Decimal(ServoInfo.GAIN_DECIMAL_SCALE)
    kp_raw = ((Decimal(kp_integer) * gain_scale + Decimal(kp_decimal)) /
              gain_scale)
    kd_raw = ((Decimal(kd_integer) * gain_scale + Decimal(kd_decimal)) /
              gain_scale)
    return (kp_raw * scales['torque'] / scales['angle'],
            kd_raw * scales['torque'] / scales['velocity'])


def format_mit_pd_si(kp_integer, kp_decimal, kd_integer, kd_decimal,
                     scales):
    kp_si, kd_si = mit_pd_raw_to_si(
        kp_integer, kp_decimal, kd_integer, kd_decimal, scales)
    return '{:.4f}, {:.4f}'.format(kp_si, kd_si)


def parse_mit_pd_si(text, scales):
    values = text.split(',')
    if len(values) != 2:
        raise ValueError('Input MIT Kp_si and Kd_si separated by a comma')
    kp_si = parse_non_negative_decimal(values[0], 'MIT Kp_si')
    kd_si = parse_non_negative_decimal(values[1], 'MIT Kd_si')
    return (encode_mit_raw_gain(
                kp_si * scales['angle'] / scales['torque'], 'MIT Kp_raw'),
            encode_mit_raw_gain(
                kd_si * scales['velocity'] / scales['torque'], 'MIT Kd_raw'))


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

        self.get_board_info_client_ = rospy.ServiceProxy(robot_ns + '/get_board_info', GetBoardInfo)
        self.set_board_config_client_ = rospy.ServiceProxy(robot_ns + '/set_board_config', SetBoardConfig)
        self.direct_servo_config_client_ = rospy.ServiceProxy(robot_ns + '/direct_servo_config', SetDirectServoConfig)
        self.servo_torque_pub_ = rospy.Publisher(robot_ns + '/servo/torque_enable', ServoTorqueCmd, queue_size = 1)

        from argparse import ArgumentParser
        parser = ArgumentParser()

        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: {}'.format(args))
            print('unknowns: {}'.format(unknowns))

        self._widget = QWidget()

        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('spinal'), 'resource', 'board_configurator.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('BoardConfigurator')

        self._board_id = None
        self._servo_index = None
        self._raw_servo_id = None
        self._current_servo_serial_index = None
        self._command = None

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

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.joint_id_name_map = {}
        self.joint_id_mit_scale_map = {}
        try:
            param_tree = rospy.get_param(robot_ns + "/servo_controller")

            for key in param_tree.keys():
                if isinstance(param_tree[key], dict):
                    for elem in [l for l in param_tree[key].keys() if 'controller' in l]:
                        controller = param_tree[key][elem]
                        servo_id = controller['id']
                        self.joint_id_name_map[servo_id] = controller['name']
                        scales = controller_mit_scales(
                            controller, param_tree[key])
                        if scales is not None:
                            self.joint_id_mit_scale_map[servo_id] = scales
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
                board.appendRow([QStandardItem('servo_pulley_skip_thresh'), QStandardItem(str(b.servo_pulley_skip_thresh))])
                board.appendRow([QStandardItem('servo_feedback_frequency'), QStandardItem(str(b.servo_feedback_frequency))])
                board.appendRow([QStandardItem('servo_target_watchdog'), QStandardItem(str(b.servo_target_watchdog_timeout))])
                servos = QStandardItem('servo (' + str(len(b.servos)) + ')')
                for j, s in enumerate(b.servos):
                    mit_scales = self.joint_id_mit_scale_map.get(
                        servo_serial_index)
                    servo = QStandardItem(str(j))
                    servo.appendRow([QStandardItem('servo_id'), QStandardItem(str(s.id))])
                    servo.appendRow([QStandardItem('servo_serial_index'), QStandardItem(str(servo_serial_index))])
                    servo.appendRow([QStandardItem('joint_name'), QStandardItem(str(self.joint_id_name_map.get(servo_serial_index)))])
                    servo_serial_index += 1
                    servo.appendRow([QStandardItem('configured'), QStandardItem(str(bool(s.configured)))])
                    servo.appendRow([QStandardItem('bus_type'), QStandardItem(SERVO_BUS_NAMES.get(s.bus_type, 'Unknown'))])
                    servo.appendRow([QStandardItem('mode'), QStandardItem(SERVO_MODE_NAMES.get(s.mode, 'Unknown'))])
                    servo.appendRow([QStandardItem('servo_mode'), QStandardItem(SERVO_INTERNAL_MODE_NAMES.get(s.servo_mode, 'Unknown'))])
                    servo.appendRow([
                        QStandardItem('mit_pd_si'),
                        QStandardItem(
                            format_mit_pd_si(
                                s.kp_integer, s.kp_decimal,
                                s.kd_integer, s.kd_decimal, mit_scales)
                            if mit_scales is not None else 'N/A')])
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

        except rospy.ServiceException as e:
            rospy.logerr("/get_board_info service call failed: %s"%e)

        self._widget.boardInfoTreeView.setColumnWidth(0, 250)
        self._widget.boardInfoTreeView.setColumnWidth(1, 160)
        self._widget.boardInfoTreeView.expandAll()
        self._widget.boardInfoTreeView.show()

    def treeClickedCallback(self):
        col = self._widget.boardInfoTreeView.currentIndex().column()
        row = self._widget.boardInfoTreeView.currentIndex().row()
        param = self._widget.boardInfoTreeView.currentIndex().sibling(row, 0).data()
        value = self._widget.boardInfoTreeView.currentIndex().sibling(row, 1).data()
        board_param_list = ['board_id', 'imu_send_data_flag',
                            'dynamixel_ttl_rs485_mixed',
                            'servo_pulley_skip_thresh',
                            'servo_feedback_frequency',
                            'servo_target_watchdog']
        servo_param_list = ['bus_type', 'mode', 'mit_pd_si',
                            'pid_gain', 'profile_velocity',
                            'current_limit', 'send_data_flag',
                            'external_encoder_flag',
                            'resolution[joint:servo]']
        if value and (param in servo_param_list):
            raw_servo_id = self._widget.boardInfoTreeView.currentIndex().sibling(0,1).data()
            servo_index = self._widget.boardInfoTreeView.currentIndex().parent().data()
            board_id = self._widget.boardInfoTreeView.currentIndex().parent().parent().parent().data()
            self._widget.paramLabel.setText('board_id: ' + board_id + ' servo_index: ' + servo_index + ' ' + param)
            self._board_id = board_id
            self._servo_index = servo_index
            self._raw_servo_id = raw_servo_id
            self._command = param
            self._current_servo_serial_index = int(self._widget.boardInfoTreeView.currentIndex().parent().child(1, 1).data())
        elif value and (param in board_param_list):
            board_id = self._widget.boardInfoTreeView.currentIndex().parent().data()
            self._widget.paramLabel.setText('board_id: ' + board_id + ' ' + param)
            self._board_id = board_id
            self._servo_index = None
            self._raw_servo_id = None
            self._current_servo_serial_index = None
            self._command = param
        else:
            self._board_id = None
            self._command = None
            self._servo_index = None
            self._raw_servo_id = None
            self._current_servo_serial_index = None
            self._widget.paramLabel.setText('')

    def disableServoTorque(self):
        servo_trq_msg = ServoTorqueCmd()
        servo_trq_msg.index = [self._current_servo_serial_index]
        servo_trq_msg.torque_enable = [0]
        self.servo_torque_pub_.publish(servo_trq_msg)
        rospy.sleep(0.5)

    def configureButtonCallback(self):
        if self._board_id is None:
            rospy.logerr("board id is not registered")
            return

        board_id = int(self._board_id)
        spinal_flag = board_id == 0
        addressed_servo_config = (
            self._command in ADDRESSED_SERVO_CONFIG_PARAMS)

        # DirectServo has no legacy board-management endpoint.  Neuron board
        # identity, IMU and pulley settings intentionally stay on
        # SetBoardConfig; all shared servo settings use the addressed service.
        if spinal_flag and not addressed_servo_config:
            rospy.logerr("This board parameter is not supported by DirectServo")
            return

        if addressed_servo_config:
            req = SetDirectServoConfigRequest()
            req.board_id = board_id
            req.servo_index = (int(self._servo_index)
                               if self._servo_index is not None else 0)
        else:
            req = SetBoardConfigRequest()
            req.data.append(board_id)

        if self._command == 'board_id':
            if spinal_flag:
                return
            try:
                req.data.append(int(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SLAVE_ID
        elif self._command == 'imu_send_data_flag':
            if spinal_flag:
                return
            try:
                req.data.append(distutils.util.strtobool(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_IMU_SEND_FLAG
        elif self._command == 'servo_feedback_frequency':
            try:
                req.data.append(int(self._widget.lineEdit.text()))
            except ValueError as e:
                rospy.logerr(str(e))
                return
            req.command = req.SET_SERVO_FEEDBACK_FREQUENCY
        elif self._command == 'servo_target_watchdog':
            try:
                timeout_ms = int(Decimal(self._widget.lineEdit.text()) * 1000)
                if timeout_ms < 0 or timeout_ms > 60000:
                    raise ValueError('Watchdog must be within 0..60 seconds')
                req.data.append(timeout_ms)
            except (InvalidOperation, ValueError) as e:
                rospy.logerr(str(e))
                return
            req.command = req.SET_SERVO_TARGET_WATCHDOG_TIMEOUT
        elif self._command == 'bus_type':
            try:
                req.data.append(parse_servo_bus(
                    self._widget.lineEdit.text()))
            except ValueError as e:
                rospy.logerr(str(e))
                return
            req.command = req.SET_SERVO_BUS_TYPE
        elif self._command == 'mode':
            try:
                req.data.append(parse_servo_mode(
                    self._widget.lineEdit.text()))
            except ValueError as e:
                rospy.logerr(str(e))
                return
            req.command = req.SET_SERVO_CONTROL_MODE
        elif self._command == 'mit_pd_si':
            try:
                mit_scales = self.joint_id_mit_scale_map.get(
                    self._current_servo_serial_index)
                if mit_scales is None:
                    raise ValueError(
                        'MIT SI conversion requires positive angle_scale, '
                        'velocity_scale and torque_scale')
                kp, kd = parse_mit_pd_si(
                    self._widget.lineEdit.text(), mit_scales)
                req.data.extend([
                    kp[0], kp[1], kd[0], kd[1]])
            except ValueError as e:
                rospy.logerr(str(e))
                return
            req.command = req.SET_SERVO_MIT_PD
        elif self._command == 'pid_gain':
            try:
                pid_gains = list(map(lambda x: int(x), self._widget.lineEdit.text().split(',')))
                if len(pid_gains) != 3:
                    raise ValueError('Input 3 gains(int)')
                req.data.extend(pid_gains)
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_PID_GAIN
        elif self._command == 'profile_velocity':
            try:
                req.data.append(int(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_PROFILE_VEL
        elif self._command == 'send_data_flag':
            try:
                req.data.append(distutils.util.strtobool(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_SEND_DATA_FLAG
        elif self._command == 'current_limit':
            try:
                req.data.append(int(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_CURRENT_LIMIT
        elif self._command == 'dynamixel_ttl_rs485_mixed':
            try:
                req.data.append(distutils.util.strtobool(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_DYNAMIXEL_TTL_RS485_MIXED
        elif self._command == 'servo_pulley_skip_thresh':
            try:
                req.data.append(int(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_PULLEY_SKIP_THRESH
        elif self._command == 'external_encoder_flag':
            try:
                req.data.append(distutils.util.strtobool(self._widget.lineEdit.text()))
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_EXTERNAL_ENCODER_FLAG
        elif self._command == 'resolution[joint:servo]':
            try:
                resolutions = list(map(lambda x: int(x), self._widget.lineEdit.text().split(':')))
                if len(resolutions) != 2:
                    raise ValueError('Input 2 resoultion(int)')
                req.data.extend(resolutions)
            except ValueError as e:
                print(e)
                return
            req.command = req.SET_SERVO_RESOLUTION_RATIO
        else:
            rospy.logerr("Unsupported board configuration parameter")
            return

        if self._command in TORQUE_OFF_SERVO_CONFIG_PARAMS:
            self.disableServoTorque()

        rospy.loginfo('published message')
        rospy.loginfo('command: ' + str(req.command))
        rospy.loginfo('data: ' + str(req.data))
        try:
            if addressed_servo_config:
                res = self.direct_servo_config_client_(req)
            else:
                res = self.set_board_config_client_(req)
            rospy.loginfo(bool(res.success))
            rospy.sleep(1)
            self.updateButtonCallback()
        except rospy.ServiceException as e:
            if addressed_servo_config:
                print("/direct_servo_config service call failed: {}".format(e))
            else:
                print("/set_board_config service call failed: {}".format(e))
