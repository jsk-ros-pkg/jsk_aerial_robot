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


class MyTableModel(QAbstractTableModel):
    def __init__(self, list, headers = [], parent = None):
        QAbstractTableModel.__init__(self, parent)
        self.list = list
        self.headers = headers

    def rowCount(self, parent):
        return len(self.list)

    def columnCount(self, parent):
        return len(self.list[0])

    def flags(self, index):
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def data(self, index, role):
        if role == Qt.EditRole:
            row = index.row()
            column = index.column()
            return self.list[row][column]

        if role == Qt.DisplayRole:
            row = index.row()
            column = index.column()
            value = self.list[row][column]
            return value

    def setData(self, index, value, role = Qt.EditRole):
        if role == Qt.EditRole:
            row = index.row()
            column = index.column()
            self.list[row][column] = value
            self.dataChanged.emit(index, index)
            return True
        return False

    def headerData(self, section, orientation, role):

        if role == Qt.DisplayRole:

            if orientation == Qt.Horizontal:

                if section < len(self.headers):
                    return self.headers[section]
                else:
                    return("not implemented")
            else:
                return("servo %d" % section)

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
        # self._widget.boardInfoTreeView.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._widget.servoTableView.clicked.connect(self.tableClickedCallback)
        # self._widget.configureButton.clicked.connect(self.configureButtonCallback)
        self._widget.servoTableView.verticalHeader().sectionClicked.connect(self.verticalHeaderClickedCallback)

        self._table_data = []
        self._board_id = None
        self._servo_id = None
        self._command = None

        self._current_servo_serial_index = None

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.headers = ["board", "id", "angle", "temperature", "load", "error", "pid_gains", "profile_velocity", "current_limit", "send_data_flag", "servo on", "servo off", "calibration"]

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

    def verticalHeaderClickedCallback(self):
        row = self._widget.servoTableView.currentIndex().row()
        print(row)
        


    def servoStateCallback(self, msg):
        for s in msg.servos:
            self._table_data[s.index][2] = s.angle
            self._table_data[s.index][3] = s.temp
            self._table_data[s.index][4] = s.load
            self._table_data[s.index][5] = s.error

    def update(self):
        model = MyTableModel(self._table_data, self.headers)
        self._widget.servoTableView.setModel(model)
        btn_cell = QPushButton("servo on", self._widget)
        i = 1
        func = lambda x=None: print(x)
        btn_cell.clicked.connect(partial(func, x=i))
        self._widget.servoTableView.setIndexWidget(self._widget.servoTableView.model().index(0, 0), btn_cell)
        btn_cell = QPushButton("servo off", self._widget)
        i = 2
        btn_cell.clicked.connect(partial(func, x=i))
        self._widget.servoTableView.setIndexWidget(self._widget.servoTableView.model().index(0, 1), btn_cell)

        self._widget.servoTableView.show()

    def servoButtonClicked(self):
        print("hoge")

    def updateButtonCallback(self):
        rospy.wait_for_service('/get_board_info')
        try:
            res = self.get_board_info_client_()

            servo_index = 0
            self._table_data = []
            for b in res.boards:
                for s in b.servos:
                    rowData = []
                    rowData.append(b.slave_id)
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
        col = self._widget.servoTableView.currentIndex().column()
        row = self._widget.servoTableView.currentIndex().row()
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
