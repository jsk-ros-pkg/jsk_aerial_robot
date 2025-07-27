#!/usr/bin/env python

import argparse
from distutils.version import LooseVersion
import os
import sys
import copy

import matplotlib
from matplotlib.collections import LineCollection, PathCollection, PolyCollection
from matplotlib.colors import colorConverter
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D  # <-- Note the capitalization!
import numpy as np
import python_qt_binding
from python_qt_binding import loadUi
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *

import rospkg
import rospy
from spinal.msg import Imu
from spinal.srv import *
from tf.transformations import euler_from_quaternion
import rosgraph
from rqt_gui_py.plugin import Plugin
from rqt_plot.rosplot import ROSData, RosPlotException
from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common.topic_helpers import is_slot_numeric

try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg \
        as FigureCanvas
except ImportError:
    # work around bug in dateutil
    import thread
    sys.modules['_thread'] = thread
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg \
        as FigureCanvas
try:
    from matplotlib.backends.backend_qt5agg \
        import NavigationToolbar2QTAgg as NavigationToolbar
except ImportError:
    from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT \
        as NavigationToolbar

class MatDataPlot3D(QWidget):
    class Canvas(FigureCanvas):
        def __init__(self, parent=None, limit=100, unit=''):
            super(MatDataPlot3D.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(1,1,1, projection='3d')

            self.axes.set_xlabel('X ' + unit)
            self.axes.set_xlim3d(-limit, limit)
            self.axes.set_ylabel('Y ' + unit)
            self.axes.set_ylim3d(-limit, limit)
            self.axes.set_zlabel('Z ' + unit)
            self.axes.set_zlim3d(-limit, limit)

            self.x = np.array([])
            self.y = np.array([])
            self.z = np.array([])
            self.axes.scatter(self.x, self.y, self.z)

            self.figure.tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()

        def update_sample(self, x, y, z):
            self.x = np.append(self.x, x)
            self.y = np.append(self.y, y)
            self.z = np.append(self.z, z)

        def clear_sample(self):
            self.x = np.array([])
            self.y = np.array([])
            self.z = np.array([])

        def resizeEvent(self, event):
            super(MatDataPlot3D.Canvas, self).resizeEvent(event)
            self.figure.tight_layout()

    def __init__(self, parent=None, limit=100, unit=''):
        super(MatDataPlot3D, self).__init__(parent)
        self._canvas = MatDataPlot3D.Canvas(parent, limit, unit)
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)
        self._draw_flag = False

    def update_sample(self, x, y, z):
        self._canvas.update_sample(x, y, z)

    def clear_sample(self):
        self._canvas.clear_sample()

    def clear_canvas(self):
        self._canvas.axes.cla()
        self._canvas.draw()

    def redraw(self):

        if len(self._canvas.x) == 0:
            return

        self._canvas.axes.grid(True, color='gray')
        self._canvas.axes.cla()

        # simple mutex
        if len(self._canvas.x) == len(self._canvas.y) == len(self._canvas.z):
            self._canvas.axes.scatter(self._canvas.x, self._canvas.y, self._canvas.z, s = 20, c = 'blue')
            self._canvas.draw()

class IMUCalibWidget(QWidget):
    def __init__(self, limit=100, unit='[mG]'):
        super(IMUCalibWidget, self).__init__()
        self.setObjectName('ImuCalibWidget')

        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('spinal'), 'resource', 'imu_calibration.ui')

        loadUi(ui_file, self)
        self.mag_data_plot = MatDataPlot3D(self, limit, unit)
        self.mag_view_layout.addWidget(self.mag_data_plot)
        self.gyro_start_calib_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.gyro_stop_calib_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.acc_start_calib_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.acc_stop_calib_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.mag_start_calib_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.mag_stop_calib_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.mag_start_lsm_calib_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.mag_stop_lsm_calib_button.setIcon(QIcon.fromTheme('media-playback-stop'))


        self.mag_view_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.mag_clear_button.setIcon(QIcon.fromTheme('edit-clear'))

        # init and start update timer for imu data and mag plot
        self.update_mag_plot_timer = QTimer(self)
        self.update_mag_plot_timer.timeout.connect(self.update_mag_plot)
        self.update_mag_plot_timer.start(100)

        self.update_imu_data_timer = QTimer(self)
        self.update_imu_data_timer.timeout.connect(self.update_imu_data)
        self.update_imu_data_timer.start(100)

        # search the robot prefix for topics
        master = rosgraph.Master('/rostopic')
        try:
            _, _, srvs = master.getSystemState()
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")
        imu_calib_srvs = [srv[0] for srv in srvs if '/imu_calib' in srv[0]]

        # choose the first robot name
        robot_ns = imu_calib_srvs[0].split('/imu_calib')[0]

        self.imu_sub = rospy.Subscriber(robot_ns + '/imu', Imu, self.imu_callback)
        self.imu_calib_data_client = rospy.ServiceProxy(robot_ns + '/imu_calib', ImuCalib)

        self.imu_stamp = rospy.get_time()
        self.mag_view_start_flag = False
        self.mag_view_clear_flag = False

        self.mag_declination_client = rospy.ServiceProxy(robot_ns + '/mag_declination', MagDeclination)

        self.calib_data_len = 12 # gyro (3) + acc (3) + mag (6)
        self.common_headers = ["value", "bias"]
        self.mag_headers = copy.copy(self.common_headers + ["scale"])
        self.gyro_table_data = [] # self.common_headers
        self.acc_table_data = []  # self.common_headers
        self.mag_table_data = []  # self.mag_headers
        self.att_table_data = []

        # get imu calibration data
        self.update_imu_calib_data()
        self.update_mag_declination()

    def imu_callback(self, msg):

        if len(self.gyro_table_data) == 0 or len(self.acc_table_data) == 0 or len(self.mag_table_data) == 0:
            return

        if rospy.get_time() - self.imu_stamp < 0.1: #hard-coding
            return

        self.gyro_table_data[0][self.common_headers.index("value")] = "{0:.5f}".format(msg.gyro[0]) + ', ' + "{0:.5f}".format(msg.gyro[1]) + ', ' + "{0:.5f}".format(msg.gyro[2])

        self.acc_table_data[0][self.common_headers.index("value")] = "{0:.5f}".format(msg.acc[0]) + ', ' + "{0:.5f}".format(msg.acc[1]) + ', ' + "{0:.5f}".format(msg.acc[2])

        self.mag_table_data[0][self.common_headers.index("value")] = "{0:.5f}".format(msg.mag[0]) + ', ' + "{0:.5f}".format(msg.mag[1]) + ', ' + "{0:.5f}".format(msg.mag[2])

        rpy = euler_from_quaternion(msg.quaternion)
        self.att_table_data[0][0] = "{0:.5f}".format(rpy[0]) + ', ' + "{0:.5f}".format(rpy[1]) + ', ' + "{0:.5f}".format(rpy[2])


        self.imu_stamp = rospy.get_time();

        if self.mag_view_clear_flag:
            self.mag_data_plot.clear_sample()

        if self.mag_view_start_flag:
            self.mag_data_plot.update_sample(msg.mag[0],
                                             msg.mag[1],
                                             msg.mag[2])
    def update_mag_plot(self):
        self.mag_data_plot.redraw()

        # clear canvas here for consistence
        if self.mag_view_clear_flag:
            self.mag_view_clear_flag = False
            self.mag_data_plot.clear_canvas()

    def update_imu_data(self):
        if len(self.gyro_table_data) == 0 or len(self.acc_table_data) == 0 or len(self.mag_table_data) == 0:
            return

        # Att
        self.att_table_widget.setRowCount(len(self.att_table_data))
        self.att_table_widget.setColumnCount(1)

        for i in range(len(self.att_table_data)):
            item = QTableWidgetItem(self.att_table_data[i][0])
            self.att_table_widget.setItem(i, 0, item)

        item = QTableWidgetItem(str("Euler Angles [RPY]"))
        self.att_table_widget.setHorizontalHeaderItem(i, item)

        for i in range(len(self.att_table_data)):
            item = QTableWidgetItem('IMU' + str(i))
            self.att_table_widget.setVerticalHeaderItem(i, item)

        self.att_table_widget.resizeColumnsToContents()
        self.att_table_widget.show()

        # Gyro
        self.gyro_table_widget.setRowCount(len(self.gyro_table_data))
        self.gyro_table_widget.setColumnCount(len(self.common_headers))

        for i in range(len(self.gyro_table_data)):
            for j in range(len(self.gyro_table_data[0])):
                item = QTableWidgetItem(self.gyro_table_data[i][j])
                self.gyro_table_widget.setItem(i, j, item)

        for i, h in enumerate(self.common_headers):
            item = QTableWidgetItem(str(h))
            self.gyro_table_widget.setHorizontalHeaderItem(i, item)

        for i in range(len(self.gyro_table_data)):
            item = QTableWidgetItem('IMU' + str(i))
            self.gyro_table_widget.setVerticalHeaderItem(i, item)

        self.gyro_table_widget.resizeColumnsToContents()
        self.gyro_table_widget.show()

        # Acc
        self.acc_table_widget.setRowCount(len(self.acc_table_data))
        self.acc_table_widget.setColumnCount(len(self.common_headers))

        for i in range(len(self.acc_table_data)):
            for j in range(len(self.acc_table_data[0])):
                item = QTableWidgetItem(self.acc_table_data[i][j])
                self.acc_table_widget.setItem(i, j, item)

        for i, h in enumerate(self.common_headers):
            item = QTableWidgetItem(str(h))
            self.acc_table_widget.setHorizontalHeaderItem(i, item)

        for i in range(len(self.acc_table_data)):
            item = QTableWidgetItem('IMU' + str(i))
            self.acc_table_widget.setVerticalHeaderItem(i, item)

        self.acc_table_widget.resizeColumnsToContents()
        self.acc_table_widget.show()

        # Mag
        self.mag_table_widget.setRowCount(len(self.mag_table_data))
        self.mag_table_widget.setColumnCount(len(self.mag_headers))

        for i in range(len(self.mag_table_data)):
            for j in range(len(self.mag_table_data[0])):
                item = QTableWidgetItem(self.mag_table_data[i][j])
                self.mag_table_widget.setItem(i, j, item)

        for i, h in enumerate(self.mag_headers):
            item = QTableWidgetItem(str(h))
            self.mag_table_widget.setHorizontalHeaderItem(i, item)

        for i in range(len(self.mag_table_data)):
            item = QTableWidgetItem('IMU' + str(i))
            self.mag_table_widget.setVerticalHeaderItem(i, item)

        self.mag_table_widget.resizeColumnsToContents()
        self.mag_table_widget.show()

    def update_imu_calib_data(self):
        try:
            req = ImuCalibRequest()
            req.command = req.GET_CALIB_DATA
            res = self.imu_calib_data_client(req)

            self.gyro_table_data = []
            self.acc_table_data = []
            self.mag_table_data = []
            self.att_table_data = []

            for i in range(len(res.data) // self.calib_data_len):
                gyro_data = []
                gyro_data.extend([None])
                gyro_data.append("{0:.5f}".format(res.data[i * self.calib_data_len + 0]) + ', ' + "{0:.5f}".format(res.data[i * self.calib_data_len + 1]) + ', ' + "{0:.5f}".format(res.data[i * self.calib_data_len + 2])) # bias
                self.gyro_table_data.append(gyro_data)

                acc_data = []
                acc_data.extend([None])
                acc_data.append("{0:.5f}".format(res.data[3 + i * self.calib_data_len]) + ', ' + "{0:.5f}".format(res.data[4 + i * self.calib_data_len]) + ', ' + "{0:.5f}".format(res.data[i * self.calib_data_len + 5])) # bias
                self.acc_table_data.append(acc_data)

                mag_data = []
                mag_data.extend([None])
                mag_data.append("{0:.5f}".format(res.data[6 + i * self.calib_data_len]) + ', ' + "{0:.5f}".format(res.data[7 + i * self.calib_data_len]) + ', ' + "{0:.5f}".format(res.data[8 + i * self.calib_data_len])) # bias
                mag_data.append("{0:.5f}".format(res.data[9 + i * self.calib_data_len]) + ', ' + "{0:.5f}".format(res.data[10 + i * self.calib_data_len]) + ', ' + "{0:.5f}".format(res.data[11 + i * self.calib_data_len])) # scale
                self.mag_table_data.append(mag_data)

                att_data = []
                att_data.extend([None])
                self.att_table_data.append(att_data)

            # change to debug log
            rospy.logdebug("number of imu: %d", len(self.gyro_table_data))

        except rospy.ServiceException as e:
            rospy.logerr("/imu_calib service call failed: %s"%e)


    def reset_imu_calib_data(self):
        try:
            req = ImuCalibRequest()
            req.command = req.RESET_CALIB_DATA
            res = self.imu_calib_data_client(req)
        except rospy.ServiceException as e:
            rospy.logerr("/imu_calib service call failed: %s"%e)


        rospy.sleep(1.0)
        self.update_imu_calib_data()

    def save_imu_calib_data(self):
        try:
            req = ImuCalibRequest()
            req.command = req.SAVE_CALIB_DATA
            res = self.imu_calib_data_client(req)
        except rospy.ServiceException as e:
            rospy.logerr("/imu_calib service call failed: %s"%e)

        rospy.sleep(2.0)
        self.update_imu_calib_data()

    def gyro_calib(self, flag):
        try:
            req = ImuCalibRequest()
            req.command = req.CALIB_GYRO
            req.data = []
            req.data.append(flag) # start or stop
            req.data.append(0) # do calibration until the stop trigger
            res = self.imu_calib_data_client(req)
        except rospy.ServiceException as e:
            rospy.logerr("/imu_calib service call failed: %s"%e)

        if not flag: # stop calibration
            rospy.sleep(0.5)
            self.update_imu_calib_data()

    def acc_calib(self, flag):
        try:
            req = ImuCalibRequest()
            req.command = req.CALIB_ACC
            req.data = []
            req.data.append(flag) # start or stop
            req.data.append(0) # do calibration until the stop trigger
            res = self.imu_calib_data_client(req)
        except rospy.ServiceException as e:
            rospy.logerr("/imu_calib service call failed: %s"%e)

        if not flag: # stop calibration
            rospy.sleep(0.5)
            self.update_imu_calib_data()

    def mag_calib(self, flag):
        try:
            req = ImuCalibRequest()
            req.command = req.CALIB_MAG
            req.data = []
            req.data.append(flag) # start or stop
            req.data.append(0) # do calibration until the stop trigger
            res = self.imu_calib_data_client(req)
        except rospy.ServiceException as e:
            rospy.logerr("/imu_calib service call failed: %s"%e)

        self.mag_view_start_flag = flag # for visualization

        rospy.sleep(1.0)

        if flag:  # start calibration
            self.mag_view_clear_flag = True # clear the sample with certain delay
        else: # stop calibration
            self.update_imu_calib_data()

    def mag_lsm_calib(self, flag):
        try:
            # Rest Mag calib data first
            req = ImuCalibRequest()
            req.command = req.SEND_CALIB_DATA
            req.data = []
            req.data.append(0) # hard-coding: imu0
            req.data.append(req.CALIB_MAG)
            req.data.extend([0, 0, 0, 1, 1, 1]) # mag bias and scale
            print(req.data)
            res = self.imu_calib_data_client(req)
        except rospy.ServiceException as e:
            rospy.logerr("/imu_calib service call failed: %s"%e)

        self.mag_view_start_flag = flag # for visualization

        rospy.sleep(1.0)

        if flag:  # start calibration
            self.mag_view_clear_flag = True # clear the sample with certain delay
            self.update_imu_calib_data()
        else: # stop calibration
            # start least-squares method.
            bias, scale = self.mag_least_squares_method()

            try:
                # Rest Mag calib data first
                req = ImuCalibRequest()
                req.command = req.SEND_CALIB_DATA
                req.data = []
                req.data.append(0) # hard-coding: imu0
                req.data.append(req.CALIB_MAG)
                req.data.extend(bias)
                req.data.extend(scale) # mag bias and scale
                res = self.imu_calib_data_client(req)
            except rospy.ServiceException as e:
                rospy.logerr("/imu_calib service call failed: %s"%e)

            rospy.sleep(0.5)
            self.update_imu_calib_data()

    def mag_least_squares_method(self):

        xyz = np.matrix([self.mag_data_plot._canvas.x, self.mag_data_plot._canvas.y, self.mag_data_plot._canvas.z]).T
        rospy.loginfo('Starting least-squared based magnetometer calibration with %d samples'%(len(self.mag_data_plot._canvas.x)))

        #compute the vectors [ x^2 y^2 z^2 2*x*y 2*y*z 2*x*z x y z 1] for every sample
        # the result for the x*y y*z and x*z components should be divided by 2
        xyz2 = np.power(xyz,2)
        xy = np.multiply(xyz[:,0],xyz[:,1])
        xz = np.multiply(xyz[:,0],xyz[:,2])
        yz = np.multiply(xyz[:,1],xyz[:,2])

        # build the data matrix
        A = np.bmat('xyz2 xy xz yz xyz')

        b = 1.0*np.ones((xyz.shape[0],1))

        # solve the system Ax = b
        q,res,rank,sing = np.linalg.lstsq(A,b)

        # build scaled ellipsoid quadric matrix (in homogeneous coordinates)
        A = np.matrix([[q[0][0],0.5*q[3][0],0.5*q[4][0],0.5*q[6][0]],
                    [0.5*q[3][0],q[1][0],0.5*q[5][0],0.5*q[7][0]],
                    [0.5*q[4][0],0.5*q[5][0],q[2][0],0.5*q[8][0]],
                    [0.5*q[6][0],0.5*q[7][0],0.5*q[8][0],-1]])

        # build scaled ellipsoid quadric matrix (in regular coordinates)
        Q = np.matrix([[q[0][0],0.5*q[3][0],0.5*q[4][0]],
                    [0.5*q[3][0],q[1][0],0.5*q[5][0]],
                    [0.5*q[4][0],0.5*q[5][0],q[2][0]]])

        # obtain the centroid of the ellipsoid
        x0 = np.linalg.inv(-1.0*Q) * np.matrix([0.5*q[6][0],0.5*q[7][0],0.5*q[8][0]]).T

        # translate the ellipsoid in homogeneous coordinates to the center
        T_x0 = np.matrix(np.eye(4))
        T_x0[0,3] = x0[0]; T_x0[1,3] = x0[1]; T_x0[2,3] = x0[2];
        A = T_x0.T*A*T_x0

        # rescale the ellipsoid quadric matrix (in regular coordinates)
        Q = Q*(-1.0/A[3,3])

        # take the cholesky decomposition of Q. this will be the matrix to transform
        # points from the ellipsoid to a sphere, after correcting for the offset x0
        L = np.eye(3)

        # deprecated to calcualte the transformation matrix from ellipsoid to sphere,
        # since the detorsion is very small for most of the case.
        '''
        try:
            L = np.linalg.cholesky(Q).transpose()
            L = L / L[1,1] # normalized with y
        except Exception,e:
            rospy.loginfo(str(e))
            L = np.eye(3)
        '''

        rospy.loginfo("Magnetometer offset:\n %s",x0)
        rospy.loginfo("Magnetometer Calibration Matrix:\n %s",L)

        # calibrate the sample data
        self.mag_data_plot._canvas.x = [i - x0[0] for i in self.mag_data_plot._canvas.x]
        self.mag_data_plot._canvas.y = [i - x0[1] for i in self.mag_data_plot._canvas.y]
        self.mag_data_plot._canvas.z = [i - x0[2] for i in self.mag_data_plot._canvas.z]

        '''
        for i in range(self.mag_data_plot._canvas.x):
            self.mag_data_plot._canvas.x[i] = self.mag_data_plot._canvas.x[i] - x0[0]
            self.mag_data_plot._canvas.y[i] = self.mag_data_plot._canvas.y[i] - x0[1]
            self.mag_data_plot._canvas.z[i] = self.mag_data_plot._canvas.z[i] - x0[2]

        print (self.mag_data_plot._canvas.x)
        '''

        return x0, np.diag(L)

    def mag_dec_configure(self):
        try:
            req = MagDeclinationRequest()
            req.command = req.SET_DECLINATION
            req.data = float(self.mag_dec_line_edit.text())
            res = self.mag_declination_client(req)

            #print(float(self.mag_dec_line_edit.text()))
        except rospy.ServiceException as e:
            rospy.logerr("/mag_declination service for set decalination call failed: %s"%e)

        rospy.sleep(3.0) # wait for the flash write

        self.update_mag_declination()

    def update_mag_declination(self):
        try:
            req = MagDeclinationRequest()
            req.command = req.GET_DECLINATION
            res = self.mag_declination_client(req)

            self.mag_dec.setText('Magnetic Declination: ' + str(res.data))
        except rospy.ServiceException as e:
            rospy.logerr("/mag_declination service for get decalination call failed: %s"%e)


    @Slot()
    def on_update_button_clicked(self):
        self.update_imu_calib_data()
        self.update_mag_declination()

    @Slot()
    def on_reset_button_clicked(self):
        self.reset_imu_calib_data()

    @Slot()
    def on_save_button_clicked(self):
        self.save_imu_calib_data()

    @Slot()
    def on_gyro_start_calib_button_clicked(self):
        self.gyro_calib(True)

    @Slot()
    def on_gyro_stop_calib_button_clicked(self):
        self.gyro_calib(False)

    @Slot()
    def on_acc_start_calib_button_clicked(self):
        self.acc_calib(True)

    @Slot()
    def on_acc_stop_calib_button_clicked(self):
        self.acc_calib(False)

    @Slot()
    def on_mag_start_calib_button_clicked(self):
        self.mag_calib(True)

    @Slot()
    def on_mag_stop_calib_button_clicked(self):
        self.mag_calib(False)

    @Slot()
    def on_mag_start_lsm_calib_button_clicked(self):
        self.mag_lsm_calib(True)

    @Slot()
    def on_mag_stop_lsm_calib_button_clicked(self):
        self.mag_lsm_calib(False)

    @Slot()
    def on_mag_declination_configure_button_clicked(self):
        self.mag_dec_configure()

    @Slot(bool)
    def on_mag_view_button_clicked(self, checked):
        if checked:
            self.mag_view_start_flag = True
        else:
            self.mag_view_start_flag = False

    @Slot()
    def on_mag_clear_button_clicked(self):
        self.mag_view_clear_flag = True

class ImuCalibrator(Plugin):
    def __init__(self, context):
        super(ImuCalibrator, self).__init__(context)
        self.setObjectName('ImuCalibrator')

        from argparse import ArgumentParser
        parser = ArgumentParser()

        parser.add_argument(
            "-q", "--quiet", dest="quiet", action="store_true",
            help="Put plugin in silent mode")
        parser.add_argument(
            '-L', '--limit', dest='limit', action="store",
            help='the bound of plot', default=100, type=int)
        parser.add_argument(
            '-u', '--unit', dest='unit', action="store",
            help='the unit of 3D vector', default='', type=str)

        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: {}'.format(args))
            print('unknowns: {}'.format(unknowns))


        self._widget = IMUCalibWidget(args.limit, args.unit)
        context.add_widget(self._widget)

