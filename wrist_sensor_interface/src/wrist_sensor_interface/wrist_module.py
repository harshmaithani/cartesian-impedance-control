import os
import rospy
import rospkg
import threading


from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget, QPixmap, QMessageBox, QStandardItemModel,QStandardItem
from PyQt4.QtGui import QApplication, QVBoxLayout, QLCDNumber, QProgressBar
from PyQt4.Qwt5 import *        #include for thermo
from PyQt4.Qwt5.qplt import *
from rospy.exceptions import ROSException

from python_qt_binding.QtGui import QWidget, QPixmap
from python_qt_binding.QtCore import QObject, pyqtSignal


from python_qt_binding.QtCore import Qt, QTimer, Slot, QBasicTimer, SIGNAL

from rqt_gui_py.plugin import Plugin

from std_msgs.msg import String
from geometry_msgs.msg import *

class WristValuesSignal(QObject):
    signal = pyqtSignal(list)

class WristSensor(Plugin):

    def __init__(self, context):
        super(WristSensor, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('WristSensor')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('wrist_sensor_interface'), 'resource', 'wrist_int.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        rp = rospkg.RosPack()

        self._wristValuesSignal = WristValuesSignal()

        self._widget.subscribe.clicked.connect(self._subscribe)


        self._wristValuesSignal.signal.connect(self._wristValuesSignalHandler)

        # Give QObjects reasonable names
        self._widget.setObjectName('FTSensor Interface')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        pixmap_blue_file = os.path.join(rp.get_path('wrist_sensor_interface'), 'resource', 'blue.png')
        pixmap_baby_file = os.path.join(rp.get_path('wrist_sensor_interface'), 'resource', 'baby.png')
        pixmap_green_file = os.path.join(rp.get_path('wrist_sensor_interface'), 'resource', 'green.png')
        pixmap_yellow_file = os.path.join(rp.get_path('wrist_sensor_interface'), 'resource', 'yellow.png')
        pixmap_orange_file = os.path.join(rp.get_path('wrist_sensor_interface'), 'resource', 'orange.png')
        pixmap_red_file = os.path.join(rp.get_path('wrist_sensor_interface'), 'resource', 'red.png')
        pixmap_empty_file = os.path.join(rp.get_path('wrist_sensor_interface'), 'resource', 'empty.png')


        self._blue_string = "background-color: rgb(0,0,255)"
        self._black_string = "background-color: rgb(76,76,76)"

        self._pixmap_red = QPixmap(pixmap_red_file)
        self._pixmap_green = QPixmap(pixmap_green_file)
        self._pixmap_baby = QPixmap(pixmap_baby_file)
        self._pixmap_blue = QPixmap(pixmap_blue_file)
        self._pixmap_orange = QPixmap(pixmap_orange_file)
        self._pixmap_yellow = QPixmap(pixmap_yellow_file)
        self._pixmap_empty = QPixmap(pixmap_empty_file)

        self._cleaning()
        self._initdisplay()
        self._initframe()

    def _blacklcd(self):

        for i in range(0,6):
            lcd_color = 'lcd_display_' + str(i)
            getattr(self._widget,lcd_color).setStyleSheet(self._black_string)


    def _initframe(self):
        for i in range(0,6):
            frame_color = 'frame_' + str(i)
            getattr(self._widget,frame_color).setStyleSheet(self._black_string)

    def _initdisplay(self):
        for i in range(0,6):
            lcd_color = 'lcd_display_' + str(i)
            getattr(self._widget,lcd_color).setStyleSheet(self._blue_string)

    def _cleaning(self):
        for i in range(0,6):
            for j in range(0,18):
                lab_color = 'clab_' + str(i) +'_' + str(j)
                getattr(self._widget,lab_color).setPixmap(self._pixmap_empty)

    def _callback(self, ftreadings):

        #defining variables
        self.force_x = int(ftreadings.wrench.force.x)
        self.force_y = int(ftreadings.wrench.force.y)
        self.force_z = int(ftreadings.wrench.force.z)

        self.torque_x = int(ftreadings.wrench.torque.x)
        self.torque_y = int(ftreadings.wrench.torque.y)
        self.torque_z = int(ftreadings.wrench.torque.z)
        self._display(self.force_x, self.force_y, self.force_z, self.torque_x, self.torque_y, self.torque_z)

        #publisher in terminal
        rospy.loginfo("Force : x=[%f] y=[%f] z=[%f] Torque x=[%f] y=[%f] z=[%f]",self.force_x, self.force_y, self.force_z, self.torque_x, self.torque_y, self.torque_z)
        wrist_list=[self.force_x, self.force_y, self.force_z, self.torque_x, self.torque_y, self.torque_z]
        self._wristValuesSignal.signal.emit(wrist_list)

    def _wristValuesSignalHandler(self, wrist_list):

        self._cleaning()
        #self._initframe()

        self.forcetop=30000
        self.stepvalue=int(self.forcetop/17.)

        for i in range (0,3):
            force=wrist_list[i]
            if force<0:
                force=force*-1

            if force> 0:
                force_color = 'clab_' + str(i) +'_0'
                getattr(self._widget,force_color).setPixmap(self._pixmap_blue)
                if force>self.stepvalue:
                    force_color = 'clab_' + str(i) +'_1'
                    getattr(self._widget,force_color).setPixmap(self._pixmap_blue)
                    if force>2*self.stepvalue:
                        force_color = 'clab_' + str(i) +'_2'
                        getattr(self._widget,force_color).setPixmap(self._pixmap_blue)
                        if force>3*self.stepvalue:
                            force_color = 'clab_' + str(i) +'_3'
                            getattr(self._widget,force_color).setPixmap(self._pixmap_baby)
                            if force>4*self.stepvalue:
                                force_color = 'clab_' + str(i) +'_4'
                                getattr(self._widget,force_color).setPixmap(self._pixmap_baby)
                                if force>5*self.stepvalue:
                                    force_color = 'clab_' + str(i) +'_5'
                                    getattr(self._widget,force_color).setPixmap(self._pixmap_baby)
                                    if force>6*self.stepvalue:
                                        force_color = 'clab_' + str(i) +'_6'
                                        getattr(self._widget,force_color).setPixmap(self._pixmap_green)
                                        if force>7*self.stepvalue:
                                            force_color = 'clab_' + str(i) +'_7'
                                            getattr(self._widget,force_color).setPixmap(self._pixmap_green)
                                            if force>8*self.stepvalue:
                                                force_color = 'clab_' + str(i) +'_8'
                                                getattr(self._widget,force_color).setPixmap(self._pixmap_green)
                                                if force>9*self.stepvalue:
                                                    force_color = 'clab_' + str(i) +'_9'
                                                    getattr(self._widget,force_color).setPixmap(self._pixmap_yellow)
                                                    if force>10*self.stepvalue:
                                                        force_color = 'clab_' + str(i) +'_10'
                                                        getattr(self._widget,force_color).setPixmap(self._pixmap_yellow)
                                                        if force>11*self.stepvalue:
                                                            force_color = 'clab_' + str(i) +'_11'
                                                            getattr(self._widget,force_color).setPixmap(self._pixmap_yellow)
                                                            if force>12*self.stepvalue:
                                                                force_color = 'clab_' + str(i) +'_12'
                                                                getattr(self._widget,force_color).setPixmap(self._pixmap_orange)
                                                                if force>13*self.stepvalue:
                                                                    force_color = 'clab_' + str(i) +'_13'
                                                                    getattr(self._widget,force_color).setPixmap(self._pixmap_orange)
                                                                    if force>14*self.stepvalue:
                                                                        force_color = 'clab_' + str(i) +'_14'
                                                                        getattr(self._widget,force_color).setPixmap(self._pixmap_orange)
                                                                        if force>15*self.stepvalue:
                                                                            force_color = 'clab_' + str(i) +'_15'
                                                                            getattr(self._widget,force_color).setPixmap(self._pixmap_red)
                                                                            if force>16*self.stepvalue:
                                                                                force_color = 'clab_' + str(i) +'_16'
                                                                                getattr(self._widget,force_color).setPixmap(self._pixmap_red)
                                                                                if force>self.topvalue:
                                                                                    force_color = 'clab_' + str(i) +'_17'
                                                                                    getattr(self._widget,force_color).setPixmap(self._pixmap_red)

        self.torquetop=15000
        self.torquestep=int(self.torquetop/17.)

        for i in range (3,6):
            torque=wrist_list[i]
            if torque<0:
                torque=torque*-1

            if torque> 0:
                torque_color = 'clab_' + str(i) +'_0'
                getattr(self._widget,torque_color).setPixmap(self._pixmap_blue)
                if torque>self.torquestep:
                    torque_color = 'clab_' + str(i) +'_1'
                    getattr(self._widget,torque_color).setPixmap(self._pixmap_blue)
                    if torque>2*self.torquestep:
                        torque_color = 'clab_' + str(i) +'_2'
                        getattr(self._widget,torque_color).setPixmap(self._pixmap_blue)
                        if torque>3*self.torquestep:
                            torque_color = 'clab_' + str(i) +'_3'
                            getattr(self._widget,torque_color).setPixmap(self._pixmap_baby)
                            if torque>4*self.torquestep:
                                torque_color = 'clab_' + str(i) +'_4'
                                getattr(self._widget,torque_color).setPixmap(self._pixmap_baby)
                                if torque>5*self.torquestep:
                                    torque_color = 'clab_' + str(i) +'_5'
                                    getattr(self._widget,torque_color).setPixmap(self._pixmap_baby)
                                    if torque>6*self.torquestep:
                                        torque_color = 'clab_' + str(i) +'_6'
                                        getattr(self._widget,torque_color).setPixmap(self._pixmap_green)
                                        if torque>7*self.torquestep:
                                            torque_color = 'clab_' + str(i) +'_7'
                                            getattr(self._widget,torque_color).setPixmap(self._pixmap_green)
                                            if torque>8*self.torquestep:
                                                torque_color = 'clab_' + str(i) +'_8'
                                                getattr(self._widget,torque_color).setPixmap(self._pixmap_green)
                                                if torque>9*self.torquestep:
                                                    torque_color = 'clab_' + str(i) +'_9'
                                                    getattr(self._widget,torque_color).setPixmap(self._pixmap_yellow)
                                                    if torque>10*self.torquestep:
                                                        torque_color = 'clab_' + str(i) +'_10'
                                                        getattr(self._widget,torque_color).setPixmap(self._pixmap_yellow)
                                                        if torque>11*self.torquestep:
                                                            torque_color = 'clab_' + str(i) +'_11'
                                                            getattr(self._widget,torque_color).setPixmap(self._pixmap_yellow)
                                                            if torque>12*self.torquestep:
                                                                torque_color = 'clab_' + str(i) +'_12'
                                                                getattr(self._widget,toque_color).setPixmap(self._pixmap_orange)
                                                                if torque>13*self.torquestep:
                                                                    torque_color = 'clab_' + str(i) +'_13'
                                                                    getattr(self._widget,torque_color).setPixmap(self._pixmap_orange)
                                                                    if torque>14*self.torquestep:
                                                                        torque_color = 'clab_' + str(i) +'_14'
                                                                        getattr(self._widget,torque_color).setPixmap(self._pixmap_orange)
                                                                        if torque>15*self.torquestep:
                                                                            torque_color = 'clab_' + str(i) +'_15'
                                                                            getattr(self._widget,torque_color).setPixmap(self._pixmap_red)
                                                                            if torque>16*self.torquestep:
                                                                                torque_color = 'clab_' + str(i) +'_16'
                                                                                getattr(self._widget,torque_color).setPixmap(self._pixmap_red)
                                                                                if torque>self.torquetop:
                                                                                    torque_color = 'clab_' + str(i) +'_17'
                                                                                    getattr(self._widget,torque_color).setPixmap(self._pixmap_red)


    def _display(self,force_x,force_y, force_z, torque_x, torque_y, torque_z):

        #self._blacklcd()

                #Force LCDs
        self._widget.lcd_display_0.display(force_x)
        self._widget.lcd_display_1.display(force_y)
        self._widget.lcd_display_2.display(force_z)


                #Torque LCDs
        self._widget.lcd_display_3.display(torque_x)
        self._widget.lcd_display_4.display(torque_y)
        self._widget.lcd_display_5.display(torque_z)


    def _subscribe(self):
        #Subscriber to the force sensor
        self._topicListener = rospy.Subscriber('sensor_readings', WrenchStamped,self._callback)
        rospy.loginfo('Subscribe to topic sensor_readings')
        self._blacklcd()

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

