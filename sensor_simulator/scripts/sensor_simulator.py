#!/usr/bin/env python3
from multiprocessing import Lock
import sys
from PyQt5 import QtWidgets, uic, QtCore

from sensor_sim_ui_main import Ui_MainWindow
from lidar_form import Ui_LidarForm
from sensor_sim_ui_object_form import Ui_ObjectForm

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from sensor_sim_msgs.msg import SimulationParam, LidarSimParam, ObjectSimParam


class  ObjectForm(QtWidgets.QWidget, Ui_ObjectForm):
    def __init__(self, *args, obj=None, name=None, parent=None, id, **kwargs, ) :
        #super(MainWindow, self).__init__(*args, **kwargs)
        super(ObjectForm, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self.updated = False
        self.closed = False
        self.osp = ObjectSimParam()
        self.id = id
        if name is not None :
            self.setObjectName(name)
            self.setWindowTitle(name)
        if parent is not None :
            self.lineEdit_frame.setText(parent)

        self.lineEdit_name.setText(self.objectName())

    def update_spinbox_pos (self) :
        self.doubleSpinBox_roll.setValue(self.dial_roll.value()/100.0)
        self.doubleSpinBox_pitch.setValue(self.dial_pitch.value()/100.0)
        self.doubleSpinBox_yaw.setValue(self.dial_yaw.value()/100.0)
        self.doubleSpinBox_pos_x.setValue(self.horizontalSlider_pos_x.value()/1000.0)
        self.doubleSpinBox_pos_y.setValue(self.horizontalSlider_pos_y.value()/1000.0)
        self.doubleSpinBox_pos_z.setValue(self.horizontalSlider_pos_z.value()/1000.0)
        self.doubleSpinBox_length.setValue(self.horizontalSlider_length.value()/1000.0)
        self.doubleSpinBox_width.setValue(self.horizontalSlider_width.value()/1000.0)
        self.doubleSpinBox_height.setValue(self.horizontalSlider_height.value()/1000.0)
        self.change_param()

    def update_slider_pos (self) :
        self.horizontalSlider_length.setValue(int(self.doubleSpinBox_length.value()*1000))
        self.horizontalSlider_width.setValue(int(self.doubleSpinBox_width.value()*1000))
        self.horizontalSlider_height.setValue(int(self.doubleSpinBox_height.value()*1000))
        self.horizontalSlider_pos_x.setValue(int(self.doubleSpinBox_pos_x.value()*1000))
        self.horizontalSlider_pos_y.setValue(int(self.doubleSpinBox_pos_y.value()*1000))
        self.horizontalSlider_pos_z.setValue(int(self.doubleSpinBox_pos_z.value()*1000))
        self.change_param()

    def update_dial_pos (self) :
        self.dial_roll.setValue(int(self.doubleSpinBox_roll.value()*100))
        self.dial_pitch.setValue(int(self.doubleSpinBox_pitch.value()*100))
        self.dial_yaw.setValue(int(self.doubleSpinBox_yaw.value()*100))
        self.change_param()

    def change_param(self) :
        osp = ObjectSimParam()
        osp.id = self.id
        osp.name = self.objectName()
        osp.topic_name = self.lineEdit_name.text()
        osp.parent_frame_id = self.lineEdit_frame.text()
        osp.length = self.doubleSpinBox_length.value()
        osp.width = self.doubleSpinBox_width.value()
        osp.height = self.doubleSpinBox_height.value()
        osp.pos_x = self.doubleSpinBox_pos_x.value()
        osp.pos_y = self.doubleSpinBox_pos_y.value()
        osp.pos_z = self.doubleSpinBox_pos_z.value()
        osp.roll = self.doubleSpinBox_roll.value()
        osp.pitch = self.doubleSpinBox_pitch.value()
        osp.yaw = self.doubleSpinBox_yaw.value()
        osp.color.r = self.doubleSpinBox_color_r.value()
        osp.color.g = self.doubleSpinBox_color_g.value()
        osp.color.b = self.doubleSpinBox_color_b.value()
        osp.color.a = self.doubleSpinBox_color_a.value()

        self.osp = osp
        self.updated = True
        #self.publisher.publish(sp)

    def closeEvent(self, event) :
        self.closed = True


#class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
class  LidarForm(QtWidgets.QWidget, Ui_LidarForm):
    def __init__(self, *args, obj=None, name=None, parent=None, id, **kwargs, ):
        #super(MainWindow, self).__init__(*args, **kwargs)
        super(LidarForm, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self.updated = False
        self.closed = False
        self.lsp = LidarSimParam()
        self.id = id
        if name is not None :
            self.setObjectName(name)
            self.setWindowTitle(name)
        if parent  is not None :
            self.lineEdit_frame.setText(parent)

        self.lineEdit_name.setText(self.objectName())

    def update_spinbox_pos (self) :
        self.doubleSpinBox_mounting_roll.setValue(self.dial_mounting_roll.value()/10.0)
        self.doubleSpinBox_mounting_pitch.setValue(self.dial_mounting_pitch.value()/10.0)
        self.doubleSpinBox_mounting_yaw.setValue(self.dial_mounting_yaw.value()/10.0)
        self.doubleSpinBox_mounting_x.setValue(self.horizontalSlider_mounting_x.value()/1000.0)
        self.doubleSpinBox_mounting_y.setValue(self.horizontalSlider_mounting_y.value()/1000.0)
        self.doubleSpinBox_mounting_z.setValue(self.horizontalSlider_mounting_z.value()/1000.0)
        self.change_param()

    def update_slider_pos (self) :
        self.horizontalSlider_mounting_x.setValue(int(self.doubleSpinBox_mounting_x.value()*1000))
        self.horizontalSlider_mounting_y.setValue(int(self.doubleSpinBox_mounting_y.value()*1000))
        self.horizontalSlider_mounting_z.setValue(int(self.doubleSpinBox_mounting_z.value()*1000))
        self.change_param()

    def update_dial_pos (self) :
        self.dial_mounting_roll.setValue(int(self.doubleSpinBox_mounting_roll.value()*10))
        self.dial_mounting_pitch.setValue(int(self.doubleSpinBox_mounting_pitch.value()*10))
        self.dial_mounting_yaw.setValue(int(self.doubleSpinBox_mounting_yaw.value()*10))
        self.change_param()

    def change_param(self) :
        lsp = LidarSimParam()
        lsp.id = self.id
        lsp.name = self.objectName()
        lsp.topic_name = self.lineEdit_name.text()
        lsp.parent_frame_id = self.lineEdit_frame.text()
        lsp.horizontal_fov = self.doubleSpinBox_horizontal_fov.value()
        lsp.vertical_fov = self.doubleSpinBox_vertical_fov.value()
        lsp.horizontal_resolution = self.doubleSpinBox_horizontal_resolution.value()
        lsp.vertical_resolution = self.doubleSpinBox_vertical_resolution.value()
        lsp.max_detection_range = self.doubleSpinBox_max_range.value()
        lsp.min_detection_range = self.doubleSpinBox_min_range.value()
        lsp.update_frequency = self.doubleSpinBox_frequency.value()
        lsp.update_frequency = self.doubleSpinBox_frequency.value()
        lsp.mouting_positon_x = self.doubleSpinBox_mounting_x.value()
        lsp.mouting_positon_y = self.doubleSpinBox_mounting_y.value()
        lsp.mouting_positon_z = self.doubleSpinBox_mounting_z.value()
        lsp.mouting_posture_roll = self.doubleSpinBox_mounting_roll.value()
        lsp.mouting_posture_pitch = self.doubleSpinBox_mounting_pitch.value()
        lsp.mouting_posture_yaw = self.doubleSpinBox_mounting_yaw.value()
        lsp.color.r = self.doubleSpinBox_color_r.value()
        lsp.color.g = self.doubleSpinBox_color_g.value()
        lsp.color.b = self.doubleSpinBox_color_b.value()
        lsp.color.a = self.doubleSpinBox_color_a.value()
        self.lsp = lsp
        self.updated = True
        #self.publisher.publish(sp)

    def closeEvent(self, event) :
        self.closed = True

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        rclpy.init(args=args)
        self.node = rclpy.create_node('simulation_param_publisher')
        self.publisher = self.node.create_publisher(SimulationParam, '/simulation_param', 1)
        #self.sim_param = SimulationParam()

        self.setupUi(self)

        self.lidar_num = 0
        self.lidar_forms = []
        self.used_lidar_id = [0 for i in range(100)]

        self.object_num = 0
        self.object_forms = []
        self.used_object_id = [0 for i in range(100)]

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start(100)

    def closeEvent(self, event) :
        for w in self.lidar_forms :
            w.close()
        for w in self.object_forms :
            w.close()

    def add_lidar(self):
        if self.lidar_num < 100 :
            name = ""
            parent = ""
            num = 0
            for i, id in enumerate(self.used_lidar_id) :
                if id == 0 :
                    self.used_lidar_id[i] = 1
                    num = i
                    break

            if ( self.lineEdit_lidar_name.text() == "" ) :
                name = "lidar_" + str(num)
            else :
                name = self.lineEdit_lidar_name.text()

            if ( self.lineEdit_lidar_frame.text() == "" ) :
                parent = "base_link"
            else :
                parent = self.lineEdit_lidar_frame.text()

            tmp_form = LidarForm(name=name, parent=parent, id=num)
            tmp_form.show()
            self.lidar_forms.append(tmp_form)
            self.lidar_num += 1
            self.spinBox_lidar.setValue(self.spinBox_lidar.value()+1)

    def add_object(self):
        if self.object_num < 100 :
            name = ""
            parent = ""

            for i, id in enumerate(self.used_object_id) :
                if id == 0 :
                    self.used_object_id[i] = 1
                    num = i
                    break

            if ( self.lineEdit_object_name.text() == "" ) :
                name = "object_" + str(num)
            else :
                name = self.lineEdit_object_name.text()

            if ( self.lineEdit_object_frame.text() == "" ) :
                parent = "base_link"
            else :
                parent = self.lineEdit_object_frame.text()

            tmp_form = ObjectForm(name=name, parent=parent, id=num)
            tmp_form.show()
            self.object_forms.append(tmp_form)
            self.object_num += 1
            self.spinBox_object.setValue(self.spinBox_object.value()+1)

    def timer_callback(self) :

        if not self.spinBox_lidar.editingFinished or not self.spinBox_object.editingFinished or \
            not self.lineEdit_lidar_name.editingFinished or not self.lineEdit_lidar_frame.editingFinished or \
                not self.lineEdit_object_name.editingFinished or not self.lineEdit_object_frame.editingFinished :
            return

        sp = SimulationParam()

        updated = False

        active_lidar_form = []
        tmp_lidar_num = self.spinBox_lidar.value()
        for i,w in enumerate(self.lidar_forms) :
            sp.lidar_params.append(w.lsp)
            if w.updated :
                updated = True
                w.updated = False

            if not w.closed :
                active_lidar_form.append(w)
            else :
                self.used_lidar_id[w.id] = 0
                tmp_lidar_num -= 1

        if ( len(active_lidar_form) < tmp_lidar_num ) :
            for i in range(len(active_lidar_form),tmp_lidar_num) :
                if self.lidar_num < 100 :
                    name = ""
                    parent = ""
                    num = 0
                    for i, id in enumerate(self.used_lidar_id) :
                        if id == 0 :
                            self.used_lidar_id[i] = 1
                            num = i
                            break

                    if ( self.lineEdit_lidar_name.text() == "" ) :
                        name = "lidar_" + str(num)
                    else :
                        name = self.lineEdit_lidar_name.text()

                    if ( self.lineEdit_lidar_frame.text() == "" ) :
                        parent = "base_link"
                    else :
                        parent = self.lineEdit_lidar_frame.text()

                    tmp_form = LidarForm(name=name, parent=parent, id=num)
                    tmp_form.show()
                    active_lidar_form.append(tmp_form)
                    self.lidar_num += 1
                    self.spinBox_lidar.setValue(self.spinBox_lidar.value()+1)

        elif ( len(active_lidar_form) > tmp_lidar_num ) :
            del active_lidar_form[tmp_lidar_num:len(active_lidar_form)]

        self.lidar_forms = active_lidar_form
        self.lidar_num = len(active_lidar_form)

        if self.spinBox_lidar.editingFinished :
            self.spinBox_lidar.setValue(self.lidar_num)

        active_object_form = []
        tmp_object_num = self.spinBox_object.value()

        for i,w in enumerate(self.object_forms) :
            sp.object_params.append(w.osp)
            if w.updated :
                updated = True
                w.updated = False
            if not w.closed :
                active_object_form.append(w)
            else :
                self.used_object_id[w.id] = 0
                tmp_object_num =-1

        if ( len(active_object_form) < tmp_object_num ) :
            for i in range(len(active_object_form),tmp_object_num) :
                if self.object_num < 100 :
                    name = ""
                    parent = ""
                    num = 0
                    for i, id in enumerate(self.used_object_id) :
                        if id == 0 :
                            self.used_object_id[i] = 1
                            num = i
                            break

                    if ( self.lineEdit_object_name.text() == "" ) :
                        name = "object_" + str(num)
                    else :
                        name = self.lineEdit_object_name.text()

                    if ( self.lineEdit_object_frame.text() == "" ) :
                        parent = "base_link"
                    else :
                        parent = self.lineEdit_object_frame.text()

                    tmp_form = ObjectForm(name=name, parent=parent, id=num)
                    tmp_form.show()
                    active_object_form.append(tmp_form)
                    self.object_num += 1
                    self.spinBox_object.setValue(self.spinBox_object.value()+1)

        elif ( len(active_object_form) > tmp_object_num ) :
            del active_object_form[tmp_object_num:len(active_object_form)]

        self.object_forms = active_object_form
        self.object_num = len(active_object_form)
        if self.spinBox_object.editingFinished :
            self.spinBox_object.setValue(self.object_num)

        if updated :
            self.publisher.publish(sp)

app = QtWidgets.QApplication(sys.argv)

#window = LidarForm()
window = MainWindow()

window.show()
app.exec()