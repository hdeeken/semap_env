#!/usr/bin/env python

import sys
import rviz
import rospy
import roslib; roslib.load_manifest('spatial_environment')

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from spatial_db_ros.srv import *
from spatial_db_ros.service_calls import *
from spatial_db_ros.instance_srv_calls import *
from spatial_db_ros.description_srv_calls import *

class ChooseObjectInstanceWidget(QWidget):
  name_ = None
  list_ = None
  choice_ = None
  choices_map_ = {}
 
  def __init__(self, name):

    QWidget.__init__(self)

    self.name_ = name 
    self.frame = QFrame()
    self.setWindowTitle( "Choose Object Instance" )
 
    layout = QVBoxLayout()
    layout.addWidget( self.frame )

    self.choice_list = QComboBox()
    self.choice_list.setEditable(False)
    self.setChoices()
    layout.addWidget( self.choice_list )

    h_layout = QHBoxLayout()
    top_button = QPushButton( "Apply" )
    top_button.clicked.connect( self.applyButtonClick )
    h_layout.addWidget( top_button )
    side_button = QPushButton( "Cancel" )
    side_button.clicked.connect( self.cancelButtonClick )
    h_layout.addWidget( side_button )

    layout.addLayout( h_layout )
    self.setLayout( layout )
 
  def applyButtonClick( self ):
    self.choice_ = self.choice_list.currentText()
    self.close()

  def getChoice(self):
    return self.choice_, self.choices_map_[self.choice_].id, self.choices_map_[self.choice_].pose

  def cancelButtonClick( self ):
    self.close()

  def setChoices(self):
    response = call_get_object_instances_list()
    print response
    for obj in response.objects:
      if(obj.name != self.name_):
        self.choice_list.addItem(obj.name)
        self.choices_map_[obj.name] = obj

class ChooseObjectDescriptionWidget(QWidget):
  name_ = None
  list_ = None
  choice_ = None
  choices_map_ = {}
 
  def __init__(self, name):
    QWidget.__init__(self)

    self.name_ = name
    self.frame = QFrame()
    self.setWindowTitle( "Choose Object Description" )
 
    layout = QVBoxLayout()
    layout.addWidget( self.frame )

    self.choice_list = QComboBox()
    self.choice_list.setEditable(True)
    self.setChoices()
    layout.addWidget( self.choice_list )
    print 'have set choices'

    h_layout = QHBoxLayout()
    top_button = QPushButton( "Apply" )
    top_button.clicked.connect( self.applyButtonClick )
    h_layout.addWidget( top_button )
    side_button = QPushButton( "Cancel" )
    side_button.clicked.connect( self.cancelButtonClick )
    h_layout.addWidget( side_button )

    layout.addLayout( h_layout )
    self.setLayout( layout )
 
  def applyButtonClick( self ):
    self.choice_ = self.choice_list.currentText()
    self.close()

  def getChoice(self):
    print 'here i am'
    
    print self.choice_

    if  self.choice_ in self.choices_map_.keys():
      print 'old obj'
      return self.choice_, self.choices_map_[self.choice_]
    else:
        print 'new obj'
        return self.choice_, -1

  def cancelButtonClick( self ):
    self.close()

  def setChoices(self):
    response = call_get_object_descriptions_list()
    print 'got response', len(response.descriptions)
    if len(response.descriptions) > 0:
      for desc in response.descriptions:
        if desc.type != self.name_:
          self.choice_list.addItem(desc.type)
          self.choices_map_[desc.type] = desc.id
          print 'add', desc.type
        else:
          print 'desc type == self.name', desc.type
    else:
      print 'no desc avialable'

class MeshTransformationWidget(QWidget):
  has_modifier_ = False
  modifier_ = None

  def __init__(self):
    QWidget.__init__(self)

    self.frame = QFrame()
    self.setWindowTitle( "Apply Tranformations?" )
 
    layout = QVBoxLayout()
    layout.addWidget( self.frame )
  
    h_layout = QHBoxLayout()
    
    top_button = QPushButton( "Apply" )
    top_button.clicked.connect( self.applyButtonClick )
    h_layout.addWidget( top_button )
    side_button = QPushButton( "Cancel" )
    side_button.clicked.connect( self.cancelButtonClick )
    h_layout.addWidget( side_button )

    items = ["0.001", "0.01", "0.1", "1.0", "-0.001", "-0.01", "-0.1", "-1.0", "0.15", "0.2", "0.25", "0.025", "0.007", "0.002", "0.003", "0.004", "0.05", "0.025"]

    self.x_modifier = QComboBox()
    self.x_modifier.addItems(items)
    self.x_modifier.setEditable(True)
    layout.addWidget( self.x_modifier )

    self.y_modifier = QComboBox()
    self.y_modifier.addItems(items)
    self.y_modifier.setEditable(True)
    layout.addWidget( self.y_modifier )

    self.z_modifier = QComboBox()
    self.z_modifier.addItems(items)
    self.z_modifier.setEditable(True)
    layout.addWidget( self.z_modifier )

    layout.addLayout( h_layout )
    self.setLayout( layout )

  def applyButtonClick( self ):
    self.has_modifier_ = True
    self.modifier_ = [float(self.x_modifier.currentText()), float(self.y_modifier.currentText()), float(self.z_modifier.currentText())]
    self.close()

  def hasModifier(self):
    return self.has_modifier_

  def getModifier(self):
    return self.modifier_

  def cancelButtonClick( self ):
    self.close()


class SetNameWidget(QWidget):
  has_name_ = False
  name_ = None

  def __init__(self):
    QWidget.__init__(self)

    self.frame = QFrame()
    self.setWindowTitle( "Name" )
 
    layout = QVBoxLayout()
    layout.addWidget( self.frame )

    self.name = QLineEdit()
    layout.addWidget( self.name )

    button_layout = QHBoxLayout()
    top_button = QPushButton( "Apply" )
    top_button.clicked.connect( self.applyButtonClick )
    button_layout.addWidget( top_button )
    side_button = QPushButton( "Cancel" )
    side_button.clicked.connect( self.cancelButtonClick )
    button_layout.addWidget( side_button )
    layout.addLayout( button_layout )

    self.setLayout( layout )

  def applyButtonClick( self ):
    self.has_pose_ = True
    self.name_ = self.name.text()
    self.close()

  def modify(self):
    return self.has_name_

  def getName(self):
    return self.name_

  def cancelButtonClick( self ):
    self.close()

class SetPose(QWidget):
  has_pose_ = False
  pose_ = None
  
  def __init__(self):
    QWidget.__init__(self)

    self.frame = QFrame()
    self.setWindowTitle( "Pose" )
 
    layout = QVBoxLayout()
    layout.addWidget( self.frame )

    pos_layout = QHBoxLayout()
    pos_val = QDoubleValidator(-100, 100, 2)

    self.pos_x_modifier = QLineEdit("0.0")
    self.pos_x_modifier.setValidator(pos_val)
    self.pos_x_modifier.textChanged.connect(self.check_state)
    self.pos_x_modifier.textChanged.emit(self.pos_x_modifier.text())
    pos_layout.addWidget( self.pos_x_modifier )

    self.pos_y_modifier = QLineEdit("0.0")
    self.pos_y_modifier.setValidator(pos_val)
    self.pos_y_modifier.textChanged.connect(self.check_state)
    self.pos_y_modifier.textChanged.emit(self.pos_y_modifier.text())
    pos_layout.addWidget( self.pos_y_modifier )

    self.pos_z_modifier = QLineEdit("0.0")
    self.pos_z_modifier.setValidator(pos_val)
    self.pos_z_modifier.textChanged.connect(self.check_state)
    self.pos_z_modifier.textChanged.emit(self.pos_z_modifier.text())
    pos_layout.addWidget( self.pos_z_modifier )

    ori_layout = QHBoxLayout()
    ori_val = QDoubleValidator(0.0, 1.0, 2)

    self.ori_x_modifier = QLineEdit("0.0")
    self.ori_x_modifier.setValidator(ori_val)
    self.ori_x_modifier.textChanged.connect(self.check_state)
    self.ori_x_modifier.textChanged.emit(self.ori_x_modifier.text())
    ori_layout.addWidget( self.ori_x_modifier )

    self.ori_y_modifier = QLineEdit("0.0")
    self.ori_y_modifier.setValidator(ori_val)
    self.ori_y_modifier.textChanged.connect(self.check_state)
    self.ori_y_modifier.textChanged.emit(self.ori_y_modifier.text())
    ori_layout.addWidget( self.ori_y_modifier )

    self.ori_z_modifier = QLineEdit("0.0")
    self.ori_z_modifier.setValidator(ori_val)
    self.ori_z_modifier.textChanged.connect(self.check_state)
    self.ori_z_modifier.textChanged.emit(self.ori_z_modifier.text())
    ori_layout.addWidget( self.ori_z_modifier )

    self.ori_w_modifier = QLineEdit("1.0")
    self.ori_w_modifier.setValidator(ori_val)
    self.ori_w_modifier.textChanged.connect(self.check_state)
    self.ori_w_modifier.textChanged.emit(self.ori_w_modifier.text())
    ori_layout.addWidget( self.ori_w_modifier )

    button_layout = QHBoxLayout()
    top_button = QPushButton( "Apply" )
    top_button.clicked.connect( self.applyButtonClick )
    button_layout.addWidget( top_button )
    side_button = QPushButton( "Cancel" )
    side_button.clicked.connect( self.cancelButtonClick )
    button_layout.addWidget( side_button )

    layout.addLayout( pos_layout )
    layout.addLayout( ori_layout )
    layout.addLayout( button_layout )
    self.setLayout( layout )

  def check_state(self, *args, **kwargs):
    sender = self.sender()
    validator = sender.validator()
    state = validator.validate(sender.text(), 0)[0]

    if state == QValidator.Acceptable:
      color = '#c4df9b' # green
    elif state == QValidator.Intermediate:
      color = '#fff79a' # yellow
    else:
      color = '#f6989d' # red
    sender.setStyleSheet('QLineEdit { background-color: %s }' % color)

  def applyButtonClick( self ):
    self.has_pose_ = True
    self.pose_ = [[float(self.pos_x_modifier.text()),
                   float(self.pos_y_modifier.text()),
                   float(self.pos_z_modifier.text())],
                  [float(self.ori_x_modifier.text()),
                   float(self.ori_y_modifier.text()),
                   float(self.ori_z_modifier.text()),
                   float(self.ori_w_modifier.text())]]
    self.close()

  def modify(self):
    return self.has_pose_

  def getPose(self):
    return self.pose_

  def cancelButtonClick( self ):
    self.close()

class SetGeometryModelTypeWidget(QWidget):
  type_list = None
  type = None
 
  def __init__(self):
    QWidget.__init__(self)

    self.frame = QFrame()
    self.setWindowTitle( "Set Model Type" )
 
    layout = QVBoxLayout()
    layout.addWidget( self.frame )
  
    h_layout = QHBoxLayout()
    
    top_button = QPushButton( "Apply" )
    top_button.clicked.connect( self.applyButtonClick )
    h_layout.addWidget( top_button )
    side_button = QPushButton( "Cancel" )
    side_button.clicked.connect( self.cancelButtonClick )
    h_layout.addWidget( side_button )

    self.type_list = QComboBox()
    self.type_list.setEditable(True)
    self.getDescriptionTypes()
    layout.addWidget( self.type_list )

    layout.addLayout( h_layout )
    self.setLayout( layout )
 
  def applyButtonClick( self ):
    self.type = self.type_list.currentText()
    self.close()

  def getType(self):
    return self.type

  def cancelButtonClick( self ):
    self.close()

  def getDescriptionTypes(self):
    response = call_get_geometry_model_types()
    if len(response.types) > 0: 
      self.type_list.addItems(response.types)
    self.type_list.addItems(["Body"])
    
class ChooseReferenceFrameWidget(QDialog):
  name_ = None
  list_ = None
  choice_ = None
  choices_map_ = {}
 
  def __init__(self, name):
    QWidget.__init__(self)
    self.name_ = name
    self.frame = QFrame()
    self.setWindowTitle( "Choose Object Description" )
 
    layout = QVBoxLayout()
    layout.addWidget( self.frame )

    self.choice_list = QComboBox()
    self.choice_list.setEditable(False)
    self.setChoices()
    layout.addWidget( self.choice_list )

    self.keep_transform_cbox = QCheckBox( "Keep Transform" )
    layout.addWidget( self.keep_transform_cbox )

    h_layout = QHBoxLayout()
    top_button = QPushButton( "Apply" )
    top_button.clicked.connect( self.applyButtonClick )
    h_layout.addWidget( top_button )
    side_button = QPushButton( "Cancel" )
    side_button.clicked.connect( self.cancelButtonClick )
    h_layout.addWidget( side_button )

    layout.addLayout( h_layout )
    self.setLayout( layout )
 
  def applyButtonClick( self ):
    self.choice_ = self.choice_list.currentText()
    self.close()

  def getChoice(self):
    return self.choice_, self.choices_map_[self.choice_].id

  def cancelButtonClick( self ):
    self.close()

  def setChoices(self):
    print 'get frames'
    response =  call_get_frame_names()
    for frame in response.frames:
      if frame != self.name_:
        self.choice_list.addItem(frame)
        print 'add', frame
    
    self.choice_list.addItem('test')
  def getChoice(self):
    return self.choice_, self.keep_transform_cbox.checkState()
