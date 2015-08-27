#!/usr/bin/env python

import sys
import copy
from random import random
from math import sin, radians
from pyassimp import pyassimp

import roslib; roslib.load_manifest("semap_env")
import rospy, tf

from tf.broadcaster import TransformBroadcaster
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PolygonStamped
from std_msgs.msg import String
from spatial_db_ros.srv import *
from spatial_db_ros.service_calls import *
from spatial_db_msgs.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from spatial_db_msgs.msg import ColorCommand, LabelCommand
from spatial_db_msgs.msg import ObjectDescription as ROSObjectDescription
from spatial_db_msgs.msg import ObjectInstance as ROSObjectInstance
from interactive_object_marker_widgets import *
from assimp_postgis_importer import importFromFileToMesh
from semap_env.service_calls import *
from object_description_marker import *
from visualization import *

#### Visu Control ###

def createVisuControl(controls, obj, visu_config):
    control = InteractiveMarkerControl()
    control.name = "VisuControl"
    control.description = "This is the objects visualization."
    control.interaction_mode = InteractiveMarkerControl.NONE
    control.always_visible = True
    control.markers = create_object_visualization_marker(obj, visu_config).markers
    controls.append(control)

def updateVisuControl(controls, obj, visu_config):
    for control in controls:
      if control.name == "VisuControl":
        controls.remove(control)
    createVisuControl(controls, obj, visu_config)

def createGhostVisu(controls, obj, visu_config):
    control = InteractiveMarkerControl()
    control.name = "GhostVisu"
    control.description = "This is the objects visualization."
    control.interaction_mode = InteractiveMarkerControl.NONE
    control.always_visible = True
    control.markers = create_inactive_object_visualization_marker(obj, visu_config).markers
    controls.append(control)

def updateGhostVisu(controls, obj, visu_config):
    for control in controls:
      if control.name == "GhostVisu":
        controls.remove(control)
    createGhostVisu(controls, obj, visu_config)

def createModelVisuControl(controls, frame, model, visu):
    control = InteractiveMarkerControl()
    control.name = "ModelVisuControl"
    control.description = "This is the model visualization."
    control.interaction_mode = InteractiveMarkerControl.NONE
    control.always_visible = True
    markers = create_model_visualization_marker(frame, model, visu).markers
    control.markers = markers
    controls.append(control)

### Motion Control

def create6DMotionControl(controls, fixed):
    control = InteractiveMarkerControl()
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.orientation.w = 1
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

def create3DMotionControl(controls, fixed):

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

def removeControl(controls, name):
    for control in controls:
      if control.name == name:
        controls.remove(control)

def createMenuControl(controls, pose, name):
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.name = "Menu"
    control.description= name
    control.always_visible = True
    marker = create_text_marker("Title", pose, [1.0, 1.0, 1.0, 1.0], [0.015, 0.15, 0.15], name)
    control.markers.append(marker)
    controls.append(control)

def updateMenuControl(controls, pose, name):
    for control in controls:
      if control.name == "Menu":
        controls.remove(control)
    createMenuControl(controls, pose, name)

def listControls(controls):
    for control in controls:
      print control.name

## Menu Util

def setTitle(menu_handler, entry_id, title):
  menu_handler.entry_contexts_[entry_id].title = title

def findParent(menu_handler, ancestor, child):
  if len(menu_handler.entry_contexts_[ancestor].sub_entries) > 0:
    for descendant in menu_handler.entry_contexts_[ancestor].sub_entries:
      if descendant == child:
        return ancestor
      else:
        parent = findParent(menu_handler, descendant, child)
        if parent:
          return parent
        else:
          continue
  else:
      return None

def transformMesh(mesh, modifier):
    for vertex in mesh.vertices:
      vertex.x *= modifier[0]
      vertex.y *= modifier[1]
      vertex.z *= modifier[2]

    return mesh

####################
####################

def switchCheckState(menu_handler, handle):
  state = menu_handler.getCheckState( handle )

  if state == MenuHandler.CHECKED:
    menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
  else:
    menu_handler.setCheckState( handle, MenuHandler.CHECKED )

def transformPolygonStamped(tf_listener, frame, polygon):

  transformed_polygon = PolygonStamped()
  transformed_polygon.header.frame_id = frame

  try:
    tf_listener.waitForTransform(frame, polygon.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))

    old_point = PointStamped()
    old_point.header = polygon.header;
    for point in polygon.polygon.points:
      old_point.point = point
      new_point = tf_listener.transformPoint(frame, old_point)
      transformed_polygon.polygon.points.append(new_point.point)

    return transformed_polygon

  except tf.Exception as e:
    print "some tf exception happened %s" % e

def getRotation(axis, degree):

    quaternion = None

    if axis == "Roll":
      if degree == "90":
        quaternion = tf.transformations.quaternion_from_euler(radians(90), 0, 0, 'sxyz')
      elif degree == "180":
        quaternion = tf.transformations.quaternion_from_euler(radians(180), 0, 0, 'sxyz')
      elif degree == "270":
        quaternion = tf.transformations.quaternion_from_euler(radians(270), 0, 0, 'sxyz')
    elif axis == "Pitch":
      if degree == "90":
        quaternion = tf.transformations.quaternion_from_euler(0, radians(90), 0, 'sxyz')
      elif degree == "180":
        quaternion = tf.transformations.quaternion_from_euler(0, radians(180), 0, 'sxyz')
      elif degree == "270":
        quaternion = tf.transformations.quaternion_from_euler(0, radians(270), 0, 'sxyz')
    elif axis == "Yaw":
      if degree == "90":
        quaternion = tf.transformations.quaternion_from_euler(0, 0, radians(90), 'sxyz')
      elif degree == "180":
        quaternion = tf.transformations.quaternion_from_euler(0, 0, radians(0), 'sxyz')
      elif degree == "270":
        quaternion = tf.transformations.quaternion_from_euler(0, 0, radians(270), 'sxyz')

    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose
