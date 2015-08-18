#!/usr/bin/env python

import sys
import copy
from random import random
from math import sin, radians
from pyassimp import pyassimp

import roslib; roslib.load_manifest("spatial_environment")
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
from spatial_db_ros.instance_srv_calls import *
from spatial_db_ros.description_srv_calls import *
from spatial_db_msgs.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel, GeometryModelSet
from spatial_db_msgs.msg import ColorCommand, LabelCommand
from spatial_db_msgs.msg import ObjectDescription as ROSObjectDescription
from spatial_db_msgs.msg import ObjectInstance as ROSObjectInstance
from interactive_object_marker_widgets import *
from assimp_postgis_importer import importFromFileToMesh
from spatial_environment.service_calls import *
from object_description_marker import *
from visualization import *

from shared_marker_util import *

class GhostObjectMarker():
  server = None
  obj = None
  marker = None
  geometry_movement_marker = None
  geometry_movement_menu_handler = None

  tf_listener = None
  visu_config = None
  model_visu = {}
  absolute_visu = {}
  model_entry = {}
  menu_handler = None
  geometry_menu_handle = None

  instance_menu_handle = None
  description_menu_handle = None
  models_menu_handle = None

  current_pose = None

  label = None
  label_pose = None
  label_movement = None
  label_movement_menu_handler = None
  geometry_movement = False
  geometry_movement_id = None
  geometry_movement_pose = None

  point2d_sub_ = None
  pose2d_sub_ = None
  polygon2d_sub_ = None
  point3d_sub_ = None
  pose3d_sub_ = None
  polygon3d_sub_ = None

  color_sub_ = None
  label_sub_ = None
  print_sub_ = None

  process_point2d_ = False
  process_pose2d_ = False
  process_polygon2d_ = False
  process_point3d_ = False
  process_pose3d_ = False
  process_polygon3d_ = False

  def __init__(self, obj, server, visu_config):
    self.obj = obj

    if visu_config:
      self.visu_config = visu_config
    else:
      self.visu_config = InstVisu()
      self.visu_config.relative = DescVisu()
      self.visu_config.absolute = DescVisu()
      self.visu_config.relative.geometries = defaultGhostRelativeDescriptionVisu(self.obj.description.geometries)
      self.visu_config.relative.abstractions = defaultGhostRelativeAbstractionVisu(self.obj.description.abstractions)
      self.visu_config.absolute.geometries = defaultGhostAbsoluteDescriptionVisu(self.obj.absolute.geometries)
      self.visu_config.absolute.abstractions = defaultGhostAbsoluteAbstractionVisu(self.obj.absolute.abstractions)

    self.server = server
    self.tf_listener = tf.TransformListener()

    self.menu_handler = MenuHandler()
    self.geometry_movement_menu_handler = MenuHandler()
    self.label_movement_menu_handler = MenuHandler()

    self.initInstanceMenu()
    self.createGhostMarker()

## Callbacks

  def infoInstanceCb(self, feedback):
    print "Id:", self.obj.id, "Name:", self.obj.name, "Alias:", self.obj.alias, "Type:", self.obj.description.type, "Type Id:", self.obj.description.id

  def processFeedback(self, feedback ):
      if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
          self.current_pose = feedback.pose
      self.server.applyChanges()

  def showModelCb(self, feedback):
    print 'Come and see the show'
    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler
    server = self.server

    # note: 1 is the id of the root handel in the menu_handler
    models = findParent(menu_handler, 1, handle)
    geo_or_abs = findParent(menu_handler, 1, models)
    rel_or_abs = findParent(menu_handler, 1, geo_or_abs)
    show = findParent(menu_handler, 1, rel_or_abs)
    inst_or_desc = findParent(menu_handler, 1, show)

    if menu_handler.getTitle(rel_or_abs) == "Relative" and menu_handler.getTitle(geo_or_abs) == "Geometries":
      model_visu = self.visu_config.relative.geometries
      print 'choose rel geo'
    elif menu_handler.getTitle(rel_or_abs) == "Relative" and menu_handler.getTitle(geo_or_abs) == "Abstractions":
      model_visu = self.visu_config.relative.abstractions
      print 'choose rel abs'
    elif menu_handler.getTitle(rel_or_abs) == "Absolute" and menu_handler.getTitle(geo_or_abs) == "Geometries":
      model_visu = self.visu_config.absolute.geometries
      print 'choose abs geo'
    elif menu_handler.getTitle(rel_or_abs) == "Absolute" and menu_handler.getTitle(geo_or_abs) == "Abstractions":
      model_visu = self.visu_config.absolute.abstractions
      print 'choose abs abs'
    else:
      print 'couldnt collect right model visu'
      return

    if menu_handler.getTitle(handle) in model_visu.keys():
      if menu_handler.getCheckState( handle ) == MenuHandler.CHECKED:
        menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
        model_visu[menu_handler.getTitle(handle)].show_geo = False
        model_visu[menu_handler.getTitle(handle)].show_text = False
      else:
        menu_handler.setCheckState( handle, MenuHandler.CHECKED )
        model_visu[menu_handler.getTitle(handle)].show_geo = True
        model_visu[menu_handler.getTitle(handle)].show_text = True

    else:
      rospy.logwarn('showGeometryModelCb: Unknown MenuEntry %s was called.' % menu_handler.getTitle(handle))

    menu_handler.reApply( server )
    updateGhostVisu(self.marker.controls, self.obj, self.visu_config)
    server.applyChanges()
      
  def showMetaCb(self, feedback):
    print ' show and destroy '
    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler
    server = self.server

    geo_or_abs = findParent(menu_handler, 1, handle)
    rel_or_abs = findParent(menu_handler, 1, geo_or_abs)

    print 'SEARCH FOR ROOT'
    show = findParent(menu_handler, 1, rel_or_abs)
    print 'show', menu_handler.getTitle(show)
    over = findParent(menu_handler, 1, show)
    print 'over', menu_handler.getTitle(over)
    if menu_handler.getTitle(rel_or_abs) == "Relative" and menu_handler.getTitle(geo_or_abs) == "Geometries":
      model_visu = self.visu_config.relative.geometries
    elif menu_handler.getTitle(rel_or_abs) == "Relative" and menu_handler.getTitle(geo_or_abs) == "Abstractions":
      model_visu = self.visu_config.relative.abstractions
    elif menu_handler.getTitle(rel_or_abs) == "Absolute" and menu_handler.getTitle(geo_or_abs) == "Geometries":
      model_visu = self.visu_config.absolute.geometries
    elif menu_handler.getTitle(rel_or_abs) == "Absolute" and menu_handler.getTitle(geo_or_abs) == "Abstractions":
      model_visu = self.visu_config.absolute.abstractions
    else:
      print 'couldnt collect right model visu'
      return

    if menu_handler.getTitle(handle) == "All":
      for key in model_visu.keys():
        model_visu[key].show_geo = True
        model_visu[key].show_text = True
        menu_handler.setCheckState( model_visu[key].handle, MenuHandler.CHECKED )
    elif menu_handler.getTitle(handle) == "None":
      for key in model_visu.keys():
        model_visu[key].show_geo = False
        model_visu[key].show_text = False
        menu_handler.setCheckState( model_visu[key].handle, MenuHandler.UNCHECKED )
    elif menu_handler.getTitle(handle) == "Inverted":
      for key in model_visu.keys():
        if menu_handler.getCheckState( model_visu[key].handle ) == MenuHandler.CHECKED:
          model_visu[key].show_geo = False
          model_visu[key].show_text = False
          menu_handler.setCheckState( model_visu[key].handle, MenuHandler.UNCHECKED )
        elif menu_handler.getCheckState( model_visu[key].handle ) == MenuHandler.UNCHECKED:
          model_visu[key].show_geo = True
          model_visu[key].show_text = True
          menu_handler.setCheckState( model_visu[key].handle, MenuHandler.CHECKED )
        else:
          print 'no checkbox... shouldnt happen'
    else:
      rospy.logwarn('showGeometryModelCb: Unknown MenuEntry %s was called.' % menu_handler.getTitle(handle))

    menu_handler.reApply( server )
    updateGhostVisu(self.marker.controls, self.obj, self.visu_config)
    server.applyChanges()

  def update(self):
    call_activate_objects([self.obj.id])

### Init Menus

  def initInstanceMenu(self):
    self.instance_menu_handle = self.menu_handler.insert("Instance")

    show = self.menu_handler.insert("Show", parent = self.instance_menu_handle)

    show_relative = self.menu_handler.insert("Relative", parent = show)
    show_relative_geos = self.menu_handler.insert("Geometries", parent = show_relative)
    models_menu_handle = self.menu_handler.insert("Models", parent = show_relative_geos)
    for key in self.visu_config.relative.geometries.keys():
      handle = self.menu_handler.insert(key, parent = models_menu_handle, callback = self.showModelCb)
      self.visu_config.relative.geometries[key].handle = handle
      if self.visu_config.relative.geometries[key].show_geo or self.visu_config.relative.geometries[key].show_text:
        self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
      else:
        self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
    self.menu_handler.insert("All", parent = show_relative_geos, callback = self. showMetaCb)
    self.menu_handler.insert("None", parent = show_relative_geos, callback = self.showMetaCb)
    self.menu_handler.insert("Inverted", parent = show_relative_geos, callback = self.showMetaCb)
    show_relative_abs = self.menu_handler.insert("Abstractions", parent = show_relative)
    models_menu_handle = self.menu_handler.insert("Models", parent = show_relative_abs)
    for key in self.visu_config.relative.abstractions.keys():
      handle = self.menu_handler.insert(key, parent = models_menu_handle, callback = self.showModelCb)
      self.visu_config.relative.abstractions[key].handle = handle
      if self.visu_config.relative.abstractions[key].show_geo or self.visu_config.relative.abstractions[key].show_text:
        self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
      else:
        self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
    self.menu_handler.insert("All", parent = show_relative_abs, callback = self.showMetaCb)
    self.menu_handler.insert("None", parent = show_relative_abs, callback = self.showMetaCb)
    self.menu_handler.insert("Inverted", parent = show_relative_abs, callback = self.showMetaCb)

    show_absolute = self.menu_handler.insert("Absolute", parent = show)
    show_absolute_geos = self.menu_handler.insert("Geometries", parent = show_absolute)
    models_menu_handle = self.menu_handler.insert("Models", parent = show_absolute_geos)
    for key in self.visu_config.absolute.geometries.keys():
      handle = self.menu_handler.insert(key, parent = models_menu_handle, callback = self.showModelCb)
      self.visu_config.absolute.geometries[key].handle = handle
      if self.visu_config.absolute.geometries[key].show_geo or self.visu_config.absolute.geometries[key].show_text:
        self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
      else:
        self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
    self.menu_handler.insert("All", parent = show_absolute_geos, callback = self.showMetaCb)
    self.menu_handler.insert("None", parent = show_absolute_geos, callback = self.showMetaCb)
    self.menu_handler.insert("Inverted", parent = show_absolute_geos, callback = self.showMetaCb)
    show_absolute_abs = self.menu_handler.insert("Abstractions", parent = show_absolute)
    models_menu_handle = self.menu_handler.insert("Models", parent = show_absolute_abs)
    for key in self.visu_config.absolute.abstractions.keys():
      handle = self.menu_handler.insert(key, parent = models_menu_handle, callback = self.showModelCb)
      self.visu_config.absolute.abstractions[key].handle = handle
      if self.visu_config.absolute.abstractions[key].show_geo or self.visu_config.absolute.abstractions[key].show_text:
        self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
      else:
        self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
    self.menu_handler.insert("All", parent = show_absolute_abs, callback = self.showMetaCb)
    self.menu_handler.insert("None", parent = show_absolute_abs, callback = self.showMetaCb)
    self.menu_handler.insert("Inverted", parent = show_absolute_abs, callback = self.showMetaCb)

  def createGhostMarker(self):
    self.marker = InteractiveMarker()
    self.marker.name = self.obj.name
    self.marker.header.frame_id = "world"
    self.marker.scale = 1.0
    self.marker.description = "This is the ghost marker for object: " + self.obj.name

    if self.obj.description.type:
      self.label = self.obj.description.type
    else:
      self.label = "Unknown"
    self.label_pose = ROSPoseStamped()
    self.label_pose.header.frame_id = "world"
    self.label_pose = self.obj.pose
    self.label_pose.pose.position.z += 1.0

    createMenuControl(self.marker.controls, self.label_pose, self.label)
    updateGhostVisu(self.marker.controls, self.obj, self.visu_config)

    self.server.insert(self.marker, self.processFeedback)
    self.menu_handler.apply( self.server, self.marker.name )
    self.server.applyChanges()
