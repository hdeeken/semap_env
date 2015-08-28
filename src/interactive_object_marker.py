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
from semap_ros.srv import *
from semap_ros.service_calls import *
from semap_ros.instance_srv_calls import *
from semap_ros.description_srv_calls import *
from semap_msgs.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel, GeometryModelSet
from semap_msgs.msg import ColorCommand, LabelCommand
from semap_msgs.msg import ObjectDescription as ROSObjectDescription
from semap_msgs.msg import ObjectInstance as ROSObjectInstance
from interactive_object_marker_widgets import *
from assimp_postgis_importer import importFromFileToMesh
from semap_env.service_calls import *
from object_description_marker import *
from visualization import *

from shared_marker_util import *

class InteractiveObjectMarker():
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
      self.visu_config.relative.geometries = defaultRelativeDescriptionVisu(self.obj.description.geometries)
      self.visu_config.relative.abstractions = defaultRelativeAbstractionVisu(self.obj.description.abstractions)
      self.visu_config.absolute.geometries = defaultAbsoluteDescriptionVisu(self.obj.absolute.geometries)
      self.visu_config.absolute.abstractions = defaultAbsoluteAbstractionVisu(self.obj.absolute.abstractions)

    self.server = server
    self.tf_listener = tf.TransformListener()
    self.initSubscriber();
    self.menu_handler = MenuHandler()
    self.geometry_movement_menu_handler = MenuHandler()
    self.label_movement_menu_handler = MenuHandler()

    self.initInstanceMenu()
    self.initDescriptionMenu()

    self.createInteractiveMarker()

## Callbacks

  def labelObjectInstanceCb(self, feedback):
    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler
    server = self.server

    rospy.loginfo( "assign %s" % menu_handler.getTitle(handle) )
    if menu_handler.getTitle(handle) == "Type":
      name = self.obj.description.type
    elif menu_handler.getTitle(handle) == "Alias":
      if self.obj.alias != "":
        name = self.obj.alias
      else:
        print "labeled object by type, there's no alias"
        name = self.obj.description.type
    elif menu_handler.getTitle(handle) == "Name":
      name = self.obj.name
    elif menu_handler.getTitle(handle) == "None":
      for control in self.marker.controls:
        if control.name == "Menu":
          self.marker.controls.remove(control)
          return

    updateMenuControl(self.marker.controls, self.label_pose, name)
    server.applyChanges()
    self.update()

  def infoInstanceCb(self, feedback):
    print "Id:", self.obj.id, "Name:", self.obj.name, "Alias:", self.obj.alias, "Type:", self.obj.description.type, "Type Id:", self.obj.description.id

  def processGeometryMovementFeedback(self, feedback ):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
      self.geometry_movement_pose = feedback.pose
    self.server.applyChanges()

  def processLabelMovementFeedback(self, feedback ):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
      self.label_pose.pose = feedback.pose
    self.server.applyChanges()

  def processFeedback(self, feedback ):
      s = "Feedback from marker '" + feedback.marker_name
      s += "' / control '" + feedback.control_name + "'"

      mp = ""
      if feedback.mouse_point_valid:
          mp = " at " + str(feedback.mouse_point.x)
          mp += ", " + str(feedback.mouse_point.y)
          mp += ", " + str(feedback.mouse_point.z)
          mp += " in frame " + feedback.header.frame_id

      #if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
      #    rospy.loginfo( s + ": button click" + mp + "." )
      #elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
      #    rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
      if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
          #rospy.loginfo( s + ": pose changed")
          #rospy.loginfo( feedback.pose)
          self.current_pose = feedback.pose
      #elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
      #    rospy.loginfo( s + ": mouse down" + mp + "." )
      #elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
          #rospy.loginfo( s + ": mouse up" + mp + "." )

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
    updateVisuControl(self.marker.controls, self.obj, self.visu_config)
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
    updateVisuControl(self.marker.controls, self.obj, self.visu_config)
    server.applyChanges()


  def showGeometryModelCb(self, feedback):
    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler
    server = self.server

    menu_handle = findParent(menu_handler, self.instance_menu_handle, handle)
    model_handle = findParent(menu_handler, self.models_menu_handle, handle)

    if menu_handler.getTitle(model_handle) in self.model_visu.keys():
      state = menu_handler.getCheckState( model_handle )

      if state == MenuHandler.CHECKED:
        menu_handler.setCheckState( model_handle, MenuHandler.UNCHECKED )
        self.model_visu[menu_handler.getTitle(model_handle)].show_geo = False
        self.model_visu[menu_handler.getTitle(model_handle)].show_text = False
      else:
        menu_handler.setCheckState( model_handle, MenuHandler.CHECKED )
        self.model_visu[menu_handler.getTitle(model_handle)].show_geo = True
        self.model_visu[menu_handler.getTitle(model_handle)].show_text = True

    elif menu_handler.getTitle(handle) == "All":
      for key in self.model_visu.keys():
        self.model_visu[key].show_geo = True
        self.model_visu[key].show_text = True
        menu_handler.setCheckState( self.model_entry[key], MenuHandler.CHECKED )

    elif menu_handler.getTitle(handle) == "None":
      for key in self.model_visu.keys():
        self.model_visu[key].show_geo = False
        self.model_visu[key].show_text = False
        menu_handler.setCheckState( self.model_entry[key], MenuHandler.UNCHECKED )

    elif menu_handler.getTitle(handle) == "Inverted":
      for key in self.model_visu.keys():
        if menu_handler.getCheckState( self.model_entry[key] ) == MenuHandler.CHECKED:
          menu_handler.setCheckState( self.model_entry[key], MenuHandler.UNCHECKED )
          self.model_visu[menu_handler.getTitle(self.model_entry[key])].show_geo = False
          self.model_visu[menu_handler.getTitle(self.model_entry[key])].show_text = False
        elif menu_handler.getCheckState( self.model_entry[key] ) == MenuHandler.UNCHECKED:
          menu_handler.setCheckState( self.model_entry[key], MenuHandler.CHECKED )
          self.model_visu[menu_handler.getTitle(self.model_entry[key])].show_geo = True
          self.model_visu[menu_handler.getTitle(self.model_entry[key])].show_text = True
        else:
          print 'no checkbox... shouldnt happen'
    else:
      rospy.logwarn('showGeometryModelCb: Unknown MenuEntry %s was called.' % menu_handler.getTitle(handle))

    menu_handler.reApply( server )
    updateVisuControl(self.marker.controls, self.obj, self.model_visu, False)
    server.applyChanges()

  def removeGeometryModelCb(self, feedback):
    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler
    server = self.server

    meta_handle = findParent(menu_handler, self.models_menu_handle, handle)
    model_handle = findParent(menu_handler, self.models_menu_handle, meta_handle)
    rospy.loginfo("Remove %s" % menu_handler.getTitle(model_handle))
    if menu_handler.getTitle(model_handle) in self.visu_config.relative.geometries.keys():
      res = call_remove_geometry_model(self.visu_config.relative.geometries[menu_handler.getTitle(model_handle)].id)
    #elif menu_handler.getTitle(handle) == "All":
      #rospy.loginfo("Remove %s" % menu_handler.getTitle(handle))
      #for key in self.model_visu.keys():
        #res = call_remove_geometry_model(self.obj.description.id, key)
    #elif menu_handler.getTitle(handle) == "Shown":
      #rospy.loginfo("Remove %s" % menu_handler.getTitle(handle))
      #for key in self.model_visu.keys():
        #if menu_handler.getCheckState( self.model_entry[key] ) == MenuHandler.CHECKED:
          #res = call_remove_geometry_model(self.obj.description.id, key)
    #elif menu_handler.getTitle(handle) == "Not Shown":
      #rospy.loginfo("Remove %s" % menu_handler.getTitle(handle))
      #for key in self.model_visu.keys():
        #if menu_handler.getCheckState( self.model_entry[key] ) == MenuHandler.UNCHECKED:
          #res = call_remove_geometry_model(self.obj.description.id, key)
    else:
      rospy.logwarn('removeGeometryModelCb: Unknown MenuEntry %s was called.' % menu_handler.getTitle(handle))

    menu_handler.reApply( server )
    #updateVisuControl(self.marker.controls, self.obj, self.model_visu)
    server.applyChanges()
    call_refresh_objects(res.ids)

  def moveObjectInstanceCb(self, feedback):
    menu_handler = self.menu_handler
    server = self.server
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState( handle )

    if state == MenuHandler.CHECKED:
      removeControl(self.marker.controls, "MotionControl")
    else:
      if menu_handler.getTitle(handle) == "3D":
        removeControl(self.marker.controls, "MotionControl")
        create3DMotionControl(self.marker.controls, True)
      elif menu_handler.getTitle(handle) == "6D":
        removeControl(self.marker.controls, "MotionControl")
        create6DMotionControl(self.marker.controls, True)

    switchCheckState(menu_handler, feedback.menu_entry_id)
    menu_handler.reApply( server )
    server.applyChanges()

  def frameCb(self, feedback):
    app = QApplication(sys.argv)
    widget = ChooseReferenceFrameWidget(self.obj.name)
    widget.exec_()
    name, keep_transform = widget.getChoice()
    res = call_change_frame(self.obj.id, name , keep_transform)
    call_refresh_objects(res.ids)

  def update(self):
    call_activate_objects([self.obj.id])

  def saveChangesCb(self, feedback):
    print 'commit all pending changes to db'

  def saveObjectDescriptionCb(self, feedback):
    print 'commit all desc changes to db'

  def saveObjectInstanceCb(self, feedback):
    if self.current_pose != None:
      res = call_update_transform(self.obj.id, self.current_pose)
    call_refresh_objects(res.ids)

  def resetObjectInstanceCb(self, feedback):
    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler

    if menu_handler.getTitle(handle) == "Origin":
      rospy.loginfo("Reset %s" % menu_handler.getTitle(handle))
      pose = Pose()
      pose.orientation.w = 1.0
      call_set_transform(self.obj.id, pose)
    elif menu_handler.getTitle(handle) == "Pose":
      rospy.loginfo("Reset %s" % menu_handler.getTitle(handle))
      app = QApplication(sys.argv)
      widget = SetPose()
      widget.show()
      app.exec_()

      tmp = widget.getPose()
      pose = Pose()
      pose.position.x = tmp[0][0]
      pose.position.y = tmp[0][1]
      pose.position.z = tmp[0][2]
      pose.orientation.x = tmp[1][0]
      pose.orientation.y = tmp[1][1]
      pose.orientation.z = tmp[1][2]
      pose.orientation.z = tmp[1][3]
      call_set_transform(self.obj.id, pose)

    elif menu_handler.getTitle(handle) == "Instance":
      rospy.loginfo("Reset %s" % menu_handler.getTitle(handle))
      app = QApplication(sys.argv)
      widget = ChooseObjectInstanceWidget(self.obj.name)
      widget.show()
      app.exec_()
      name, id, pose = widget.getChoice()

      res = call_get_transform(self.obj.name, name)
      pose = Pose()
      pose.position.x = res.transform.transform.translation.x
      pose.position.y = res.transform.transform.translation.y
      pose.position.z = res.transform.transform.translation.z
      pose.orientation.x = res.transform.transform.rotation.x
      pose.orientation.y = res.transform.transform.rotation.y
      pose.orientation.z = res.transform.transform.rotation.z
      pose.orientation.w = res.transform.transform.rotation.w
      call_update_transform(self.obj.id, pose)
    else:
      rospy.logwarn('removeGeometryModelCb: Unknown MenuEntry %s was called.' % menu_handler.getTitle(handle))

    self.update()

  def rotateObjectInstanceCb(self, feedback):
    axis_handle = findParent(self.menu_handler, self.instance_menu_handle,  feedback.menu_entry_id)
    #print 'handle', self.menu_handler.getTitle(axis_handle), self.menu_handler.getTitle(feedback.menu_entry_id)
    pose = getRotation(self.menu_handler.getTitle(axis_handle), self.menu_handler.getTitle(feedback.menu_entry_id))
    #print 'pose',pose
    call_update_transform(self.obj.id, pose)
    self.update()

  def deleteObjectInstanceCb(self, feedback):
    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler

    if menu_handler.getTitle(handle) == "Keep Children":
      delete_res = call_delete_object_instances([self.obj.id], True)
    elif menu_handler.getTitle(handle) == "Redirect Children":
      app = QApplication(sys.argv)
      widget = ChooseReferenceFrameWidget(self.obj.name)
      widget.show()
      app.exec_()
      self.update()
      name, keep_transform = widget.getChoice()
      delete_res = call_delete_object_instances([self.obj.id], True, name, keep_transform)
      self.app.reactivate_objects()
    elif menu_handler.getTitle(handle) == "Remove Children":
      delete_res = call_delete_object_instances([self.obj.id], False)

    deactivate_res = call_deactivate_objects(delete_res.ids)

    for id in delete_res.ids:
      self.server.erase( "object" + str(id) )

    self.server.applyChanges()

  def copyObjectInstanceCb(self, feedback):
    res = call_copy_object_instances([self.obj.id])
    call_refresh_objects(res.ids)
    self.server.applyChanges()

  def renameObjectInstanceCb(self, feedback):
    app = QApplication(sys.argv)
    widget = SetNameWidget()
    widget.show()
    app.exec_()
    call_rename_object_instance(self.obj.id, widget.getName())
    self.update()

  def renameObjectDescriptionCb(self, feedback):
    app = QApplication(sys.argv)
    widget = SetNameWidget()
    widget.show()
    app.exec_()
    res = call_rename_object_description(self.obj.description.id, widget.getName())
    call_refresh_objects( res.ids )

  def renameGeometryModelCb(self, feedback):

    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler
    meta_handle = findParent(menu_handler, self.models_menu_handle, handle)
    model_handle = findParent(menu_handler, self.models_menu_handle, meta_handle)
    model_id = self.visu_config.relative.geometries[menu_handler.getTitle(model_handle)].id

    app = QApplication(sys.argv)
    widget = SetNameWidget()
    widget.show()
    app.exec_()
    res = call_rename_geometry_model( model_id, widget.getName() )
    call_refresh_objects( res.ids )

  def resetGeometryModelCb(self, feedback):
    menu_handler = self.menu_handler

    handle = handle = feedback.menu_entry_id
    reset_handle = findParent(menu_handler, self.models_menu_handle, handle)
    move_handle = findParent(menu_handler, self.models_menu_handle, reset_handle)
    model_handle = findParent(menu_handler, self.models_menu_handle, move_handle)
    model_id = self.visu_config.relative.geometries[menu_handler.getTitle(model_handle)].id
    print 'reset model:', model_id
    if menu_handler.getTitle(handle) == "Origin":
      rospy.loginfo("Reset %s" % menu_handler.getTitle(handle))
      pose = Pose()
      pose.orientation.w = 1.0
      res= call_set_geometry_model_pose(model_id, pose)
    elif menu_handler.getTitle(handle) == "Pose":
      rospy.loginfo("Reset %s" % menu_handler.getTitle(handle))
      app = QApplication(sys.argv)
      widget = SetPose()
      widget.show()
      app.exec_()

      tmp = widget.getPose()
      pose = Pose()
      pose.position.x = tmp[0][0]
      pose.position.y = tmp[0][1]
      pose.position.z = tmp[0][2]
      pose.orientation.x = tmp[1][0]
      pose.orientation.y = tmp[1][1]
      pose.orientation.z = tmp[1][2]
      pose.orientation.z = tmp[1][3]
      res = call_set_geometry_model_pose(model_id, pose)

    elif menu_handler.getTitle(handle) == "Z-Align":
      print 'get da bb'
      bb_res = call_get_geometry_model_bb(model_id)
      print 'da BB is', bb_res
      pose = ROSPose()
      pose.position.z = 0.0 - bb_res.min_z
      res = call_update_geometry_model_pose(model_id, pose)

    call_refresh_objects( res.ids )


  def rotateGeometryModelCb(self, feedback):
  
    menu_handler = self.menu_handler
    degree_handle = handle = feedback.menu_entry_id
    axis_handle = findParent(menu_handler, self.models_menu_handle, degree_handle)
    rotate_handle = findParent(menu_handler, self.models_menu_handle, axis_handle)
    move_handle = findParent(menu_handler, self.models_menu_handle, rotate_handle)
    model_handle = findParent(menu_handler, self.models_menu_handle, move_handle)
    print 'rotate', menu_handler.getTitle(model_handle),' about ', menu_handler.getTitle(axis_handle), menu_handler.getTitle(degree_handle)
   
    pose = getRotation( menu_handler.getTitle(axis_handle), menu_handler.getTitle(degree_handle))
    #call_update_geometry_model_pose(self.visu_config.relative.geometries[menu_handler.getTitle(model_handle)].id, pose)
    print 'in', pose
    call_update_and_transform_geometry_model_pose(self.visu_config.relative.geometries[menu_handler.getTitle(model_handle)].id, pose)
    res = call_update_object_descriptions( [self.obj.description.id] )
    call_refresh_objects( res.ids )
    print 'DAT UPDATE IS OVER, VISU IS UP'
    
  def moveLabelCb(self, feedback):
    server = self.server
    menu_handler = self.menu_handler

    if not self.label_movement:
      self.label_movement_id = self.obj.id
      self.label_movement = True
      self.createLabelMovement()

    server.applyChanges()

  def saveLabelMovementCb(self, feedback):
    if self.label_movement:
      self.server.erase(self.label)
      self.label_movement = False
      updateMenuControl(self.marker.controls, self.label_pose, self.label)
      self.server.applyChanges()

  def moveGeometryModelCb(self, feedback):
    server = self.server
    menu_handler = self.menu_handler

    handle = feedback.menu_entry_id
    move_handle = findParent(menu_handler, self.models_menu_handle, handle)
    model_handle = findParent(menu_handler, self.models_menu_handle, move_handle)
    model_id = self.visu_config.relative.geometries[menu_handler.getTitle(model_handle)].id

    if not self.geometry_movement:
      self.geometry_movement_id = model_id
      self.geometry_movement = True
      model = self.getModelFromGeometrySet(self.obj.description.geometries, model_id)
      self.createGeometryMovement(model, self.visu_config.relative.geometries)

    server.applyChanges()

  def saveGeometryModelCb(self, feedback):
    if self.geometry_movement:
      res = call_set_geometry_model_pose(self.geometry_movement_id, self.geometry_movement_pose)
      self.server.erase("GeometryMotion")
      self.geometry_movement = False
      self.geometry_movement_id = None
      self.server.applyChanges()
      call_refresh_objects( res.ids )

  def switchObjectDescriptionCb(self, feedback):
    app = QApplication(sys.argv)
    widget = ChooseObjectDescriptionWidget(self.obj.description.type)
    widget.show()
    app.exec_()

   # print 'get choices from wid'
    desc_name, desc_id = widget.getChoice()
  #  print 'got choices from wid'
    del app, widget
   # print 'got:', desc_name, desc_id
    if(desc_id == -1):
      new_desc = ROSObjectDescription()
      new_desc.type = desc_name
      res = call_add_object_descriptions([new_desc])
      call_switch_object_descriptions([self.obj.id], res.ids[0])
    else:
      call_switch_object_descriptions([self.obj.id], desc_id)

    self.update()

  def deleteObjectDescriptionCb(self, feedback):
    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler
    if menu_handler.getTitle(handle) == "Remove Corresponding Instances":
      delete_res = call_delete_object_descriptions([self.obj.description.id], False)
      deactivate_res = call_deactivate_objects(delete_res.ids)
      for id in delete_res.ids:
        self.server.erase( "object" + str(id) )
      self.server.applyChanges()
      return
    elif menu_handler.getTitle(handle) == "Switch Instances To...":
      app = QApplication(sys.argv)
      widget = ChooseObjectDescriptionWidget(self.obj.description.type)
      widget.show()
      app.exec_()
      desc_name, desc_id = widget.getChoice()
      delete_res = call_delete_object_descriptions([self.obj.description.id], True, desc_id)
    elif menu_handler.getTitle(handle) == "Strip Instances of Description":
      delete_res = call_delete_object_descriptions([self.obj.description.id], True)
    else:
      print 'fuck nothin happend'
      return

    deactivate_res = call_refresh_objects(delete_res.ids)
    self.server.applyChanges()
    self.update()

  def copyObjectDescriptionCb(self, feedback):
    res = call_copy_object_descriptions([self.obj.description.id])

### ADD GEOMETRY MODELS ##################################

  def point2dCb(self, feedback):
    self.process_point2d_ = True

  def point2dtopicCb(self, point):
    if self.process_point2d_:
      self.process_point2d_ = False

      try:
        self.tf_listener.waitForTransform(self.obj.name, point.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
        new_point = self.tf_listener.transformPoint(self.obj.name, point)
        model = Point2DModel()
        model.geometry = new_point.point

        app = QApplication(sys.argv)
        widget = SetGeometryModelTypeWidget()
        widget.show()
        app.exec_()


        model.type = widget.getType()
        call_add_point_2d_model(self.obj.description.id, model)
        res = call_update_object_descriptions( [self.obj.description.id] )
        call_refresh_objects( res.ids )
      except tf.Exception as e:
        print "some tf exception happened %s" % e

  def pose2dCb(self, feedback):
    print 'waiting for 2d pose'
    self.process_pose2d_ = True

  def pose2dtopicCb(self, pose):
    if self.process_pose2d_:
      self.process_pose2d_ = False
      print 'processing 2d pose - IS NOT IMPLEMENTED YET'

  def polygon2dCb(self, feedback):
    print 'waiting for 2d polygon'
    self.process_polygon2d_ = True

  def polygon2dtopicCb(self, polygon):
    if self.process_polygon2d_:
      self.process_polygon2d_ = False
      transformed_polygon = transformPolygonStamped(self.tf_listener, self.obj.name, polygon)
      model = Polygon2DModel()
      model.geometry = transformed_polygon.polygon

      app = QApplication(sys.argv)
      widget = SetGeometryModelTypeWidget()
      widget.show()
      app.exec_()

      model.type = widget.getType()
      call_add_polygon_2d_model(self.obj.description.id, model)
      res = call_update_object_descriptions( [self.obj.description.id] )
      call_refresh_objects( res.ids )

  def point3dCb(self, feedback):
    print 'waiting for 3d point'
    self.process_point3d_ = True

  def point3dtopicCb(self, point):
    if self.process_point3d_:
      self.process_point3d_ = False
      print 'processing 3d point'
      try:
        self.tf_listener.waitForTransform(self.obj.name, point.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
        new_point = self.tf_listener.transformPoint(self.obj.name, point)
        model = Point3DModel()
        model.geometry = new_point.point

        app = QApplication(sys.argv)
        widget = SetGeometryModelTypeWidget()
        widget.show()
        app.exec_()

        model.type = widget.getType()
        call_add_point_3d_model(self.obj.description.id, model)
        res = call_update_object_descriptions( [self.obj.description.id] )
        call_refresh_objects( res.ids )
      except tf.Exception as e:
        print "some tf exception happened %s" % e

  def pose3dCb(self, feedback):
    print 'waiting for 3d pose'
    self.process_pose3d_ = True

  def pose3dtopicCb(self, pose):
    if self.process_pose3d_:
      self.process_pose3d_ = False
      print 'processing 3d pose'
      try:
        self.tf_listener.waitForTransform(self.obj.name, pose.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
        new_point = self.tf_listener.transformPose(self.obj.name, pose)
        model = Pose3DModel()
        model.pose = new_point.pose

        app = QApplication(sys.argv)
        widget = SetGeometryModelTypeWidget()
        widget.show()
        app.exec_()

        model.type = widget.getType()
        call_add_pose_3d_model(self.obj.description.id, model)
        res = call_update_object_descriptions( [self.obj.description.id] )
        call_refresh_objects( res.ids )
      except tf.Exception as e:
        print "some tf exception happened %s" % e

  def polygon3dCb(self, feedback):
    print 'waiting for 3d polygon'
    self.process_polygon3d_ = True

  def polygon3dtopicCb(self, polygon):
    if self.process_polygon3d_:
      self.process_polygon3d_ = False
      print 'processing 3d point'
      transformed_polygon = transformPolygonStamped(self.tf_listener, self.obj.name, polygon)

      model = Polygon2DModel()
      model.geometry = transformed_polygon.polygon

      app = QApplication(sys.argv)
      widget = SetGeometryModelTypeWidget()
      widget.show()
      app.exec_()

      model.type = widget.getType()

      call_add_polygon_3d_model(self.obj.description.id, model)
      res = call_update_object_descriptions( [self.obj.description.id] )
      call_refresh_objects( res.ids )

  def meshFileCb(self, feedback):
    app = QApplication(sys.argv)
    dlg=QFileDialog( )
    dlg.setWindowTitle( 'Select MeshFile' )
    dlg.setViewMode( QFileDialog.Detail )
    dlg.setFileMode( QFileDialog.ExistingFile )
    #dlg.setNameFilters( [self.tr('Text Files (*.txt)'), self.tr('All Files (*)')] )
    if dlg.exec_() :
        mesh = importFromFileToMesh(dlg.selectedFiles()[0].encode('ascii','ignore'))
        model = TriangleMesh3DModel()

        widget = SetGeometryModelTypeWidget()
        widget.show()
        app.exec_()
        model.type = widget.getType()

        widget = MeshTransformationWidget()
        widget.show()
        app.exec_()

        if widget.hasModifier() :
          mesh = transformMesh(mesh, widget.getModifier())

        model.geometry = mesh
        call_add_triangle_mesh_3d_model(self.obj.description.id, model)
        res = call_update_object_descriptions( [self.obj.description.id] )
        call_refresh_objects( res.ids )

  def meshToolCb(self, feedback):
    print 'waiting for mesh via tool'

  def polymeshCb(self, feedback):
    print 'waiting for polymesh'

  def colorCb(self, feedback):
  
    
    if feedback.obj_name == self.obj.name or feedback.desc_type == self.obj.description.type:
      print 'label command incoming'
      print feedback.model_type, self.visu_config.relative.geometries.keys()
      
      if feedback.model_type in self.visu_config.relative.geometries.keys():
        self.visu_config.relative.geometries[feedback.model_type].show_geo = feedback.show_geo
        self.visu_config.relative.geometries[feedback.model_type].geo_color = feedback.geo_color
        self.visu_config.relative.geometries[feedback.model_type].geo_scale = feedback.geo_scale
        self.visu_config.relative.geometries[feedback.model_type].show_text = feedback.geo_color
        self.visu_config.relative.geometries[feedback.model_type].text_color = feedback.text_color
        self.visu_config.relative.geometries[feedback.model_type].text_scale = feedback.text_scale
        self.visu_config.relative.geometries[feedback.model_type].text_offset = feedback.text_offset
        updateVisuControl(self.marker.controls, self.obj, self.visu_config)
        self.server.applyChanges()
      self.update()

  def labelCb(self, feedback):
    
    if feedback.obj_name == self.obj.name or feedback.desc_type == self.obj.description.type:

      if feedback.set_pose:
        self.label_pose.pose = feedback.label_pose
        updateMenuControl(self.marker.controls, self.label_pose, self.label)

      if feedback.label_type == "Type":
        self.label = self.obj.description.type
        updateMenuControl(self.marker.controls, self.label_pose, self.label)
      elif feedback.label_type == "Alias":
        if self.obj.alias != "":
          self.label = self.obj.alias
          updateMenuControl(self.marker.controls, self.label_pose, self.label)
        else:
          self.label = self.obj.description.type
          updateMenuControl(self.marker.controls, self.label_pose, self.label)
      elif feedback.label_type == "Name":
        self.label = self.obj.name
        updateMenuControl(self.marker.controls, self.label_pose, self.label)

      elif feedback.label_type == "None":
        for control in self.marker.controls:
          if control.name == "Menu":
            self.marker.controls.remove(control)
            return

    self.server.applyChanges()


  def printLabelCb(self, feedback):

    if feedback.data == "label":
      string = 'rostopic pub /label_cmd semap_msgs/LabelCommand '
      string += '"obj_name: ' + "%r" % self.obj.name +"\n"
      string += "desc_type: ''" +"\n"
      string += "label_type: 'Alias'" +"\n"
      string += "set_pose: true" +"\n"
      string += "label_pose: " +"\n"
      string += "  position:" +"\n"
      string += "    x: " + str(self.label_pose.pose.position.x) +"\n"
      string += "    y: " + str(self.label_pose.pose.position.y) +"\n"
      string += "    z: " + str(self.label_pose.pose.position.z)+"\n"
      string += "  orientation:" +"\n"
      string += "    x: 0.0" +"\n"
      string += "    y: 0.0" +"\n"
      string += "    z: 0.0" +"\n"
      string += '    w: 0.0"' +" -1 \n"

      print string

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

    move = self.menu_handler.insert("Move", parent = self.instance_menu_handle)
    move_6d = self.menu_handler.insert("6D", parent = move, callback = self.moveObjectInstanceCb)
    self.menu_handler.setCheckState(move_6d, MenuHandler.UNCHECKED)
    move_3d = self.menu_handler.insert("3D", parent = move, callback = self.moveObjectInstanceCb)
    self.menu_handler.setCheckState(move_3d, MenuHandler.UNCHECKED)

    rotate = self.menu_handler.insert("Rotate...", parent = move)
    roll = self.menu_handler.insert("Roll", parent = rotate)
    self.menu_handler.insert("90", parent = roll, callback = self.rotateObjectInstanceCb)
    self.menu_handler.insert("180", parent = roll, callback = self.rotateObjectInstanceCb)
    self.menu_handler.insert("270", parent = roll, callback = self.rotateObjectInstanceCb)
    pitch = self.menu_handler.insert("Pitch", parent = rotate)
    self.menu_handler.insert("90", parent = pitch, callback = self.rotateObjectInstanceCb)
    self.menu_handler.insert("180", parent = pitch, callback = self.rotateObjectInstanceCb)
    self.menu_handler.insert("270", parent = pitch, callback = self.rotateObjectInstanceCb)
    yaw = self.menu_handler.insert("Yaw", parent = rotate)
    self.menu_handler.insert("90", parent = yaw, callback = self.rotateObjectInstanceCb)
    self.menu_handler.insert("180", parent = yaw, callback = self.rotateObjectInstanceCb)
    self.menu_handler.insert("270", parent = yaw, callback = self.rotateObjectInstanceCb)
    reset = self.menu_handler.insert("Reset To...", parent = move)
    self.menu_handler.insert("Origin", parent = reset, callback = self.resetObjectInstanceCb)
    self.menu_handler.insert("Pose", parent = reset, callback = self.resetObjectInstanceCb)
    self.menu_handler.insert("Instance", parent = reset, callback = self.resetObjectInstanceCb)
    self.menu_handler.insert("Change Reference Frame", parent = move, callback = self.frameCb)

    save = self.menu_handler.insert("Save Changes", parent = self.instance_menu_handle, callback = self.saveObjectInstanceCb)
    self.menu_handler.setVisible(save, True)

    meta = self.menu_handler.insert("Meta", parent = self.instance_menu_handle)
    self.menu_handler.insert("Info", parent = meta, callback = self.infoInstanceCb)
    self.menu_handler.insert("Rename", parent = meta, callback = self.renameObjectInstanceCb)
    self.menu_handler.insert("Switch", parent = meta, callback = self.switchObjectDescriptionCb)
    self.menu_handler.insert("Copy", parent = meta, callback = self.copyObjectInstanceCb)
    delete = self.menu_handler.insert("Delete", parent = meta, callback = self.deleteObjectInstanceCb)
    self.menu_handler.insert("Keep Children", parent = delete, callback = self.deleteObjectInstanceCb)
    self.menu_handler.insert("Redirect Children", parent = delete, callback = self.deleteObjectInstanceCb)
    self.menu_handler.insert("Remove Children", parent = delete, callback = self.deleteObjectInstanceCb)
    label = self.menu_handler.insert("Label", parent = meta)
    self.menu_handler.insert("Type",  parent = label, callback = self.labelObjectInstanceCb)
    self.menu_handler.insert("Alias", parent = label, callback = self.labelObjectInstanceCb)
    self.menu_handler.insert("Name", parent = label, callback = self.labelObjectInstanceCb)
    self.menu_handler.insert("None", parent = label, callback = self.labelObjectInstanceCb)
    self.menu_handler.insert("Move", parent = label, callback = self.moveLabelCb )

  def initDescriptionMenu(self):
    self.description_menu_handle = self.menu_handler.insert("Description")

    save = self.menu_handler.insert("Save Changes", parent = self.description_menu_handle, callback = self.saveObjectDescriptionCb)
    self.menu_handler.setVisible(save, False)

    #show = self.menu_handler.insert("Show", parent = self.description_menu_handle)
    #self.menu_handler.insert("All", parent = show, callback = self.showModelCb)
    #self.menu_handler.insert("None", parent = show, callback = self.showModelCb)
    #self.menu_handler.insert("Inverted", parent = show, callback = self.showModelCb)

    #remove = self.menu_handler.insert("Remove", parent = self.description_menu_handle)
    #self.menu_handler.insert("All", parent = remove, callback = self.removeGeometryModelCb)
    #self.menu_handler.insert("Shown", parent = remove, callback = self.removeGeometryModelCb)
    #self.menu_handler.insert("Not Shown", parent = remove, callback = self.removeGeometryModelCb)

    add = self.menu_handler.insert("Add", parent = self.description_menu_handle)
    self.menu_handler.insert("2D Point", parent = add, callback = self.point2dCb)
    self.menu_handler.insert("2D Pose", parent = add, callback = self.pose2dCb)
    self.menu_handler.insert("2D Polygon", parent = add, callback = self.polygon2dCb)
    self.menu_handler.insert("3D Point", parent = add, callback = self.point3dCb)
    self.menu_handler.insert("3D Pose", parent = add, callback = self.pose3dCb)
    self.menu_handler.insert("3D Polygon", parent = add, callback = self.polygon3dCb)
    self.menu_handler.insert("3D Triangle Mesh", parent = add, callback = self.meshFileCb)
    self.menu_handler.insert("3D Polygon Mesh", parent = add, callback = self.polymeshCb)

    self.models_menu_handle = self.menu_handler.insert("Models", parent = self.description_menu_handle)

    for key in self.visu_config.relative.geometries:
      model_menu = self.menu_handler.insert(key, parent = self.models_menu_handle)

      move = self.menu_handler.insert("Move", parent = model_menu)
      self.menu_handler.insert("Freely", parent = move, callback = self.moveGeometryModelCb )

      rotate = self.menu_handler.insert("Rotate...", parent = move)
      roll = self.menu_handler.insert("Roll", parent = rotate)
      self.menu_handler.insert("90", parent = roll, callback = self.rotateGeometryModelCb)
      self.menu_handler.insert("180", parent = roll, callback = self.rotateGeometryModelCb)
      self.menu_handler.insert("270", parent = roll, callback = self.rotateGeometryModelCb)
      pitch = self.menu_handler.insert("Pitch", parent = rotate)
      self.menu_handler.insert("90", parent = pitch, callback = self.rotateGeometryModelCb)
      self.menu_handler.insert("180", parent = pitch, callback = self.rotateGeometryModelCb)
      self.menu_handler.insert("270", parent = pitch, callback = self.rotateGeometryModelCb)
      yaw = self.menu_handler.insert("Yaw", parent = rotate)
      self.menu_handler.insert("90", parent = yaw, callback = self.rotateGeometryModelCb)
      self.menu_handler.insert("180", parent = yaw, callback = self.rotateGeometryModelCb)
      self.menu_handler.insert("270", parent = yaw, callback = self.rotateGeometryModelCb)
      reset = self.menu_handler.insert("Reset To...", parent = move)
      self.menu_handler.insert("Origin", parent = reset, callback = self.resetGeometryModelCb)
      self.menu_handler.insert("Pose", parent = reset, callback = self.resetGeometryModelCb)
      self.menu_handler.insert("Z-Align", parent = reset, callback = self.resetGeometryModelCb)
      

      meta = self.menu_handler.insert("Meta", parent = model_menu)
      self.menu_handler.insert("Rename", parent = meta, callback = self.renameGeometryModelCb)
      self.menu_handler.insert("Remove", parent = meta, callback = self.removeGeometryModelCb)

    meta = self.menu_handler.insert("Meta", parent = self.description_menu_handle)
    self.menu_handler.insert("Rename", parent = meta, callback = self.renameObjectDescriptionCb)
    self.menu_handler.insert("Copy", parent = meta, callback = self.copyObjectDescriptionCb)
    delete = self.menu_handler.insert("Delete", parent = meta, callback = self.deleteObjectDescriptionCb)
    self.menu_handler.insert("Remove Corresponding Instances", parent = delete, callback = self.deleteObjectDescriptionCb)
    self.menu_handler.insert("Switch Instances To...", parent = delete, callback = self.deleteObjectDescriptionCb)
    self.menu_handler.insert("Strip Instances of Description", parent = delete, callback = self.deleteObjectDescriptionCb)

  def createInteractiveMarker(self):
    self.marker = InteractiveMarker()
    self.marker.name = self.obj.name
    self.marker.header.frame_id = self.obj.name
    self.marker.scale = 1.0
    self.marker.description = "This is the interactive marker for object: " + self.obj.name

    if self.obj.description.type:
      self.label = self.obj.description.type + str(self.obj.id)
    else:
      self.label = "Unknown"
    self.label_pose = ROSPoseStamped()
    self.label_pose.header.frame_id = self.obj.name
    self.label_pose.pose.position.z += 1.0

    createMenuControl(self.marker.controls, self.label_pose, self.label)
    createVisuControl(self.marker.controls, self.obj, self.visu_config)

    self.server.insert(self.marker, self.processFeedback)
    self.menu_handler.apply( self.server, self.marker.name )
    self.server.applyChanges()

  def getModelFromGeometrySet(self, geo_set, model_id):
    for model in geo_set.point2d_models:
      if model_id == model.id:
        return model

    for model in geo_set.pose2d_models:
      if model_id == model.id:
        return model

    for model in geo_set.polygon2d_models:
      if model_id == model.id:
        return model

    for model in geo_set.point3d_models:
      if model_id == model.id:
        return model

    for model in geo_set.pose3d_models:
      if model_id == model.id:
        return model

    for model in geo_set.polygon3d_models:
      if model_id == model.id:
        return model

    for model in geo_set.trianglemesh3d_models:
      if model_id == model.id:
        return model

    for model in geo_set.polygonmesh3d_models:
      if model_id == model.id:
        return model

  def createGeometryMovement(self, model, model_visu):
    self.geometry_movement_marker = InteractiveMarker()

    pose = ROSPoseStamped()
    pose.header.frame_id = self.marker.name
    pose.pose.orientation.w = 1.0

    if type(model) is Point2DModel:
      pose.pose.position.x = model.geometry.x
      pose.pose.position.y = model.geometry.y
      pose.pose.position.z = 0.0

    elif type(model) is Pose2DModel:
      quat = quaternion_from_euler(0, 0, model.pose.theta)
      pose.pose.position.x = model.pose.x
      pose.pose.position.y = model.pose.y
      pose.pose.position.z = 0.0
      pose.pose.orientation.x = quat[0]
      pose.pose.orientation.y = quat[1]
      pose.pose.orientation.z = quat[2]
      pose.pose.orientation.w = quat[3]

    elif type(model) is Point3DModel:
      pose.pose.position = model.geometry
    else:
      pose.pose = model.pose

    self.geometry_movement_marker.name = "GeometryMotion"
    self.geometry_movement_marker.pose = pose.pose
    self.geometry_movement_marker.header.frame_id = self.marker.name
    self.geometry_movement_marker.scale = 1.0
    self.geometry_movement_marker.description = "This is a GeometryMovementMarker"

    createMenuControl(self.geometry_movement_marker.controls, pose, self.geometry_movement_marker.name)
    createModelVisuControl(self.geometry_movement_marker.controls, self.marker.name, model, model_visu)
    if type(model) == Point2DModel or type(model) == Pose2DModel or type(model) == Polygon2DModel:
      create3DMotionControl(self.geometry_movement_marker.controls, True)
    else:
      create6DMotionControl(self.geometry_movement_marker.controls, True)

    self.geometry_movement_menu_handler.insert("Save Changes", callback = self.saveGeometryModelCb)

    self.server.insert(self.geometry_movement_marker, self.processGeometryMovementFeedback)
    self.geometry_movement_menu_handler.apply( self.server, self.geometry_movement_marker.name)
    self.server.applyChanges()

  def createLabelMovement(self):
    self.label_movement_marker = InteractiveMarker()

    self.label_movement_marker.name =  self.label
    self.label_movement_marker.pose = self.label_pose.pose
    self.label_movement_marker.header = self.label_pose.header
    self.label_movement_marker.scale = 1.0
    self.label_movement_marker.description = ""

    createMenuControl(self.label_movement_marker.controls, self.label_pose, self.label_movement_marker.name)
    create6DMotionControl(self.label_movement_marker.controls, True)

    self.label_movement_menu_handler.insert("Save Changes", callback = self.saveLabelMovementCb)

    self.server.insert(self.label_movement_marker, self.processLabelMovementFeedback)
    self.label_movement_menu_handler.apply( self.server, self.label_movement_marker.name)
    self.server.applyChanges()

  def initSubscriber(self):
    self.point2d_sub_ = rospy.Subscriber("clicked_point", PointStamped, self.point2dtopicCb)
    self.pose2d_sub_ = rospy.Subscriber("pose", PoseStamped, self.pose2dtopicCb)
    self.polygon2d_sub_ = rospy.Subscriber("polygon", PolygonStamped, self.polygon2dtopicCb)
    self.point3d_sub_ = rospy.Subscriber("clicked_point", PointStamped, self.point3dtopicCb)
    self.pose3d_sub_ = rospy.Subscriber("pose", PoseStamped, self.pose3dtopicCb)
    self.polygon3d_sub_ = rospy.Subscriber("polygon", PolygonStamped, self.polygon3dtopicCb)

    self.color_sub_ = rospy.Subscriber("color_cmd", ColorCommand, self.colorCb)
    self.label_sub_ = rospy.Subscriber("label_cmd", LabelCommand, self.labelCb)
    self.print_sub_ = rospy.Subscriber("print_cmd", String, self.printLabelCb)

### Debug Node

if __name__=="__main__":
    rospy.init_node("interactive_object_marker")
    server = InteractiveMarkerServer("interactive_object_marker")

    obj = create_object_instance()
    int_obj = InteractiveObjectMarker(obj, server)

    rospy.spin()
