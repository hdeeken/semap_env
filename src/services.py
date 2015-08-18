#!/usr/bin/env python

import sys
import rospy
import roslib; roslib.load_manifest('spatial_environment')

import tf
from tf.broadcaster import TransformBroadcaster
from tf.listener import TransformListener
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
 
from db_model import *
from db_environment import db, initializeConnection
from interactive_object_marker import *
from ghost_object_marker import *
from spatial_environment.srv import *
from spatial_db_ros.instance_srv_calls import *
from spatial_db_ros.description_srv_calls import *
from spatial_db_msgs.msg import ObjectDescription as ROSObjectDescription
from spatial_db_msgs.msg import ObjectInstance as ROSObjectInstance
from spatial_db_msgs.msg import GeometryCollection

class EnvironmentObject:
  id = None
  name = None
  status = None
  object = None
  marker = None

  def __init__(self, id, name, object, marker, status):
    self.id = id
    self.name = name
    self.object = object
    self.marker = marker
    self.status = status

class SpatialEnvironmentServices():

  server = None
  objects = {}
  tf_broadcaster = None
  tf_listener = None
  marker_pub = None
  range_query2d_marker = None
  range_query3d_marker = None

  def __init__(self):
    self.setup_node()
    self.server = InteractiveMarkerServer("semap_environment")
    self.tf_broadcaster = TransformBroadcaster()
    self.tf_listener = TransformListener()

  def setup_node(self):
      rospy.init_node('spatial_environment_services')
      rospy.loginfo( "SEMAP Environment Services are initializing...\n" )
      
      rospy.Timer(rospy.Duration(0.02), self.publishTF)
      #rospy.Timer(rospy.Duration(1.0), self.activate_robot_surroundings)
      rospy.Subscriber("create_object", PoseStamped, self.createObjectCb)
      rospy.Subscriber("polygon", PolygonStamped, self.findObjectWithinPolygonCb)
      rospy.Subscriber("range2d", PointStamped, self.findObjectWithinRange2DCb)
      rospy.Subscriber("range3d", PointStamped, self.findObjectWithinRange3DCb)

      self.range_query2d_marker = rospy.Publisher("range_query2d_marker", Marker, queue_size=10)
      self.range_query3d_marker = rospy.Publisher("range_query3d_marker", Marker, queue_size=10)
      self.marker_pub = rospy.Publisher("collection_marker", MarkerArray, queue_size=10)

      user = rospy.get_param('~user')
      password = rospy.get_param('~password')
      host = rospy.get_param('~host')
      database = rospy.get_param('~database')

      initializeConnection(user, password, host, database, False)

      ## Active Objects
      srv_refresh_objects = rospy.Service('refresh_objects', ActivateObjects, self.refresh_objects)
      srv_refresh_all_objects = rospy.Service('refresh_all_objects', ActivateAllObjects, self.refresh_all_objects)
      srv_activate_objects = rospy.Service('activate_objects', ActivateObjects, self.activate_objects)
      srv_activate_all_objects = rospy.Service('activate_all_objects', ActivateAllObjects, self.activate_all_objects)
      srv_deactivate_objects = rospy.Service('deactivate_objects', DeactivateObjects, self.deactivate_objects)
      srv_deactivate_all_object = rospy.Service('deactivate_all_objects', DeactivateAllObjects, self.deactivate_all_objects)
      srv_show_objects = rospy.Service('show_objects', ActivateObjects, self.show_objects)
      srv_show_all_objects = rospy.Service('show_all_objects', ActivateAllObjects, self.show_all_objects)
      #srv_unshow_objects = rospy.Service('unshow_objects', ActivateObjects, self.activate_all_objects)
      #srv_unshow_all_objects = rospy.Service('unshow_all_objects', ActivateAllObjects, self.activate_all_objects)
      
      rospy.loginfo( "SEMAP DB Services are running...\n" )

  def spin(self):
      rospy.spin()

  ### Services
  def activate_objects(self, req):
    if len( req.ids) > 0:
      rospy.loginfo("SpatialEnv SRVs: activate_objects")
      self.deactivate_objects(req)
      get_res = call_get_object_instances(req.ids)
      for obj in get_res.objects:
        now = rospy.Time.now()
        rospy.loginfo("Activate object: %s" % obj.name)

        # get inst visu from db
        # if exits: convertNonDBtype
        # else create one, store and pass on
        
        if obj.name in self.objects.keys():
          if self.objects[obj.name].status == "active":
            print 'object', obj.name, 'is already active and gets reactivated'
          elif self.objects[obj.name].status == "inactive":
            del self.objects[obj.name]
            print 'object', obj.name, 'is inactive and gets removed from inactive pool'
        else:
          print 'object was unknown so far, now its active'

        active_object = EnvironmentObject(obj.id, obj.name, obj, InteractiveObjectMarker(obj, self.server, None ), status = "active")
        self.objects[obj.name] = active_object

        rospy.loginfo("Took %f seconds" % (rospy.Time.now() - now).to_sec())
      self.publishTF(None)
      rospy.loginfo("SpatialEnv SRVs: activate_objects - done")
    return ActivateObjectsResponse()

  def show_objects(self, req):
    if len( req.ids) > 0:
      rospy.loginfo("SpatialEnv SRVs: show_objects")
      get_res = call_get_object_instances(req.ids)
      for obj in get_res.objects:
        if obj.name in self.objects.keys():
          if self.objects[obj.name].status == "active":
            print 'object', obj.name, 'is already active'
            return
          elif self.objects[obj.name].status == "inactive":
            print 'object', obj.name, 'is inactive'
            ghost = EnvironmentObject(obj.id, obj.name, obj, GhostObjectMarker(obj, self.server, None ), status = "inactive")
            self.objects[obj.name] = ghost
        else:
          print 'object was unknown so far, now its inactive'
          ghost = EnvironmentObject(obj.id, obj.name, obj, GhostObjectMarker(obj, self.server, None ), status = "inactive")
          self.objects[obj.name] = ghost
      rospy.loginfo("SpatialEnv SRVs: show_objects - done")
    return ActivateObjectsResponse()

  def refresh_objects(self, req):
    rospy.loginfo("SpatialEnv SRVs: reactivate_objects")
    print req.ids
    active_req = ActivateObjectsRequest()
    inactive_req = ActivateObjectsRequest()

    for key in self.objects.keys():
      if self.objects[key].id in req.ids:
        if self.objects[key].status == "active":
          active_req.ids.append( self.objects[key].id )
        elif self.objects[key].status == "inactive":
          inactive_req.ids.append( self.objects[key].id )

    self.activate_objects(active_req)
    self.show_objects(inactive_req)

    res = ActivateObjectsResponse()
    rospy.loginfo("SpatialEnv SRVs: reactivate_objects - done")
    return res

  def refresh_all_objects(self, req):
    rospy.loginfo("SpatialEnv SRVs: reactivate_all_objects")
    active_req = ActivateObjectsRequest()
    inactive_req = ActivateObjectsRequest()

    for key in self.objects.keys():
      if self.objects[key].status == "active":
        active_req.ids.append( self.objects[key].id )
      elif self.objects[key].status == "inactive":
        inactive_req.ids.append( self.objects[key].id )

    self.activate_objects(active_req)
    self.show_objects(inactive_req)

    res = ActivateAllObjectsResponse()
    rospy.loginfo("SpatialEnv SRVs: reactivate_all_objects - done")
    return res

  def deactivate_objects(self, req):
    rospy.loginfo("SpatialEnv SRVs: deactivate_objects")
    res = DeactivateObjectsResponse()
    inactive_req = ActivateObjectsRequest()
    for key in self.objects.keys():
      if self.objects[key].id in req.ids:
        del self.objects[key]
    self.show_objects(req)
    self.publishTF(None)
    return res

  def deactivate_all_objects(self, req):
    rospy.loginfo("SpatialEnv SRVs: deactivate_all_objects")
    self.objects = {}
    self.publishTF(None)
    self.show_all_objects(req)
    return DeactivateAllObjectsResponse()

  def activate_all_objects(self, req):
      rospy.loginfo("SpatialEnv SRVs: activate_all_objects")
      ids = db().query(ObjectInstance.id).all()
      req = ActivateObjectsRequest()
      req.ids = [id for id, in ids]
      self.activate_objects(req)
      res = ActivateAllObjectsResponse()
      rospy.loginfo("SpatialEnv SRVs: activate_all_objects - done")
      return res

  def show_all_objects(self, req):
      rospy.loginfo("SpatialEnv SRVs: show_all_objects")
      ids = db().query(ObjectInstance.id).all()
      req = ActivateObjectsRequest()
      req.ids = [id for id, in ids]
      self.show_objects(req)
      res = ActivateAllObjectsResponse()
      rospy.loginfo("SpatialEnv SRVs: show_all_objects - done")
      return res

  def publishTF(self, msg):
      if len(self.objects) > 0:
        for key in self.objects.keys():
          if self.objects[key].status == "active":
            object = self.objects[key].object
            translation = object.pose.pose.position
            rotation = object.pose.pose.orientation
            time = rospy.Time.now()
            self.tf_broadcaster.sendTransform( (translation.x, translation.y, translation.z), \
                                          (rotation.x, rotation.y, rotation.z, rotation.w), \
                                          time, object.name, object.pose.header.frame_id)

  def activate_robot_surroundings(self, msg):

    #if activate_robot_surroundings:
      try:
        (trans,rot) = self.tf_listener.lookupTransform('/world', '/base_link', rospy.Time(0))
        print 'current robot position', trans
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print 'could not determine current robot position'
    #else:
    #  robot inactive

  def createObjectCb(self, pose):
    print 'Create Object from RViz pose'
    app = QApplication(sys.argv)
    widget = ChooseObjectDescriptionWidget(0)
    widget.show()
    app.exec_()
    desc_name, desc_id = widget.getChoice()
    del app, widget

    if(desc_id == -1):
      new_desc = ROSObjectDescription()
      new_desc.type = desc_name
      res = call_add_object_descriptions( [ new_desc ] )
      print res
      desc_id = res.ids[0]

    obj = ROSObjectInstance()
    obj.description.id = desc_id
    obj.pose = pose

    res = call_add_object_instances( [obj] )
    call_activate_objects( res.ids )

  def findObjectWithinPolygonCb(self, polygon_stamped):
    print 'find Objects Within Polygon2D'
    res = call_get_objects_within_polygon2d([], "FootprintHull", polygon_stamped.polygon, fully_within = False)
    #res = call_get_objects_within_polygon2d(["ConferenceTable", "ConferenceChair"], "Position2D", polygon_stamped.polygon)
    #res = call_get_objects_within_polygon2d(["ConferenceTable", "ConferenceChair"], "AxisAligned2D", polygon_stamped.polygon)
    #res = call_get_objects_within_polygon2d(["ConferenceTable", "ConferenceChair"], "FootprintBox", polygon_stamped.polygon)
    #res = call_get_objects_within_polygon2d(["ConferenceTable", "ConferenceChair"], "FootprintHull", polygon_stamped.polygon)
    call_activate_objects( res.ids )

  def findObjectWithinRange2DCb(self, point_stamped):
    distance = 1.5
    print 'find Objects Within Range2D'
    res = call_get_objects_within_range2d([], "FootprintHull", point_stamped.point, distance, fully_within = True)
    #res = call_get_objects_within_range2d([], "Position2D", point_stamped.point, distance, fully_within = False)
    #res = call_get_objects_within_range2d(["ConferenceTable", "ConferenceChair"], "Position2D", point_stamped.point, 3.0)
    call_activate_objects( res.ids )

    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "2d_range_query_marker"
    marker.id = 0
    marker.type = 3 #cylinder
    marker.scale.x = distance*2
    marker.scale.y = distance*2
    marker.scale.z = 0.01
    marker.pose.position = point_stamped.point
    marker.pose.position.z = 0.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5
    self.range_query2d_marker.publish(marker)

  def findObjectWithinRange3DCb(self, point_stamped):
    distance = 1.5
    print 'find Objects Within Range3D'
    res = call_get_objects_within_range3d([], "Body", point_stamped.point, distance, fully_within = False)
    #res = call_get_objects_within_range2d([], "Position2D", point_stamped.point, distance, fully_within = False)
    #res = call_get_objects_within_range2d(["ConferenceTable", "ConferenceChair"], "Position2D", point_stamped.point, 3.0)
    call_activate_objects( res.ids )

    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "3d_range_query_marker"
    marker.id = 0
    marker.type = 2 #sphere
    marker.scale.x = distance*2
    marker.scale.y = distance*2
    marker.scale.z = distance*2
    marker.pose.position = point_stamped.point
    marker.pose.position.z = 0.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5
    self.range_query3d_marker.publish(marker)

  def list_active_objects(self, msg):
      for key in self.objects.keys():
        active = active_objects[key]
        print 'ID    :', active.id
        print 'TYPE  :', active.object.description.type
        print 'NAME  :', active.object.name
        print 'ALIAS :', active.object.alias
        print 'POSE  :', active.object.pose

if __name__ == "__main__":
    services = SpatialEnvironmentServices()
    services.spin()
