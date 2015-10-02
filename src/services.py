#!/usr/bin/env python

import sys
import rospy
import roslib; roslib.load_manifest("semap_env")

import tf
from tf.broadcaster import TransformBroadcaster
from tf.listener import TransformListener
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from db_model import *
from db_environment import db, initializeConnection
from interactive_object_marker import *
from ghost_object_marker import *
from semap_env.srv import *
from object_description_marker import *
from semap_ros.instance_srv_calls import *
from semap_ros.description_srv_calls import *
from semap_msgs.msg import ObjectDescription as ROSObjectDescription
from semap_msgs.msg import ObjectInstance as ROSObjectInstance
from semap_msgs.msg import GeometryCollection
from mesh_msgs.msg import PolygonMeshStamped

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

class SEMAPEnvironmentServices():

  server = None
  objects = {}
  tf_broadcaster = None
  tf_listener = None
  marker_pub = None
  range_query2d_marker = None
  range_query3d_marker = None
  range_query_marker = None
  area_query_marker = None
  volume_query_marker = None

  def __init__(self):
    self.setup_node()
    self.server = InteractiveMarkerServer("semap_environment")
    self.tf_broadcaster = TransformBroadcaster()
    self.tf_listener = TransformListener()

  def setup_node(self):
      rospy.init_node('semap_environment_services')
      rospy.loginfo( "SEMAP Environment Services are initializing...\n" )

      rospy.Timer(rospy.Duration(0.02), self.publishTF)
      #rospy.Timer(rospy.Duration(1.0), self.activate_robot_surroundings)
      rospy.Subscriber("create_object", PoseStamped, self.createObjectCb)
      #rospy.Subscriber("polygon", PolygonStamped, self.findObjectWithinPolygonCb)
      #rospy.Subscriber("polygon", PolygonStamped, self.copyObjectWithinPolygonCb)
      rospy.Subscriber("polygon", PolygonStamped, self.findObjectWithinVolumeCb)
      rospy.Subscriber("range2d", PointStamped, self.findObjectWithinRange2DCb)
      rospy.Subscriber("range_query", PointStamped, self.findObjectWithinRangeCb)

      self.range_query_marker = rospy.Publisher("distance_query", MarkerArray, queue_size=10)
      self.area_query_marker = rospy.Publisher("area_query", PolygonStamped, queue_size=10)
      self.volume_query_marker = rospy.Publisher("volume_query", MarkerArray, queue_size=10)

      self.range_query2d_marker = rospy.Publisher("range_query2d_marker", Marker, queue_size=10)
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

      srv_show_distance_between_objects = rospy.Service('show_distance_between_objects', GetDistanceBetweenObjects, self.showDistanceBetweenObjects)
      srv_show_objects_within_range = rospy.Service('show_objects_within_range', GetObjectsWithinRange, self.showObjectWithinRange)
      srv_show_objects_within_area = rospy.Service('show_objects_within_area', GetObjectsWithinArea, self.showObjectWithinArea)
      srv_show_objects_within_volume = rospy.Service('show_objects_within_volume', GetObjectsWithinVolume, self.showObjectWithinVolume)

      srv_activate_objects_in_objects = rospy.Service('activate_objects_in_objects', GetObjectsInObjects, self.activateObjectsInObjects)
      srv_show_objects_in_objects = rospy.Service('show_objects_in_objects', GetObjectsInObjects, self.showObjectsInObjects)

      srv_activate_objects_on_objects = rospy.Service('activate_objects_on_objects', GetObjectsOnObjects, self.activateObjectsOnObjects)
      srv_show_objects_on_objects = rospy.Service('show_objects_on_objects', GetObjectsOnObjects, self.showObjectsOnObjects)

      rospy.loginfo( "SEMAP DB Services are running...\n" )

  def spin(self):
      rospy.spin()

  ### Services
  def activate_objects(self, req):
    then = rospy.Time.now()
    if len( req.ids) > 0:
      rospy.loginfo("SpatialEnv SRVs: activate_objects")
      #self.deactivate_objects(req)
      get_res = call_get_object_instances(req.ids)
      rospy.loginfo("Call Get Object Instances took %f seconds" % (rospy.Time.now() - then).to_sec())
      for obj in get_res.objects:

        #rospy.loginfo("Activate object: %s" % obj.name)

        # get inst visu from db
        # if exits: convertNonDBtype
        # else create one, store and pass on

        if obj.name in self.objects.keys():
          #if self.objects[obj.name].status == "active":
            #print 'object', obj.name, 'is already active and gets reactivated'
          if self.objects[obj.name].status == "inactive":
            del self.objects[obj.name]
            #print 'object', obj.name, 'is inactive and gets removed from inactive pool'
        #else:
        #  print 'object was unknown so far, now its active'

        then2 = rospy.Time.now()
        active_object = EnvironmentObject(obj.id, obj.name, obj, InteractiveObjectMarker(obj, self.server, None ), status = "active")
        #rospy.loginfo("Marker took %f seconds" % (rospy.Time.now() - then2).to_sec())
        self.objects[obj.name] = active_object



        #rospy.loginfo("Took %f seconds" % (rospy.Time.now() - now).to_sec())
      self.publishTF(None)
      rospy.loginfo("Took %f seconds" % (rospy.Time.now() - then).to_sec())
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
            break
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
            #print 'publish tf'
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

  def activateObjectsInObjects(self, req):
    res = call_get_objects_in_objects(req.reference, req.target, req.fully_within)
    ids = []
    for pair in res.pairs:
      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)
    call_activate_objects( ids )
    return res

  def showObjectsInObjects(self, req):
    res = call_get_objects_in_objects(req.reference, req.target, req.fully_within)
    ids = []
    for pair in res.pairs:
      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)
    call_show_objects( ids )
    return res

  def activateObjectsOnObjects(self, req):
    print 'GOT SO FAR'
    res = call_get_objects_on_objects(req.reference_object_types, req.target_object_types, req.threshold)
    print 'NOW WHAT'
    ids = []
    for pair in res.pairs:
      #if pair.reference_id not in ids:
      #  ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)
    call_activate_objects( ids )
    return res

  def showObjectsOnObjects(self, req):
    print 'GOT SO FAR'
    res = call_get_objects_on_objects(req.reference_object_types, req.target_object_types, req.threshold)
    print 'NOW WHAT'
    ids = []
    for pair in res.pairs:
      #if pair.reference_id not in ids:
      #  ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)
    call_show_objects( ids )
    return res

  def findObjectWithinRange2DCb(self, point_stamped):
    distance = 2.0
    print 'find Objects Within Range2D'
    #res = call_get_objects_within_range2d([], "Position2D", point_stamped.point, distance, fully_within = False)
    #res = call_get_objects_within_range2d([], "FootprintBox", point_stamped.point, distance, fully_within = False)
    #res = call_get_objects_within_range2d(["ConferenceTable", "ConferenceChair"], "Position2D", point_stamped.point, 3.0)
    #call_activate_objects( res.ids )

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

  def findObjectWithinRangeCb(self, point_stamped):
    distance = 1.5
    res = call_get_objects_within_range(point_stamped.point, [], "FootprintHull", distance, fully_within = False)
    ids = []
    array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "range"
    marker.id = 0
    marker.type = 3 #sphere
    marker.scale.x = distance*2
    marker.scale.y = distance*2
    marker.scale.z = 0.005
    marker.pose.position = point_stamped.point
    marker.pose.position.z = 0.0
    marker.color.r = 0.25
    marker.color.g = 0.25
    marker.color.b = 0.75
    marker.color.a = 0.5
    array.markers.append(marker)

    lines = Marker()
    lines.header.frame_id = "world"
    lines.header.stamp = rospy.get_rostime()
    lines.ns = "lines"
    lines.id = 0
    lines.type = 5 #line list
    lines.scale.x = 0.05
    lines.scale.y = 0.0
    lines.scale.z = 0.0
    lines.color.r = 0.75
    lines.color.g = 0.750
    lines.color.b = 0.750
    lines.color.a = 1.0
    for pair in res.pairs:
      lines.points += pair.min_dist_line
      lines.points += pair.max_dist_line

      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)

      lines.colors.append(lines.color)
      text = Marker()
      text.header.frame_id = "world"
      text.header.stamp = rospy.get_rostime()
      text.ns = "distances"
      text.id = 0
      text.type = 9 #line list
      text.pose.position.x = (pair.min_dist_line[1].x + pair.min_dist_line[0].x)/2
      text.pose.position.y = (pair.min_dist_line[1].y + pair.min_dist_line[0].y)/2
      text.pose.position.z = (pair.min_dist_line[1].z + pair.min_dist_line[0].z)/2 + 0.05
      text.scale.z = 0.2
      text.color.r = 1.0
      text.color.g = 1.0
      text.color.b = 1.0
      text.color.a = 1.0
      text.text = str(round(pair.min_dist,3))
      array.markers.append(text)
    array.markers.append(lines)
    id = 0
    for m in array.markers:
      m.id = id
      id += 1
    self.range_query_marker.publish(array)
    call_activate_objects( ids )

  def showObjectWithinRange(self, req):
    res = call_get_objects_within_range(req.reference_point, req.target_object_types, req.target_object_geometry_type,
                                              req.distance, req.fully_within)

    ids = []
    array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "range"
    marker.id = 0
    marker.type = 3 #sphere
    marker.scale.x = req.distance*2
    marker.scale.y = req.distance*2
    marker.scale.z = 0.005
    marker.pose.position = req.reference_point
    marker.pose.position.z = 0.0
    marker.color.r = 0.25
    marker.color.g = 0.25
    marker.color.b = 0.75
    marker.color.a = 0.5
    array.markers.append(marker)

    lines = Marker()
    lines.header.frame_id = "world"
    lines.header.stamp = rospy.get_rostime()
    lines.ns = "lines"
    lines.id = 0
    lines.type = 5 #line list
    lines.scale.x = 0.05
    lines.scale.y = 0.0
    lines.scale.z = 0.0
    lines.color.r = 0.75
    lines.color.g = 0.750
    lines.color.b = 0.750
    lines.color.a = 1.0
    for pair in res.pairs:
      lines.points += pair.min_dist_line
      lines.points += pair.max_dist_line

      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)

      lines.colors.append(lines.color)
      text = Marker()
      text.header.frame_id = "world"
      text.header.stamp = rospy.get_rostime()
      text.ns = "distances"
      text.id = 0
      text.type = 9 #line list
      text.pose.position.x = (pair.min_dist_line[1].x + pair.min_dist_line[0].x)/2
      text.pose.position.y = (pair.min_dist_line[1].y + pair.min_dist_line[0].y)/2
      text.pose.position.z = (pair.min_dist_line[1].z + pair.min_dist_line[0].z)/2 + 0.05
      text.scale.z = 0.2
      text.color.r = 1.0
      text.color.g = 1.0
      text.color.b = 1.0
      text.color.a = 1.0
      text.text = str(round(pair.min_dist,3))
      array.markers.append(text)
    array.markers.append(lines)
    id = 0
    for m in array.markers:
      m.id = id
      id += 1
    self.range_query_marker.publish(array)
    call_activate_objects( ids )
    return res

  def findObjectWithinVolumeCb(self, polygon_stamped):
    extrude = call_extrude_polygon(polygon_stamped.polygon, 0.0, 0.0, 0.9)
    mesh = PolygonMeshStamped()
    mesh.header.frame_id = "world"
    mesh.mesh = extrude.mesh
    print extrude.mesh
    res = call_get_objects_within_volume(extrude.mesh, ['Mug','Teapot','Monitor','Keyboard'], 'BoundingBox', True)
    ids=[]
    for pair in res.pairs:
      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)

    call_activate_objects( ids )

    array = MarkerArray()
    pose = ROSPoseStamped()
    pose.header.frame_id = 'world'
    for polygon in extrude.mesh.polygons:
        poly = Polygon()
        for point in polygon.vertex_indices:
          poly.points.append(extrude.mesh.vertices[point])
        geo_marker = create_polygon_marker("VolumeQuery", pose, [0,1.0,0,1.0], [0.01, 0.01, 0.01], poly)
        array.markers.append(geo_marker)

    id = 0
    for m in array.markers:
      m.header.frame_id = 'world'
      m.id = id
      id += 1

    self.volume_query_marker.publish(array)

  def showObjectWithinVolume(self, req):
    res = call_get_objects_within_volume(req.reference_mesh, req.target_object_types, req.target_object_geometry_type, req.fully_within)

    ids=[]
    for pair in res.pairs:
      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)

    call_activate_objects( ids )

    array = MarkerArray()
    pose = ROSPoseStamped()
    pose.header.frame_id = 'world'
    for polygon in req.reference_mesh.polygons:
        poly = Polygon()
        for point in polygon.vertex_indices:
          poly.points.append(req.reference_mesh.vertices[point])
        geo_marker = create_polygon_marker("VolumeQuery", pose, [0,1.0,0,1.0], [0.01, 0.01, 0.01], poly)
        array.markers.append(geo_marker)

    id = 0
    for m in array.markers:
      m.header.frame_id = 'world'
      m.id = id
      id += 1

    self.volume_query_marker.publish(array)


    return res

  def findObjectWithinPolygonCb(self, polygon_stamped):
    res = call_get_objects_within_area(polygon_stamped.polygon, ["Chair"], "Position2D", fully_within = False)
    ids=[]
    for pair in res.pairs:
      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)

    call_activate_objects( ids )
    self.area_query_marker.publish(polygon_stamped)
    return res

  def copyObjectWithinPolygonCb(self, polygon_stamped):
    res = call_get_objects_within_area(polygon_stamped.polygon, [], "Position2D", fully_within = False)
    ids=[]
    for pair in res.pairs:
      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)

    print 'COPY'
    copy_res = call_copy_object_instances(ids)

    if len( copy_res.ids ) > 1:
      print 'copy_res', copy_res
      obj_res = call_get_object_instances( [copy_res.ids[0]] )
      print 'bind to ', obj_res.objects[0].name

      frame = obj_res.objects[0].name
      for id in copy_res.ids:
        if id != obj_res.objects[0].id:
          print 'bind', id, 'to', obj_res.objects[0].id
          call_change_frame(id, frame, False)

    call_activate_objects( copy_res.ids )
    call_refresh_objects( copy_res.ids )
    return res

  def showObjectWithinArea(self, req):
    res = call_get_objects_within_area(req.reference_polygon, req.target_object_types, req.target_object_geometry_type, req.fully_within)
    ids=[]
    for pair in res.pairs:
      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)

    call_activate_objects( ids )
    polygon_stamped = PolygonStamped()
    polygon_stamped.header.frame_id = "world"
    polygon_stamped.polygon = req.reference_polygon
    self.area_query_marker.publish(polygon_stamped)
    return res

  def showDistanceBetweenObjects(self, req):
    res = call_get_distance_between_objects(req.reference_object_types, req.reference_object_geometry_type,
                                              req.target_object_types, req.target_object_geometry_type,
                                              req.min_range, req.max_range,
                                              req.sort_descending, req.max_distance, return_points = True)

    array = MarkerArray()

    lines = Marker()
    lines.header.frame_id = "world"
    lines.header.stamp = rospy.get_rostime()
    lines.ns = "lines"
    lines.id = 0
    lines.type = 5 #line list
    lines.scale.x = 0.05
    lines.scale.y = 0.0
    lines.scale.z = 0.0
    lines.color.r = 0.75
    lines.color.g = 0.750
    lines.color.b = 0.750
    lines.color.a = 1.0
    ids =[]
    for pair in res.pairs:
      lines.points += pair.min_dist_line
      lines.points += pair.max_dist_line

      if pair.reference_id not in ids:
        ids.append(pair.reference_id)
      if pair.target_id not in ids:
        ids.append(pair.target_id)

      lines.colors.append(lines.color)
      text = Marker()
      text.header.frame_id = "world"
      text.header.stamp = rospy.get_rostime()
      text.ns = "distances"
      text.id = 0
      text.type = 9 #line list
      text.pose.position.x = (pair.min_dist_line[1].x + pair.min_dist_line[0].x)/2
      text.pose.position.y = (pair.min_dist_line[1].y + pair.min_dist_line[0].y)/2
      text.pose.position.z = (pair.min_dist_line[1].z + pair.min_dist_line[0].z)/2 + 0.05
      text.scale.z = 0.2
      text.color.r = 1.0
      text.color.g = 1.0
      text.color.b = 1.0
      text.color.a = 1.0
      text.text = str(round(pair.min_dist,3))
      array.markers.append(text)
    array.markers.append(lines)
    id = 0
    for m in array.markers:
      m.id = id
      id += 1
    self.range_query_marker.publish(array)
    call_activate_objects( ids )
    return res

  def list_active_objects(self, msg):
      for key in self.objects.keys():
        active = active_objects[key]
        print 'ID    :', active.id
        print 'TYPE  :', active.object.description.type
        print 'NAME  :', active.object.name
        print 'ALIAS :', active.object.alias
        print 'POSE  :', active.object.pose

if __name__ == "__main__":
    services = SEMAPEnvironmentServices()
    services.spin()
