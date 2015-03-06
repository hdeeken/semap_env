#!/usr/bin/env python

import sys
import rospy
import roslib; roslib.load_manifest('spatial_environment')

from tf.broadcaster import TransformBroadcaster
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from db_model import *
from db_environment import db, initializeConnection
from interactive_object_marker import *
from spatial_environment.srv import *
from spatial_db_msgs.msg import ObjectDescription as ROSObjectDescription
from spatial_db_msgs.msg import ObjectInstance as ROSObjectInstance
from spatial_db_msgs.msg import GeometryCollection
 
class ActiveObject:
  id = None
  name = None
  object = None
  marker = None

  def __init__(self, id, name, object, marker):
    self.id = id
    self.name = name
    self.object = object
    self.marker = marker

class SpatialEnvironmentServices():

  server = None
  active_objects = {}
  tf_broadcaster = None
  marker_pub = None

  def __init__(self ):
    self.setup_node()
    self.server = InteractiveMarkerServer("interactive_object_marker")
    self.tf_broadcaster = TransformBroadcaster()

  def setup_node(self):
      rospy.init_node('spatial_environment_services')
      rospy.Timer(rospy.Duration(0.02), self.publishTF)
      rospy.Subscriber("create_object", PoseStamped, self.createObjectCb)
      marker_pub = rospy.Publisher("collection_marker", MarkerArray)

      user = rospy.get_param('~user')
      password = rospy.get_param('~password')
      host = rospy.get_param('~host')
      database = rospy.get_param('~database')
      
      initializeConnection(user, password, host, database, False)

      ## Active Objects
      srv_activate_objects = rospy.Service('activate_objects', ActivateObjects, self.activate_objects)
      srv_activate_all_objects = rospy.Service('activate_all_objects', ActivateAllObjects, self.activate_all_objects)
      srv_deactivate_objects = rospy.Service('deactivate_objects', DeactivateObjects, self.deactivate_objects)
      srv_deactivate_all_object = rospy.Service('deactivate_all_objects', DeactivateAllObjects, self.deactivate_all_objects)

      print "SpatialEnvironment Services are online."

  def spin(self):
      rospy.spin()

  ### Services
  def activate_objects(self, req):
    #rospy.loginfo("SpatialEnv SRVs: activate_objects")
    self.deactivate_objects(req)
    get_res = call_get_object_instances(req.ids)
    for obj in get_res.objects:
      now = rospy.Time.now()
      #rospy.loginfo("Activate object: %s" % obj.name)
      active_object = ActiveObject(obj.id, obj.name, obj, InteractiveObjectMarker(obj, self.server))
      self.active_objects[obj.name] = active_object
      #rospy.loginfo("Took %f seconds" % (rospy.Time.now() - now).to_sec())
    self.publishTF(None)
    return ActivateObjectsResponse()

  def deactivate_objects(self, req):
    #rospy.loginfo("SpatialEnv SRVs: deactivate_objects")
    res = DeactivateObjectsResponse()
    for key in self.active_objects.keys():
      if self.active_objects[key].id in req.ids:
          res.names.append(self.active_objects[key].name)
          del self.active_objects[key]
    self.publishTF(None)
    return res

  def deactivate_all_objects(self, req):
    #rospy.loginfo("SpatialEnv SRVs: deactivate_all_objects")
    self.active_objects = {}
    self.publishTF(None)
    return DeactivateAllObjectsResponse()

  def activate_all_objects(self, req):
      #rospy.loginfo("SpatialEnv SRVs: activate_all_objects")
      ids = db().query(ObjectInstance.id).all()
      req = ActivateObjectsRequest()
      req.ids = [id for id, in ids]
      self.activate_objects(req)
      res = ActivateAllObjectsResponse()
      return res

  def publishTF(self, msg):
      if len(self.active_objects) > 0:
        for key in self.active_objects.keys():
          object = self.active_objects[key].object
          translation = object.pose.pose.position
          rotation = object.pose.pose.orientation
          time = rospy.Time.now()
          self.tf_broadcaster.sendTransform( (translation.x, translation.y, translation.z), \
                                        (rotation.x, rotation.y, rotation.z, rotation.w), \
                                        time, object.name, object.pose.header.frame_id)

  def createObjectCb(self, pose):
    app = QApplication(sys.argv)
    widget = ChooseObjectDescriptionWidget(0)
    widget.show()
    app.exec_()
    
    desc_name, desc_id = widget.getChoice()

    if(desc_name == "Empty"):
      new_desc = ROSObjectDescription()
      #desc.type = widget.getName()
      new_desc.type = "New Description"
      res = call_add_object_descriptions([new_desc])
      desc_id = res.ids[0]

    obj = ROSObjectInstance()
    obj.description.id = desc_id
    obj.pose = pose

    res = call_add_object_instances([obj])
    call_activate_objects(res.ids)

  ### Utility
  def list_active_objects(self, msg):
      #rospy.loginfo("SpatialEnv SRVs: active objects")
      for key in self.active_objects.keys():
        active = active_objects[key]
        print 'ID    :', active.id
        print 'TYPE  :', active.object.description.type
        print 'NAME  :', active.object.name
        print 'ALIAS :', active.object.alias
        print 'POSE  :', active.object.pose

if __name__ == "__main__":
    services = SpatialEnvironmentServices()
    services.spin()
