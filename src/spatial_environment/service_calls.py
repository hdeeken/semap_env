#!/usr/bin/env python

'''
SpatialEnvironment Service Calls
'''

import roslib; roslib.load_manifest('spatial_environment')
import rospy

from spatial_environment.srv import *

def call_refresh_objects(ids):
  try:
    rospy.wait_for_service('refresh_objects')
    call = rospy.ServiceProxy('refresh_objects', ActivateObjects)
    request = ActivateObjectsRequest()
    request.ids = ids
    response = call(request)
    rospy.loginfo('RefreshObjects service call succeeded!')
    return response
  except rospy.ServiceException as e:
      return None, "RefreshObjects service call failed: %s" % e

def call_activate_objects(ids):
  try:
    rospy.wait_for_service('activate_objects')
    call = rospy.ServiceProxy('activate_objects', ActivateObjects)
    request = ActivateObjectsRequest()
    request.ids = ids
    response = call(request)
    rospy.loginfo('ActivateObjects service call succeeded!')
    return response
  except rospy.ServiceException as e:
      return None, "ActivateObjects service call failed: %s" % e

def call_deactivate_objects(ids):
  try:
    rospy.wait_for_service('deactivate_objects')
    call = rospy.ServiceProxy('deactivate_objects', DeactivateObjects)
    request = DeactivateObjectsRequest()
    request.ids = ids
    response = call(request)
    rospy.loginfo('DeactivateObjects service call succeeded!')
    return response
  except rospy.ServiceException as e:
      return None, "DeactivateObjects service call failed: %s" % e

def call_activate_all_objects():
  try:
    rospy.wait_for_service('activate_all_objects')
    call = rospy.ServiceProxy('activate_all_objects', ActivateAllObjects)
    request = ActivateAllObjectsRequest()
    response = call(request)
    rospy.loginfo('ActivateAllObjects service call succeeded!')
    return response
  except rospy.ServiceException as e:
      return None, "ActivateAllObjects service call failed: %s" % e

def call_deactivate_all_objects():
  try:
    rospy.wait_for_service('deactivate_all_objects')
    call = rospy.ServiceProxy('deactivate_all_objects', DeactivateAllObjects)
    request = DeactivateAllObjectsRequest()
    response = call(request)
    rospy.loginfo('DeactivateAllObjects service call succeeded!')
    return response
  except rospy.ServiceException as e:
      return None, "DeactivateAllObjects service call failed: %s" % e
