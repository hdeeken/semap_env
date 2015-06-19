#!/usr/bin/env python

import rospy, tf
import roslib; roslib.load_manifest("spatial_environment")

from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PolygonStamped, Polygon
from spatial_db_ros.srv import *
from spatial_db_ros.service_calls import *
from spatial_db_msgs.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from spatial_db_msgs.msg import ColorCommand
from spatial_db_msgs.msg import ObjectDescription as ROSObjectDescription
from spatial_db_msgs.msg import ObjectInstance as ROSObjectInstance
from object_description_marker import *

class SceneVisu:
  name = None
  descriptions = None
  instances = None

class InstanceVisu:
  id = None
  name = None
  models = {}

class DescriptionVisu:
  id = None
  type = None
  models = {}

 #def __init__(self, type, id, model,\
               #show_geo = False, \
               #geo_color = [0,0,0,1], \
               #geo_scale = [0.2,0.2,0.2], \
               #show_text = False, \
               #text_color = [0,0,0,1], \
               #text_scale = [0.1,0.1,0.1], \
               #text_offset = [0.0,0.0,0.25]):

    #self.type = type
    #self.id = id
    #self.model = model
    #self.show_geo = show_geo
    #self.geo_color = geo_color
    #self.geo_scale = geo_scale
    #self.show_text = show_text
    #self.text_color = text_color
    #self.text_scale = text_scale
    #self.text_offset = text_offset

class ModelVisu:
  type = None
  id = None
  model = None
  show_geo = False
  geo_color = []
  geo_scale = []
  show_text = False
  text_color = []
  text_scale = []
  text_offset = []

  def __init__(self, type, id, model,\
               show_geo = False, \
               geo_color = [0,0,0,1], \
               geo_scale = [0.2,0.2,0.2], \
               show_text = False, \
               text_color = [0,0,0,1], \
               text_scale = [0.1,0.1,0.1], \
               text_offset = [0.0,0.0,0.25]):

    self.type = type
    self.id = id
    self.model = model
    self.show_geo = show_geo
    self.geo_color = geo_color
    self.geo_scale = geo_scale
    self.show_text = show_text
    self.text_color = text_color
    self.text_scale = text_scale
    self.text_offset = text_offset

  def __repr__(self):
    string = "Visu(type=%r, id=%r)" % (
                                        self.type,
                                        self.id)
                                        #self.model)
                                        #self.show_geo,
                                        #self.geo_color,
                                        #self.geo_scale,
                                        #self.show_text,
                                        #self.text_color,
                                        #self.text_scale,
                                        #self.text_offset)
    return string

def lookupModelVisuConfig(desc, absolute = False):
  model_dict = {}
  for model in desc.point2d_models:
    type = model.type
    id = model.id
    show_geo = True
    geo_color = [1.0, 0.0, 0.0, 1.0]
    geo_scale = [0.05, 0.05, 0.05]
    show_text = False
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, id, model, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)
  for model in desc.pose2d_models:
    type = model.type
    id = model.id
    show_geo = True
    geo_color = [1.0, 1.0, 0.0, 1.0]
    geo_scale = [0.1, 0.025, 0.025]
    show_text = False
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, id, model, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)
  for model in desc.polygon2d_models:
    type = model.type
    id = model.id
    show_geo = True
    geo_color = [0.0, 1.0, 1.0, 1.0]
    geo_scale = [0.05, 0.05, 0.05]

    if model.type == "FootprintBox":
      show_geo = False
      geo_color = [0.0, 0.0, 0.50, 1.0]
      geo_scale = [0.01, 0.01, 0.01]

    if model.type == "FootprintHull":
      show_geo = False
      geo_color = [0.0, 0.50, 0.0, 1.0]
      geo_scale = [0.01, 0.01, 0.01]

    if model.type == "FootprintBox" and absolute:
      show_geo = False
      geo_color = [0.5, 0.0, 0.0, 1.0]
      geo_scale = [0.01, 0.01, 0.01]

    if model.type == "FootprintHull" and absolute:
      show_geo = False
      geo_color = [0.5, 0.5, 0.0, 1.0]
      geo_scale = [0.01, 0.01, 0.01]

    show_text = False
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, id, model, show_geo, geo_color, geo_scale, show_text,text_color, text_scale, text_offset)
  for model in desc.point3d_models:
    type = model.type
    id = model.id
    show_geo = True
    geo_color = [0.0, 0.0, 1.0, 1.0]
    geo_scale = [0.05, 0.05, 0.05]
    show_text = False
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, id, model, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)
  for model in desc.pose3d_models:
    type = model.type
    id = model.id
    show_geo = True
    geo_color = [0.0, 0.0, 1.0, 1.0]
    geo_scale = [0.1, 0.025, 0.025]
    show_text = False
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, id, model, show_geo, geo_color, geo_scale, show_text,text_color, text_scale, text_offset)
  for model in desc.polygon3d_models:
    type = model.type
    show_geo = True
    geo_color = [0.0, 1.0, 0.0, 1.0]
    geo_scale = [0.05, 0.05, 0.05]
    show_text = False
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, id, model, show_geo, geo_color, geo_scale, show_text,text_color, text_scale, text_offset)
  for model in desc.trianglemesh3d_models:
    type = model.type
    id = model.id
    show_geo = True
    geo_color = [0.0, 0.5, 0.5, 1.0]
    geo_scale = [1.0, 1.0, 1.0]

    if absolute:
      geo_color = [1.0, 0.5, 0.5, 1.0]
      geo_scale = [1.0, 1.0, 1.0]

    show_text = False
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, id, model, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)
  for model in desc.polygonmesh3d_models:
    type = model.type
    id = model.id
    show_geo = True
    geo_color = [0.5, 1.0, 0.5, 1.0]
    geo_scale = [0.01, 0.01, 0.01]

    if absolute:
      geo_color = [1.0, 0.5, 0.5, 1.0]
      geo_scale = [0.01, 0.01, 0.01]
      
    if model.type == "BoundingBox":
      show_geo = False
      geo_color = [0.0, 0.0, 0.75, 1.0]
      geo_scale = [0.01, 0.01, 0.01]

    if model.type == "BoundingHull":
      show_geo = False
      geo_color = [0.0, 0.75, 0.0, 1.0]
      geo_scale = [0.005, 0.005, 0.005]

    if model.type == "BoundingBox" and absolute:
      show_geo = False
      geo_color = [0.75, 0.0, 0.0, 1.0]
      geo_scale = [0.01, 0.01, 0.01]

    if model.type == "BoundingHull" and absolute:
      show_geo = False
      geo_color = [0.75, 0.75, 0.0, 1.0]
      geo_scale = [0.005, 0.005, 0.005]

    show_text = False
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.05, 0.05, 0.05]
    text_offset = [0.0, 0.0, 0.15]

    model_dict[model.type] = ModelVisu(type, id, model, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)

  return model_dict

def create_model_visualization_marker(frame, model, model_visu):

    array = MarkerArray()

    if type(model) is Point2DModel:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0
      pose.pose.position.x = model.geometry.x
      pose.pose.position.y = model.geometry.y
      pose.pose.position.z = 0.0

      if model_visu[model.type].show_geo:
        geo_marker = create_point_marker("Point2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    if type(model) is Pose2DModel:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0
      quat = quaternion_from_euler(0, 0, model.pose.theta)
      pose.pose.position.x = model.pose.x
      pose.pose.position.y = model.pose.y
      pose.pose.position.z = 0.0
      pose.pose.orientation.x = quat[0]
      pose.pose.orientation.y = quat[1]
      pose.pose.orientation.z = quat[2]
      pose.pose.orientation.w = quat[3]

      if model_visu[model.type].show_geo:
        geo_marker = create_pose_marker("Pose2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    if type(model) is Polygon2DModel:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0
      pose.pose = model.pose

      if model.type == "AbsoluteFootprintBox" or model.type == "AbsoluteFootprintHull":
        pose.header.frame_id = "world"

      if model_visu[model.type].show_geo:
        geo_marker = create_polygon_marker("Polygon2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)
      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    if type(model) is Point3DModel:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0
      pose.pose.position = model.geometry

      if model_visu[model.type].show_geo:
        geo_marker = create_point_marker("Point3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    if type(model) is Pose3DModel:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0
      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_pose_marker("Pose3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    if type(model) is Polygon3DModel:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0
      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_polygon_marker("Polygon3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    if type(model) is TriangleMesh3DModel:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0
      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_mesh_marker("TriangleMesh3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    if type(model) is PolygonMesh3DModel:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0
      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        for polygon in model.geometry.polygons:
          geo_marker = create_polygon_marker("PolygonMesh3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, polygon)
          array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type, model_visu[model.type].text_offset)
        array.markers.append(text_marker)

    id = 0
    for m in array.markers:
      m.id = id
      id += 1

    return array

def create_object_visualization_marker(obj, model_visu, absolute = False):

    array = MarkerArray()

    if absolute:
      desc = obj.absolute
      frame = "world"
    else:
      desc = obj.description
      frame = obj.name

    for model in desc.point2d_models:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0
      pose.pose.position.x = model.geometry.x
      pose.pose.position.y = model.geometry.y
      pose.pose.position.z = 0.0

      if model_visu[model.type].show_geo:
        geo_marker = create_point_marker("Point2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.pose2d_models:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0

      quat = quaternion_from_euler(0, 0, model.pose.theta)
      pose.pose.position.x = model.pose.x
      pose.pose.position.y = model.pose.y
      pose.pose.position.z = 0.0
      pose.pose.orientation.x = quat[0]
      pose.pose.orientation.y = quat[1]
      pose.pose.orientation.z = quat[2]
      pose.pose.orientation.w = quat[3]

      if model_visu[model.type].show_geo:
        geo_marker = create_pose_marker("Pose2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.polygon2d_models:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model.type == "AbsoluteFootprintBox" or model.type == "AbsoluteFootprintHull":
        pose.header.frame_id = "world"

      if model_visu[model.type].show_geo:
        geo_marker = create_polygon_marker("Polygon2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)
      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.point3d_models:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0

      pose.pose.position = model.geometry

      if model_visu[model.type].show_geo:
        geo_marker = create_point_marker("Point3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.pose3d_models:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_pose_marker("Pose3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)
    for model in desc.polygon3d_models:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_polygon_marker("Polygon3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)
    for model in desc.trianglemesh3d_models:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_mesh_marker("TriangleMesh3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.polygonmesh3d_models:
      pose = ROSPoseStamped()
      pose.header.frame_id = frame
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model.type == "AbsoluteBoundingBox" or model.type == "AbsoluteBoundingHull":
        pose.header.frame_id = "world"

      if model_visu[model.type].show_geo:
        for polygon in model.geometry.polygons:
          poly= ROSPolygon()

          for point in polygon.vertex_indices:
            poly.points.append(model.geometry.vertices[point])

          geo_marker = create_polygon_marker("PolygonMesh3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, poly)
          array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type, model_visu[model.type].text_offset)
        array.markers.append(text_marker)

    id = 0
    for m in array.markers:
      m.id = id
      id += 1

    return array
