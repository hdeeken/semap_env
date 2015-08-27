#!/usr/bin/env python

import copy
import rospy
import roslib; roslib.load_manifest("semap_env")

from shape_msgs.msg import Mesh as ROSMesh
from shape_msgs.msg import MeshTriangle as ROSMeshTriangle
from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import Point32 as ROSPoint32
from geometry_msgs.msg import Pose2D as ROSPose2D
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import Polygon as ROSPolygon
from spatial_db_msgs.msg import PolygonMesh as ROSPolygonMesh
from spatial_db_msgs.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from spatial_db_msgs.msg import ObjectDescription as ROSObjectDescription
from spatial_db_msgs.msg import ObjectInstance as ROSObjectInstance
from spatial_db_msgs.msg import ObjectInstanceOverview as ROSObjectInstanceOverview
from visualization_msgs.msg import Marker, MarkerArray

from semap_ros.srv import *
from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix, euler_matrix, quaternion_from_euler

###
### BASIC MARKER
###

def create_marker(name, pose, color, scale):
  marker = Marker()
  marker.ns = name
  marker.header.frame_id = pose.header.frame_id
  marker.pose = pose.pose
  marker.action = marker.ADD
  marker.scale.x = scale[0]
  marker.scale.y = scale[1]
  marker.scale.z = scale[2]
  marker.color.r = color[0]
  marker.color.g = color[1]
  marker.color.b = color[2]
  marker.color.a = color[3]
  return marker

def create_point_marker(name, pose, color, scale):
  marker = create_marker(name, pose, color, scale)
  marker.type = marker.SPHERE
  return marker

def create_polygon_marker(name, pose, color, scale, polygon):
  marker = create_marker(name, pose, color, scale)
  marker.type = marker.LINE_STRIP
  marker.points = polygon.points
  marker.points.append(polygon.points[0])
  return marker

def create_mesh_marker(name, pose, color, scale, mesh):
  marker = create_marker(name, pose, color, scale)
  marker.type = marker.TRIANGLE_LIST
  for triangle in mesh.triangles:
    marker.points.append(mesh.vertices[triangle.vertex_indices[0]])
    marker.points.append(mesh.vertices[triangle.vertex_indices[1]])
    marker.points.append(mesh.vertices[triangle.vertex_indices[2]])
  return marker

def create_pose_marker(name, pose, color, scale):
  marker = create_marker(name, pose, color, scale)
  marker.type = marker.ARROW
  return marker

def create_text_marker(name, pose, color, scale, text, offset = [0,0,0]):
  text_pose = copy.deepcopy(pose)
  text_pose.pose.position.x += offset[0]
  text_pose.pose.position.y += offset[1]
  text_pose.pose.position.z += offset[2]
  marker = create_marker(name, text_pose, color, scale)
  marker.type = marker.TEXT_VIEW_FACING
  marker.text = text
  return marker

###
### COMPOSITE MARKERS
###

def create_title_marker(frame, name):
  pose = ROSPoseStamped()
  pose.header.frame_id = frame
  pose.pose.orientation.w = 1.0
  marker = create_text_marker("Title", pose, [1.0, 1.0, 1.0, 1.0], [0.1, 0.1, 0.1], name)
  return marker

def create_object_visualization_marker(frame, description, model_visu):
    array = MarkerArray()
    desc = description

    pose = ROSPoseStamped()
    pose.header.frame_id = frame
    pose.pose.orientation.w = 1.0

    for model in desc.point2d_models:
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
      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_polygon_marker("Polygon2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)
      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.point3d_models:
      pose.pose.position = model.geometry

      if model_visu[model.type].show_geo:
        geo_marker = create_point_marker("Point3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.pose3d_models:
      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_pose_marker("Pose3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.polygon3d_models:
      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_polygon_marker("Polygon3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.trianglemesh3d_models:
      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_mesh_marker("TriangleMesh3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.polygonmesh3d_models:
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
