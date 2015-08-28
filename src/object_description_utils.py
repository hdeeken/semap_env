#!/usr/bin/env python

'''
Object Description Utils
'''

from numpy import radians
import roslib; roslib.load_manifest("semap_env")

from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix, euler_matrix
from assimp_postgis_importer import importFromFileToMesh

from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import Point32 as ROSPoint32
from geometry_msgs.msg import Pose2D as ROSPose2D
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import Polygon as ROSPolygon
from shape_msgs.msg import Mesh as ROSMesh
from shape_msgs.msg import MeshTriangle as ROSMeshTriangle
from semap_msgs.msg import PolygonMesh as ROSPolygonMesh
from semap_msgs.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from semap_msgs.msg import ObjectDescription as ROSObjectDescription
from semap_msgs.msg import ObjectInstance as ROSObjectInstance

###
### Basic Geometry Models
###

def create_point2d_model(type, point):
  model = Point2DModel()
  model.type = type
  model.geometry = point
  return model

def create_pose2d_model(type, pose):
  model = Pose2DModel()
  model.type = type
  model.pose = pose
  return model

def create_polygon2d_model(type, pose, polygon):
  model = Polygon2DModel()
  model.type = type
  model.pose = pose
  model.geometry = polygon
  return model

def create_point3d_model(type, point):
  model = Point3DModel()
  model.type = type
  model.geometry = point
  return model

def create_pose3d_model(type, pose):
  pose3dmodel = Pose3DModel()
  pose3dmodel.type = type
  pose3dmodel.pose = pose
  return model

def create_pose3d_model(type, pose, polygon):
  model = Polygon3DModel()
  model.type = type
  model.pose = pose
  model.geometry = polygon
  return model

def create_trianglemesh3d_model(type, pose, mesh):
  model = TriangleMesh3DModel()
  model.type = type
  model.pose = pose
  model.geometry = mesh
  return model

def create_trianglemesh3d_model(type, pose, mesh):
  model = PolygonMesh3DModel()
  model.type = type
  model.pose = pose
  model.geometry = mesh
  return model

###
### Composite Geometry Models
###

def create_object_description(type, pose, path):

  pose_ = ROSPose()
  pose_.position.x = pose[0][0]
  pose_.position.y = pose[0][1]
  pose_.position.z = pose[0][2]
  pose_.orientation.x = pose[1][0]
  pose_.orientation.y = pose[1][1]
  pose_.orientation.z = pose[1][2]
  pose_.orientation.w = pose[1][3]

  desc_ros = ROSObjectDescription()
  desc_ros.type = type
  mesh = importFromFileToMesh(path)
  model = create_trianglemesh3d_model("Body", pose_, mesh)
  desc_ros.trianglemesh3d_models.append(model)

  return desc_ros

def create_complete_object_description():
  point2dmodel = Point2DModel()
  point2dmodel.type = '2dpoint'
  point2dmodel.geometry.x = -0.5
  point2dmodel.geometry.y = 0.0
  point2dmodel.geometry.z = 0.0

  pose2dmodel = Pose2DModel()
  pose2dmodel.type = '2dpose'
  pose2dmodel.pose = ROSPose2D()
  pose2dmodel.pose.x = 0.5
  pose2dmodel.pose.y = 1.5
  pose2dmodel.pose.theta = 1.5

  point0 = ROSPoint32(0, 0, 0)
  point1 = ROSPoint32(1, 0, 0)
  point2 = ROSPoint32(1, 1, 0)
  point3 = ROSPoint32(0, 1, 0)
  point4 = ROSPoint32(-1, 0, 0.5)
  point5 = ROSPoint32(-1, 1, 0.5)

  point6 = ROSPoint32(1, 1, 0)
  point7 = ROSPoint32(2, 1, 0)
  point8 = ROSPoint32(2, 2, 0)
  point9 = ROSPoint32(1, 2, 0)
  point10 = ROSPoint32(-2, 1, 0)
  point11 = ROSPoint32(-2, 2, 0)

  polygon0 = ROSPolygon()
  polygon0.points.append(point0)
  polygon0.points.append(point1)
  polygon0.points.append(point2)
  polygon0.points.append(point3)

  polygon1 = ROSPolygon()
  polygon1.points.append(point4)
  polygon1.points.append(point0)
  polygon1.points.append(point3)
  polygon1.points.append(point5)

  polygon2 = ROSPolygon()
  polygon2.points.append(point6)
  polygon2.points.append(point7)
  polygon2.points.append(point8)
  polygon2.points.append(point9)

  polygon3 = ROSPolygon()
  polygon3.points.append(point10)
  polygon3.points.append(point6)
  polygon3.points.append(point9)
  polygon3.points.append(point11)

  polygon2dmodel = Polygon2DModel()
  polygon2dmodel.type = '2dpolygon'
  polygon2dmodel.pose = ROSPose()
  polygon2dmodel.geometry = polygon0

  point3dmodel = Point3DModel()
  point3dmodel.type = '3dpoint'
  point3dmodel.geometry.x = 2.0
  point3dmodel.geometry.y = 2.0
  point3dmodel.geometry.z = 1.0

  pose3dmodel = Pose3DModel()
  pose3dmodel.type = '3dpose'
  pose3dmodel.pose = ROSPose()
  pose3dmodel.pose.position.x = -0.5
  pose3dmodel.pose.position.y = -0.5
  pose3dmodel.pose.position.z = 0.5

  rand_quat = random_quaternion()
  pose3dmodel.pose.orientation.x = rand_quat[0]
  pose3dmodel.pose.orientation.y = rand_quat[1]
  pose3dmodel.pose.orientation.z = rand_quat[2]
  pose3dmodel.pose.orientation.w = rand_quat[3]

  polygon3dmodel = Polygon3DModel()
  polygon3dmodel.type = '3dpolygon'
  polygon3dmodel.pose = ROSPose()
  polygon3dmodel.pose.position.x = 2.0
  polygon3dmodel.geometry = polygon1

  tri0 = ROSMeshTriangle()
  tri1 = ROSMeshTriangle()
  tri0.vertex_indices[0] = 0
  tri0.vertex_indices[1] = 1
  tri0.vertex_indices[2] = 2

  tri1.vertex_indices[0] = 0
  tri1.vertex_indices[1] = 2
  tri1.vertex_indices[2] = 3

  tpoint0 = ROSPoint( 0.0, 0.0, 0.0)
  tpoint1 = ROSPoint( 1.0, 0.0, 1.0)
  tpoint2 = ROSPoint( 1.0, 1.0, 0.0)
  tpoint3 = ROSPoint( 0.0, 1.0, 1.0)
  tpoint4 = ROSPoint(-1.0, 0.0, 0.0)
  tpoint5 = ROSPoint(-1.0, 1.0, 1.0)

  trianglemesh3dmodel = TriangleMesh3DModel()
  trianglemesh3dmodel.type = '3dtrianglemesh'
  trianglemesh3dmodel.pose = ROSPose()
  rand_quat = random_quaternion()
  trianglemesh3dmodel.pose.position.x = 1.0
  trianglemesh3dmodel.pose.position.y = -2.0
  trianglemesh3dmodel.pose.position.z = 0.25
  trianglemesh3dmodel.pose.orientation.x = 0.0
  trianglemesh3dmodel.pose.orientation.y = 0.0
  trianglemesh3dmodel.pose.orientation.z = 0.0
  trianglemesh3dmodel.pose.orientation.w = 0.0
  trianglemesh3dmodel.geometry.vertices.append(tpoint0)
  trianglemesh3dmodel.geometry.vertices.append(tpoint1)
  trianglemesh3dmodel.geometry.vertices.append(tpoint2)
  trianglemesh3dmodel.geometry.vertices.append(tpoint3)
  trianglemesh3dmodel.geometry.triangles.append(tri0)
  trianglemesh3dmodel.geometry.triangles.append(tri1)

  polygonmesh3dmodel = PolygonMesh3DModel()
  polygonmesh3dmodel.type = '3dpolygonmesh'
  polygonmesh3dmodel.pose = ROSPose()
  rand_quat = random_quaternion()
  polygonmesh3dmodel.pose.position.x = 0.5
  polygonmesh3dmodel.pose.position.y = 1.0
  polygonmesh3dmodel.pose.position.z = 0.0
  polygonmesh3dmodel.pose.orientation.x = 0.0
  polygonmesh3dmodel.pose.orientation.y = 0.0
  polygonmesh3dmodel.pose.orientation.z = 0.0
  polygonmesh3dmodel.pose.orientation.w = 0.0
  polygonmesh3dmodel.geometry.polygons.append(polygon2)
  polygonmesh3dmodel.geometry.polygons.append(polygon3)

  desc_ros = ROSObjectDescription()
  desc_ros.type = "Geometry"
  desc_ros.point2d_models.append(point2dmodel)
  desc_ros.pose2d_models.append(pose2dmodel)
  desc_ros.polygon2d_models.append(polygon2dmodel)
  desc_ros.point3d_models.append(point3dmodel)
  desc_ros.pose3d_models.append(pose3dmodel)
  desc_ros.polygon3d_models.append(polygon3dmodel)
  desc_ros.trianglemesh3d_models.append(trianglemesh3dmodel)
  desc_ros.polygonmesh3d_models.append(polygonmesh3dmodel)

  return desc_ros
