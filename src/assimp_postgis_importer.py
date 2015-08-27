#!/usr/bin/env python
#-*- coding: UTF-8 -*-

import os
import sys
from pyassimp import pyassimp
import roslib; roslib.load_manifest("semap_env")

from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import Point32 as ROSPoint32
from mesh_msgs.msg import TriangleMesh as ROSMeshTriangleMesh
from mesh_msgs.msg import TriangleIndices as ROSTriangleIndices

from mesh_msgs.msg import PolygonIndices as ROSPolygonIndices
from mesh_msgs.msg import PolygonMesh as ROSPolygonMesh

def importFromFileToTIN(path):
  scene = pyassimp.load(path)
  results = []
  for index, mesh in enumerate(scene.meshes):
    prefix = 'TIN('
    postfix = ')'
    infix = ''
    triangles = mesh.faces
    vertices = mesh.vertices
    triangle_strings = []
    for triangle in triangles:
      indices = triangle.indices
      triangle_strings.append('((%f %f %f, %f %f %f, %f %f %f, %f %f %f))' \
              % (vertices[indices[0]].x, vertices[indices[0]].y, vertices[indices[0]].z, \
                 vertices[indices[1]].x, vertices[indices[1]].y, vertices[indices[1]].z, \
                 vertices[indices[2]].x, vertices[indices[2]].y, vertices[indices[2]].z, \
                 vertices[indices[0]].x, vertices[indices[0]].y, vertices[indices[0]].z))
    infix = ",".join(triangle_strings)
  tin_string = prefix + infix + postfix
  results.append(tin_string)
  pyassimp.release(scene)
  return results

def importFromFileToMesh(path):
  scene = pyassimp.load(path)
  if len(scene.meshes) == 0:
    print "assimp_importer: found no mesh, abort"
    return None
  elif len(scene.meshes) > 1:
    print "assimp_importer: found %s meshes, abort", len(scene.meshes)
    return None
  else:
    faces = scene.meshes[0].faces
    vertices = scene.meshes[0].vertices
    result = ROSMeshTriangleMesh()
    for vertex in vertices:
      ros_point = ROSPoint()
      ros_point.x = vertex[0]
      ros_point.y = vertex[1]
      ros_point.z = vertex[2]
      result.vertices.append(ros_point)
    for face in faces:
      if len(face.indices) != 3:
        print "WARNING: Triangle contained $s instead of 3 indicies", len(face.indices)
      ros_triangle = ROSTriangleIndices()
      ros_triangle.vertex_indices = [face.indices[0], face.indices[1], face.indices[2]]
      result.triangles.append(ros_triangle)
  pyassimp.release(scene)
  return result
