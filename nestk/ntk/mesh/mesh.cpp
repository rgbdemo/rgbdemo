/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#include "mesh.h"
#include "ply.h"

#include <ntk/utils/debug.h>
#include <ntk/utils/opencv_utils.h>

#include <opencv/cv.h>

#include <ntk/utils/time.h>

#include <fstream>

using namespace cv;

namespace
{

  struct PlyVertex
  {
    float x,y,z;
    float nx,ny,nz;
    float u,v; // tex coords.
    unsigned char r,g,b;     /* vertex color */
  };

  struct PlyFace
  {
    unsigned char nverts;    /* number of vertex indices in list */
    int *verts;              /* vertex index list */
    unsigned char ntexcoord; /* number of tex coords */
    float *texcoord;         /* texture coordinates */
    double nx,ny,nz;         /* normal vector */
  };

  /* list of the kinds of elements in the user's object */
  const char *elem_names[] =
  {
    "vertex", "face"
  };

  ply::PlyProperty available_vertex_properties[] = {
    {"x", Float32, Float32, offsetof(PlyVertex,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(PlyVertex,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(PlyVertex,z), 0, 0, 0, 0},
    {"nx", Float32, Float32, offsetof(PlyVertex,nx), 0, 0, 0, 0},
    {"ny", Float32, Float32, offsetof(PlyVertex,ny), 0, 0, 0, 0},
    {"nz", Float32, Float32, offsetof(PlyVertex,nz), 0, 0, 0, 0},
    {"s", Float32, Float32, offsetof(PlyVertex,u), 0, 0, 0, 0},
    {"t", Float32, Float32, offsetof(PlyVertex,v), 0, 0, 0, 0},
    {"red", Uint8, Uint8, offsetof(PlyVertex,r), 0, 0, 0, 0},
    {"green", Uint8, Uint8, offsetof(PlyVertex,g), 0, 0, 0, 0},
    {"blue", Uint8, Uint8, offsetof(PlyVertex,b), 0, 0, 0, 0},
  };

  ply::PlyProperty available_face_properties[] = { /* list of property information for a face */
    {"vertex_indices", PLY_Int32, PLY_Int32, offsetof(PlyFace,verts),
     1, Uint8, Uint8, offsetof(PlyFace,nverts)},
  };



}

namespace ntk
{

  void Mesh::applyTransform(const Pose3D& pose)
  {
    foreach_idx(i, vertices)
      vertices[i] = pose.cameraTransform(vertices[i]);
  }

  Point3f Mesh :: centerize()
  {
    Point3f center(0,0,0);
    foreach_idx(i, vertices)
    {
      center += vertices[i];
    }
    center *= 1.0/vertices.size();

    foreach_idx(i, vertices)
    {
      vertices[i] -= center;
    }
    return center;
  }

  void Mesh::saveToPlyFile(const char* filename) const
  {
    if (texture.data)
      imwrite(cv::format("%s.texture.png", filename), texture);

    std::ofstream ply_file (filename);
    ply_file << "ply\n";
    ply_file << "format ascii 1.0\n";
    ply_file << "element vertex " << vertices.size() << "\n";
    ply_file << "property float x\n";
    ply_file << "property float y\n";
    ply_file << "property float z\n";

    if (hasNormals())
    {
      ply_file << "property float nx\n";
      ply_file << "property float ny\n";
      ply_file << "property float nz\n";
    }

    if (hasTexcoords())
    {
      // put it twice, blender uses (s,t) and meshlab (u,v)
      ply_file << "property float s\n";
      ply_file << "property float t\n";
    }

    if (hasColors())
    {
      ply_file << "property uchar red\n";
      ply_file << "property uchar green\n";
      ply_file << "property uchar blue\n";
    }

    if (hasFaces())
    {
      ply_file << "element face " << faces.size() << "\n";
      ply_file << "property list uchar uint vertex_indices\n";
      // For meshlab wedges.
      if (hasTexcoords())
        ply_file << "property list uchar float texcoord\n";
    }


    ply_file << "end_header\n";

    foreach_idx(i, vertices)
    {
      ply_file << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z;

      if (hasNormals())
        ply_file << " " << normals[i].x << " " << normals[i].y << " " << normals[i].z;

      if (hasTexcoords())
      {
        ply_file << " " << texcoords[i].x << " " << texcoords[i].y;
      }

      if (hasColors())
        ply_file << " " << (int)colors[i][0] << " " << (int)colors[i][1] << " " << (int)colors[i][2];

      ply_file << "\n";
    }

    if (hasFaces())
    {
      foreach_idx(i, faces)
      {
        ply_file << faces[i].numVertices();
        for (unsigned j = 0; j < faces[i].numVertices(); ++j)
          ply_file << " " << faces[i].indices[j];
        if (hasTexcoords())
        {
          ply_file << " 6";
          for (unsigned j = 0; j < faces[i].numVertices(); ++j)
          {
            ply_file << " " << texcoords[faces[i].indices[j]].x;
            ply_file << " " << 1.0 - texcoords[faces[i].indices[j]].y;
          }
        }
        ply_file << "\n";
      }
    }

    ply_file.close();
  }

  void Mesh::loadFromPlyFile(const char* filename)
  {
    vertices.clear();
    colors.clear();
    texcoords.clear();
    normals.clear();
    faces.clear();

    bool has_colors = false;
    bool has_normals = false;
    bool has_texcoords = false;
    bool has_faces = false;

    std::vector<ply::PlyProperty> vertex_properties;
    std::vector<ply::PlyProperty> face_properties;

    std::vector<PlyVertex> ply_vertices;
    std::vector<PlyFace> ply_faces;

    FILE* mesh_file = fopen(filename, "r");
    ntk_ensure(mesh_file, "Could not open mesh file.");

    ply::PlyFile* ply_file = ply::read_ply(mesh_file);
    ntk_ensure(ply_file, "Could not parse mesh file.");

    for (int i = 0; i < ply_file->num_elem_types; i++)
    {
      /* prepare to read the i'th list of elements */
      int elem_count = 0;
      const char* elem_name = ply::setup_element_read_ply (ply_file, i, &elem_count);

      if (ply::equal_strings("vertex", elem_name))
      {
        /* create a vertex list to hold all the vertices */
        ply_vertices.resize(elem_count);

        /* set up for getting vertex elements */
        ply::setup_property_ply (ply_file, &available_vertex_properties[0]);
        ply::setup_property_ply (ply_file, &available_vertex_properties[1]);
        ply::setup_property_ply (ply_file, &available_vertex_properties[2]);

        if (has_property(ply_file, "vertex", "nx"))
        {
          has_normals = true;
          ply::setup_property_ply (ply_file, &available_vertex_properties[3]);
          ply::setup_property_ply (ply_file, &available_vertex_properties[4]);
          ply::setup_property_ply (ply_file, &available_vertex_properties[5]);
        }

        if (has_property(ply_file, "vertex", "s"))
        {
          has_texcoords = true;
          ply::setup_property_ply (ply_file, &available_vertex_properties[6]);
          ply::setup_property_ply (ply_file, &available_vertex_properties[7]);
        }

        if (has_property(ply_file, "vertex", "red"))
        {
          has_colors = true;
          ply::setup_property_ply (ply_file, &available_vertex_properties[8]);
          ply::setup_property_ply (ply_file, &available_vertex_properties[9]);
          ply::setup_property_ply (ply_file, &available_vertex_properties[10]);
        }

        /* grab all the vertex elements */
        for (int j = 0; j < elem_count; j++)
          ply::get_element_ply (ply_file, &ply_vertices[j]);
      }
      else if (ply::equal_strings("face", elem_name))
      {
        has_faces = true;

        /* create a list to hold all the face elements */
        ply_faces.resize(elem_count);

        /* set up for getting face elements */
        setup_property_ply (ply_file, &available_face_properties[0]);
        // setup_property_ply (ply_file, &global::face_props[1]);

        /* grab all the face elements */
        for (int j = 0; j < elem_count; j++)
        {
          get_element_ply (ply_file, &ply_faces[j]);
        }
      }
    }
    fclose(mesh_file);

    vertices.resize(ply_vertices.size());
    if (has_colors) colors.resize(vertices.size());
    if (has_normals) normals.resize(vertices.size());

    foreach_idx(i, ply_vertices)
    {
      const PlyVertex& v = ply_vertices[i];
      vertices[i] = Point3f(v.x, v.y, v.z);
      if (has_colors)
        colors[i] = Vec3b(v.r, v.g, v.b);
      if (has_normals)
        normals[i] = Point3f(v.nx, v.ny, v.nz);
    }

    if (has_faces)
    {
      faces.resize(ply_faces.size());
      foreach_idx(i, ply_faces)
      {
        const PlyFace& f = ply_faces[i];
        ntk_ensure(f.nverts == 3, "Only triangles are supported.");
        for (int j = 0; j < f.nverts; ++j)
          faces[i].indices[j] = f.verts[j];
      }
    }

  }

  void Mesh:: clear()
  {
    vertices.clear();
    colors.clear();
    normals.clear();
    texcoords.clear();
    faces.clear();
    texture = cv::Mat3b();
  }

  void Mesh :: buildFromSurfels(const std::vector<Surfel>& surfels, int min_views)
  {
    clear();

    unsigned long start = ntk::Time::getMillisecondCounter();
    int idx = 0;

    foreach_idx(i, surfels)
    {
      if (!surfels[i].enabled())
        continue;

      if (surfels[i].n_views < min_views)
        continue;

      // FIXME: temp
      vertices.push_back(surfels[i].location);
      colors.push_back(surfels[i].color);
      normals.push_back(surfels[i].normal);
      continue;

      const Surfel& surfel = surfels[i];
      Vec3f v1, v2;
      orthogonal_basis(v1, v2, surfel.normal);
      Point3f p0 = surfel.location + Point3f(v1 * surfel.radius);
      Point3f p1 = surfel.location + Point3f(v1 * (surfel.radius/2.0f) + v2 * surfel.radius);
      Point3f p2 = surfel.location + Point3f(v1 * (-surfel.radius/2.0f) + v2 * surfel.radius);
      Point3f p3 = surfel.location + Point3f(v1 * -surfel.radius);
      Point3f p4 = surfel.location + Point3f(v1 * (-surfel.radius/2.0f) + v2 * (-surfel.radius));
      Point3f p5 = surfel.location + Point3f(v1 * (surfel.radius/2.0f) + v2 * (-surfel.radius));
      vertices.push_back(p0);
      vertices.push_back(p1);
      vertices.push_back(p2);
      vertices.push_back(p3);
      vertices.push_back(p4);
      vertices.push_back(p5);

      for (int k = 0; k < 6; ++k)
        colors.push_back(surfel.color);

      for (int k = 0; k < 6; ++k)
        normals.push_back(surfel.normal);

      {
        Face f;
        f.indices[0] = idx+5;
        f.indices[1] = idx+0;
        f.indices[2] = idx+1;
        faces.push_back(f);
      }

      {
        Face f;
        f.indices[0] = idx+5;
        f.indices[1] = idx+1;
        f.indices[2] = idx+2;
        faces.push_back(f);
      }

      {
        Face f;
        f.indices[0] = idx+4;
        f.indices[1] = idx+5;
        f.indices[2] = idx+2;
        faces.push_back(f);
      }

      {
        Face f;
        f.indices[0] = idx+4;
        f.indices[1] = idx+2;
        f.indices[2] = idx+3;
        faces.push_back(f);
      }

      idx += 6;
    }

    unsigned long end = ntk::Time::getMillisecondCounter();
   // ntk_dbg_print((end-start) / 1000., 1);
  }

  void generate_mesh_from_plane(Mesh& mesh, const ntk::Plane& plane,
                                const Point3f& center, float plane_size)
  {
    Point3f line1[2] = { Point3f(center.x-plane_size, center.y-plane_size, center.z-plane_size),
                         Point3f(center.x-plane_size, center.y+plane_size, center.z-plane_size) };
    Point3f line2[2] = { Point3f(center.x+plane_size, center.y-plane_size, center.z-plane_size),
                         Point3f(center.x+plane_size, center.y+plane_size, center.z-plane_size) };
    Point3f line3[2] = { Point3f(center.x-plane_size, center.y-plane_size, center.z+plane_size),
                         Point3f(center.x-plane_size, center.y+plane_size, center.z+plane_size) };
    Point3f line4[2] = { Point3f(center.x+plane_size, center.y-plane_size, center.z+plane_size),
                         Point3f(center.x+plane_size, center.y+plane_size, center.z+plane_size) };

    Point3f plane_p1 = plane.intersectionWithLine(line1[0], line1[1]);
    Point3f plane_p2 = plane.intersectionWithLine(line2[0], line2[1]);
    Point3f plane_p3 = plane.intersectionWithLine(line3[0], line3[1]);
    Point3f plane_p4 = plane.intersectionWithLine(line4[0], line4[1]);

    mesh.vertices.push_back(plane_p1);
    mesh.vertices.push_back(plane_p2);
    mesh.vertices.push_back(plane_p3);
    mesh.vertices.push_back(plane_p4);

    {
      Face f;
      f.indices[0] = 0;
      f.indices[1] = 1;
      f.indices[2] = 2;
      mesh.faces.push_back(f);
    }

    {
      Face f;
      f.indices[0] = 2;
      f.indices[1] = 1;
      f.indices[2] = 3;
      mesh.faces.push_back(f);
    }
  }

  void Mesh::addCube(const cv::Point3f& center, const cv::Point3f& sizes)
  {
    const int links[12][3] = { {0, 1, 3},
                               {0, 3, 2},

                               {0, 5, 1},
                               {0, 4, 5},

                               {3, 1, 5},
                               {3, 5, 7},

                               {2, 3, 7},
                               {2, 7, 6},

                               {6, 5, 4},
                               {6, 7, 5},

                               {0, 2, 6},
                               {0, 6, 4} };

    Point3f cube_points[8];
    Point3f proj_cube_points[8];

    const double xvals [] = {center.x-(sizes.x/2), center.x+(sizes.x/2)};
    const double yvals [] = {center.y-(sizes.y/2), center.y+(sizes.y/2)};
    const double zvals [] = {center.z-(sizes.z/2), center.z+(sizes.z/2)};

    int first_vertex_index = vertices.size();
    for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
    for (int k = 0; k < 2; ++k)
    {
      Point3f p(xvals[i], yvals[j], zvals[k]);
      vertices.push_back(p);
    }

    for (int f = 0; f < 12; ++f)
    {
      Face face;
      for (int i = 0; i < 3; ++i)
      {
        face.indices[i] = first_vertex_index + links[f][i];
      }
      faces.push_back(face);
    }
  }

} // end of ntk
