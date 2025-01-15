# Nikita Akimov
# interplanety@interplanety.org
#
# GitHub
#    https://github.com/Korchy/1d_knife_imprint

import bmesh
import bpy
from bmesh.types import BMEdge
from bpy_extras.view3d_utils import region_2d_to_vector_3d, location_3d_to_region_2d
from bpy.props import FloatProperty, IntProperty, EnumProperty
from bpy.types import Operator, Panel, Scene
from bpy.utils import register_class, unregister_class
from math import isclose
from mathutils import kdtree, Vector
from mathutils.bvhtree import BVHTree

bl_info = {
    "name": "Knife Imprint",
    "description": "The variation of Knife Project by with casting all edges of source mesh, not only boundary.",
    "author": "Nikita Akimov, Paul Kotelevets",
    "version": (1, 5, 1),
    "blender": (2, 79, 0),
    "location": "View3D > Tool panel > 1D > Knife Imprint",
    "doc_url": "https://github.com/Korchy/1d_knife_imprint",
    "tracker_url": "https://github.com/Korchy/1d_knife_imprint",
    "category": "All"
}


# MAIN CLASS

class KnifeImprint:

    @classmethod
    def selection_project_verts(cls, context, dest_object, src_object, threshold_r):
        # make vertices on dest_object selected by vertices on src_object
        #   project all vertices to XY plane, for each src_object's vertex find nearest dest_object's vertex
        #   if it is in radius threshold (threshold_r) - select it
        if src_object and dest_object:
            # current mode
            mode = dest_object.mode
            if dest_object.mode == 'EDIT':
                bpy.ops.object.mode_set(mode='OBJECT')
            # deselect all
            cls._deselect_all(obj=dest_object)
            # switch to vertex selection mode
            context.tool_settings.mesh_select_mode = (True, False, False)
            # create KDTree for dest_object to quickly find nearest vertices
            dest_world_matrix = dest_object.matrix_world.copy()
            bm_dest = bmesh.new()
            bm_dest.from_mesh(dest_object.data)
            bm_dest.verts.ensure_lookup_table()
            bm_dest.edges.ensure_lookup_table()
            kd = kdtree.KDTree(len(dest_object.data.vertices))
            for vertex in dest_object.data.vertices:
                vertex_global = dest_world_matrix * vertex.co
                kd.insert(Vector((vertex_global.x, vertex_global.y, 0.0)), vertex.index)  # in xy projection
            kd.balance()
            # src_object's edges
            src_world_matrix = src_object.matrix_world.copy()
            bm_src = bmesh.new()
            bm_src.from_mesh(src_object.data)
            bm_src.verts.ensure_lookup_table()
            bm_src.edges.ensure_lookup_table()
            # for each src_object's vertex
            src_verts = [src_world_matrix * _vertex.co for _vertex in bm_src.verts]
            for vertex in src_verts:
                dest_vertex = kd.find(co=Vector((vertex.x, vertex.y, 0.0)))
                distance = dest_vertex[2]
                dest_vertex_index = dest_vertex[1]
                if distance < threshold_r:
                    bm_dest.verts[dest_vertex_index].select = True
            # save changed data to mesh
            bm_dest.to_mesh(dest_object.data)
            bm_dest.free()
            bm_src.free()
            # return mode back
            bpy.ops.object.mode_set(mode=mode)

    @classmethod
    def selection_project_edge_isolines(cls, context, dest_object, src_object, threshold_r):
        # special case for selecting edges on dest object by isolines (edges) on src object
        #   works the same way as selection_project_verts but for selecting edges
        if src_object and dest_object:
            # current mode
            mode = dest_object.mode
            if dest_object.mode == 'EDIT':
                bpy.ops.object.mode_set(mode='OBJECT')
            # switch to edge selection mode
            context.tool_settings.mesh_select_mode = (False, True, False)
            # create KDTree for dest_object to quickly find nearest vertices
            dest_world_matrix = dest_object.matrix_world.copy()
            bm_dest = bmesh.new()
            bm_dest.from_mesh(dest_object.data)
            bm_dest.verts.ensure_lookup_table()
            bm_dest.edges.ensure_lookup_table()
            kd = kdtree.KDTree(len(dest_object.data.vertices))
            for vertex in dest_object.data.vertices:
                vertex_global = dest_world_matrix * vertex.co
                kd.insert(Vector((vertex_global.x, vertex_global.y, 0.0)), vertex.index)  # in xy projection
            kd.balance()
            # deselect all bm_dest
            cls._deselect_bm_all(bm=bm_dest)
            # src_object's edges
            src_world_matrix = src_object.matrix_world.copy()
            bm_src = bmesh.new()
            bm_src.from_mesh(src_object.data)
            bm_src.verts.ensure_lookup_table()
            bm_src.edges.ensure_lookup_table()
            # get global co-s for each edge's vertices on src object
            # ((co, co), (co, co), ...)
            src_edge_verts_co = ((src_world_matrix * edge.verts[0].co, src_world_matrix * edge.verts[1].co)
                                 for edge in bm_src.edges)
            # for each src edge's vertices
            for verts_pair in src_edge_verts_co:
                # try to find edge with correspondence vertices on dest object
                position0, index0, distance0 = kd.find(co=Vector((verts_pair[0].x, verts_pair[0].y, 0.0)))
                position1, index1, distance1 = kd.find(co=Vector((verts_pair[1].x, verts_pair[1].y, 0.0)))
                if distance0 < threshold_r and distance1 < threshold_r:
                    # got two vertices on dest object - find the edge between them
                    edge = next((edge for edge in bm_dest.verts[index0].link_edges
                                 if edge.other_vert(bm_dest.verts[index0]).index == index1), None)
                    if edge:
                        edge.select = True
            # save changed data to mesh
            bm_dest.to_mesh(dest_object.data)
            bm_dest.free()
            bm_src.free()
            # return mode back
            bpy.ops.object.mode_set(mode=mode)

    @classmethod
    def selection_project_edge_triangle_any(cls, context, dest_object, src_object, bvh_epsilon=0.0):
        # make edges on dest_object selected by edges on src_object
        #   this variant works same as knife imprint with 'TRIANGLE ANY' mode
        #   virtually triangulating dest object, raycast to src object, as a result we will have to linked
        #   lists: polygons on dest object - polygons on src object, next - select border edges on dest object polygons
        if src_object and dest_object:
            # current mode
            mode = dest_object.mode
            if dest_object.mode == 'EDIT':
                bpy.ops.object.mode_set(mode='OBJECT')
            # deselect all
            cls._deselect_all(obj=dest_object)
            # bm dest object
            dest_world_matrix = dest_object.matrix_world.copy()
            bm = bmesh.new()
            bm.from_mesh(dest_object.data)
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bm.faces.ensure_lookup_table()
            # try to triangulate mesh, get face center of each triangle to use it as a raycast point
            # create a copy to save face indices (triangulation will replace current geometry)
            bm_copy = bm.copy()
            # triangulate
            triangulation = bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method=0, ngon_method=0)
            # triangulation['face_map'] -> dict {tris: source_face, tris: source_face, ...}
            # triangulation['face_map'].items() -> a list of pairs [(tris, source face), (tris, source face), ...]
            # invert face mapping to get format like: {source_face: [tris, tris,...], ...}
            inv_map = {}
            for k, v in triangulation['face_map'].items():
                inv_map[v] = inv_map.get(v, []) + [k]
            # get vertices from inverted map
            dest_vertices_co = [
                (source_face.index, tuple(cls._face_center(tuple(dest_world_matrix * vert.co for vert in tris.verts))
                                  for tris in trises))
                for source_face, trises in inv_map.items()
            ]
            # tris wasn't add to face_map - why?
            #   add them from bmesh copy
            dest_vertices_tris = [
                (face.index, (Vector(cls._face_center([dest_world_matrix * vertex.co for vertex in face.verts])),))
                for face in bm_copy.faces if len(face.verts) <= 3]
            dest_vertices_co += dest_vertices_tris
            # now we have the following format: [(face.index, (vertex.co, vertex.co, ...)), ...]
            # viewport direction vector
            region = next((_region for _region in context.area.regions if _region.type == 'WINDOW'), None)
            region_3d = context.area.spaces[0].region_3d
            viewport_view_direction = region_2d_to_vector_3d(
                region,
                region_3d,
                coord=(region.width / 2.0, region.height / 2.0)
            )
            viewport_view_direction.negate()    # from dest_object geometry to viewport
            # create BVH tree for src object
            src_world_matrix = src_object.matrix_world.copy()
            src_vertices_world = [src_world_matrix * vertex.co for vertex in src_object.data.vertices]
            src_faces = [polygon.vertices for polygon in src_object.data.polygons]
            src_bvh_tree = BVHTree.FromPolygons(src_vertices_world, src_faces, all_triangles=False, epsilon=bvh_epsilon)
            # from each dest_vertices_co vertex cast ray to viewport and check hit with src_bvh_tree
            # dest_vertices_co: [(face.index, (vertex.co, vertex.co, ...)), ...]
            # write to dict {src_face_index: [dest_face_index, dest_face_index, ...], ...}
            src_dest_refers = {}
            for dest_face_index, vertex_cos in dest_vertices_co:
                dest_face = dest_object.data.polygons[dest_face_index]
                # only for face side of dest_object
                if viewport_view_direction.dot(dest_face.normal) >= 0.0:
                    for co in vertex_cos:
                        raycast = src_bvh_tree.ray_cast(co, viewport_view_direction)
                        # raycast = (position, normal, index, distance)
                        if raycast[2] is not None:
                            src_dest_refers[raycast[2]] = src_dest_refers.get(raycast[2], []) + [dest_face_index]
            # for each src-dest group of faces get only boundary edges and select them
            cls._deselect_bm_all(bm=bm)
            for src_face_index, dest_faces_indices in src_dest_refers.items():
                dest_edges = [[edge for edge in face.edges] for face in bm_copy.faces if face.index in dest_faces_indices]
                # convert from list with edges in nested lists to flat set with edges
                dest_edges = set(cls._nested_lists_to_list(lst=dest_edges))
                boundary_edges = [edge for edge in dest_edges
                                  if len(edge.link_faces) >= 2
                                  and not((edge.link_faces[0].index in dest_faces_indices)
                                         and (edge.link_faces[1].index in dest_faces_indices))]
                # select boundary edges
                for edge in boundary_edges:
                    edge.select = True
            # copy selection from bm to original mesh
            bm_copy.to_mesh(dest_object.data)
            # clear bm objects
            bm.free()
            bm_copy.free()
            # switch to edge selection mode
            context.tool_settings.mesh_select_mode = (False, True, False)
            # return mode back
            bpy.ops.object.mode_set(mode=mode)

    @classmethod
    def selection_project_edge(cls, context, dest_object, src_object, threshold_d=0.001, threshold_r=0.001):
        # make edges on dest_object selected by edges on src_object
        #   for each src_object's edges, project its vertices on XY projection, find the nearest vertices
        #   on dest_object projection to XY, check radius threshold (threshold_r) if less - find the
        #   shortest path between them, count it length and compare with src_object's edge length
        #   if equal (by threshold_d) - add this edges to selection
        if src_object and dest_object:
            # current mode
            mode = dest_object.mode
            if dest_object.mode == 'EDIT':
                bpy.ops.object.mode_set(mode='OBJECT')
            # deselect all
            cls._deselect_all(obj=dest_object)
            # create KDTree for dest_object to quickly find nearest vertices
            dest_world_matrix = dest_object.matrix_world.copy()
            bm_dest = bmesh.new()
            bm_dest.from_mesh(dest_object.data)
            bm_dest.verts.ensure_lookup_table()
            bm_dest.edges.ensure_lookup_table()
            kd = kdtree.KDTree(len(dest_object.data.vertices))
            for vertex in dest_object.data.vertices:
                vertex_global = dest_world_matrix * vertex.co
                kd.insert(Vector((vertex_global.x, vertex_global.y, 0.0)), vertex.index)  # in xy projection
            kd.balance()
            # src_object's edges
            src_world_matrix = src_object.matrix_world.copy()
            bm_src = bmesh.new()
            bm_src.from_mesh(src_object.data)
            bm_src.verts.ensure_lookup_table()
            bm_src.edges.ensure_lookup_table()
            # [(edge, vert0_co, vert1_co), ...]
            src_edges = [
                (edge, src_world_matrix * edge.verts[0].co, src_world_matrix * edge.verts[1].co)
                for edge in bm_src.edges
            ]
            edges_to_select = []    # here we will save edges that will be selected after all iterations
            for edge in src_edges:
            # for edge in (_e for _e in src_edges if _e[0].index == 3):
            # for edge in (_e for _e in src_edges if _e[0].index == 6):     # A
                src_edge = edge[0]
                src_edge_length = cls._edge_xy_length(src_edge)
                src_v0 = edge[1]
                src_v1 = edge[2]
                # find nearest vertices on dest_object by projecting geometry to XY
                dest_v0 = kd.find(co=Vector((src_v0.x, src_v0.y, 0.0)))
                dest_v1 = kd.find(co=Vector((src_v1.x, src_v1.y, 0.0)))
                # print('dest_v0', dest_v0, threshold_r)
                # print('dest_v1', dest_v1, threshold_r)
                # if two founded vertices less threshold_r by distance
                if (dest_v1[2] < threshold_r) and (dest_v0[2] < threshold_r):
                    # try to get the shortest_path between them
                    # check if this is the single edge, this needs because bpy.ops.mesh.shortest_path_select() drops
                    #   selection if two vertices of one edge are selected
                    single_edge = next((_edge for _edge in bm_dest.edges if
                                        {_edge.verts[0].index, _edge.verts[1].index} == {dest_v0[1], dest_v1[1]} ), None)
                    if single_edge:
                        # add this edge to future selection by length compare
                        # if isclose(cls._edge_xy_length(edge=single_edge), src_edge_length, rel_tol=threshold):
                        # print('edge lenght', cls._edge_xy_length(edge=single_edge))
                        # print('template edge lenght', src_edge_length)
                        # print('diff', cls._edge_xy_length(edge=single_edge) - src_edge_length, 'threshold', threshold_d)
                        if abs(cls._edge_xy_length(edge=single_edge) - src_edge_length) <= threshold_d:
                            edges_to_select += [single_edge.index,]
                    else:
                        # select two founded vertices on dest_object
                        cls._deselect_all(obj=dest_object)
                        dest_object.data.vertices[dest_v0[1]].select = True
                        dest_object.data.vertices[dest_v1[1]].select = True
                        bpy.ops.object.mode_set(mode='EDIT')
                        context.tool_settings.mesh_select_mode = (True, False, False)   # require fo shortest_path_select
                        bpy.ops.mesh.shortest_path_select()
                        bpy.ops.object.mode_set(mode='OBJECT')
                        # get selected edges
                        shortest_path_selected_edges = [_edge.index for _edge in dest_object.data.edges if _edge.select]
                        # get selected edges in bm_dest for calc their length
                        bm_dest_selected_edges = [_edge for _edge in bm_dest.edges
                                                  if _edge.index in shortest_path_selected_edges]
                        # get sum of their length
                        shortest_path_length = sum(map(cls._edge_xy_length, bm_dest_selected_edges))
                        # add to future selection by compare length
                        # if isclose(shortest_path_length, src_edge_length, abs_tol=threshold):
                        if abs(shortest_path_length - src_edge_length) <= threshold_d:
                            edges_to_select += shortest_path_selected_edges
            # select all previously saved edges
            bpy.ops.object.mode_set(mode='OBJECT')
            cls._deselect_all(obj=dest_object)
            # switch to edge selection mode
            context.tool_settings.mesh_select_mode = (False, True, False)
            for edge_index in edges_to_select:
                dest_object.data.edges[edge_index].select = True
            # return mode back
            bpy.ops.object.mode_set(mode=mode)

    @classmethod
    def selection_project_edge_v0(cls, context, dest_object, src_object, threshold=0.001):
        # make edges on dest_object selected by edges on src_object
        #   This variant is scanning all dest_object edges and for each edge check if this edge belongs to
        #   any src_object's edge (if both vertices belongs, the edge belongs itself)
        #   Slow and not precise
        if src_object and dest_object:
            # current mode
            mode = dest_object.mode
            if dest_object.mode == 'EDIT':
                bpy.ops.object.mode_set(mode='OBJECT')
            # switch to select mode = Face
            context.tool_settings.mesh_select_mode = (False, True, False)
            # deselect all
            cls._deselect_all(obj=dest_object)
            # bmesh for dest_object
            dest_world_matrix = dest_object.matrix_world.copy()
            bm_dest = bmesh.new()
            bm_dest.from_mesh(dest_object.data)
            bm_dest.verts.ensure_lookup_table()
            bm_dest.edges.ensure_lookup_table()
            # bmesh for src_object
            src_world_matrix = src_object.matrix_world.copy()
            bm_src = bmesh.new()
            bm_src.from_mesh(src_object.data)
            bm_src.verts.ensure_lookup_table()
            bm_src.edges.ensure_lookup_table()

            # project all edges of dest_object to viewport
            #   this way we have already 2d vectors in this sequences
            # region = next((_region for _region in context.area.regions if _region.type == 'WINDOW'), None)
            # region_3d = context.area.spaces[0].region_3d
            # region_3d = context.space_data.region_3d
            # src_edges_proj = [
            #     (edge, [(vert.index, location_3d_to_region_2d(region=region, rv3d=region_3d, coord=(src_world_matrix * vert.co)))
            #                   for vert in edge.verts])
            #     for edge in bm_src.edges
            # ]
            # # print(src_edges_proj)
            # dest_edges_proj = [
            #     (edge, [(vert.index, location_3d_to_region_2d(region=region, rv3d=region_3d, coord=(dest_world_matrix * vert.co)))
            #                   for vert in edge.verts])
            #     for edge in bm_dest.edges
            # ]
            # # print(dest_edges_proj)

            # get all edges for src_object and dest_object
            src_edges_proj = [
                (edge, [(vert.index, src_world_matrix * vert.co) for vert in edge.verts])
                for edge in bm_src.edges
            ]
            # print(src_edges_proj)
            dest_edges_proj = [
                (edge, [(vert.index, dest_world_matrix * vert.co) for vert in edge.verts])
                for edge in bm_dest.edges
            ]
            # print(dest_edges_proj)

            # check each dest_object's edge for belonging to any src_object's edge
            for dest_edge in dest_edges_proj:
                edge_verts = dest_edge[1]   # [(index, co), (index, co)]
                dv0 = edge_verts[0][1]
                dv1 = edge_verts[1][1]
                # use projection to the XY plane (to work with 2d coordinates)
                dv0.resize_2d()
                dv1.resize_2d()
                # check each src_object's edge if dest_object's edge belongs to it
                for src_edge in src_edges_proj:
                    src_verts = src_edge[1]   # [(index, co), (index, co)]
                    sv0 = src_verts[0][1]
                    sv1 = src_verts[1][1]
                    # use projection to the XY plane (to work with 2d coordinates)
                    sv0.resize_2d()
                    sv1.resize_2d()
                    # if both vertices of dest_objects edge belongs to source_object's edge - dest_object's edge belongs src_object's edge
                    if cls._point_belongs_segment(p=dv0, v0=sv0, v1=sv1, threshold=threshold) \
                            and cls._point_belongs_segment(p=dv1, v0=sv0, v1=sv1, threshold=threshold):
                        dest_object.data.edges[dest_edge[0].index].select = True
            bm_src.free()
            bm_dest.free()
            # return mode back
            bpy.ops.object.mode_set(mode=mode)

    @classmethod
    def selection_project(cls, context, dest_object, src_object, raycast_mode='AREA', bvh_epsilon=0.0,
                          hybrid_threshold=1):
        # make selected polygons on the dest_object by visible overlap of polygons from src_object
        # raycast_mode = 'AREA' - raycast from all face vertices (select face if even one hits)
        #               'FACEDOT' - raycast from face center
        #               'AREA_OR' - raycast from all vertices (select face if all hits)
        #               'AREA_FACEDOT' - raycast from all face vertices and from face center (select face if even one hits)
        #               'HYBRID' - process n-gons like AREA, triangles and quads like FACEDOT
        #               'HYBRID+' - process like HYBRID but select face (for ngons) only if hybrid_threshold vertices or more are hit
        #               'HYBRID-' - process like HYBRID but not select face (for ngons) if more than hybrid_threshold vertices are not hit
        #               'VERTEX_SELECT' - raycast and select only vertices
        #               'TRIANGLE_ALL', 'TRIANGLE_ANY' - virtually triangulate mesh, raycast from face center of each tris
        #                   ALL - select parent face if all hit
        #                   ANY - select parent face if at least one hit
        # bvh_epsilon - epsilon threshold for BVH tree
        # hybrid_threshold - how many vertices include/exclude for detecting select face or not
        if src_object and dest_object:
            # ray_cast from each vertex of dest_object along the viewport direction vector to check
            #   intersections with src_object
            # current mode
            mode = dest_object.mode
            if dest_object.mode == 'EDIT':
                bpy.ops.object.mode_set(mode='OBJECT')
            # list of dest_object vertices with links to linked faces
            dest_world_matrix = dest_object.matrix_world.copy()
            bm = bmesh.new()
            bm.from_mesh(dest_object.data)
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bm.faces.ensure_lookup_table()
            if raycast_mode == 'AREA':
                # 'AREA'
                # format: [(vertex.co, (face.index, face.index, ...)), ...]
                dest_vertices_co = [(dest_world_matrix * vertex.co, [face.index for face in vertex.link_faces])
                                    for vertex in bm.verts]
            elif raycast_mode == 'AREA_OR':
                # 'AREA_OR'
                # format: [(face.index, (vertex.co, vertex.co, ...)), ...]
                dest_vertices_co = [(face.index, tuple(dest_world_matrix * vertex.co for vertex in face.verts))
                                    for face in bm.faces]
            elif raycast_mode == 'AREA_FACEDOT':
                # 'AREA + FACEDOT'
                # format: [(vertex.co, (face.index, face.index, ...)), ...]
                dest_vertices_area = [(dest_world_matrix * vertex.co, [face.index for face in vertex.link_faces])
                                    for vertex in bm.verts]
                dest_vertices_facedot = [
                    (Vector(cls._face_center([dest_world_matrix * vertex.co for vertex in face.verts])), [face.index,])
                    for face in bm.faces
                ]
                dest_vertices_co = dest_vertices_area + dest_vertices_facedot
            elif raycast_mode in {'HYBRID', 'HYBRID+', 'HYBRID-'}:
                # 'HYBRID'
                # format: [(face.index, (vertex.co, vertex.co, ...)), ...]
                # n-gons will process by all vertices
                dest_vertices_ngons = [(face.index, tuple(dest_world_matrix * vertex.co for vertex in face.verts))
                                    for face in bm.faces if len(face.verts) > 4]
                # tris and quads will process only by center (facedot)
                dest_vertices_tris_quads = [
                    (face.index, (Vector(cls._face_center([dest_world_matrix * vertex.co for vertex in face.verts])),))
                    for face in bm.faces if len(face.verts) <= 4]
                dest_vertices_co = dest_vertices_ngons + dest_vertices_tris_quads
            elif raycast_mode == 'VERTEX_SELECT':
                # 'VERTEX SELECT'
                # format: ((vertex.index, vertex.co), ...)
                dest_vertices_co = [(vertex.index, dest_world_matrix * vertex.co) for vertex in bm.verts]
            elif raycast_mode in {'TRIANGLE_ALL', 'TRIANGLE_ANY'}:
                # 'TRIANGLE'
                # format: [(face.index, (vertex.co, vertex.co, ...)), ...]
                # try to triangulate mesh, get face center of each triangle to use it as a raycast point
                # create a copy to save face indices (triangulation will replace current geometry)
                bm_copy = bm.copy()
                # triangulate
                triangulation = bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method=0, ngon_method=0)
                # triangulation['face_map'] -> dict {tris: source_face, tris: source_face, ...}
                # triangulation['face_map'].items() -> a list of pairs [(tris, source face), (tris, source face), ...]
                # invert face mapping to get format like: {source_face: [tris, tris,...], ...}
                inv_map = {}
                for k, v in triangulation['face_map'].items():
                    inv_map[v] = inv_map.get(v, []) + [k]
                # get vertices from inverted map
                dest_vertices_co = [
                    (key.index, tuple(cls._face_center(tuple(dest_world_matrix * vert.co for vert in face.verts))
                                      for face in value))
                    for key, value in inv_map.items()
                ]
                # tris wasn't add to face_map - why?
                #   add them from bmesh copy
                dest_vertices_tris = [
                    (face.index, (Vector(cls._face_center([dest_world_matrix * vertex.co for vertex in face.verts])),))
                    for face in bm_copy.faces if len(face.verts) <= 3]
                dest_vertices_co += dest_vertices_tris
            else:
                # 'FACEDOT'
                # format: [(vertex.co, (face.index, face.index, ...)), ...]
                dest_vertices_co = [
                    [cls._face_center([dest_world_matrix * vertex.co for vertex in face.verts]), [face.index,]]
                    for face in bm.faces
                ]
            bm.free()
            # viewport direction vector
            region = next((_region for _region in context.area.regions if _region.type == 'WINDOW'), None)
            region_3d = context.area.spaces[0].region_3d
            viewport_view_direction = region_2d_to_vector_3d(
                region,
                region_3d,
                coord=(region.width / 2.0, region.height / 2.0)
            )
            viewport_view_direction.negate()    # from dest_object geometry to viewport
            # create BVH tree for src object
            src_world_matrix = src_object.matrix_world.copy()
            src_vertices_world = [src_world_matrix * vertex.co for vertex in src_object.data.vertices]
            src_faces = [polygon.vertices for polygon in src_object.data.polygons]
            src_bvh_tree = BVHTree.FromPolygons(src_vertices_world, src_faces, all_triangles=False, epsilon=bvh_epsilon)
            # from each des_object vertex cast ray to viewport and check hit with src_bvh_tree
            cls._deselect_all(obj=dest_object)
            if raycast_mode == 'AREA_OR':
                # format: [(face.index, (vertex.co, vertex.co, ...)), ...]
                for face_index, vertex_cos in dest_vertices_co:
                    # only if raycasts from all vertices were success
                    if (None, None, None, None) not in (src_bvh_tree.ray_cast(co, viewport_view_direction) for co in vertex_cos):
                        face = dest_object.data.polygons[face_index]
                        # only for face side of dest_object
                        if viewport_view_direction.dot(face.normal) >= 0.0:
                            face.select = True
            elif raycast_mode == 'HYBRID':
                # format: [(face.index, (vertex.co, vertex.co, ...)), ...]
                for face_index, vertex_cos in dest_vertices_co:
                    # if raycasts from any vertex was success
                    for co in vertex_cos:
                        hit = src_bvh_tree.ray_cast(co, viewport_view_direction)
                        if hit[0] is not None:
                            face = dest_object.data.polygons[face_index]
                            # only for face side of dest_object
                            if viewport_view_direction.dot(face.normal) >= 0.0:
                                face.select = True
                            continue
            elif raycast_mode == 'HYBRID+':
                # format: [(face.index, (vertex.co, vertex.co, ...)), ...]
                for face_index, vertex_cos in dest_vertices_co:
                    face = dest_object.data.polygons[face_index]
                    # only for face side of dest_object
                    if viewport_view_direction.dot(face.normal) >= 0.0:
                        raycasts = tuple(src_bvh_tree.ray_cast(co, viewport_view_direction) for co in vertex_cos)
                        hits_amount = len(raycasts) - raycasts.count((None, None, None, None))
                        if len(face.vertices) > 4:
                            # ngon
                            if hits_amount >= hybrid_threshold:
                                face.select = True
                        else:
                            # tris/quad
                            if raycasts[0][0] is not None:
                                face.select = True
            elif raycast_mode == 'HYBRID-':
                # format: [(face.index, (vertex.co, vertex.co, ...)), ...]
                for face_index, vertex_cos in dest_vertices_co:
                    face = dest_object.data.polygons[face_index]
                    # only for face side of dest_object
                    if viewport_view_direction.dot(face.normal) >= 0.0:
                        raycasts = tuple(src_bvh_tree.ray_cast(co, viewport_view_direction) for co in vertex_cos)
                        hits_amount = len(raycasts) - raycasts.count((None, None, None, None))
                        if len(face.vertices) > 4:
                            # ngon
                            if hits_amount >= len(face.vertices) - hybrid_threshold:
                                face.select = True
                        else:
                            # tris/quad
                            if raycasts[0][0] is not None:
                                face.select = True
            elif raycast_mode == 'VERTEX_SELECT':
                # format: ((vertex.index, vertex.co), ...)
                for index, co in dest_vertices_co:
                    hit = src_bvh_tree.ray_cast(co, viewport_view_direction)
                    if hit[0] is not None:
                        dest_object.data.vertices[index].select = True
            elif raycast_mode in {'TRIANGLE_ALL', 'TRIANGLE_ANY'}:
                # format: [(face.index, (vertex.co, vertex.co, ...)), ...]
                for face_index, vertex_cos in dest_vertices_co:
                    face = dest_object.data.polygons[face_index]
                    # only for face side of dest_object
                    if viewport_view_direction.dot(face.normal) >= 0.0:
                        raycasts = tuple(src_bvh_tree.ray_cast(co, viewport_view_direction) for co in vertex_cos)
                        misses = raycasts.count((None, None, None, None))
                        if raycast_mode == 'TRIANGLE_ALL' and misses == 0:
                            face.select = True
                        elif raycast_mode == 'TRIANGLE_ANY' and misses < len(raycasts):
                            face.select = True
            else:
                # format: [(vertex.co, (face.index, face.index, ...)), ...]
                for co, link_faces_ids in dest_vertices_co:
                    hit = src_bvh_tree.ray_cast(co, viewport_view_direction)
                    if hit[0] is not None:
                        for face_index in link_faces_ids:
                            face = dest_object.data.polygons[face_index]
                            # only for face side of dest_object
                            if viewport_view_direction.dot(face.normal) >= 0.0:
                                face.select = True
            # switch to select mode = Face
            context.tool_settings.mesh_select_mode = (False, False, True)
            # return mode back
            bpy.ops.object.mode_set(mode=mode)

    @classmethod
    def knife_imprint(cls, context, dest_object, src_object):
        # cast edges like "knife project" from src_object to dest_object with creating geometry by all edges
        if src_object and dest_object:
            # make a copy of selected object
            selected_object_copy = src_object.copy()
            selected_object_copy.data = selected_object_copy.data.copy()
            context.scene.objects.link(selected_object_copy)
            # remove faces on a copy
            bm = bmesh.new()
            bm.from_mesh(selected_object_copy.data)
            cls._remove_faces(bm=bm)
            bm.to_mesh(selected_object_copy.data)
            bm.free()
            # hide src_object
            src_object.hide = True
            # current mode
            mode = dest_object.mode
            # save edges for further selection
            # switch active_object to EDIT mode
            if dest_object.mode == 'OBJECT':
                bpy.ops.object.mode_set(mode='EDIT')
            # call "knife project"
            #   calling with object without faces it works for all edges not only for boundary
            bpy.ops.mesh.knife_project()
            # make new selection;    commented by Paul, staying with the default selection
            # bpy.ops.object.mode_set(mode='OBJECT')
            # cls.selection_project(context=context, dest_object=dest_object, src_object=src_object)
            # remove copy
            bpy.data.objects.remove(selected_object_copy)
            # return mode back
            bpy.ops.object.mode_set(mode=mode)
            # unhide src object
            src_object.hide = False
            # force redraw areas
            cls._refresh_areas(context=context)

    @classmethod
    def _edge_xy_length(cls, edge: BMEdge):
        # return length of the edge's projection to XY plane
        vect = edge.verts[1].co - edge.verts[0].co
        proj = Vector((vect.x, vect.y, 0.0))
        return proj.length

    @staticmethod
    def _proj_xy(v: Vector):
        # return project of v on XY
        return Vector((v.x, v.y, 0.0))

    @classmethod
    def _edge_xy_len2(cls, v: Vector):
        # return square length of the edge's projection to XY plane
        return cls._segment_len2(v0=Vector((0.0, 0.0, 0.0)), v1=Vector((v.x, v.y, 0.0)))

    @staticmethod
    def _segment_len2(v0, v1):
        # return square of the segment length
        return (v1.x - v0.x) * (v1.x - v0.x) + (v1.y - v0.y) * (v1.y - v0.y)

    @classmethod
    def _point_belongs_segment(cls, p: Vector, v0: Vector, v1: Vector, threshold = 0.001):
        # return True if point p belongs segment v0-v1
        # point on one of the ends of the segment
        if (isclose(p.x, v0.x, rel_tol=threshold) and isclose(p.y, v0.y, rel_tol=threshold)
                or isclose(p.x, v1.x, rel_tol=threshold) and isclose(p.y, v1.y, rel_tol=threshold)):
            return True
        # all points are collinear (are on the one line if cross of v1-v0 and p-v0 is 0.0)
        cross = (p.y - v0.y) * (v1.x - v0.x) - (p.x - v0.x) * (v1.y - v0.y)
        # print('cross', cross)
        if isclose(abs(cross), 0.0, abs_tol=threshold) :
            # point is between v0-v1 if dot of v1-v0 and p-v0 is positive and less than square length of v0-v1
            dot = (p.x - v0.x) * (v1.x - v0.x) + (p.y - v0.y) * (v1.y - v0.y)
            # print('dot', dot, 'segment len 2', cls._segment_len2(v0, v1))
            if 0 < dot < cls._segment_len2(v0, v1):
                return True
        return False

    # @staticmethod
    # def _point_belongs_segment(point: Vector, segment_v0: Vector, segment_v1: Vector):
    #     # return True if point belongs segment
    #     t = 0.001
    #     x = point.x
    #     y = point.y
    #     x1 = segment_v0.x
    #     y1 = segment_v0.y
    #     x2 = segment_v1.x
    #     y2 = segment_v1.y
    #     # flag = (x1 == x2 and x1 == x and min(y1, y2) <= y <= max(y1, y2))
    #     flag = (isclose(x1, x2, abs_tol=t) and isclose(x1, x, abs_tol=t) and min(y1, y2) <= y <= max(y1, y2))
    #     if not flag:
    #         # flag = (y1 == y2 and y1 == y and min(x1, x2) <= x <= max(x1, x2))
    #         flag = (isclose(y1, y2, abs_tol=t) and isclose(y1, y, abs_tol=t) and min(x1, x2) <= x <= max(x1, x2))
    #     if not flag:
    #         flag = (isclose(abs(y1 - y) + abs(y2 - y), abs(y2 - y1), abs_tol=t)
    #                 and isclose(abs(x1 - x) + abs(x2 - x), abs(x2 - x1), abs_tol=t)
    #                 and isclose((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1), 0, abs_tol=t))
    #
    #         # a = y1 - y2
    #         # b = x2 - x1
    #         # c = x1 * y2 - x2 * y1
    #         # # flag = (a * x + b * y + c == 0)
    #         print('flag', isclose(abs(y1 - y) + abs(y2 - y), abs(y2 - y1), abs_tol=t), isclose(abs(x1 - x) + abs(x2 - x), abs(x2 - x1), abs_tol=t), isclose((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1), 0, abs_tol=t))
    #         # flag = isclose(a * x + b * y + c,  0.0, abs_tol=t)
    #     return flag
    #     # print("YES" if flag else "NO")
    #     # return True if flag else False
    #
    #     # segment_vector = segment_v1 - segment_v0
    #     # print('segment v', segment_vector)
    #     # point_vector = segment_v1 - point
    #     # print('point v', point_vector)
    #     # dot = segment_vector.dot(point_vector)
    #     # # dot = point_vector.dot(segment_vector)
    #     # print('dot', dot, 'point vector len', point_vector.length)
    #     # return isclose(dot, point_vector.length, abs_tol=0.01)

    @staticmethod
    def _nested_lists_to_list(lst):
        # convert list with nested lists to flat list
        return [x for xs in lst for x in xs]

    @staticmethod
    def _face_center(vertices):
        # returns the coordinates of the polygon center by coordinates if its vertexes
        # vertices format = ((0.1, 0.0, 1.0), (1.0, 0.0, 1.0), ...)
        x_list = [vertex[0] for vertex in vertices]
        y_list = [vertex[1] for vertex in vertices]
        z_list = [vertex[2] for vertex in vertices]
        length = len(vertices)
        x = sum(x_list) / length
        y = sum(y_list) / length
        z = sum(z_list) / length
        return tuple((x, y, z))

    @staticmethod
    def _deselect_all(obj):
        # deselect all vertices/edges/faced on mesh data
        for face in obj.data.polygons:
            face.select = False
        for edge in obj.data.edges:
            edge.select = False
        for vertex in obj.data.vertices:
            vertex.select = False

    @staticmethod
    def _deselect_bm_all(bm):
        # remove all selection from edges and vertices in bmesh
        for face in bm.faces:
            face.select = False
        for edge in bm.edges:
            edge.select = False
        for vertex in bm.verts:
            vertex.select = False

    @staticmethod
    def _remove_faces(bm):
        # remove faces
        for face in bm.faces:
            bm.faces.remove(face)

    @staticmethod
    def _refresh_areas(context):
        # force redraw all areal
        if context.screen:
            for area in context.screen.areas:
                area.tag_redraw()

    @staticmethod
    def ui(layout, context):
        # ui panel
        # Knife Imprint
        layout.operator(
            operator='knifeimprint.knife_imprint',
            icon='MOD_UVPROJECT'
        )
        # Face Project selection
        box = layout.box()
        op = box.operator(
            operator='knifeimprint.selection_project',
            icon='MOD_BEVEL'
        )
        op.raycast_mode = context.scene.knifeimprint_prop_selection_mode
        op.bvh_epsilon = context.scene.knifeimprint_prop_bvh_epsilon
        op.hybrid_threshold = context.scene.knifeimprint_prop_hybrid_threshold
        col = box.column()
        col.prop(
            data=context.scene,
            property='knifeimprint_prop_selection_mode',
            expand=False,
            text=''
        )
        box.prop(
            data=context.scene,
            property='knifeimprint_prop_bvh_epsilon',
            text='Raycast threshold'
        )
        box.prop(
            data=context.scene,
            property='knifeimprint_prop_hybrid_threshold',
            text='Hybrid threshold'
        )
        # Selection Project Edges
        box = layout.box()
        op = box.operator(
            operator='knifeimprint.selection_project_edges',
            icon='MOD_LATTICE'
        )
        op.threshold_d = context.scene.knifeimprint_prop_selection_edges_d_threshold
        op.threshold_r = context.scene.knifeimprint_prop_selection_edges_r_threshold
        op.bvh_epsilon = context.scene.knifeimprint_prop_bvh_epsilon
        op.mode = context.scene.knifeimprint_prop_selection_edges_mode
        box.prop(
            data=context.scene,
            property='knifeimprint_prop_selection_edges_mode',
            expand=True
        )
        if context.scene.knifeimprint_prop_selection_edges_mode == 'SHORTEST_PATH':
            box.prop(
                data=context.scene,
                property='knifeimprint_prop_selection_edges_d_threshold',
                text='Distance Threshold'
            )
            box.prop(
                data=context.scene,
                property='knifeimprint_prop_selection_edges_r_threshold',
                text='Radius Threshold'
            )
        elif context.scene.knifeimprint_prop_selection_edges_mode == 'ISOLINES':
            box.prop(
                data=context.scene,
                property='knifeimprint_prop_selection_edges_r_threshold',
                text='Radius Threshold'
            )
        # Selection Project Verts
        box = layout.box()
        op = box.operator(
            operator='knifeimprint.selection_project_verts',
            icon='MOD_MESHDEFORM'
        )
        op.threshold_r = context.scene.knifeimprint_prop_selection_verts_r_threshold
        box.prop(
            data=context.scene,
            property='knifeimprint_prop_selection_verts_r_threshold',
            text='Radius Threshold'
        )



# OPERATORS

class KnifeImpring_OT_selection_project_verts(Operator):
    bl_idname = 'knifeimprint.selection_project_verts'
    bl_label = 'Verts Selection'
    bl_description = 'top projection, selects matching vertices'
    bl_options = {'REGISTER', 'UNDO'}

    threshold_r = FloatProperty(
        name='Radius Threshold',
        default=0.05
    )

    def execute(self, context):
        selected_object = context.selected_objects[:]
        selected_object.remove(context.active_object)
        selected_object = selected_object[0] if selected_object else selected_object
        KnifeImprint.selection_project_verts(
            context=context,
            dest_object=context.active_object,
            src_object=selected_object,
            threshold_r=self.threshold_r
        )
        return {'FINISHED'}

class KnifeImpring_OT_selection_project_edges(Operator):
    bl_idname = 'knifeimprint.selection_project_edges'
    bl_label = 'Edges Selection'
    bl_description = 'Select active object edges from passive object'
    bl_options = {'REGISTER', 'UNDO'}

    threshold_d = FloatProperty(
        name='Distance Threshold',
        default=0.2
    )

    threshold_r = FloatProperty(
        name='Radius Threshold',
        default=0.21
    )

    bvh_epsilon = FloatProperty(
        name='bvh_epsilon',
        default=0.0
    )

    mode = EnumProperty(
        name='Selection Edges Mode',
        items=[
            ('SHORTEST_PATH', 'SHORTEST PATH', 'top projection, selects vertices and guess shortest paths between them, slow', '', 0),
            ('TRIANGLE_ANY', 'TRIANGLE ANY', 'view projection, selects edges of passive objects faces boundaries', '', 1),
            ('ISOLINES', 'ISOLINES', 'top projection, selects exactly matching edges', '', 2)
        ],
        default='TRIANGLE_ANY'
    )

    def execute(self, context):
        selected_object = context.selected_objects[:]
        selected_object.remove(context.active_object)
        selected_object = selected_object[0] if selected_object else selected_object
        if self.mode == 'SHORTEST_PATH':
            KnifeImprint.selection_project_edge(
                context=context,
                dest_object=context.active_object,
                src_object=selected_object,
                threshold_d=self.threshold_d,
                threshold_r=self.threshold_r
            )
        elif self.mode == 'TRIANGLE_ANY':
            KnifeImprint.selection_project_edge_triangle_any(
                context=context,
                dest_object=context.active_object,
                src_object=selected_object,
                bvh_epsilon=self.bvh_epsilon
            )
        elif self.mode == 'ISOLINES':
            KnifeImprint.selection_project_edge_isolines(
                context=context,
                dest_object=context.active_object,
                src_object=selected_object,
                threshold_r=self.threshold_r
            )
        return {'FINISHED'}

class KnifeImpring_OT_selection_project(Operator):
    bl_idname = 'knifeimprint.selection_project'
    bl_label = 'Face Project selection'
    bl_description = 'Select faces by view projection, blocked by passive object polygons'
    bl_options = {'REGISTER', 'UNDO'}

    raycast_mode = EnumProperty(
        name='Selection Mode',
        items=[
            ('AREA', 'AREA', 'AREA', '', 0),
            ('FACEDOT', 'FACEDOT', 'FACEDOT', '', 1),
            ('AREA_OR', 'AREA OR', 'AREA OR', '', 2),
            ('AREA_FACEDOT', 'AREA + FACEDOT', 'AREA + FACEDOT', '', 3),
            ('HYBRID', 'HYBRID', 'HYBRID', '', 4),
            ('VERTEX_SELECT', 'VERTEX SELECT', 'VERTEX SELECT', '', 5),
            ('HYBRID-', 'HYBRID-', 'HYBRID-', '', 6),
            ('HYBRID+', 'HYBRID+', 'HYBRID+', '', 7),
            ('TRIANGLE_ALL', 'TRIANGLE ALL', 'TRIANGLE ALL', '', 8),
            ('TRIANGLE_ANY', 'TRIANGLE_ANY', 'view projection, selects edges of passive objects faces boundaries', '', 9)
        ],
        default='TRIANGLE_ANY'
    )

    bvh_epsilon = FloatProperty(
        name='bvh_epsilon',
        default=0.0
    )

    hybrid_threshold = IntProperty(
        name='Hybrid threshold',
        default=1,
        min=1
    )

    def execute(self, context):
        selected_object = context.selected_objects[:]
        selected_object.remove(context.active_object)
        selected_object = selected_object[0] if selected_object else selected_object
        KnifeImprint.selection_project(
            context=context,
            dest_object=context.active_object,
            src_object=selected_object,
            raycast_mode=self.raycast_mode,
            bvh_epsilon=self.bvh_epsilon,
            hybrid_threshold=self.hybrid_threshold
        )
        return {'FINISHED'}

class KnifeImpring_OT_knife_imprint(Operator):
    bl_idname = 'knifeimprint.knife_imprint'
    bl_label = 'Knife Imprint'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        selected_object = context.selected_objects[:]
        selected_object.remove(context.active_object)
        selected_object = selected_object[0] if selected_object else selected_object
        KnifeImprint.knife_imprint(
            context=context,
            dest_object=context.active_object,
            src_object=selected_object
        )
        return {'FINISHED'}


# PANELS

class KnifeImprint_PT_panel(Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_label = "Knife Imprint"
    bl_category = '1D'

    def draw(self, context):
        KnifeImprint.ui(
            layout=self.layout,
            context=context
        )


# REGISTER

def register(ui=True):
    Scene.knifeimprint_prop_selection_edges_mode = EnumProperty(
        name='Selection Edges Mode',
        items=[
            ('SHORTEST_PATH', 'SHORTEST PATH', 'top projection, selects vertices and guess shortest paths between them, slow', '', 0),
            ('TRIANGLE_ANY', 'TRIANGLE ANY', 'view projection, selects edges of passive objects faces boundaries', '', 1),
            ('ISOLINES', 'ISOLINES', 'top projection, selects exactly matching edges', '', 2)
        ],
        default='TRIANGLE_ANY'
    )
    Scene.knifeimprint_prop_selection_mode = EnumProperty(
        name='Selection Mode',
        items=[
            ('AREA', 'AREA', 'AREA', '', 0),
            ('FACEDOT', 'FACEDOT', 'FACEDOT', '', 1),
            ('AREA_OR', 'AREA OR', 'AREA OR', '', 2),
            ('AREA_FACEDOT', 'AREA + FACEDOT', 'AREA + FACEDOT', '', 3),
            ('HYBRID', 'HYBRID', 'HYBRID', '', 4),
            ('VERTEX_SELECT', 'VERTEX SELECT', 'VERTEX SELECT', '', 5),
            ('HYBRID-', 'HYBRID-', 'HYBRID-', '', 6),
            ('HYBRID+', 'HYBRID+', 'HYBRID+', '', 7),
            ('TRIANGLE_ALL', 'TRIANGLE ALL', 'TRIANGLE ALL', '', 8),
            ('TRIANGLE_ANY', 'TRIANGLE_ANY', 'view projection, selects edges of passive objects faces boundaries', '', 9)
        ],
        default='TRIANGLE_ANY'
    )
    Scene.knifeimprint_prop_bvh_epsilon = FloatProperty(
        name='bvh_epsilon',
        default=0.0
    )
    Scene.knifeimprint_prop_hybrid_threshold = IntProperty(
        name='Hybrid threshold',
        default=1,
        min=1
    )
    Scene.knifeimprint_prop_selection_edges_d_threshold = FloatProperty(
        name='Edges Selection Distance Threshold',
        default=0.2
    )
    Scene.knifeimprint_prop_selection_edges_r_threshold = FloatProperty(
        name='Edges Selection Radius Threshold',
        default=0.21
    )
    Scene.knifeimprint_prop_selection_verts_r_threshold = FloatProperty(
        name='Verts Selection Radius Threshold',
        default=0.05
    )
    register_class(KnifeImpring_OT_knife_imprint)
    register_class(KnifeImpring_OT_selection_project)
    register_class(KnifeImpring_OT_selection_project_edges)
    register_class(KnifeImpring_OT_selection_project_verts)
    if ui:
        register_class(KnifeImprint_PT_panel)


def unregister(ui=True):
    if ui:
        unregister_class(KnifeImprint_PT_panel)
    unregister_class(KnifeImpring_OT_selection_project_verts)
    unregister_class(KnifeImpring_OT_selection_project_edges)
    unregister_class(KnifeImpring_OT_selection_project)
    unregister_class(KnifeImpring_OT_knife_imprint)
    del Scene.knifeimprint_prop_selection_verts_r_threshold
    del Scene.knifeimprint_prop_selection_edges_d_threshold
    del Scene.knifeimprint_prop_selection_edges_r_threshold
    del Scene.knifeimprint_prop_hybrid_threshold
    del Scene.knifeimprint_prop_bvh_epsilon
    del Scene.knifeimprint_prop_selection_mode
    del Scene.knifeimprint_prop_selection_edges_mode


if __name__ == "__main__":
    register()
