# Nikita Akimov
# interplanety@interplanety.org
#
# GitHub
#    https://github.com/Korchy/1d_knife_imprint

import bmesh
import bpy
from bpy_extras.view3d_utils import region_2d_to_vector_3d
from bpy.props import FloatProperty, IntProperty, EnumProperty
from bpy.types import Operator, Panel, Scene
from bpy.utils import register_class, unregister_class
from mathutils import Vector
from mathutils.bvhtree import BVHTree

bl_info = {
    "name": "Knife Imprint",
    "description": "The variation of Knife Project by with casting all edges of source mesh, not only boundary.",
    "author": "Nikita Akimov, Paul Kotelevets",
    "version": (1, 1, 4),
    "blender": (2, 79, 0),
    "location": "View3D > Tool panel > 1D > Knife Imprint",
    "doc_url": "https://github.com/Korchy/1d_knife_imprint",
    "tracker_url": "https://github.com/Korchy/1d_knife_imprint",
    "category": "All"
}


# MAIN CLASS

class KnifeImprint:

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
                # triangulation['face_map'].items() -> a list of pairs [(tris, source face), (tris, source face), ...]
                dest_vertices_co = [
                    (
                    face.index, tuple(Vector(cls._face_center([dest_world_matrix * vertex.co for vertex in tris_face[0].verts]))
                                      for tris_face in triangulation['face_map'].items()
                                      if tris_face[1].index == face.index)
                    ) for face in bm_copy.faces
                ]
                # tris don't add to face_map - ?
                #   so, we have some empty co if face was the tris [..., (face.index, ()), ...]
                #   remove them and replace with correct data from bmesh copy
                dest_vertices_co = [_e for _e in dest_vertices_co if _e[1] != ()]
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
        # Selection Project
        op = layout.operator(
            operator='knifeimprint.selection_project',
            icon='MOD_BEVEL'
        )
        op.raycast_mode = context.scene.knifeimprint_prop_selection_mode
        op.bvh_epsilon = context.scene.knifeimprint_prop_bvh_epsilon
        op.hybrid_threshold = context.scene.knifeimprint_prop_hybrid_threshold
        col = layout.column()
        col.prop(
            data=context.scene,
            property='knifeimprint_prop_selection_mode',
            expand=True
        )
        layout.prop(
            data=context.scene,
            property='knifeimprint_prop_bvh_epsilon',
            text='Raycast threshold'
        )
        layout.prop(
            data=context.scene,
            property='knifeimprint_prop_hybrid_threshold',
            text='Hybrid threshold'
        )


# OPERATORS

class KnifeImpring_OT_selection_project(Operator):
    bl_idname = 'knifeimprint.selection_project'
    bl_label = 'Selection Project'
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
            ('TRIANGLE_ANY', 'TRIANGLE_ANY', 'TRIANGLE_ANY', '', 9)
        ],
        default='AREA'
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
            ('TRIANGLE_ANY', 'TRIANGLE_ANY', 'TRIANGLE_ANY', '', 9)
        ],
        default='AREA'
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
    register_class(KnifeImpring_OT_knife_imprint)
    register_class(KnifeImpring_OT_selection_project)
    if ui:
        register_class(KnifeImprint_PT_panel)


def unregister(ui=True):
    if ui:
        unregister_class(KnifeImprint_PT_panel)
    unregister_class(KnifeImpring_OT_selection_project)
    unregister_class(KnifeImpring_OT_knife_imprint)
    del Scene.knifeimprint_prop_hybrid_threshold
    del Scene.knifeimprint_prop_bvh_epsilon
    del Scene.knifeimprint_prop_selection_mode


if __name__ == "__main__":
    register()
