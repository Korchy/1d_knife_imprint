# Nikita Akimov
# interplanety@interplanety.org
#
# GitHub
#    https://github.com/Korchy/1d_knife_imprint

import bmesh
import bpy
from bpy_extras.view3d_utils import region_2d_to_vector_3d
from bpy.types import Operator, Panel
from bpy.utils import register_class, unregister_class
from mathutils.bvhtree import BVHTree

bl_info = {
    "name": "Knife Imprint",
    "description": "The variation of Knife Project by with casting all edges of source mesh, not only boundary.",
    "author": "Nikita Akimov, Paul Kotelevets",
    "version": (1, 1, 0),
    "blender": (2, 79, 0),
    "location": "View3D > Tool panel > 1D > Knife Imprint",
    "doc_url": "https://github.com/Korchy/1d_knife_imprint",
    "tracker_url": "https://github.com/Korchy/1d_knife_imprint",
    "category": "All"
}


# MAIN CLASS

class KnifeImprint:

    @classmethod
    def selection_project(cls, context, dest_object, src_object):
        # make selected polygons on the dest_object by visible overlap of polygons from src_object
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
            dest_vertices_co = [(dest_world_matrix * vertex.co, [face.index for face in vertex.link_faces])
                                for vertex in bm.verts]
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
            src_bvh_tree = BVHTree.FromPolygons(src_vertices_world, src_faces)
            # from each des_object vertex cast ray to viewport and check hit with src_bvh_tree
            cls._deselect_all(obj=dest_object)
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
            # make new selection
            bpy.ops.object.mode_set(mode='OBJECT')
            cls.selection_project(context=context, dest_object=dest_object, src_object=src_object)
            # remove copy
            bpy.data.objects.remove(selected_object_copy)
            # return mode back
            bpy.ops.object.mode_set(mode=mode)
            # unhide src object
            src_object.hide = False
            # force redraw areas
            cls._refresh_areas(context=context)

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
        layout.operator(
            operator='knifeimprint.selection_project',
            icon='MOD_BEVEL'
        )


# OPERATORS

class KnifeImpring_OT_selection_project(Operator):
    bl_idname = 'knifeimprint.selection_project'
    bl_label = 'Selection Project'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        selected_object = context.selected_objects[:]
        selected_object.remove(context.active_object)
        selected_object = selected_object[0] if selected_object else selected_object
        KnifeImprint.selection_project(
            context=context,
            dest_object=context.active_object,
            src_object=selected_object
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
    register_class(KnifeImpring_OT_knife_imprint)
    register_class(KnifeImpring_OT_selection_project)
    if ui:
        register_class(KnifeImprint_PT_panel)


def unregister(ui=True):
    if ui:
        unregister_class(KnifeImprint_PT_panel)
    unregister_class(KnifeImpring_OT_selection_project)
    unregister_class(KnifeImpring_OT_knife_imprint)


if __name__ == "__main__":
    register()
