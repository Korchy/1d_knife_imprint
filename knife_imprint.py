# Nikita Akimov
# interplanety@interplanety.org
#
# GitHub
#    https://github.com/Korchy/1d_knife_imprint

import bmesh
import bpy
from bpy.types import Operator, Panel
from bpy.utils import register_class, unregister_class

bl_info = {
    "name": "Knife Imprint",
    "description": "The variation of Knife Project by with casting all edges of source mesh, not only boundary.",
    "author": "Nikita Akimov, Paul Kotelevets",
    "version": (1, 3, 0),
    "blender": (2, 79, 0),
    "location": "View3D > Tool panel > 1D > Knife Imprint",
    "doc_url": "https://github.com/Korchy/1d_knife_imprint",
    "tracker_url": "https://github.com/Korchy/1d_knife_imprint",
    "category": "All"
}


# MAIN CLASS

class KnifeImprint:

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
            old_edges = [e.index for e in dest_object.data.edges]
            old_faces = [f.index for f in dest_object.data.polygons]
            # switch active_object to EDIT mode
            if dest_object.mode == 'OBJECT':
                bpy.ops.object.mode_set(mode='EDIT')
            # call "knife project"
            #   calling with object without faces it works for all edges not only for boundary
            bpy.ops.mesh.knife_project()
            # make new selection
            # bpy.ops.object.mode_set(mode='OBJECT')
            # cls._deselect_all(obj=dest_object)
            # for f in dest_object.data.polygons:
            #     f.select = True if f.index not in old_faces else False
            # for edge in dest_object.data.edges:
            #     edge.select = True if edge.index not in old_edges else False
            # bpy.ops.object.mode_set(mode='EDIT')
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


# OPERATORS

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
    if ui:
        register_class(KnifeImprint_PT_panel)


def unregister(ui=True):
    if ui:
        unregister_class(KnifeImprint_PT_panel)
    unregister_class(KnifeImpring_OT_knife_imprint)


if __name__ == "__main__":
    register()
