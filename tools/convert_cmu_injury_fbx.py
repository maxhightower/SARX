#!/usr/bin/env python3
import bpy
import sys
from pathlib import Path

CLIP_NAMES = {
    "91_16": "CMU_Limp",
    "91_24": "CMU_HurtLegWalk",
    "91_25": "CMU_DragBadLegWalk",
}

def clean_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for datablocks in (
        bpy.data.actions,
        bpy.data.armatures,
        bpy.data.meshes,
        bpy.data.materials,
    ):
        # actions in use are handled after objects are removed.
        pass

def main():
    argv = sys.argv[sys.argv.index("--") + 1 :]
    if len(argv) != 2:
        raise RuntimeError("usage: blender --python convert_cmu_injury_fbx.py -- INPUT_FBX OUTPUT_GLB")

    source = Path(argv[0]).resolve()
    destination = Path(argv[1]).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)

    clean_scene()

    bpy.ops.import_scene.fbx(
        filepath=str(source),
        use_anim=True,
        automatic_bone_orientation=False,
    )

    armatures = [obj for obj in bpy.context.scene.objects if obj.type == "ARMATURE"]
    if len(armatures) != 1:
        raise RuntimeError(f"expected one armature in {source.name}, found {len(armatures)}")

    armature = armatures[0]
    bone_names = [bone.name for bone in armature.data.bones]
    print("SARX_CMU_SOURCE_ARMATURE", source.name, armature.name)
    print("SARX_CMU_SOURCE_BONES", "|".join(bone_names))

    clip_key = source.stem
    clip_name = CLIP_NAMES.get(clip_key, f"CMU_{clip_key}")

    actions = list(bpy.data.actions)
    if not actions and armature.animation_data and armature.animation_data.action:
        actions = [armature.animation_data.action]

    if not actions:
        raise RuntimeError(f"{source.name} contains no Blender actions")

    # Keep a single authored motion per output GLB and give it a stable semantic ID.
    action = actions[0]
    action.name = clip_name
    if armature.animation_data is None:
        armature.animation_data_create()
    armature.animation_data.action = action

    for other in list(bpy.data.actions):
        if other != action:
            bpy.data.actions.remove(other)

    bpy.context.view_layer.objects.active = armature
    armature.select_set(True)

    bpy.ops.export_scene.gltf(
        filepath=str(destination),
        export_format="GLB",
        use_selection=True,
        export_animations=True,
        export_skins=True,
        export_yup=True,
    )

    print("SARX_CMU_EXPORTED", clip_name, destination)

if __name__ == "__main__":
    main()
