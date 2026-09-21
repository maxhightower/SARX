#!/usr/bin/env python3
import bpy
import json
import struct
import sys
from pathlib import Path


BONE_MAP = {
    "hip": "pelvis",
    "abdomen": "spine_01",
    "chest": "spine_03",
    "neck": "neck_01",
    "head": "Head",
    "rCollar": "clavicle_r",
    "rShldr": "upperarm_r",
    "rForeArm": "lowerarm_r",
    "rHand": "hand_r",
    "lCollar": "clavicle_l",
    "lShldr": "upperarm_l",
    "lForeArm": "lowerarm_l",
    "lHand": "hand_l",
    "rThigh": "thigh_r",
    "rShin": "calf_r",
    "rFoot": "foot_r",
    "lThigh": "thigh_l",
    "lShin": "calf_l",
    "lFoot": "foot_l",
}

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

def strip_non_rotation_animation_channels(glb_path: Path):
    data = glb_path.read_bytes()
    if len(data) < 20:
        raise RuntimeError(f"invalid GLB: {glb_path}")

    magic, version, total_length = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67 or version != 2 or total_length != len(data):
        raise RuntimeError(f"unexpected GLB header: {glb_path}")

    json_length, json_type = struct.unpack_from("<II", data, 12)
    if json_type != 0x4E4F534A:
        raise RuntimeError(f"first GLB chunk is not JSON: {glb_path}")

    json_start = 20
    json_end = json_start + json_length
    document = json.loads(data[json_start:json_end].decode("utf-8").rstrip(" \t\r\n\x00"))

    kept = 0
    removed = 0
    for animation in document.get("animations", []):
        channels = animation.get("channels", [])
        filtered = []
        for channel in channels:
            path = channel.get("target", {}).get("path")
            if path == "rotation":
                filtered.append(channel)
                kept += 1
            else:
                removed += 1
        animation["channels"] = filtered

    encoded = json.dumps(
        document,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")
    encoded += b" " * ((4 - len(encoded) % 4) % 4)

    remainder = data[json_end:]
    new_total = 12 + 8 + len(encoded) + len(remainder)

    rebuilt = bytearray()
    rebuilt += struct.pack("<III", magic, version, new_total)
    rebuilt += struct.pack("<II", len(encoded), json_type)
    rebuilt += encoded
    rebuilt += remainder

    glb_path.write_bytes(rebuilt)
    print("SARX_CMU_GLB_ROTATION_CHANNELS", kept)
    print("SARX_CMU_GLB_REMOVED_NONROTATION_CHANNELS", removed)


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

    renamed = {}
    for source_name, target_name in BONE_MAP.items():
        bone = armature.data.bones.get(source_name)
        if bone is None:
            continue
        renamed[source_name] = target_name
        bone.name = target_name

    # Blender normally updates action RNA paths when bones are renamed, but
    # enforce that rewrite explicitly so the exported glTF channels cannot
    # silently retain the CMU names.
    for action in bpy.data.actions:
        for curve in action.fcurves:
            for source_name, target_name in renamed.items():
                old = f'pose.bones["{source_name}"]'
                new = f'pose.bones["{target_name}"]'
                if old in curve.data_path:
                    curve.data_path = curve.data_path.replace(old, new)

    target_names = [bone.name for bone in armature.data.bones if bone.name in BONE_MAP.values()]
    print("SARX_CMU_RETARGETED_BONES", "|".join(sorted(target_names)))
    if len(target_names) < 17:
        raise RuntimeError(
            f"CMU retarget mapped only {len(target_names)} major Quaternius bones"
        )

    # SARX owns world/root translation. The source FBXs use CMU translation
    # units/rest offsets that are not compatible with the SARX Quaternius
    # character. Preserve the authored joint rotations, but let the target
    # character's own rest translations/scales remain authoritative.
    removed_transform_curves = 0
    for action in bpy.data.actions:
        for curve in list(action.fcurves):
            path = curve.data_path
            if (
                path == "location"
                or path == "scale"
                or path.endswith(".location")
                or path.endswith(".scale")
            ):
                action.fcurves.remove(curve)
                removed_transform_curves += 1

    print("SARX_CMU_REMOVED_TRANSLATION_SCALE_CURVES", removed_transform_curves)

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

    strip_non_rotation_animation_channels(destination)
    print("SARX_CMU_EXPORTED", clip_name, destination)

if __name__ == "__main__":
    main()
