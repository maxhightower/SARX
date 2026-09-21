#!/usr/bin/env python3
import bpy
import csv
import json
import struct
import sys
from pathlib import Path

BONE_MAP = {
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
    "139_19": "CMU_WalkWoundedLeg",
    "142_12": "CMU_PainfulLeftKnee",
}

def clean_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    for action in list(bpy.data.actions):
        bpy.data.actions.remove(action)

def find_armature_with_bones(required_names):
    required = set(required_names)

    for obj in bpy.context.scene.objects:
        if obj.type != "ARMATURE":
            continue

        names = {bone.name for bone in obj.data.bones}
        if required.issubset(names):
            return obj

    raise RuntimeError(
        "could not find armature containing required bones: "
        + ",".join(sorted(required))
    )

def bone_depth(bone):
    depth = 0
    current = bone.parent
    while current is not None:
        depth += 1
        current = current.parent
    return depth

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
    document = json.loads(
        data[json_start:json_end]
        .decode("utf-8")
        .rstrip(" \t\r\n\x00")
    )

    kept = 0
    removed = 0

    for animation in document.get("animations", []):
        filtered = []

        for channel in animation.get("channels", []):
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

    if len(argv) != 3:
        raise RuntimeError(
            "usage: blender --python convert_cmu_injury_fbx.py "
            "-- SOURCE_FBX TARGET_CHARACTER_GLB OUTPUT_GLB"
        )

    source = Path(argv[0]).resolve()
    target_character = Path(argv[1]).resolve()
    destination = Path(argv[2]).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)

    clean_scene()

    # Import the actual SARX Quaternius target rig first.
    bpy.ops.import_scene.gltf(
        filepath=str(target_character),
    )

    target_armature = find_armature_with_bones(
        {"pelvis", "spine_01", "thigh_l", "calf_l", "foot_l"}
    )

    target_armature.name = "SARX_Quaternius_Target"

    # The character fixture should contribute rest pose only.
    if target_armature.animation_data is not None:
        target_armature.animation_data_clear()

    target_meshes = [
        obj
        for obj in bpy.context.scene.objects
        if obj.type == "MESH"
    ]

    # Import the authored CMU motion and its source skeleton.
    bpy.ops.import_scene.fbx(
        filepath=str(source),
        use_anim=True,
        automatic_bone_orientation=False,
    )

    source_armature = find_armature_with_bones(
        {"hip", "abdomen", "rThigh", "rShin", "rFoot"}
    )

    if source_armature == target_armature:
        raise RuntimeError("source and target armatures resolved to the same object")

    source_action = (
        source_armature.animation_data.action
        if source_armature.animation_data is not None
        else None
    )

    if source_action is None:
        raise RuntimeError(f"{source.name} contains no source action")

    source_bones = [bone.name for bone in source_armature.data.bones]
    target_bones = [bone.name for bone in target_armature.data.bones]

    print("SARX_CMU_SOURCE_ARMATURE", source.name, source_armature.name)
    print("SARX_CMU_SOURCE_BONES", "|".join(source_bones))
    print("SARX_QUATERNIUS_TARGET_BONES", "|".join(target_bones))

    mapped = []

    for source_name, target_name in BONE_MAP.items():
        source_bone = source_armature.data.bones.get(source_name)
        target_bone = target_armature.data.bones.get(target_name)

        if source_bone is None or target_bone is None:
            continue

        source_rest_world = (
            source_armature.matrix_world
            @ source_bone.matrix_local
        )

        target_rest_world = (
            target_armature.matrix_world
            @ target_bone.matrix_local
        )

        # Map a rotation expressed in the source bone's rest-space
        # coordinates into the target bone's rest-space coordinates.
        #
        # Do NOT copy absolute source world orientation onto the target pose.
        # That makes child world rotations look individually plausible while
        # corrupting the target hierarchy (the first authored SARX evidence
        # literally turned the whole character sideways/upside-down).
        source_rest_rotation = (
            source_rest_world.to_quaternion()
        )
        target_rest_rotation = (
            target_rest_world.to_quaternion()
        )

        source_to_target_basis = (
            target_rest_rotation.inverted()
            @ source_rest_rotation
        )
        source_to_target_basis.normalize()

        mapped.append(
            (
                source_name,
                target_name,
                source_to_target_basis,
                bone_depth(target_bone),
            )
        )

    mapped.sort(key=lambda item: item[3])

    print(
        "SARX_CMU_RETARGETED_BONES",
        "|".join(sorted(target for _, target, _, _ in mapped)),
    )

    if len(mapped) < 17:
        raise RuntimeError(
            f"CMU retarget mapped only {len(mapped)} major Quaternius bones"
        )

    clip_key = source.stem
    clip_name = CLIP_NAMES.get(
        clip_key,
        f"CMU_{clip_key}",
    )

    target_action = bpy.data.actions.new(clip_name)
    target_armature.animation_data_create()
    target_armature.animation_data.action = target_action

    for pose_bone in target_armature.pose.bones:
        pose_bone.rotation_mode = "QUATERNION"

    scene = bpy.context.scene

    source_start = int(round(source_action.frame_range[0]))
    source_end = int(round(source_action.frame_range[1]))

    if source_end <= source_start:
        raise RuntimeError(
            f"invalid source frame range {source_action.frame_range}"
        )

    print(
        "SARX_CMU_SOURCE_TIMING",
        source_start,
        source_end,
        scene.render.fps,
        scene.render.fps_base,
    )

    source_root_pose = source_armature.pose.bones.get("hip")
    if source_root_pose is None:
        raise RuntimeError(f"{source.name} has no hip/root pose bone")

    source_fps = (
        float(scene.render.fps)
        / float(scene.render.fps_base)
    )

    def rest_world_position(armature, bone_name):
        bone = armature.data.bones.get(bone_name)
        if bone is None:
            raise RuntimeError(
                f"{armature.name} missing rest bone {bone_name}"
            )
        return (
            armature.matrix_world
            @ bone.matrix_local
        ).translation.copy()

    source_head_rest = rest_world_position(
        source_armature,
        "head",
    )
    source_foot_rest = (
        rest_world_position(source_armature, "lFoot")
        + rest_world_position(source_armature, "rFoot")
    ) * 0.5

    target_head_rest = rest_world_position(
        target_armature,
        "Head",
    )
    target_foot_rest = (
        rest_world_position(target_armature, "foot_l")
        + rest_world_position(target_armature, "foot_r")
    ) * 0.5

    source_height_units = (
        source_head_rest
        - source_foot_rest
    ).length

    target_height_m = (
        target_head_rest
        - target_foot_rest
    ).length

    if source_height_units <= 1e-9:
        raise RuntimeError(
            "CMU source skeleton height is degenerate"
        )

    root_unit_scale = (
        target_height_m
        / source_height_units
    )

    root_positions = []

    # Bake the authored source pose into the actual Quaternius target
    # coordinate frames. Target bone translations/lengths remain those
    # of the Quaternius rest skeleton; only rotations are keyed.
    #
    # Deliberately do NOT transfer the CMU hip/root rotation to Quaternius
    # pelvis. CMU root orientation contains capture heading/root motion.
    # SARX owns world facing and continuity, so authored injury clips may
    # animate the spine and limbs but may not silently rotate the agent.
    for frame in range(source_start, source_end + 1):
        scene.frame_set(frame)

        root_positions.append(
            (
                (frame - source_start) / source_fps,
                (
                    source_armature.matrix_world
                    @ source_root_pose.matrix
                ).translation.copy(),
            )
        )

        for source_name, target_name, source_to_target_basis, _ in mapped:
            source_pose = source_armature.pose.bones.get(source_name)
            target_pose = target_armature.pose.bones.get(target_name)

            if (
                source_pose is None
                or target_pose is None
            ):
                continue

            # matrix_basis is the animation delta relative to the source
            # bone's own rest pose and parent.  Transfer that delta into
            # the target bone's rest basis by conjugation, so Quaternius
            # keeps its own hierarchy/rest orientation while inheriting
            # the authored CMU motion.
            source_delta = (
                source_pose.matrix_basis
                .to_quaternion()
            )
            source_delta.normalize()

            target_delta = (
                source_to_target_basis
                @ source_delta
                @ source_to_target_basis.inverted()
            )
            target_delta.normalize()

            target_pose.rotation_mode = "QUATERNION"
            target_pose.rotation_quaternion = target_delta

            # Injury GLBs are rotation-only. Translation and scale stay
            # at the Quaternius target rest values; world/root travel is
            # owned by SARX's continuity layer.
            target_pose.location = (0.0, 0.0, 0.0)
            target_pose.scale = (1.0, 1.0, 1.0)

            target_pose.keyframe_insert(
                data_path="rotation_quaternion",
                frame=frame,
                group=target_name,
            )

    # Preserve original timing. Blender's FBX importer sets scene FPS from
    # the source file, so glTF timestamps remain in source seconds.
    scene.frame_start = source_start
    scene.frame_end = source_end

    # Remove the source skeleton and any imported source meshes so export
    # cannot accidentally include or target the CMU rig.
    bpy.data.objects.remove(source_armature, do_unlink=True)

    for obj in list(bpy.context.scene.objects):
        if obj.type == "MESH" and obj not in target_meshes:
            bpy.data.objects.remove(obj, do_unlink=True)

    # Export only the target armature.  The SARX loader combines these
    # animation nodes with assets/quaternius/character.glb by node name.
    bpy.ops.object.select_all(action="DESELECT")
    target_armature.select_set(True)
    bpy.context.view_layer.objects.active = target_armature

    bpy.ops.export_scene.gltf(
        filepath=str(destination),
        export_format="GLB",
        use_selection=True,
        export_animations=True,
        export_skins=True,
        export_yup=True,
        export_force_sampling=False,
    )

    strip_non_rotation_animation_channels(destination)

    if len(root_positions) < 3:
        raise RuntimeError(
            "CMU authored motion has too few root samples"
        )

    horizontal_steps = []
    for index in range(1, len(root_positions)):
        previous = root_positions[index - 1][1]
        current = root_positions[index][1]
        dx = current.x - previous.x
        dy = current.y - previous.y
        horizontal_steps.append(
            (dx * dx + dy * dy) ** 0.5
        )

    sorted_steps = sorted(horizontal_steps[1:])
    median_step = (
        sorted_steps[len(sorted_steps) // 2]
        if sorted_steps
        else 0.0
    )

    stable_start_index = 0

    first_step_m = (
        horizontal_steps[0]
        * root_unit_scale
        if horizontal_steps
        else 0.0
    )

    if (
        len(horizontal_steps) >= 2
        and (
            first_step_m > 0.05
            or (
                median_step > 1e-9
                and horizontal_steps[0]
                    > median_step * 8.0
            )
        )
    ):
        stable_start_index = 1

    stable_start = root_positions[
        stable_start_index
    ][1]

    end_root = root_positions[-1][1]

    forward_x = end_root.x - stable_start.x
    forward_y = end_root.y - stable_start.y
    forward_length = (
        forward_x * forward_x
        + forward_y * forward_y
    ) ** 0.5

    if forward_length <= 1e-9:
        # Fall back to the dominant early displacement rather than
        # inventing movement if a clip is effectively in-place.
        for index in range(
            stable_start_index + 1,
            len(root_positions),
        ):
            candidate = root_positions[index][1]
            forward_x = candidate.x - stable_start.x
            forward_y = candidate.y - stable_start.y
            forward_length = (
                forward_x * forward_x
                + forward_y * forward_y
            ) ** 0.5
            if forward_length > median_step * 4.0:
                break

    if forward_length > 1e-9:
        forward_x /= forward_length
        forward_y /= forward_length
    else:
        forward_x = 0.0
        forward_y = 0.0

    root_samples = []
    min_vertical = 0.0
    max_vertical = 0.0

    for time_seconds, root_world in root_positions:
        dx = root_world.x - stable_start.x
        dy = root_world.y - stable_start.y

        distance_m = (
            dx * forward_x
            + dy * forward_y
        ) * root_unit_scale

        vertical_m = (
            root_world.z
            - stable_start.z
        ) * root_unit_scale

        # Before the stable motion origin, suppress the FBX bind-to-motion
        # discontinuity instead of turning it into instantaneous travel.
        if time_seconds < root_positions[
            stable_start_index
        ][0]:
            distance_m = 0.0
            vertical_m = 0.0

        min_vertical = min(
            min_vertical,
            vertical_m,
        )
        max_vertical = max(
            max_vertical,
            vertical_m,
        )

        root_samples.append(
            (
                time_seconds,
                distance_m,
                vertical_m,
            )
        )

    root_path = destination.with_suffix(".root.csv")
    with root_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time_seconds",
                "distance_m",
                "vertical_m",
            ]
        )
        writer.writerows(root_samples)

    final_distance_m = root_samples[-1][1]

    print(
        "SARX_CMU_ROOT_TRAJECTORY",
        clip_name,
        f"samples={len(root_samples)}",
        f"stable_start_index={stable_start_index}",
        f"unit_scale={root_unit_scale:.9f}",
        f"distance_m={final_distance_m:.9f}",
        f"vertical_min_m={min_vertical:.9f}",
        f"vertical_max_m={max_vertical:.9f}",
        f"path={root_path}",
    )
    print(
        "SARX_CMU_BAKED_RETARGET",
        clip_name,
        len(mapped),
        source_start,
        source_end,
    )
    print("SARX_CMU_EXPORTED", clip_name, destination)

if __name__ == "__main__":
    main()
