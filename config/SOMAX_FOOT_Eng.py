from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import os
import traceback
from pathlib import Path

import numpy as np
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils

app_utils.enable_extension("isaacsim.robot.experimental.manipulators.examples")
app_utils.enable_extension("omni.anim.skelJoint")

from isaacsim.core.experimental.materials import RigidBodyMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.manipulators.examples.franka import Franka


DEVICE = "cpu"

SOMAX_FOLDER_VALUE = os.environ.get("SOMAX_FOLDER")
if not SOMAX_FOLDER_VALUE:
    raise RuntimeError(
        "Set SOMAX_FOLDER first, for example:\n"
        "$env:SOMAX_FOLDER='C:\\Users\\98275\\Desktop\\SOMA\\SOMA-X\\outputs\\demo\\shape'"
    )

SOMAX_FOLDER = Path(SOMAX_FOLDER_VALUE)
SOMAX_ROOT = "/World/SOMAX"
SOMAX_MODEL = "/World/SOMAX/Model"
SOMAX_FEET = np.array([-0.45, 0.65, 0.0], dtype=np.float64)
SOMAX_GROUND_Z = -0.005

TABLE_X = 1.65
TABLE_Y = 1.65
TABLE_H = 0.90
TABLE_T = 0.05
LEG_SIZE = 0.05

FRANKA_BASE = np.array([0.0, 0.0, TABLE_H], dtype=np.float32)

CUBE_SIZE = 0.0515
CUBE_HALF = CUBE_SIZE / 2.0
CUBE_START = np.array(
    [0.40, 0.20, TABLE_H + CUBE_HALF + 0.001],
    dtype=np.float32,
)

GRASP_Z = 0.075
ABOVE_CUBE = 0.20
LIFT_HEIGHT = 0.18
HAND_APPROACH = 0.18

SOFT_FINGER = 0.0245
FINGER_EFFORT = 6.0

MOVE_STEPS = 120
ALIGN_STEPS = 90
DESCEND_STEPS = 120
CLOSE_STEPS = 220
SETTLE_STEPS = 60
LIFT_STEPS = 180
HAND_MOVE_STEPS = 180
HAND_LOWER_STEPS = 140
RELEASE_STEPS = 100


def np_value(value):
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy(), dtype=np.float32).copy()
    return np.asarray(value, dtype=np.float32).copy()


def smoothstep(x):
    x = float(np.clip(x, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def q_normalize(q):
    q = np.asarray(q, dtype=np.float64).reshape(4)
    return q / np.linalg.norm(q)


def q_multiply(a, b):
    w1, x1, y1, z1 = q_normalize(a)
    w2, x2, y2, z2 = q_normalize(b)
    return q_normalize(
        np.array(
            [
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2,
            ],
            dtype=np.float64,
        )
    )


def q_yaw(q):
    w, x, y, z = q_normalize(q)
    return float(
        np.arctan2(
            2.0 * (w*z + x*y),
            1.0 - 2.0 * (y*y + z*z),
        )
    )


def axis_error(target, current):
    e = (target - current + np.pi) % (2.0*np.pi) - np.pi
    if e > np.pi/2:
        e -= np.pi
    elif e < -np.pi/2:
        e += np.pi
    return float(e)


def yaw_orientation(base_orientation, yaw_delta):
    base = np_value(base_orientation)
    shape = base.shape
    base_q = base.reshape(-1, 4)[0]

    half = 0.5 * yaw_delta
    yaw_q = np.array(
        [np.cos(half), 0.0, 0.0, np.sin(half)],
        dtype=np.float64,
    )

    result = q_multiply(yaw_q, base_q).astype(np.float32)
    return result.reshape(1, 4) if len(shape) == 2 else result


def find_somax_usd(folder):
    if folder.is_file():
        return folder

    rest = list(folder.rglob("RestPose.usd"))
    if rest:
        return rest[0]

    for pattern in ("*.usd", "*.usda", "*.usdc"):
        files = list(folder.rglob(pattern))
        if files:
            return files[0]

    raise FileNotFoundError(f"No SOMA-X USD found in {folder}")


def bbox(prim, stage, timeline):
    time_code = Usd.TimeCode(
        timeline.get_current_time() * stage.GetTimeCodesPerSecond()
    )
    cache = UsdGeom.BBoxCache(
        time_code,
        [
            UsdGeom.Tokens.default_,
            UsdGeom.Tokens.render,
            UsdGeom.Tokens.proxy,
        ],
        useExtentsHint=True,
    )
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return (
        np.array(r.GetMin(), dtype=np.float64),
        np.array(r.GetMax(), dtype=np.float64),
    )


def load_somax(stage, timeline):
    usd = find_somax_usd(SOMAX_FOLDER)
    print("[SOMA-X]", usd.resolve().as_posix())

    root_xform = UsdGeom.Xform.Define(stage, SOMAX_ROOT)
    stage_utils.add_reference_to_stage(usd.resolve().as_posix(), SOMAX_MODEL)
    app_utils.update_app(steps=60)

    root = stage.GetPrimAtPath(SOMAX_ROOT)

    bmin, bmax = bbox(root, stage, timeline)
    feet = np.array(
        [
            0.5*(bmin[0] + bmax[0]),
            0.5*(bmin[1] + bmax[1]),
            bmin[2],
        ],
        dtype=np.float64,
    )

    t = SOMAX_FEET - feet
    op = root_xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    op.Set(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])))
    app_utils.update_app(steps=20)

    bmin, _ = bbox(root, stage, timeline)
    t[2] += SOMAX_GROUND_Z - float(bmin[2])
    op.Set(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])))
    app_utils.update_app(steps=20)

    return root


def normalize_name(name):
    return "".join(c.lower() for c in name if c.isalnum())


def find_left_hand(root):
    skeleton = None
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdSkel.Skeleton):
            skeleton = UsdSkel.Skeleton(prim)
            break

    if skeleton is None:
        raise RuntimeError("SOMA-X Skeleton not found")

    query = UsdSkel.Cache().GetSkelQuery(skeleton)
    joints = list(query.GetJointOrder())

    names = ("lefthand", "lhand", "handl", "leftwrist", "lwrist", "wristl")

    for wanted in names:
        for i, token in enumerate(joints):
            token_string = str(token)
            terminal = Sdf.Path(token_string).name or token_string.split("/")[-1]
            if normalize_name(terminal) == wanted:
                print("[SOMA-X] Left hand:", token)
                return skeleton.GetPrim().GetPath(), i

    raise RuntimeError("SOMA-X left hand/wrist joint not found")


def left_hand_position(stage, timeline, skeleton_path, joint_index):
    skeleton = UsdSkel.Skeleton(stage.GetPrimAtPath(skeleton_path))
    query = UsdSkel.Cache().GetSkelQuery(skeleton)

    time_code = Usd.TimeCode(
        timeline.get_current_time() * stage.GetTimeCodesPerSecond()
    )

    transforms = query.ComputeJointWorldTransforms(
        UsdGeom.XformCache(time_code)
    )

    p = transforms[joint_index].ExtractTranslation()
    return np.array([float(p[0]), float(p[1]), float(p[2])], dtype=np.float32)


def create_table():
    top = Cube(
        paths="/World/Table/Top",
        positions=[0.0, 0.0, TABLE_H - TABLE_T/2.0],
        sizes=1.0,
        scales=[TABLE_X, TABLE_Y, TABLE_T],
        colors="saddlebrown",
    )
    GeomPrim(paths=top.paths, apply_collision_apis=True)

    lx = TABLE_X/2.0 - LEG_SIZE
    ly = TABLE_Y/2.0 - LEG_SIZE
    lh = TABLE_H - TABLE_T

    for i, pos in enumerate(
        [
            [ lx,  ly, lh/2.0],
            [ lx, -ly, lh/2.0],
            [-lx,  ly, lh/2.0],
            [-lx, -ly, lh/2.0],
        ]
    ):
        leg = Cube(
            paths=f"/World/Table/Leg_{i}",
            positions=pos,
            sizes=1.0,
            scales=[LEG_SIZE, LEG_SIZE, lh],
            colors="saddlebrown",
        )
        GeomPrim(paths=leg.paths, apply_collision_apis=True)


def run():
    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()

    GroundPlane("/World/Ground")
    create_table()

    light = DomeLight("/World/Light")
    light.set_intensities(1000)

    somax_root = load_somax(stage, timeline)
    skeleton_path, hand_index = find_left_hand(somax_root)

    robot = Franka(robot_path="/World/Franka", create_robot=True)
    XformPrim(paths="/World/Franka").set_world_poses(
        positions=np.array([FRANKA_BASE], dtype=np.float32)
    )

    cube_shape = Cube(
        paths="/World/Cube",
        positions=CUBE_START.tolist(),
        sizes=1.0,
        scales=[CUBE_SIZE, CUBE_SIZE, CUBE_SIZE],
        colors="blue",
    )

    cube_geom = GeomPrim(paths=cube_shape.paths, apply_collision_apis=True)
    cube = RigidPrim(paths=cube_shape.paths)

    mat = RigidBodyMaterial(
        "/World/Materials/CubeGrip",
        static_frictions=[1.5],
        dynamic_frictions=[1.2],
        restitutions=[0.0],
    )
    mat.set_combine_modes(frictions=["max"], restitutions=["min"])
    cube_geom.apply_physics_materials(mat)

    SimulationManager.setup_simulation(dt=1.0/60.0, device=DEVICE)

    scenes = SimulationManager.get_physics_scenes()
    if scenes:
        scenes[0].set_enabled_gpu_dynamics(False)

    app_utils.play()
    app_utils.update_app(steps=30)

    cube.set_masses(0.05)
    robot.reset_to_default_pose()
    robot.open_gripper()
    app_utils.update_app(steps=60)

    downward = robot.get_downward_orientation()

    finger_dofs = [
        i for i, name in enumerate(robot.dof_names)
        if "finger" in name.lower()
    ]
    if len(finger_dofs) != 2:
        raise RuntimeError("Franka finger DOFs not found")

    efforts = np_value(robot.get_dof_max_efforts())
    effort_view = efforts.reshape(-1, efforts.shape[-1])
    effort_view[:, finger_dofs[0]] = FINGER_EFFORT
    effort_view[:, finger_dofs[1]] = FINGER_EFFORT
    robot.set_dof_max_efforts(efforts)

    left_finger = None
    right_finger = None

    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Franka")):
        name = prim.GetName().lower()
        if left_finger is None and ("leftfinger" in name or "left_finger" in name):
            left_finger = prim
        if right_finger is None and ("rightfinger" in name or "right_finger" in name):
            right_finger = prim

    if left_finger is None or right_finger is None:
        raise RuntimeError("Franka finger links not found")

    def ee(position, orientation):
        robot.set_end_effector_pose(
            position=np.array([position], dtype=np.float32),
            orientation=orientation,
        )

    def cube_pose():
        p, q = cube.get_world_poses()
        return (
            np_value(p).reshape(-1, 3)[0],
            np_value(q).reshape(-1, 4)[0],
        )

    def finger_axis():
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        lp = cache.GetLocalToWorldTransform(left_finger).ExtractTranslation()
        rp = cache.GetLocalToWorldTransform(right_finger).ExtractTranslation()

        dx = float(lp[0]) - float(rp[0])
        dy = float(lp[1]) - float(rp[1])
        return float(np.arctan2(dy, dx))

    def set_fingers(value):
        targets = np_value(robot.get_dof_position_targets())
        v = targets.reshape(-1, targets.shape[-1])
        v[:, finger_dofs[0]] = value
        v[:, finger_dofs[1]] = value
        robot.set_dof_position_targets(targets)

    # Lock Cube pose once.
    cube_pos, cube_q = cube_pose()
    cube_yaw = q_yaw(cube_q)

    print("[PICK] Cube:", cube_pos)
    print("[PICK] Yaw:", np.degrees(cube_yaw))

    MOVE_ABOVE = 0
    ALIGN = 1
    DESCEND = 2
    CLOSE = 3
    LIFT = 4
    READ_HAND = 5
    MOVE_HAND = 6
    LOWER_HAND = 7
    RELEASE = 8
    DONE = 9

    state = MOVE_ABOVE
    steps = 0
    grasp_orientation = np_value(downward).copy()

    close_start = np.array([0.04, 0.04], dtype=np.float32)

    grasp_ee = np.array(
        [cube_pos[0], cube_pos[1], cube_pos[2] + GRASP_Z],
        dtype=np.float32,
    )

    lift_ee = np.array(
        [cube_pos[0], cube_pos[1], cube_pos[2] + LIFT_HEIGHT],
        dtype=np.float32,
    )

    hand_above = None
    hand_release = None

    while simulation_app.is_running():
        simulation_app.update()

        if not app_utils.is_playing():
            continue

        steps += 1

        if state == MOVE_ABOVE:
            robot.open_gripper()

            ee(
                [
                    cube_pos[0],
                    cube_pos[1],
                    cube_pos[2] + ABOVE_CUBE,
                ],
                downward,
            )

            if steps >= MOVE_STEPS:
                actual_axis = finger_axis()

                error_x = axis_error(cube_yaw, actual_axis)
                error_y = axis_error(cube_yaw + np.pi/2.0, actual_axis)

                correction = error_x if abs(error_x) <= abs(error_y) else error_y

                grasp_orientation = yaw_orientation(downward, correction)

                print("[PICK] Finger axis:", np.degrees(actual_axis))
                print("[PICK] Yaw correction:", np.degrees(correction))

                state = ALIGN
                steps = 0

        elif state == ALIGN:
            robot.open_gripper()

            ee(
                [
                    cube_pos[0],
                    cube_pos[1],
                    cube_pos[2] + ABOVE_CUBE,
                ],
                grasp_orientation,
            )

            if steps >= ALIGN_STEPS:
                state = DESCEND
                steps = 0

        elif state == DESCEND:
            robot.open_gripper()
            ee(grasp_ee, grasp_orientation)

            if steps >= DESCEND_STEPS:
                p = np_value(robot.get_dof_positions())
                p = p.reshape(-1, p.shape[-1])[0]

                close_start = np.array(
                    [p[finger_dofs[0]], p[finger_dofs[1]]],
                    dtype=np.float32,
                )

                state = CLOSE
                steps = 0

        elif state == CLOSE:
            ee(grasp_ee, grasp_orientation)

            if steps <= CLOSE_STEPS:
                s = smoothstep(steps / CLOSE_STEPS)
                value = close_start + s * (
                    np.array([SOFT_FINGER, SOFT_FINGER], dtype=np.float32)
                    - close_start
                )

                targets = np_value(robot.get_dof_position_targets())
                v = targets.reshape(-1, targets.shape[-1])
                v[:, finger_dofs[0]] = value[0]
                v[:, finger_dofs[1]] = value[1]
                robot.set_dof_position_targets(targets)
            else:
                set_fingers(SOFT_FINGER)

            if steps >= CLOSE_STEPS + SETTLE_STEPS:
                state = LIFT
                steps = 0

        elif state == LIFT:
            set_fingers(SOFT_FINGER)

            s = smoothstep(min(steps / LIFT_STEPS, 1.0))
            position = grasp_ee + s * (lift_ee - grasp_ee)

            ee(position, grasp_orientation)

            if steps >= LIFT_STEPS:
                state = READ_HAND
                steps = 0

        elif state == READ_HAND:
            set_fingers(SOFT_FINGER)

            hand = left_hand_position(
                stage,
                timeline,
                skeleton_path,
                hand_index,
            )

            hand_release = np.array(
                [hand[0], hand[1], hand[2] + GRASP_Z],
                dtype=np.float32,
            )

            hand_above = hand_release.copy()
            hand_above[2] += HAND_APPROACH

            print("[TARGET] Left hand locked:", hand)

            state = MOVE_HAND
            steps = 0

        elif state == MOVE_HAND:
            set_fingers(SOFT_FINGER)
            ee(hand_above, grasp_orientation)

            if steps >= HAND_MOVE_STEPS:
                state = LOWER_HAND
                steps = 0

        elif state == LOWER_HAND:
            set_fingers(SOFT_FINGER)
            ee(hand_release, grasp_orientation)

            if steps >= HAND_LOWER_STEPS:
                state = RELEASE
                steps = 0

        elif state == RELEASE:
            ee(hand_release, grasp_orientation)
            robot.open_gripper()

            if steps >= RELEASE_STEPS:
                print("[DONE] Cube released at SOMA-X left hand.")
                state = DONE
                steps = 0

        elif state == DONE:
            robot.open_gripper()


try:
    run()

except KeyboardInterrupt:
    print("\nInterrupted.")

except Exception:
    print("\n========== ERROR ==========")
    traceback.print_exc()
    print("===========================\n")

    while simulation_app.is_running():
        simulation_app.update()

simulation_app.close()
