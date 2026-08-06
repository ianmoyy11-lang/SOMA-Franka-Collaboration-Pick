from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})


import os
from pathlib import Path
import traceback

import numpy as np
import omni.timeline
import omni.usd

from pxr import Gf, Usd, UsdGeom

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils

app_utils.enable_extension(
    "isaacsim.robot.experimental.manipulators.examples"
)

from isaacsim.core.experimental.materials import RigidBodyMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.manipulators.examples.franka import Franka



DEVICE = "cpu"

SOMAX_FOLDER_VALUE = os.environ.get("SOMAX_FOLDER")
if not SOMAX_FOLDER_VALUE:
    raise RuntimeError(
        "Environment variable SOMAX_FOLDER is not set.\n"
        "Set it before running the script, for example in PowerShell:\n"
        "  $env:SOMAX_FOLDER = 'C:\\Your Projects\\SOMA-X'\n"
        "Then run: C:\\isaacsim\\python.bat .\\SOMAX_FOOT_Eng.py"
    )

SOMAX_FOLDER = Path(SOMAX_FOLDER_VALUE)

SOMAX_ROOT_PATH = "/World/SOMAX"
SOMAX_MODEL_PATH = "/World/SOMAX/Model"


DESIRED_INITIAL_FEET_CENTER = np.array(
    [-0.45, 0.65, 0.0],
    dtype=np.float64,
)


FOOT_SIDE_DISTANCE = 0.25


TARGET_UPDATE_INTERVAL = 5


TARGET_CHANGE_THRESHOLD = 0.005


CUBE_SIZE = 0.0515
CUBE_HALF_HEIGHT = CUBE_SIZE / 2.0

CUBE_START = np.array(
    [0.40, 0.20, CUBE_HALF_HEIGHT],
    dtype=np.float32,
)


CUBE_MASS = 0.05


CUBE_STATIC_FRICTION = 1.5
CUBE_DYNAMIC_FRICTION = 1.2

TARGET_TOLERANCE = 0.06


MIN_REACH_RADIUS = 0.25
MAX_REACH_RADIUS = 0.75


GRASP_Z_OFFSET = 0.075


GRIP_CLOSE_STEPS = 140

TEST_LIFT_HEIGHT = 0.18

MIN_GRASP_LIFT_DELTA = 0.055

MAX_GRASP_XY_ERROR = 0.10

HOME_RETURN_STEPS = 180



def to_numpy(value) -> np.ndarray:

    if hasattr(value, "numpy"):
        return np.asarray(
            value.numpy(),
            dtype=np.float32,
        ).copy()

    return np.asarray(
        value,
        dtype=np.float32,
    ).copy()


def smoothstep(value: float) -> float:

    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def is_position_reachable(position: np.ndarray) -> bool:

    horizontal_distance = float(
        np.linalg.norm(position[:2])
    )

    return (
        MIN_REACH_RADIUS
        <= horizontal_distance
        <= MAX_REACH_RADIUS
        and 0.0
        <= float(position[2])
        <= 0.45
    )



def find_somax_usd(folder: Path) -> Path:

    if folder.is_file():
        if folder.suffix.lower() in {".usd", ".usda", ".usdc"}:
            return folder

        raise ValueError(
            f"does not point to a USD file: {folder}"
        )

    if not folder.exists():
        raise FileNotFoundError(
            f"Folder not found: {folder}"
        )

    candidates = []

    for pattern in ("*.usd", "*.usda", "*.usdc"):
        candidates.extend(folder.rglob(pattern))

    candidates = [
        path
        for path in candidates
        if path.is_file()
    ]

    if not candidates:
        raise FileNotFoundError(
            "No USD files found in the specified folder:\n"
            f"{folder}"
        )

    def score(path: Path):
        name = path.stem.lower()
        filename = path.name.lower()

        value = 0

        if filename in {
            "soma_body.usd",
            "soma_body.usda",
            "soma_body.usdc",
        }:
            value += 1000

        if "soma" in name:
            value += 200

        if "body" in name:
            value += 150

        if "shape" in name:
            value += 100

        if "human" in name:
            value += 80

        if "mesh" in name:
            value += 30

        if "animation" in name:
            value -= 500

        if "motion" in name:
            value -= 400

        if "anim" in name:
            value -= 300

        return value, -len(path.parts), -len(str(path))

    candidates.sort(
        key=score,
        reverse=True,
    )

    return candidates[0]


def compute_world_bbox(
    prim,
    stage,
    timeline,
) -> tuple[np.ndarray, np.ndarray]:

    if not prim or not prim.IsValid():
        raise RuntimeError(
            f"SOMA-X Prim {SOMAX_ROOT_PATH} is invalid."
        )

    current_seconds = timeline.get_current_time()
    time_codes_per_second = stage.GetTimeCodesPerSecond()

    time_code = Usd.TimeCode(
        current_seconds * time_codes_per_second
    )

    bbox_cache = UsdGeom.BBoxCache(
        time_code,
        [
            UsdGeom.Tokens.default_,
            UsdGeom.Tokens.render,
            UsdGeom.Tokens.proxy,
        ],
        useExtentsHint=True,
    )

    world_bound = bbox_cache.ComputeWorldBound(prim)
    aligned_range = world_bound.ComputeAlignedRange()

    if aligned_range.IsEmpty():
        raise RuntimeError(
            f"SOMA-X {SOMAX_ROOT_PATH} has an empty world bounding box."
        )

    bbox_min = np.array(
        aligned_range.GetMin(),
        dtype=np.float64,
    )

    bbox_max = np.array(
        aligned_range.GetMax(),
        dtype=np.float64,
    )

    return bbox_min, bbox_max


def estimate_feet_center(
    bbox_min: np.ndarray,
    bbox_max: np.ndarray,
) -> np.ndarray:

    return np.array(
        [
            0.5 * (bbox_min[0] + bbox_max[0]),
            0.5 * (bbox_min[1] + bbox_max[1]),
            bbox_min[2],
        ],
        dtype=np.float64,
    )


def calculate_foot_side_target(
    feet_center: np.ndarray,
) -> np.ndarray:

    feet_xy = feet_center[:2].astype(np.float64)

    direction_to_franka = -feet_xy
    direction_length = float(
        np.linalg.norm(direction_to_franka)
    )

    if direction_length < 1.0e-8:
        direction_to_franka = np.array(
            [0.0, -1.0],
            dtype=np.float64,
        )
    else:
        direction_to_franka /= direction_length

    target_xy = (
        feet_xy
        + direction_to_franka * FOOT_SIDE_DISTANCE
    )

    target_z = max(
        CUBE_HALF_HEIGHT,
        float(feet_center[2]) + CUBE_HALF_HEIGHT,
    )

    return np.array(
        [
            target_xy[0],
            target_xy[1],
            target_z,
        ],
        dtype=np.float32,
    )


def load_and_position_somax(
    stage,
    timeline,
):

    somax_usd = find_somax_usd(
        SOMAX_FOLDER
    )

    somax_usd_path = (
        somax_usd.resolve().as_posix()
    )

    print("\n[SOMA-X] Selected asset:")
    print(somax_usd_path)

    somax_root_xform = UsdGeom.Xform.Define(
        stage,
        SOMAX_ROOT_PATH,
    )

    stage_utils.add_reference_to_stage(
        somax_usd_path,
        SOMAX_MODEL_PATH,
    )

    app_utils.update_app(steps=60)

    somax_root_prim = stage.GetPrimAtPath(
        SOMAX_ROOT_PATH
    )

    bbox_min, bbox_max = compute_world_bbox(
        somax_root_prim,
        stage,
        timeline,
    )

    initial_feet_center = estimate_feet_center(
        bbox_min,
        bbox_max,
    )

    translation = (
        DESIRED_INITIAL_FEET_CENTER
        - initial_feet_center
    )

    translate_op = somax_root_xform.AddTranslateOp(
        UsdGeom.XformOp.PrecisionDouble
    )

    translate_op.Set(
        Gf.Vec3d(
            float(translation[0]),
            float(translation[1]),
            float(translation[2]),
        )
    )

    app_utils.update_app(steps=30)

    bbox_min, bbox_max = compute_world_bbox(
        somax_root_prim,
        stage,
        timeline,
    )

    final_feet_center = estimate_feet_center(
        bbox_min,
        bbox_max,
    )

    print(
        "[SOMA-X] Initial feet center:",
        final_feet_center,
    )

    return somax_root_prim, final_feet_center


def run_simulation():
    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()

    if stage is None:
        raise RuntimeError(
            "无法获得当前 USD Stage。"
        )


    GroundPlane("/World/ground_plane")

    dome_light = DomeLight("/World/DomeLight")
    dome_light.set_intensities(1000)


    somax_root_prim, feet_center = (
        load_and_position_somax(
            stage,
            timeline,
        )
    )

    latest_target = calculate_foot_side_target(
        feet_center
    )

    active_target = latest_target.copy()

    release_target = latest_target.copy()

    print(
        "[TARGET] Initial target:",
        latest_target,
    )

    robot = Franka(
        robot_path="/World/Franka",
        create_robot=True,
    )


    cube_shape = Cube(
        paths="/World/Cube",
        positions=CUBE_START.tolist(),
        sizes=1.0,
        scales=[
            CUBE_SIZE,
            CUBE_SIZE,
            CUBE_SIZE,
        ],
        colors="blue",
    )

    cube_geom = GeomPrim(
        paths=cube_shape.paths,
        apply_collision_apis=True,
    )

    cube = RigidPrim(
        paths=cube_shape.paths,
    )

    cube_grip_material = RigidBodyMaterial(
        "/World/Materials/CubeGripMaterial",
        static_frictions=[CUBE_STATIC_FRICTION],
        dynamic_frictions=[CUBE_DYNAMIC_FRICTION],
        restitutions=[0.0],
    )

    cube_grip_material.set_combine_modes(
        frictions=["max"],
        restitutions=["min"],
    )

    cube_geom.apply_physics_materials(
        cube_grip_material
    )



    SimulationManager.setup_simulation(
        dt=1.0 / 60.0,
        device=DEVICE,
    )

    physics_scenes = (
        SimulationManager.get_physics_scenes()
    )

    if not physics_scenes:
        raise RuntimeError(
            "No physics scenes found in the simulation.")

    physics_scene = physics_scenes[0]
    physics_scene.set_enabled_gpu_dynamics(False)

    app_utils.play()
    app_utils.update_app(steps=30)

    cube.set_masses(CUBE_MASS)

    robot.reset_to_default_pose()
    robot.open_gripper()

    app_utils.update_app(steps=60)

    downward_orientation = (
        robot.get_downward_orientation()
    )

    home_state = robot.get_current_state()
    home_dof_positions = to_numpy(
        home_state[0]
    )


    IDLE = 0

    MOVE_ABOVE_CUBE = 1
    LOWER_TO_CUBE = 2
    CLOSE_GRIPPER = 3

    TEST_LIFT = 4
    VERIFY_GRASP = 5
    FAILED_GRASP_RETRACT = 6

    MOVE_ABOVE_TARGET = 7
    LOWER_TO_TARGET = 8
    OPEN_GRIPPER = 9
    RETRACT = 10
    VERIFY_PLACE = 11

    RETURN_HOME = 12

    state = IDLE
    state_steps = 0
    frame_count = 0

    carrying_cube = False

    grasp_reference_position = CUBE_START.copy()
    pickup_ground_z = float(CUBE_START[2])
    test_lift_position = CUBE_START.copy()

    grasp_retry_count = 0

    return_start_dof_positions = (
        home_dof_positions.copy()
    )

    last_bbox_error_message = ""



    def change_state(new_state: int):
        nonlocal state, state_steps

        state = new_state
        state_steps = 0

    def command_end_effector(position: np.ndarray):
        robot.set_end_effector_pose(
            position=np.array(
                [position],
                dtype=np.float32,
            ),
            orientation=downward_orientation,
        )

    def get_cube_position() -> np.ndarray:
        positions, _ = cube.get_world_poses()

        return np.asarray(
            positions.numpy()[0],
            dtype=np.float32,
        )

    def get_current_end_effector_position() -> np.ndarray:
        current_state = robot.get_current_state()

        current_position = to_numpy(
            current_state[1]
        )

        return current_position.reshape(-1, 3)[0]

    def cube_is_at_target(
        cube_position: np.ndarray,
        target_position: np.ndarray,
    ) -> bool:
        return (
            np.linalg.norm(
                cube_position - target_position
            )
            <= TARGET_TOLERANCE
        )

    def begin_smooth_home_return():
        nonlocal return_start_dof_positions

        current_state = robot.get_current_state()

        return_start_dof_positions = to_numpy(
            current_state[0]
        )

        change_state(RETURN_HOME)


    while simulation_app.is_running():
        simulation_app.update()

        if not app_utils.is_playing():
            continue

        frame_count += 1
        state_steps += 1

        if (
            frame_count
            % TARGET_UPDATE_INTERVAL
            == 0
        ):
            try:
                bbox_min, bbox_max = (
                    compute_world_bbox(
                        somax_root_prim,
                        stage,
                        timeline,
                    )
                )

                new_feet_center = (
                    estimate_feet_center(
                        bbox_min,
                        bbox_max,
                    )
                )

                new_latest_target = (
                    calculate_foot_side_target(
                        new_feet_center
                    )
                )

                target_shift = float(
                    np.linalg.norm(
                        new_latest_target
                        - latest_target
                    )
                )

                feet_center = new_feet_center

                if (
                    target_shift
                    > TARGET_CHANGE_THRESHOLD
                ):
                    old_latest_target = (
                        latest_target.copy()
                    )

                    latest_target = (
                        new_latest_target
                    )

                    print(
                        "\n[HUMAN TARGET OBSERVED]"
                    )

                    print(
                        "Old:",
                        old_latest_target,
                    )

                    print(
                        "New:",
                        latest_target,
                    )

                    print(
                        "Shift:",
                        f"{target_shift:.3f} m",
                    )

                    if not carrying_cube:
                        print(
                            "Target saved only. "
                            "Cube has not been grasped yet."
                        )

                    elif state in {
                        MOVE_ABOVE_TARGET,
                        LOWER_TO_TARGET,
                    }:
                        active_target = (
                            latest_target.copy()
                        )

                        state_steps = 0

                        print(
                            "Active transport target updated."
                        )

                last_bbox_error_message = ""

            except Exception as error:
                message = str(error)

                if (
                    message
                    != last_bbox_error_message
                ):
                    print(
                        message,
                    )

                    last_bbox_error_message = (
                        message
                    )

        cube_position = get_cube_position()


        if state == IDLE:
            robot.open_gripper()

            if frame_count % 60 == 0:
                error = float(
                    np.linalg.norm(
                        cube_position
                        - latest_target
                    )
                )

                print(
                    f"[MONITOR] "
                    f"Cube={cube_position}, "
                    f"LatestTarget={latest_target}, "
                    f"Error={error:.3f} m"
                )

            if cube_is_at_target(
                cube_position,
                latest_target,
            ):
                continue

            if not is_position_reachable(
                cube_position
            ):
                if frame_count % 60 == 0:
                    print(
                        "[WAIT] Cube is outside the reachable workspace. "
                    )

                continue

            print(
                "\n[ACTION] Cube is not at the latest human target."
            )

            print(
                "Priority 1: grasp Cube first."
            )

            carrying_cube = False
            grasp_retry_count = 0

            robot.open_gripper()

            change_state(MOVE_ABOVE_CUBE)


        elif state == MOVE_ABOVE_CUBE:
            robot.open_gripper()

            if not is_position_reachable(
                cube_position
            ):
                print(
                    "[ABORT] Cube is outside the reachable workspace."
                )

                begin_smooth_home_return()
                continue

            command_end_effector(
                np.array(
                    [
                        cube_position[0],
                        cube_position[1],
                        cube_position[2] + 0.20,
                    ],
                    dtype=np.float32,
                )
            )

            if state_steps >= 120:
                change_state(LOWER_TO_CUBE)


        elif state == LOWER_TO_CUBE:
            robot.open_gripper()

            if not is_position_reachable(
                cube_position
            ):
                print(
                    "[ABORT] Cube is outside the reachable workspace."
                )

                begin_smooth_home_return()
                continue

            command_end_effector(
                np.array(
                    [
                        cube_position[0],
                        cube_position[1],
                        cube_position[2] + GRASP_Z_OFFSET,
                    ],
                    dtype=np.float32,
                )
            )

            if state_steps >= 110:
                grasp_reference_position = (
                    cube_position.copy()
                )

                pickup_ground_z = float(
                    cube_position[2]
                )

                change_state(CLOSE_GRIPPER)


        elif state == CLOSE_GRIPPER:
            grasp_pose = np.array(
                [
                    grasp_reference_position[0],
                    grasp_reference_position[1],
                    grasp_reference_position[2]
                    + GRASP_Z_OFFSET,
                ],
                dtype=np.float32,
            )

            command_end_effector(grasp_pose)

            robot.close_gripper()

            if state_steps >= GRIP_CLOSE_STEPS:
                lift_z = max(
                    0.22,
                    pickup_ground_z
                    + TEST_LIFT_HEIGHT,
                )

                test_lift_position = np.array(
                    [
                        grasp_reference_position[0],
                        grasp_reference_position[1],
                        lift_z,
                    ],
                    dtype=np.float32,
                )

                change_state(TEST_LIFT)


        elif state == TEST_LIFT:
            robot.close_gripper()

            command_end_effector(
                test_lift_position
            )

            if state_steps >= 150:
                change_state(VERIFY_GRASP)


        elif state == VERIFY_GRASP:
            robot.close_gripper()

            command_end_effector(
                test_lift_position
            )

            if state_steps < 30:
                continue

            cube_position = get_cube_position()

            lift_delta = float(
                cube_position[2]
                - pickup_ground_z
            )

            xy_error = float(
                np.linalg.norm(
                    cube_position[:2]
                    - grasp_reference_position[:2]
                )
            )

            grasp_success = (
                lift_delta
                >= MIN_GRASP_LIFT_DELTA
                and xy_error
                <= MAX_GRASP_XY_ERROR
            )

            if grasp_success:
                carrying_cube = True

                active_target = (
                    latest_target.copy()
                )

                print(
                    "\n[GRASP VERIFIED]"
                )

                print(
                    "Cube lift delta:",
                    f"{lift_delta:.3f} m",
                )

                print(
                    "Active human target committed:",
                    active_target,
                )

                change_state(
                    MOVE_ABOVE_TARGET
                )

            else:
                carrying_cube = False
                grasp_retry_count += 1

                print(
                    "\n[GRASP FAILED]"
                )

                print(
                    "Lift delta:",
                    f"{lift_delta:.3f} m",
                )

                print(
                    "XY error:",
                    f"{xy_error:.3f} m",
                )

                print(
                    "Retry number:",
                    grasp_retry_count,
                )

                robot.open_gripper()

                change_state(
                    FAILED_GRASP_RETRACT
                )


        elif state == FAILED_GRASP_RETRACT:
            robot.open_gripper()

            if is_position_reachable(
                cube_position
            ):
                command_end_effector(
                    np.array(
                        [
                            cube_position[0],
                            cube_position[1],
                            cube_position[2] + 0.20,
                        ],
                        dtype=np.float32,
                    )
                )

            if state_steps >= 100:
                if is_position_reachable(
                    cube_position
                ):
                    change_state(
                        MOVE_ABOVE_CUBE
                    )
                else:
                    begin_smooth_home_return()


        elif state == MOVE_ABOVE_TARGET:
            robot.close_gripper()

            if (
                cube_position[2]
                < pickup_ground_z + 0.035
            ):
                print(
                    "\n[DROP DETECTED] "
                    "Cube dropped during transport."
                )

                carrying_cube = False
                robot.open_gripper()

                change_state(
                    FAILED_GRASP_RETRACT
                )

                continue

            if not is_position_reachable(
                active_target
            ):
                hold_position = (
                    get_current_end_effector_position()
                )

                command_end_effector(
                    hold_position
                )

                state_steps = 0

                if frame_count % 60 == 0:
                    print(
                        "[HOLD] Cube is grasped, "
                        "but the human target is temporarily unreachable."
                    )

                continue

            transport_height = max(
                0.30,
                float(active_target[2]) + 0.25,
            )

            command_end_effector(
                np.array(
                    [
                        active_target[0],
                        active_target[1],
                        transport_height,
                    ],
                    dtype=np.float32,
                )
            )

            if state_steps >= 170:
                change_state(
                    LOWER_TO_TARGET
                )


        elif state == LOWER_TO_TARGET:
            robot.close_gripper()

            if not is_position_reachable(
                active_target
            ):
                print(
                    "[REPLAN] Human target is outside the reachable workspace,"
                    "robot will hold current position."
                )

                change_state(
                    MOVE_ABOVE_TARGET
                )

                continue

            command_end_effector(
                np.array(
                    [
                        active_target[0],
                        active_target[1],
                        active_target[2] + 0.095,
                    ],
                    dtype=np.float32,
                )
            )

            if state_steps >= 130:
                release_target = (
                    active_target.copy()
                )

                change_state(
                    OPEN_GRIPPER
                )


        elif state == OPEN_GRIPPER:
            command_end_effector(
                np.array(
                    [
                        release_target[0],
                        release_target[1],
                        release_target[2] + 0.095,
                    ],
                    dtype=np.float32,
                )
            )

            robot.open_gripper()
            carrying_cube = False

            if state_steps >= 100:
                change_state(RETRACT)


        elif state == RETRACT:
            robot.open_gripper()

            command_end_effector(
                np.array(
                    [
                        release_target[0],
                        release_target[1],
                        max(
                            0.30,
                            float(release_target[2])
                            + 0.25,
                        ),
                    ],
                    dtype=np.float32,
                )
            )

            if state_steps >= 130:
                change_state(
                    VERIFY_PLACE
                )


        elif state == VERIFY_PLACE:
            robot.open_gripper()

            if state_steps < 60:
                continue

            verified_cube_position = (
                get_cube_position()
            )

            placement_success = (
                cube_is_at_target(
                    verified_cube_position,
                    release_target,
                )
            )

            if placement_success:
                print(
                    "\n[SUCCESS] "
                    "Cube reached the committed foot-side target."
                )

                print(
                    "Starting smooth home return."
                )

            else:
                print(
                    "\n[PLACE WARNING] "
                    "Cube did not settle inside the committed target."
                )

                print(
                    "Robot will return smoothly, "
                    "then monitor and retry."
                )

            begin_smooth_home_return()


        elif state == RETURN_HOME:
            robot.open_gripper()

            progress = min(
                float(state_steps)
                / float(HOME_RETURN_STEPS),
                1.0,
            )

            interpolation = smoothstep(
                progress
            )

            interpolated_dof_positions = (
                return_start_dof_positions
                + interpolation
                * (
                    home_dof_positions
                    - return_start_dof_positions
                )
            )

            robot.set_dof_positions(
                interpolated_dof_positions
            )

            if progress >= 1.0:
                robot.set_dof_positions(
                    home_dof_positions
                )

                carrying_cube = False

                print(
                    "[HOME] Franka returned smoothly."
                )

                change_state(IDLE)



try:
    run_simulation()

except KeyboardInterrupt:
    print(
        "\nSimulation interrupted by user."
    )

except Exception:
    print(
        "\n=================================================="
    )

    print(
        "[ERROR] An unexpected exception occurred:"
    )

    traceback.print_exc()

    print(
        "==================================================\n"
    )

    while simulation_app.is_running():
        simulation_app.update()

simulation_app.close()