from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


import os
from pathlib import Path
import traceback

import numpy as np
import omni.timeline
import omni.usd

from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils

app_utils.enable_extension(
    "isaacsim.robot.experimental.manipulators.examples"
)

app_utils.enable_extension(
    "omni.anim.skelJoint"
)

from isaacsim.core.experimental.objects import (
    Cube,
    DomeLight,
    GroundPlane,
)

from isaacsim.core.experimental.prims import (
    GeomPrim,
    RigidPrim,
    XformPrim,
)

from isaacsim.core.simulation_manager import SimulationManager

from isaacsim.robot.experimental.manipulators.examples.franka import (
    Franka,
)






DEVICE = "cpu"





START_PAUSED_FOR_SOMAX_EDIT = True

SOMAX_FOLDER_VALUE = os.environ.get("SOMAX_FOLDER")

if not SOMAX_FOLDER_VALUE:
    raise RuntimeError(
        "Environment variable SOMAX_FOLDER is not set.\n"
    )

SOMAX_FOLDER = Path(SOMAX_FOLDER_VALUE)

SOMAX_ROOT_PATH = "/World/SOMAX"
SOMAX_MODEL_PATH = "/World/SOMAX/Model"




TABLE_HEIGHT = 0.90
TABLE_SIZE_X = 1.65
TABLE_SIZE_Y = 1.65
TABLE_TOP_THICKNESS = 0.05
TABLE_LEG_SIZE = 0.05

TABLE_TOP_Z = 0.0
FLOOR_Z = -TABLE_HEIGHT

DESIRED_INITIAL_FEET_CENTER = np.array(
    [-0.45, 1.05, FLOOR_Z],
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

TARGET_TOLERANCE = 0.06




MIN_GRASP_LIFT_DELTA = 0.06
MAX_GRASP_XY_DRIFT = 0.10
GRASP_VERIFY_STEPS = 30




MAX_GRASP_RELATIVE_POSITION_ERROR = 0.045
CONTINUOUS_GRASP_CHECK_INTERVAL = 5




HAND_HOLD_EE_Z_OFFSET = 0.10


MIN_REACH_RADIUS = 0.25
MAX_REACH_RADIUS = 0.75






def find_somax_usd(folder: Path) -> Path:
    
    if folder.is_file():
        if folder.suffix.lower() in {".usd", ".usda", ".usdc"}:
            return folder

        raise ValueError(
            f"Path is not a USD file: {folder}"
        )

    if not folder.exists():
        raise FileNotFoundError(
            f"SOMA-X folder does not exist: {folder}"
        )

    candidates = []

    for pattern in ("*.usd", "*.usda", "*.usdc"):
        candidates.extend(folder.rglob(pattern))

    candidates = [
        path for path in candidates
        if path.is_file()
    ]

    if not candidates:
        raise FileNotFoundError(
            "No USD files were found in the following directory:\n"
            f"{folder}"
        )

    def asset_score(path: Path):
        name = path.stem.lower()
        filename = path.name.lower()

        score = 0

        if filename in {
            "soma_body.usd",
            "soma_body.usda",
            "soma_body.usdc",
        }:
            score += 1000

        if "soma" in name:
            score += 200

        if "body" in name:
            score += 150

        if "shape" in name:
            score += 100

        if "human" in name:
            score += 80

        if "mesh" in name:
            score += 30

        if "animation" in name:
            score -= 500

        if "motion" in name:
            score -= 400

        if "anim" in name:
            score -= 300

        return score, -len(path.parts), -len(str(path))

    candidates.sort(
        key=asset_score,
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
            f"Invalid SOMA-X Prim: {SOMAX_ROOT_PATH}"
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
            "The SOMA-X world bounding box is empty."
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


def _normalize_joint_name(
    value: str,
) -> str:
    return "".join(
        character.lower()
        for character in value
        if character.isalnum()
    )


def find_somax_left_hand(
    somax_root_prim,
):
    
    skeleton = None

    for prim in Usd.PrimRange(
        somax_root_prim
    ):
        if prim.IsA(
            UsdSkel.Skeleton
        ):
            skeleton = UsdSkel.Skeleton(
                prim
            )
            break

    if skeleton is None:
        raise RuntimeError(
            "No UsdSkel.Skeleton was found in SOMA-X."
        )

    cache = UsdSkel.Cache()

    query = cache.GetSkelQuery(
        skeleton
    )

    if not query:
        raise RuntimeError(
            "Unable to create SOMA-X SkeletonQuery."
        )

    joint_order = list(
        query.GetJointOrder()
    )

    preferred_names = (
        "lefthand",
        "lhand",
        "handl",
        "leftwrist",
        "lwrist",
        "wristl",
    )

    for preferred_name in preferred_names:
        for index, token in enumerate(
            joint_order
        ):
            token_string = str(
                token
            )

            terminal_name = (
                Sdf.Path(
                    token_string
                ).name
                or token_string.split("/")[-1]
            )

            if (
                _normalize_joint_name(
                    terminal_name
                )
                == preferred_name
            ):
                skeleton_path = (
                    skeleton
                    .GetPrim()
                    .GetPath()
                )

                print(
                    "[SOMA-X] Left hand target joint:",
                    token,
                )

                return (
                    skeleton_path,
                    index,
                )

    print(
        "[SOMA-X] Available joints:"
    )

    for index, token in enumerate(
        joint_order
    ):
        print(
            f"  [{index:02d}] {token}"
        )

    raise RuntimeError(
        "Unable to automatically identify the SOMA-X left-hand/left-wrist joint."
    )


def get_left_hand_world_position(
    stage,
    timeline,
    skeleton_path,
    joint_index: int,
) -> np.ndarray:
    






    skeleton_prim = stage.GetPrimAtPath(
        skeleton_path
    )

    if (
        not skeleton_prim
        or not skeleton_prim.IsValid()
    ):
        raise RuntimeError(
            f"Invalid SOMA-X Skeleton: {skeleton_path}"
        )

    skeleton = UsdSkel.Skeleton(
        skeleton_prim
    )

    cache = UsdSkel.Cache()

    query = cache.GetSkelQuery(
        skeleton
    )

    if not query:
        raise RuntimeError(
            "Unable to read the current SOMA-X Skeleton state."
        )

    current_seconds = (
        timeline.get_current_time()
    )

    time_code = Usd.TimeCode(
        current_seconds
        * stage.GetTimeCodesPerSecond()
    )

    xform_cache = UsdGeom.XformCache(
        time_code
    )

    transforms = (
        query.ComputeJointWorldTransforms(
            xform_cache
        )
    )

    if not transforms:
        raise RuntimeError(
            "Unable to compute SOMA-X joint world coordinates."
        )

    if (
        joint_index < 0
        or joint_index >= len(transforms)
    ):
        raise RuntimeError(
            "The left-hand joint index is outside the current Skeleton range."
        )

    translation = (
        transforms[
            joint_index
        ].ExtractTranslation()
    )

    return np.array(
        [
            float(translation[0]),
            float(translation[1]),
            float(translation[2]),
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

    initial_translation = (
        DESIRED_INITIAL_FEET_CENTER
        - initial_feet_center
    )

    translate_op = somax_root_xform.AddTranslateOp(
        UsdGeom.XformOp.PrecisionDouble
    )

    translate_op.Set(
        Gf.Vec3d(
            float(initial_translation[0]),
            float(initial_translation[1]),
            float(initial_translation[2]),
        )
    )

    app_utils.update_app(steps=30)

    bbox_min, bbox_max = compute_world_bbox(
        somax_root_prim,
        stage,
        timeline,
    )

    feet_center = estimate_feet_center(
        bbox_min,
        bbox_max,
    )

    print(
        "[SOMA-X] Initial feet center:",
        feet_center,
    )

    return (
        somax_root_prim,
        feet_center,
        somax_usd,
    )






def is_position_reachable(
    position: np.ndarray,
) -> bool:
    
    horizontal_distance = np.linalg.norm(
        position[:2]
    )

    return (
        MIN_REACH_RADIUS
        <= horizontal_distance
        <= MAX_REACH_RADIUS
        and 0.0
        <= float(position[2])
        <= 0.40
    )






def create_table():
    
    tabletop = Cube(
        paths="/World/Table/TableTop",
        positions=[
            0.0,
            0.0,
            TABLE_TOP_Z - TABLE_TOP_THICKNESS / 2.0,
        ],
        sizes=1.0,
        scales=[
            TABLE_SIZE_X,
            TABLE_SIZE_Y,
            TABLE_TOP_THICKNESS,
        ],
        colors="saddlebrown",
    )

    GeomPrim(
        paths=tabletop.paths,
        apply_collision_apis=True,
    )

    leg_height = (
        TABLE_HEIGHT - TABLE_TOP_THICKNESS
    )

    leg_x = (
        TABLE_SIZE_X / 2.0 - TABLE_LEG_SIZE
    )

    leg_y = (
        TABLE_SIZE_Y / 2.0 - TABLE_LEG_SIZE
    )

    leg_z = (
        FLOOR_Z + leg_height / 2.0
    )

    for index, position in enumerate(
        (
            (+leg_x, +leg_y, leg_z),
            (+leg_x, -leg_y, leg_z),
            (-leg_x, +leg_y, leg_z),
            (-leg_x, -leg_y, leg_z),
        ),
        start=1,
    ):
        leg = Cube(
            paths=f"/World/Table/Leg_{index}",
            positions=list(position),
            sizes=1.0,
            scales=[
                TABLE_LEG_SIZE,
                TABLE_LEG_SIZE,
                leg_height,
            ],
            colors="saddlebrown",
        )

        GeomPrim(
            paths=leg.paths,
            apply_collision_apis=True,
        )






def run_simulation():
    
    
    

    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()

    if stage is None:
        raise RuntimeError(
            "Unable to obtain the current USD Stage."
        )

    
    
    

    GroundPlane("/World/ground_plane")

    
    
    ground_xform = XformPrim(
        paths="/World/ground_plane"
    )

    ground_xform.set_world_poses(
        positions=np.array(
            [[0.0, 0.0, FLOOR_Z]],
            dtype=np.float32,
        )
    )

    create_table()

    dome_light = DomeLight("/World/DomeLight")
    dome_light.set_intensities(1000)

    
    
    

    (
        somax_root_prim,
        feet_center,
        selected_asset,
    ) = load_and_position_somax(
        stage,
        timeline,
    )

    (
        somax_skeleton_path,
        left_hand_joint_index,
    ) = find_somax_left_hand(
        somax_root_prim
    )

    target_cube_position = (
        get_left_hand_world_position(
            stage,
            timeline,
            somax_skeleton_path,
            left_hand_joint_index,
        )
    )

    print(
        "[TARGET] Initial SOMA-X left-hand destination:",
        target_cube_position,
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

    GeomPrim(
        paths=cube_shape.paths,
        apply_collision_apis=True,
    )

    cube = RigidPrim(
        paths=cube_shape.paths,
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
            "No Physics Scene was found."
        )

    physics_scene = physics_scenes[0]
    physics_scene.set_enabled_gpu_dynamics(False)

    app_utils.play()
    app_utils.update_app(steps=30)

    robot.reset_to_default_pose()
    robot.open_gripper()

    app_utils.update_app(steps=30)

    downward_orientation = (
        robot.get_downward_orientation()
    )

    if START_PAUSED_FOR_SOMAX_EDIT:
        timeline.pause()
        app_utils.update_app(
            steps=5
        )

        print(
            "\n=================================================="
        )
        print(
            "[SOMA-X EDIT MODE] Scene initialized and paused."
        )
        print(
            "You can now do the following in Isaac Sim:"
        )
        print(
            "1. Select an OmniJoint and modify the Rotate XYZ of an individual bone;"
        )
        print(
            "2. Change or bind the Animation Source for the Skeleton;"
        )
        print(
            "3. Switch between Animation / Rest Pose / Default Bind Pose."
        )
        print(
            "After finishing the setup, click Play in Isaac Sim."
        )
        print(
            "Franka will use the current SOMA-X left hand as the Cube target position."
        )
        print(
            "==================================================\n"
        )

    
    
    

    IDLE = 0
    MOVE_ABOVE_CUBE = 1
    LOWER_TO_CUBE = 2
    CLOSE_GRIPPER = 3
    LIFT_CUBE = 4
    VERIFY_GRASP = 5
    MOVE_TO_TARGET = 6
    HOLD_AND_FOLLOW_TARGET = 7

    state = IDLE
    state_steps = 0
    frame_count = 0

    pickup_xy = CUBE_START[:2].copy()

    
    grasp_reference_position = CUBE_START.copy()

    
    grasp_confirmed = False

    
    
    
    grasp_ee_to_cube_offset = np.zeros(
        3,
        dtype=np.float32,
    )

    
    target_tracking_states = {
        MOVE_TO_TARGET,
    }

    last_bbox_error_message = ""

    def change_state(new_state: int):
        nonlocal state, state_steps

        state = new_state
        state_steps = 0

    def command_end_effector(
        position: np.ndarray,
    ):
        robot.set_end_effector_pose(
            position=np.array(
                [position],
                dtype=np.float32,
            ),
            orientation=downward_orientation,
        )

    def get_current_end_effector_position() -> np.ndarray:
        
        _, current_position, _ = (
            robot.get_current_state()
        )

        current_position = np.asarray(
            current_position,
            dtype=np.float32,
        )

        return (
            current_position
            .reshape(-1, 3)[0]
            .copy()
        )

    def cube_is_still_grasped(
        current_cube_position: np.ndarray,
    ) -> tuple[bool, float]:
        







        current_ee_position = (
            get_current_end_effector_position()
        )

        current_offset = (
            np.asarray(
                current_cube_position,
                dtype=np.float32,
            )
            - current_ee_position
        )

        relative_error = float(
            np.linalg.norm(
                current_offset
                - grasp_ee_to_cube_offset
            )
        )

        still_grasped = (
            grasp_confirmed
            and relative_error
            <= MAX_GRASP_RELATIVE_POSITION_ERROR
        )

        return (
            still_grasped,
            relative_error,
        )

    print(
        "\nContinuous SOMA-X LEFT-HAND target tracking started."
    )

    print(
        "Flow: grasp Cube first -> verify grasp success -> "
        "continuously confirm that the Cube remains in the gripper -> "
        "only then allow moving/following the SOMA-X left hand.\n"
    )

    
    
    

    while simulation_app.is_running():
        simulation_app.update()

        if not app_utils.is_playing():
            continue

        frame_count += 1
        state_steps += 1

        
        
        

        if (
            frame_count == 1
            or frame_count
            % TARGET_UPDATE_INTERVAL
            == 0
        ):
            try:
                new_target_position = (
                    get_left_hand_world_position(
                        stage,
                        timeline,
                        somax_skeleton_path,
                        left_hand_joint_index,
                    )
                )

                target_shift = np.linalg.norm(
                    new_target_position
                    - target_cube_position
                )

                if (
                    target_shift
                    > TARGET_CHANGE_THRESHOLD
                ):
                    old_target = (
                        target_cube_position.copy()
                    )

                    target_cube_position = (
                        new_target_position
                    )

                    print(
                        "\n[TARGET UPDATED]"
                    )

                    print(
                        "SOMA-X left hand:",
                        new_target_position,
                    )

                    print(
                        "Old target:",
                        old_target,
                    )

                    print(
                        "New target:",
                        target_cube_position,
                    )

                    print(
                        "Target moved:",
                        f"{target_shift:.3f} m",
                    )

                    
                    
                    if (
                        grasp_confirmed
                        and state in target_tracking_states
                    ):
                        state_steps = 0

                last_bbox_error_message = ""

            except Exception as error:
                
                
                message = str(error)

                if (
                    message
                    != last_bbox_error_message
                ):
                    print(
                        "[TARGET WARNING] "
                        "Unable to update the human position in this frame. "
                        "Keeping the previous target temporarily:",
                        message,
                    )

                    last_bbox_error_message = (
                        message
                    )

        
        
        

        cube_positions, _ = (
            cube.get_world_poses()
        )

        cube_position = (
            cube_positions.numpy()[0]
        )

        target_is_reachable = (
            is_position_reachable(
                target_cube_position
            )
        )

        
        
        
        
        
        

        if state == IDLE:
            grasp_confirmed = False

            if frame_count % 60 == 0:
                print(
                    f"[MONITOR] Cube={cube_position}"
                )

            if not is_position_reachable(
                cube_position
            ):
                if frame_count % 60 == 0:
                    print(
                        "[WAIT] Cube is outside Franka\'s reachable workspace."
                    )

                continue

            pickup_xy = (
                cube_position[:2].copy()
            )

            robot.open_gripper()

            print(
                "\n[ACTION] Starting Cube grasp."
            )

            change_state(
                MOVE_ABOVE_CUBE
            )

        
        
        

        elif state == MOVE_ABOVE_CUBE:
            pickup_xy = (
                cube_position[:2].copy()
            )

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

            robot.open_gripper()

            if state_steps >= 120:
                change_state(
                    LOWER_TO_CUBE
                )

        
        
        

        elif state == LOWER_TO_CUBE:
            pickup_xy = (
                cube_position[:2].copy()
            )

            command_end_effector(
                np.array(
                    [
                        cube_position[0],
                        cube_position[1],
                        cube_position[2] + 0.10,
                    ],
                    dtype=np.float32,
                )
            )

            robot.open_gripper()

            if state_steps >= 100:
                
                grasp_reference_position = (
                    cube_position.copy()
                )

                change_state(
                    CLOSE_GRIPPER
                )

        
        
        

        elif state == CLOSE_GRIPPER:
            
            command_end_effector(
                np.array(
                    [
                        grasp_reference_position[0],
                        grasp_reference_position[1],
                        grasp_reference_position[2] + 0.10,
                    ],
                    dtype=np.float32,
                )
            )

            robot.close_gripper()

            if state_steps >= 60:
                change_state(
                    LIFT_CUBE
                )

        
        
        

        elif state == LIFT_CUBE:
            
            robot.close_gripper()

            command_end_effector(
                np.array(
                    [
                        grasp_reference_position[0],
                        grasp_reference_position[1],
                        0.30,
                    ],
                    dtype=np.float32,
                )
            )

            if state_steps >= 120:
                change_state(
                    VERIFY_GRASP
                )

        
        
        
        
        

        elif state == VERIFY_GRASP:
            robot.close_gripper()

            command_end_effector(
                np.array(
                    [
                        grasp_reference_position[0],
                        grasp_reference_position[1],
                        0.30,
                    ],
                    dtype=np.float32,
                )
            )

            if state_steps < GRASP_VERIFY_STEPS:
                continue

            verified_positions, _ = (
                cube.get_world_poses()
            )

            verified_cube_position = (
                verified_positions.numpy()[0]
            )

            lift_delta = float(
                verified_cube_position[2]
                - grasp_reference_position[2]
            )

            xy_drift = float(
                np.linalg.norm(
                    verified_cube_position[:2]
                    - grasp_reference_position[:2]
                )
            )

            grasp_success = (
                lift_delta
                >= MIN_GRASP_LIFT_DELTA
                and xy_drift
                <= MAX_GRASP_XY_DRIFT
            )

            if grasp_success:
                grasp_confirmed = True

                current_ee_position = (
                    get_current_end_effector_position()
                )

                grasp_ee_to_cube_offset = (
                    np.asarray(
                        verified_cube_position,
                        dtype=np.float32,
                    )
                    - current_ee_position
                )

                print(
                    "\n[GRASP SUCCESS]"
                )

                print(
                    "Cube lift delta:",
                    f"{lift_delta:.3f} m",
                )

                print(
                    "Cube XY drift:",
                    f"{xy_drift:.3f} m",
                )

                print(
                    "EE -> Cube offset:",
                    grasp_ee_to_cube_offset,
                )

                print(
                    "Grasp confirmed. Now starting to follow the SOMA-X left hand."
                )

                
                
                try:
                    target_cube_position = (
                        get_left_hand_world_position(
                            stage,
                            timeline,
                            somax_skeleton_path,
                            left_hand_joint_index,
                        )
                    )

                    print(
                        "[TARGET] Confirmed left-hand target:",
                        target_cube_position,
                    )

                except Exception as error:
                    print(
                        "[TARGET WARNING] Grasp succeeded, "
                        "but the left hand cannot currently be read:",
                        error,
                    )

                change_state(
                    MOVE_TO_TARGET
                )

            else:
                grasp_confirmed = False

                print(
                    "\n[GRASP FAILED]"
                )

                print(
                    "Cube lift delta:",
                    f"{lift_delta:.3f} m",
                )

                print(
                    "Cube XY drift:",
                    f"{xy_drift:.3f} m",
                )

                print(
                    "Cube grasp was not confirmed, so the robot will not move to the human left hand."
                )

                robot.open_gripper()

                change_state(
                    IDLE
                )

        
        
        

        elif state == MOVE_TO_TARGET:
            
            
            
            

            robot.close_gripper()

            if (
                frame_count
                % CONTINUOUS_GRASP_CHECK_INTERVAL
                == 0
            ):
                (
                    still_grasped,
                    relative_error,
                ) = cube_is_still_grasped(
                    cube_position
                )

                if not still_grasped:
                    grasp_confirmed = False

                    print(
                        "\n[GRASP LOST]"
                    )

                    print(
                        "Cube no longer follows the gripper."
                    )

                    print(
                        "Relative EE-Cube error:",
                        f"{relative_error:.3f} m",
                    )

                    print(
                        "Stopping SOMA-X left-hand following and restarting the grasping process."
                    )

                    robot.open_gripper()

                    change_state(
                        IDLE
                    )

                    continue

            if not grasp_confirmed:
                
                
                change_state(
                    IDLE
                )
                continue

            if not target_is_reachable:
                
                command_end_effector(
                    np.array(
                        [
                            grasp_reference_position[0],
                            grasp_reference_position[1],
                            0.30,
                        ],
                        dtype=np.float32,
                    )
                )

                if frame_count % 60 == 0:
                    print(
                        "[HOLD] Cube is still grasped, "
                        "but the current left-hand target is unreachable. Holding position."
                    )

                state_steps = 0
                continue

            command_end_effector(
                np.array(
                    [
                        target_cube_position[0],
                        target_cube_position[1],
                        target_cube_position[2]
                        + HAND_HOLD_EE_Z_OFFSET,
                    ],
                    dtype=np.float32,
                )
            )

            if state_steps >= 160:
                
                
                print(
                    "\n[HOLD] Reached the SOMA-X left-hand position."
                )

                print(
                    "The manipulator will continue following the left hand "
                    "only while the Cube remains in the gripper."
                )

                change_state(
                    HOLD_AND_FOLLOW_TARGET
                )

        
        
        
        
        
        
        

        elif state == HOLD_AND_FOLLOW_TARGET:
            
            
            
            
            
            
            
            
            
            

            robot.close_gripper()

            if (
                frame_count
                % CONTINUOUS_GRASP_CHECK_INTERVAL
                == 0
            ):
                (
                    still_grasped,
                    relative_error,
                ) = cube_is_still_grasped(
                    cube_position
                )

                if not still_grasped:
                    grasp_confirmed = False

                    print(
                        "\n[GRASP LOST DURING FOLLOW]"
                    )

                    print(
                        "Relative EE-Cube error:",
                        f"{relative_error:.3f} m",
                    )

                    print(
                        "Cube has left the gripper."
                    )

                    print(
                        "Stopping human left-hand following immediately."
                    )

                    robot.open_gripper()

                    change_state(
                        IDLE
                    )

                    continue

            if not grasp_confirmed:
                change_state(
                    IDLE
                )
                continue

            if not target_is_reachable:
                
                
                if frame_count % 60 == 0:
                    print(
                        "[HOLD] Cube is still grasped, "
                        "but the left hand is temporarily outside the workspace."
                    )

                continue

            
            
            
            
            
            
            command_end_effector(
                np.array(
                    [
                        target_cube_position[0],
                        target_cube_position[1],
                        target_cube_position[2]
                        + HAND_HOLD_EE_Z_OFFSET,
                    ],
                    dtype=np.float32,
                )
            )






try:
    run_simulation()

except KeyboardInterrupt:
    print(
        "\nSimulation interrupted by user."
    )

except Exception:
    print(
        "\n============================================"
    )

    print(
        "An exception occurred. The Isaac Sim window will remain open."
    )

    traceback.print_exc()

    print(
        "============================================\n"
    )

    
    
    while simulation_app.is_running():
        simulation_app.update()


simulation_app.close()