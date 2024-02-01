from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robo_arm", package_name="robo_arm").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
