import numpy as np

from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion


class CobotMagic(ManipulatorModel):
    """
    Cobot Magic is a Mobile Aloha spec dual-armed robot designed by AgileX.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["left", "right"]

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/cobot_magic/robot.xml"), idn=idn)

        # Set joint damping
        self.set_joint_attribute(attrib="damping", values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01,    0.1, 0.1, 0.1, 0.1, 0.1, 0.01)))

    @property
    def default_base(self):
        return "TracerMobileBase"

    @property
    def default_gripper(self):
        return {"right": "CobotMagicGripper", "left": "CobotMagicGripper"}

    @property
    def default_controller_config(self):
        return {"right": "default_cobot_magic", "left": "default_cobot_magic"}

    @property
    def init_qpos(self):
        # 6 joints x 2, all zeroes for rest positions
        return np.array([0.0, 0.3, 0.3, -0.6, 0.0, 0.0,   0.0, 0.3, 0.3, -0.6, 0.0, 0.0])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.29, 0, 0),
            "table": lambda table_length: (-0.26 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 0.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "bimanual"
    
    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        return {"right": "fr_link6", "left": "fl_link6"}
