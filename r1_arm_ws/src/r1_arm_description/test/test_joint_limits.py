"""关节限值测试：joint_limits.yaml 必须用保守关节级速度覆盖 URDF 中
疑似电机侧的 velocity 值，且官方 effort 限值不得改变。"""
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
OFFICIAL_EFFORT = {"J1": 60.0, "J2": 60.0, "J3": 33.0, "J4": 33.0,
                   "J5": 33.0, "J6": 10.0, "J7": 10.0}
# URDF 官方值（疑似电机侧），安全限值必须显著低于它们
URDF_VELOCITY = {"J1": 18.7, "J2": 18.7, "J3": 33.4, "J4": 33.4,
                 "J5": 33.4, "J6": 37.7, "J7": 37.7}
MAX_SAFE_VELOCITY = 3.15  # rad/s，官方整机规格 ~180°/s


def load_limits():
    path = os.environ.get(
        "R1_ARM_JOINT_LIMITS", os.path.join(PKG_DIR, "config", "joint_limits.yaml")
    )
    with open(path) as f:
        return yaml.safe_load(f)["joint_limits"]


def load_urdf_joints():
    path = os.environ.get("R1_ARM_URDF")
    if not path or not os.path.exists(path):
        xacro_file = os.path.join(PKG_DIR, "urdf", "r1_arm.urdf.xacro")
        tmp = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False)
        subprocess.run(["xacro", xacro_file, "-o", tmp.name], check=True)
        path = tmp.name
    return {
        j.get("name"): j.find("limit")
        for j in ET.parse(path).getroot().iter("joint")
        if j.get("type") == "revolute"
    }


def test_all_joints_covered():
    limits = load_limits()
    for j in JOINTS:
        assert j in limits, f"{j} 缺少安全限值"


def test_velocity_limits_conservative():
    limits = load_limits()
    for j in JOINTS:
        v = limits[j]["max_velocity"]
        assert limits[j]["has_velocity_limits"] is True
        assert 0 < v <= MAX_SAFE_VELOCITY, (
            f"{j} max_velocity={v} 超过保守关节级上限 {MAX_SAFE_VELOCITY} rad/s"
        )
        assert v < URDF_VELOCITY[j], f"{j} 未覆盖 URDF 电机侧速度值"


def test_urdf_effort_limits_preserved():
    limits = load_limits()
    urdf_joints = load_urdf_joints()
    for j in JOINTS:
        assert float(urdf_joints[j].get("effort")) == OFFICIAL_EFFORT[j]
        assert limits[j]["max_effort"] == OFFICIAL_EFFORT[j]


def test_urdf_position_limits_preserved():
    """官方位置限值是权威几何，不得修改。"""
    official = {
        "J1": (-2.723, 2.723), "J2": (0.0, 3.1416), "J3": (-0.349, 3.491),
        "J4": (-3.1415, 0.0), "J5": (-2.269, 2.269),
        "J6": (-0.524, 0.524), "J7": (-1.571, 1.571),
    }
    urdf_joints = load_urdf_joints()
    for j, (lo, hi) in official.items():
        assert abs(float(urdf_joints[j].get("lower")) - lo) < 1e-6
        assert abs(float(urdf_joints[j].get("upper")) - hi) < 1e-6
