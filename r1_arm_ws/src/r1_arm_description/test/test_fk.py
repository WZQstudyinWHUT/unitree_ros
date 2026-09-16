"""FK 一致性测试：生成的 URDF 必须与官方 upstream 运动学完全一致，
且 tool0 法兰坐标系定义正确（Z 轴 = J7 法兰轴指向外侧）。"""
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_urdf(path):
    if path and os.path.exists(path):
        return path
    # 回退：现场从 xacro 生成
    xacro_file = os.path.join(PKG_DIR, "urdf", "r1_arm.urdf.xacro")
    tmp = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False)
    subprocess.run(["xacro", xacro_file, "-o", tmp.name], check=True)
    return tmp.name


def rpy2mat(r, p, y):
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def parse_chain(path):
    """返回 {joint_name: (parent, child, xyz, rpy, axis, type)}。"""
    joints = {}
    for j in ET.parse(path).getroot().iter("joint"):
        origin = j.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        axis_el = j.find("axis")
        axis = np.fromstring(axis_el.get("xyz"), sep=" ") if axis_el is not None else None
        joints[j.get("name")] = (
            j.find("parent").get("link"),
            j.find("child").get("link"),
            xyz,
            rpy,
            axis,
            j.get("type"),
        )
    return joints


def fk(joints, root, tip, q=None):
    """零位（或给定 q）下 tip 相对 root 的 4x4 变换。"""
    q = q or {}
    child2joint = {v[1]: (k, v) for k, v in joints.items()}
    T = np.eye(4)
    link = tip
    while link != root:
        name, (parent, child, xyz, rpy, axis, jtype) = child2joint[link]
        Tj = np.eye(4)
        Tj[:3, :3] = rpy2mat(*rpy)
        Tj[:3, 3] = xyz
        if jtype == "revolute":
            ang = q.get(name, 0.0)
            ax = axis / np.linalg.norm(axis)
            x, y, z = ax
            c, s = math.cos(ang), math.sin(ang)
            C = 1 - c
            R = np.array([
                [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
            ])
            Rj = np.eye(4)
            Rj[:3, :3] = R
            Tj = Tj @ Rj
        T = Tj @ T
        link = parent
    return T


GEN = load_urdf(os.environ.get("R1_ARM_URDF"))
UPSTREAM = os.environ.get(
    "R1_ARM_UPSTREAM_URDF", os.path.join(PKG_DIR, "upstream", "R1_Robotic_Arm.urdf")
)
gen_joints = parse_chain(GEN)
up_joints = parse_chain(UPSTREAM)


def test_seven_revolute_joints_preserved():
    revolute = [n for n, v in gen_joints.items() if v[5] == "revolute"]
    assert sorted(revolute) == ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]


def test_fk_matches_upstream_at_zero():
    """生成模型的 T(base->Link7) 必须与官方文件逐位一致。"""
    T_gen = fk(gen_joints, "base_link", "Link7")
    T_up = fk(up_joints, "base_link", "Link7")
    np.testing.assert_allclose(T_gen, T_up, atol=1e-9)


def test_fk_matches_upstream_at_random_config():
    rng = np.random.default_rng(42)
    q = {f"J{i}": float(rng.uniform(-0.5, 0.5)) for i in range(1, 8)}
    T_gen = fk(gen_joints, "base_link", "Link7", q)
    T_up = fk(up_joints, "base_link", "Link7", q)
    np.testing.assert_allclose(T_gen, T_up, atol=1e-9)


def test_tool0_exists_and_flange_axis_points_outward():
    assert "tool0_joint" in gen_joints
    assert gen_joints["tool0_joint"][5] == "fixed"
    T = fk(gen_joints, "base_link", "tool0")
    z_tool = T[:3, 2]
    # 零位时法兰轴应近似指向 base +Z（链条残余倾斜 ~1.9° 以内）
    np.testing.assert_allclose(z_tool, [0, 0, 1], atol=math.sin(math.radians(2.0)))


def test_tool0_residual_tilt_is_real_chain_geometry():
    """零位 tool0 相对 base 的残余倾斜应 ≈1.3°（来自 J5/J6 官方 origin），
    且明显小于 J7 翻转（证明翻转已被 frame 定义正确处理）。"""
    T = fk(gen_joints, "base_link", "tool0")
    # 与最近合法姿态（Z 朝上、X 朝前）的角偏差
    angle = math.degrees(math.acos(np.clip(T[2, 2], -1, 1)))
    assert 1.0 < angle < 1.7, f"残余倾斜 {angle}°，预期 ~1.33°"
