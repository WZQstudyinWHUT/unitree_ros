"""碰撞网格回归测试：base_link 不得被占位立方体替换。"""
import struct
from pathlib import Path

import numpy as np


PKG_DIR = Path(__file__).resolve().parent.parent


def load_binary_stl_vertices(path):
    """读取二进制 STL 的全部三角形顶点；本包网格均为 binary STL。"""
    data = path.read_bytes()
    face_count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + 50 * face_count, f"{path} 不是有效的二进制 STL"
    vertices = []
    for index in range(face_count):
        offset = 84 + 50 * index + 12  # 跳过法线，读取 3 个顶点
        vertices.extend(struct.unpack_from("<9f", data, offset))
    return np.asarray(vertices, dtype=float).reshape(-1, 3)


def test_base_collision_matches_original_visual_envelope():
    """底座 collision 应贴合原始 visual 尺寸，不能是 2 m 占位立方体。"""
    visual = load_binary_stl_vertices(PKG_DIR / "meshes" / "visual" / "base_link.STL")
    collision = load_binary_stl_vertices(
        PKG_DIR / "meshes" / "collision" / "base_link.STL"
    )
    visual_extent = visual.max(axis=0) - visual.min(axis=0)
    collision_extent = collision.max(axis=0) - collision.min(axis=0)
    np.testing.assert_allclose(collision_extent, visual_extent, atol=1e-6)
    assert np.all(collision_extent < 0.2), (
        f"base_link collision 尺寸异常：{collision_extent.tolist()} m"
    )
