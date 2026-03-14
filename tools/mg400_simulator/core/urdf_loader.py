#!/usr/bin/env python3
"""
Hardcoded URDF FK for MG400 (from mg400.xacro).
All distances in mm.  Mesh files loaded via trimesh (m → mm ×1000).
"""

import os
import numpy as np

# ── 4×4 transform helpers ─────────────────────────────────────────────────────

def _T(xyz):
    M = np.eye(4)
    M[:3, 3] = xyz
    return M

def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4)
    M[0, 0] =  c; M[0, 1] = -s
    M[1, 0] =  s; M[1, 1] =  c
    return M

def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4)
    M[0, 0] =  c; M[0, 2] =  s
    M[2, 0] = -s; M[2, 2] =  c
    return M

def _joint_T(jt, angles):
    """4×4 transform for one joint given angles dict."""
    T = _T(jt['xyz'])
    if jt['type'] == 'fixed':
        return T
    a = np.deg2rad(angles.get(jt['akey'], 0.0))
    R = _Rz(a) if jt['axis'] == 'z' else _Ry(a)
    return T @ R


# ── URDF joint tree (xyz in mm = URDF metres × 1000) ─────────────────────────
#
# Parallel-link angle keys:
#   j1, j2, neg_j2=-j2, j3, neg_j3=-j3, j4
#
# Main FK chain:
#   base_link →[j1 Z]→ link1 →[j2_1 Y]→ link2_1 →[j3_1 Y=-J2]→ link3_1
#              →[j4_1 Y=J3]→ link4_1 →[j5_1 fixed]→ link5
#              →[j5 Z=J4]→ flange (EE)
# Parallel chain:
#   link1 →[j2_2 Y=J2]→ link2_2 →[j3_2 Y=-J2]→ link3_2
#          →[j4_2 Y=-J3]→ link4_2

LINK_TREE = {
    'link1':   {'parent': 'base_link', 'jt': {'type': 'revolute', 'axis': 'z',
                    'xyz': [-5.0, 0.0, 109.0],        'akey': 'j1'}},
    'link2_1': {'parent': 'link1',     'jt': {'type': 'revolute', 'axis': 'y',
                    'xyz': [43.5, -35.775, 119.0],     'akey': 'j2'}},
    'link2_2': {'parent': 'link1',     'jt': {'type': 'revolute', 'axis': 'y',
                    'xyz': [4.529, -30.5, 141.5],      'akey': 'j2'}},
    'link3_1': {'parent': 'link2_1',   'jt': {'type': 'revolute', 'axis': 'y',
                    'xyz': [-1.051, 35.775, 175.001],  'akey': 'j3_1'}},
    'link3_2': {'parent': 'link2_2',   'jt': {'type': 'revolute', 'axis': 'y',
                    'xyz': [-1.05, 6.5, 175.0],        'akey': 'j3_2'}},
    'link4_1': {'parent': 'link3_1',   'jt': {'type': 'revolute', 'axis': 'y',
                    'xyz': [175.0, -17.0, 3.25],       'akey': 'j4_1'}},
    'link4_2': {'parent': 'link3_2',   'jt': {'type': 'revolute', 'axis': 'y',
                    'xyz': [67.9, 0.5, 11.972],        'akey': 'j4_2'}},
    'link5':   {'parent': 'link4_1',   'jt': {'type': 'fixed',
                    'xyz': [66.0, 17.0, 31.0]}},
    'link4_3': {'parent': 'link5',     'jt': {'type': 'fixed',
                    'xyz': [0.0, 0.0, 2.0]}},
    'flange':  {'parent': 'link5',     'jt': {'type': 'revolute', 'axis': 'z',
                    'xyz': [0.0, 0.0, -84.0],          'akey': 'j4'}},
}

DRAW_ORDER = [
    'link1', 'link2_1', 'link2_2',
    'link3_1', 'link3_2',
    'link4_1', 'link4_2',
    'link5', 'link4_3', 'flange',
]

MESH_FILES = {
    'base_link': 'base_link.dae',
    'link1':     'link1.dae',
    'link2_1':   'link2_1.dae',
    'link2_2':   'link2_2.dae',
    'link3_1':   'link3_1.dae',
    'link3_2':   'link3_2.dae',
    'link4_1':   'link4_1.dae',
    'link4_2':   'link4_2.dae',
    'link4_3':   'link4_3.dae',
    'link5':     'link5.dae',
    'flange':    'flange.dae',
}

# Per-link display colours (RGB) - Realistic MG400 colors
LINK_COLORS = {
    'base_link': (0.15, 0.15, 0.15), # Dark base
    'link1':     (0.90, 0.90, 0.90), # White main body
    'link2_1':   (0.90, 0.90, 0.90), # White main arm
    'link2_2':   (0.60, 0.60, 0.60), # Silver rear rod
    'link3_1':   (0.90, 0.90, 0.90), # White forearm
    'link3_2':   (0.60, 0.60, 0.60), # Silver upper rod
    'link4_1':   (0.90, 0.90, 0.90), # White wrist
    'link4_2':   (0.60, 0.60, 0.60), # Silver front rod
    'link4_3':   (0.20, 0.20, 0.20), # Dark grey wrist details
    'link5':     (0.25, 0.25, 0.25), # Dark grey EE base
    'flange':    (0.40, 0.40, 0.40), # Metallic flange
}


# ── FK ────────────────────────────────────────────────────────────────────────

def compute_link_transforms(joints_deg):
    """
    Return dict  link_name → 4×4 world transform (mm)
    for [J1, J2, J3, J4] in degrees.
    """
    j1, j2, j3, j4 = (float(x) for x in joints_deg[:4])
    # The real MG400 linkage:
    angles = {
        'j1': j1,
        'j2': j2,
        'j4': j4,
        # The forearm link3_1 must have absolute angle J3. Since it is connected to link2_1 (abs angle J2),
        # its relative angle is J3 - J2.
        'j3_1': j3 - j2,
        # The rear parallel rod link3_2 must stay vertical (abs angle 0). Connected to link2_2 (abs angle J2),
        # its relative angle is -J2.
        'j3_2': -j2,
        # The wrist link4_1 must stay horizontal (abs angle 0). Connected to link3_1 (abs angle J3),
        # its relative angle is -J3.
        'j4_1': -j3,
        # The front parallel rod link4_2 must have absolute angle J3. Connected to link3_2 (abs angle 0),
        # its relative angle is J3.
        'j4_2': j3,
    }
    
    transforms = {'base_link': np.eye(4)}
    for link in DRAW_ORDER:
        info = LINK_TREE[link]
        transforms[link] = transforms[info['parent']] @ _joint_T(info['jt'], angles)
    return transforms


def get_ee_transform(joints_deg):
    """4×4 world transform of the flange (mm)."""
    return compute_link_transforms(joints_deg)['flange']


def get_ee_pos(joints_deg):
    """EE XYZ world position (mm)."""
    return get_ee_transform(joints_deg)[:3, 3].copy()


# ── Mesh loading ──────────────────────────────────────────────────────────────

_MESH_DIR   = None
_MESH_CACHE = {}


def _find_mesh_dir():
    # core/ → mg400_simulator/ → tools/ → project_teleop/
    here      = os.path.dirname(os.path.abspath(__file__))
    project   = os.path.dirname(os.path.dirname(os.path.dirname(here)))
    # Primary: project_teleop/src/dobot_mg400/mg400_description/meshes
    candidate = os.path.join(project, 'src', 'dobot_mg400', 'mg400_description', 'meshes')
    if os.path.isdir(candidate):
        return candidate
    # Fallback: workspace-level (e.g. ~/project_teleop_ws/src/...)
    ws = os.path.dirname(project)
    fallback = os.path.join(ws, 'src', 'dobot_mg400', 'mg400_description', 'meshes')
    return fallback if os.path.isdir(fallback) else None


def load_mesh(link_name):
    """
    Load .dae mesh for link_name.
    Returns (vertices_mm: Nx3 f32, faces: Mx3 i32, vertex_normals: Nx3 f32)
    or (None, None, None) on failure.
    """
    global _MESH_DIR
    if link_name in _MESH_CACHE:
        return _MESH_CACHE[link_name]

    if _MESH_DIR is None:
        _MESH_DIR = _find_mesh_dir()

    fname = MESH_FILES.get(link_name)
    if not fname or _MESH_DIR is None:
        _MESH_CACHE[link_name] = (None, None, None)
        return (None, None, None)

    filepath = os.path.join(_MESH_DIR, fname)
    try:
        import trimesh
        raw = trimesh.load(str(filepath))
        # Scene -> flatten with transforms applied
        if isinstance(raw, trimesh.Scene):
            mesh = raw.dump(concatenate=True)
        else:
            mesh = raw
            
        # Convert meters to millimeters
        verts  = np.array(mesh.vertices,      dtype=np.float32) * 1000.0
        faces  = np.array(mesh.faces,         dtype=np.int32)
        vnorms = np.array(mesh.vertex_normals, dtype=np.float32)
        result = (verts, faces, vnorms)
        print(f'[urdf_loader] loaded {link_name}: {len(verts)} verts, {len(faces)} tris')
    except Exception as exc:
        print(f'[urdf_loader] WARN: cannot load {link_name}: {exc}')
        result = (None, None, None)

    _MESH_CACHE[link_name] = result
    return result
