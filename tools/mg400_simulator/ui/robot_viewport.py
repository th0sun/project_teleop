#!/usr/bin/env python3
"""
MG400 3-D OpenGL Viewport
* Renders actual URDF mesh files (.dae) via trimesh
* Full parallel-link FK from mg400.xacro joint tree
* Ball-follows-cursor: sphere at EE tracks mouse at fixed depth (Unity-style)
* Axis arrows: ray-plane drag constrained to one world axis
* Orbit / Pan / Zoom camera
"""

import math
import numpy as np

from PyQt5.QtWidgets import QOpenGLWidget
from PyQt5.QtCore    import Qt, pyqtSignal
from PyQt5.QtGui     import QMouseEvent, QWheelEvent, QSurfaceFormat

from OpenGL.GL   import *
from OpenGL.GLU  import *

from core.mg400_kinematics import inverse_kinematics, JOINT_LIMITS
from core import urdf_loader

# ── Gizmo constants ───────────────────────────────────────────────────────────

AXIS_NONE   = -1
AXIS_X      =  0
AXIS_Y      =  1
AXIS_Z      =  2
AXIS_SPHERE =  3

_AXIS_DIRS = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
_AXIS_COLS = [(0.90, 0.20, 0.20), (0.15, 0.85, 0.25), (0.20, 0.45, 1.00)]

GIZMO_LEN = 65.0   # mm shaft length
GIZMO_HIT = 12.0   # mm hit-test radius
EE_SPHERE = 14.0   # mm indicator sphere radius


# ── Viewport ──────────────────────────────────────────────────────────────────


class RobotViewport(QOpenGLWidget):

    joints_changed = pyqtSignal(list)
    ee_dragged     = pyqtSignal(float, float, float)

    def __init__(self, parent=None):
        fmt = QSurfaceFormat()
        fmt.setSamples(4)
        fmt.setDepthBufferSize(24)
        QSurfaceFormat.setDefaultFormat(fmt)
        super().__init__(parent)

        self._joints = np.array([0.0, 0.0, -45.0, 0.0])
        self._ghost  = None

        # Camera
        self._cam_dist  = 650.0
        self._cam_azim  = 45.0
        self._cam_elev  = 28.0
        self._cam_focus = np.array([0.0, 0.0, 100.0])

        # Gizmo / drag state
        self._hover_axis = AXIS_NONE
        self._drag_axis  = AXIS_NONE
        self._mouse_mode = 'none'
        self._last_pos   = None

        # Drag internals
        self._drag_start_ee  = None   # EE world pos at drag start
        self._drag_depth     = 0.5    # NDC depth of EE when drag started
        self._drag_offset    = np.zeros(3)   # EE minus mouse-world at drag start
        self._drag_axis_dir  = None   # unit world axis for constrained drag
        self._drag_plane_n   = None   # normal of drag plane (axis drags)
        self._drag_start_hit = None   # ray-plane hit at drag start (axis drags)

        # Cached OpenGL matrices (set every paintGL, used in mouse events)
        self._cached_vp   = None
        self._cached_mv   = None
        self._cached_proj = None

        # Compiled OpenGL display lists
        self._meshes = {}

    # ── Public API ─────────────────────────────────────────────────────────────

    def get_joints(self) -> np.ndarray:
        return self._joints.copy()

    def set_joints(self, joints):
        self._joints = np.array(joints, dtype=float)
        self.update()

    def set_ghost_joints(self, joints):
        self._ghost = np.array(joints, dtype=float) if joints is not None else None
        self.update()

    # ── OpenGL initialisation ──────────────────────────────────────────────────

    def initializeGL(self):
        glClearColor(0.12, 0.12, 0.14, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glShadeModel(GL_SMOOTH)
        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 2.0, 0.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  [0.9, 0.9, 0.9, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.3, 0.3, 0.3, 1.0])
        self._load_meshes()

    def _load_meshes(self):
        for link in ['base_link'] + list(urdf_loader.DRAW_ORDER):
            verts, faces, vnorms = urdf_loader.load_mesh(link)
            if verts is not None:
                self._meshes[link] = self._compile_mesh(verts, faces, vnorms)
            else:
                self._meshes[link] = self._compile_fallback()

    def _compile_mesh(self, verts, faces, vnorms):
        dl = glGenLists(1)
        glNewList(dl, GL_COMPILE)
        glBegin(GL_TRIANGLES)
        for tri in faces:
            for vi in tri:
                glNormal3fv(vnorms[vi])
                glVertex3fv(verts[vi])
        glEnd()
        glEndList()
        return dl

    def _compile_fallback(self):
        dl = glGenLists(1)
        glNewList(dl, GL_COMPILE)
        q = gluNewQuadric()
        gluSphere(q, 8.0, 10, 10)
        gluDeleteQuadric(q)
        glEndList()
        return dl

    def resizeGL(self, w, h):
        h = max(h, 1)
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, w / h, 1.0, 8000.0)
        glMatrixMode(GL_MODELVIEW)

    # ── Camera ─────────────────────────────────────────────────────────────────

    def _camera_pos(self) -> np.ndarray:
        az = math.radians(self._cam_azim)
        el = math.radians(self._cam_elev)
        r  = self._cam_dist
        return np.array([
            self._cam_focus[0] + r * math.cos(el) * math.cos(az),
            self._cam_focus[1] + r * math.cos(el) * math.sin(az),
            self._cam_focus[2] + r * math.sin(el),
        ])

    # ── Paint ──────────────────────────────────────────────────────────────────

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        cp = self._camera_pos()
        gluLookAt(cp[0], cp[1], cp[2],
                  self._cam_focus[0], self._cam_focus[1], self._cam_focus[2],
                  0.0, 0.0, 1.0)

        # Cache matrices so mouse-event handlers can use them safely
        self._cached_mv   = glGetDoublev(GL_MODELVIEW_MATRIX)
        self._cached_proj = glGetDoublev(GL_PROJECTION_MATRIX)
        self._cached_vp   = glGetIntegerv(GL_VIEWPORT)

        self._draw_grid()
        self._draw_robot(self._joints, alpha=1.0)
        if self._ghost is not None:
            self._draw_robot(self._ghost, alpha=0.35)
        self._draw_gizmo()

    def _draw_grid(self):
        glDisable(GL_LIGHTING)
        glColor3f(0.28, 0.28, 0.30)
        glBegin(GL_LINES)
        for i in range(-10, 11):
            glVertex3f(i * 50, -500, 0); glVertex3f(i * 50,  500, 0)
            glVertex3f(-500, i * 50, 0); glVertex3f( 500, i * 50, 0)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_robot(self, joints, alpha=1.0):
        transforms = urdf_loader.compute_link_transforms(joints)
        for link in ['base_link'] + list(urdf_loader.DRAW_ORDER):
            T   = transforms.get(link, np.eye(4))
            col = urdf_loader.LINK_COLORS.get(link, (0.7, 0.7, 0.7))
            dl  = self._meshes.get(link)
            if dl is None:
                continue
            glPushMatrix()
            glMultMatrixf(T.T.astype(np.float32))
            glColor4f(col[0], col[1], col[2], alpha)
            glCallList(dl)
            glPopMatrix()

    def _draw_gizmo(self):
        active = self._drag_axis if self._drag_axis != AXIS_NONE else self._hover_axis
        ee = self._ee_pos()

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glLineWidth(3.0)

        for i, (d, c) in enumerate(zip(_AXIS_DIRS, _AXIS_COLS)):
            tip    = ee + d * GIZMO_LEN
            bright = 1.3 if i == active else 0.80
            r, g, b = c
            glColor3f(min(r * bright, 1.0), min(g * bright, 1.0), min(b * bright, 1.0))
            glBegin(GL_LINES)
            glVertex3fv(ee.tolist()); glVertex3fv(tip.tolist())
            glEnd()
            # Arrow head: two diagonal lines
            for pi in range(3):
                if abs(d[pi]) < 0.9:
                    perp = np.zeros(3); perp[pi] = 1.0
                    break
            side = np.cross(d, perp)
            side /= np.linalg.norm(side)
            glBegin(GL_LINES)
            glVertex3fv(tip.tolist())
            glVertex3fv((tip - d * 9 + side * 5).tolist())
            glEnd()
            glBegin(GL_LINES)
            glVertex3fv(tip.tolist())
            glVertex3fv((tip - d * 9 - side * 5).tolist())
            glEnd()

        glLineWidth(1.0)

        # EE sphere
        if active == AXIS_SPHERE:
            glColor3f(1.0, 0.85, 0.0)
        elif self._hover_axis == AXIS_SPHERE:
            glColor3f(0.90, 0.75, 0.10)
        else:
            glColor3f(0.68, 0.68, 0.72)
        glPushMatrix()
        glTranslatef(float(ee[0]), float(ee[1]), float(ee[2]))
        q = gluNewQuadric()
        gluSphere(q, EE_SPHERE, 20, 20)
        gluDeleteQuadric(q)
        glPopMatrix()

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

    # ── EE helper ──────────────────────────────────────────────────────────────

    def _ee_pos(self) -> np.ndarray:
        return urdf_loader.get_ee_pos(self._joints)

    # ── Screen ↔ World helpers ─────────────────────────────────────────────────

    def _lp(self, lx, ly):
        """Qt logical pixels → physical (Retina-aware) pixels."""
        if self._cached_vp is None:
            return float(lx), float(ly)
        rx = self._cached_vp[2] / max(self.width(),  1)
        ry = self._cached_vp[3] / max(self.height(), 1)
        return float(lx) * rx, float(ly) * ry

    def _project(self, pt) -> tuple:
        """World 3-D → (screen_px, screen_py, ndc_depth). screen_py is 0 at TOP."""
        if self._cached_mv is None:
            return 0.0, 0.0, 0.5
        vp = self._cached_vp
        sx, sy, sz = gluProject(float(pt[0]), float(pt[1]), float(pt[2]),
                                 self._cached_mv, self._cached_proj, vp)
        return float(sx), float(vp[3] - sy), float(sz)

    def _unproject(self, px, py, depth) -> np.ndarray:
        """Physical screen (px, py) + NDC depth → world 3-D point. py is 0 at TOP."""
        if self._cached_mv is None:
            return np.zeros(3)
        vp = self._cached_vp
        wy = vp[3] - float(py)     # OpenGL Y is flipped (0 at bottom)
        pt = gluUnProject(float(px), wy, float(depth),
                          self._cached_mv, self._cached_proj, vp)
        return np.array(pt, dtype=float)

    def _mouse_ray(self, px, py):
        """Physical screen pixel → (ray_origin, ray_direction) in world space."""
        near = self._unproject(px, py, 0.0)
        far  = self._unproject(px, py, 1.0)
        d    = far - near
        n    = np.linalg.norm(d)
        return near, (d / n if n > 1e-12 else d)

    def _ray_plane(self, ray_o, ray_d, plane_p, plane_n):
        """Ray-plane intersection. Returns None if parallel or behind ray."""
        denom = np.dot(ray_d, plane_n)
        if abs(denom) < 1e-9:
            return None
        t = np.dot(plane_p - ray_o, plane_n) / denom
        return None if t < 0 else ray_o + t * ray_d

    def _best_drag_plane_normal(self, axis_dir) -> np.ndarray:
        """
        Normal for the axis-drag plane.
        The plane contains the drag axis and faces the camera as much as possible,
        so the ray-plane intersection is well-conditioned.
        """
        cam_fwd = self._drag_start_ee - self._camera_pos()
        n = np.linalg.norm(cam_fwd)
        cam_fwd = cam_fwd / n if n > 1e-6 else np.array([0., 0., -1.])
        # Remove axis component → what remains faces the camera
        proj = cam_fwd - np.dot(cam_fwd, axis_dir) * axis_dir
        pn   = np.linalg.norm(proj)
        if pn > 1e-6:
            return proj / pn
        # Axis is pointing directly at camera – pick arbitrary perpendicular
        idx  = int(np.argmin(np.abs(axis_dir)))
        temp = np.zeros(3); temp[idx] = 1.0
        fb   = np.cross(axis_dir, temp)
        return fb / np.linalg.norm(fb)

    # ── Hit testing ────────────────────────────────────────────────────────────

    def _pick(self, lx, ly) -> int:
        """Return which gizmo element is under logical pixel (lx, ly)."""
        if self._cached_mv is None:
            return AXIS_NONE
        ee = self._ee_pos()
        px, py = self._lp(lx, ly)
        
        # Calculate scale factor for physical pixels vs logical pixels
        phys_scale = self._cached_vp[3] / max(self.height(), 1)

        # EE sphere – generous hit radius in logical pixels
        ex, ey, _ = self._project(ee)
        if math.hypot(px - ex, py - ey) <= 22.0 * phys_scale:
            return AXIS_SPHERE

        # Axis arrows – hit along the shaft segment
        for i, d in enumerate(_AXIS_DIRS):
            tip        = ee + d * GIZMO_LEN
            tx, ty, _  = self._project(tip)
            seg        = np.array([tx - ex, ty - ey])
            slen       = np.linalg.norm(seg)
            if slen < 1e-3:
                continue
            v       = np.array([px - ex, py - ey])
            t       = np.clip(np.dot(v, seg) / (slen * slen), 0.0, 1.0)
            closest = np.array([ex, ey]) + t * seg
            if np.linalg.norm(np.array([px, py]) - closest) <= 12.0 * phys_scale:
                return i

        return AXIS_NONE

    # ── Mouse events ───────────────────────────────────────────────────────────

    def mousePressEvent(self, event: QMouseEvent):
        self._last_pos = event.pos()
        lx, ly = event.x(), event.y()
        btn    = event.button()

        if btn == Qt.LeftButton:
            hit = self._pick(lx, ly)
            if hit != AXIS_NONE:
                self._drag_axis  = hit
                self._mouse_mode = 'drag'
                ee = self._ee_pos()
                self._drag_start_ee = ee.copy()
                _, _, self._drag_depth = self._project(ee)
                px, py = self._lp(lx, ly)
                
                # Import forward_kinematics to get the exact analytical start position
                from core.mg400_kinematics import forward_kinematics
                # fk returns [px, py, pz, rx, ry, rz]
                ik_start_pose = forward_kinematics(self._joints)
                self._drag_ik_start_pos = ik_start_pose[:3]
                
                # We need the mouse's starting 3D position to compute pure deltas
                self._drag_start_mouse_world = self._unproject(px, py, self._drag_depth)

                if hit == AXIS_SPHERE:
                    self._drag_axis_dir  = None
                    self._drag_plane_n   = None
                else:
                    # Axis drag: compute best plane to project rays onto
                    axis_d               = _AXIS_DIRS[hit].copy()
                    self._drag_axis_dir  = axis_d
                    self._drag_plane_n   = self._best_drag_plane_normal(axis_d)
                    ray_o, ray_d         = self._mouse_ray(px, py)
                    start_hit            = self._ray_plane(ray_o, ray_d, ee, self._drag_plane_n)
                    self._drag_start_hit = start_hit if start_hit is not None else ee.copy()
                return

            self._mouse_mode = 'orbit'
        elif btn == Qt.MiddleButton:
            self._mouse_mode = 'pan'
        elif btn == Qt.RightButton:
            self._mouse_mode = 'zoom'

    def mouseMoveEvent(self, event: QMouseEvent):
        if self._last_pos is None:
            self._last_pos = event.pos()
            return

        dx_log = event.x() - self._last_pos.x()
        dy_log = event.y() - self._last_pos.y()
        self._last_pos = event.pos()
        lx, ly = event.x(), event.y()
        px, py = self._lp(lx, ly)

        # ── Gizmo drag ──────────────────────────────────────────────────────
        if self._mouse_mode == 'drag':
            if self._drag_axis == AXIS_SPHERE:
                # 1. Get current mouse 3D world pos at fixed depth
                mouse_curr_world = self._unproject(px, py, self._drag_depth)
                # 2. Compute absolute delta from when drag started
                delta = mouse_curr_world - self._drag_start_mouse_world
                
            else:
                # 1. Cast ray to the axis constraint plane
                ray_o, ray_d = self._mouse_ray(px, py)
                hit = self._ray_plane(ray_o, ray_d, self._ee_pos(), self._drag_plane_n)
                if hit is None:
                    self.update()
                    return
                # 2. Compute raw delta on plane, then project onto the constrained axis
                raw_delta = hit - self._drag_start_hit
                proj_mm   = np.dot(raw_delta, self._drag_axis_dir)
                delta     = self._drag_axis_dir * proj_mm

            # Apply the pure delta to the ANALYTICAL IK starting position
            # This completely bypasses any coordinate mismatch between URDF visuals and IK math
            target_ik_pos = self._drag_ik_start_pos + delta

            result = inverse_kinematics(
                float(target_ik_pos[0]), float(target_ik_pos[1]), float(target_ik_pos[2]),
                j4_deg=float(self._joints[3]),
                prefer_elbow='up',
                current_joints=self._joints)
                
            if result is not None:
                self._joints = result
                self.joints_changed.emit(self._joints.tolist())
                # For the UI event, we can just emit the target IK pos
                self.ee_dragged.emit(float(target_ik_pos[0]), float(target_ik_pos[1]), float(target_ik_pos[2]))
            self.update()
            return

        # ── Hover highlight (no button held) ────────────────────────────────
        if self._mouse_mode == 'none':
            new_hover = self._pick(lx, ly)
            if new_hover != self._hover_axis:
                self._hover_axis = new_hover
                self.update()
            return

        # ── Camera controls ─────────────────────────────────────────────────
        if self._mouse_mode == 'orbit':
            self._cam_azim -= dx_log * 0.4
            self._cam_elev  = float(np.clip(self._cam_elev + dy_log * 0.4, -89.0, 89.0))
        elif self._mouse_mode == 'pan':
            az  = math.radians(self._cam_azim)
            el  = math.radians(self._cam_elev)
            spd = self._cam_dist * 0.0012
            right   = np.array([-math.sin(az),  math.cos(az), 0.0])
            up_proj = np.array([-math.cos(az) * math.sin(el),
                                 -math.sin(az) * math.sin(el),
                                  math.cos(el)])
            self._cam_focus += right * (-dx_log * spd) + up_proj * (dy_log * spd)
        elif self._mouse_mode == 'zoom':
            self._cam_dist *= 1.0 + dy_log * 0.005
            self._cam_dist  = float(np.clip(self._cam_dist, 50.0, 3000.0))
        self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        self._drag_axis  = AXIS_NONE
        self._mouse_mode = 'none'
        self._last_pos   = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        self._cam_dist *= 0.999 ** delta
        self._cam_dist  = max(50.0, min(3000.0, self._cam_dist))
        self.update()

    def keyPressEvent(self, event):
        k = event.key()
        if k == Qt.Key_R:
            self._cam_dist = 650.0; self._cam_azim = 45.0
            self._cam_elev = 28.0;  self._cam_focus = np.array([0., 0., 100.])
        elif k == Qt.Key_F:
            self._cam_focus = self._ee_pos().copy()
        self.update()
