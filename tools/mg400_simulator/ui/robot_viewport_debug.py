#!/usr/bin/env python3
"""
MG400 3-D OpenGL Viewport
* Renders actual URDF mesh files (.dae) via trimesh
* Full parallel-link FK from mg400.xacro joint tree
* Unity-style XYZ gizmo at end-effector for precise axis-constrained drag
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

AXIS_NONE = -1
AXIS_X    =  0
AXIS_Y    =  1
AXIS_Z    =  2
AXIS_SPHERE = 3

_AXIS_DIRS = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
_AXIS_COLS = [(0.90,0.20,0.20), (0.15,0.85,0.25), (0.20,0.45,1.00)]

GIZMO_LEN = 65.0   # mm shaft length
GIZMO_HIT = 12.0   # mm hit-test radius
EE_SPHERE = 12.0   # mm indicator sphere radius


# ── Viewport ─────────────────────────────────────────────────────────────────


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

        # Gizmo / drag
        self._hover_axis = AXIS_NONE
        self._drag_axis  = AXIS_NONE
        self._drag_ref   = None
        self._mouse_mode = 'none'
        self._last_pos   = None

        # GL resources
        self._mesh_dl  = {}
        self._gl_ready = False

        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

    # ── Public API ────────────────────────────────────────────────────────────

    def set_joints(self, joints_deg):
        self._joints = np.array(joints_deg[:4], dtype=float)
        self.update()

    def set_ghost_joints(self, joints_deg):
        self._ghost = np.array(joints_deg[:4], dtype=float) if joints_deg is not None else None
        self.update()

    def get_joints(self):
        return self._joints.copy()

    # ── OpenGL lifecycle ──────────────────────────────────────────────────────

    def initializeGL(self):
        glClearColor(0.07, 0.07, 0.11, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_NORMALIZE)
        glEnable(GL_MULTISAMPLE)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_LIGHT1)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE)

        glLightfv(GL_LIGHT0, GL_POSITION, [ 300.0,  400.0,  600.0, 0.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  [0.85, 0.85, 0.85, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.20, 0.20, 0.22, 1.0])
        glLightfv(GL_LIGHT1, GL_POSITION, [-300.0, -200.0,  200.0, 0.0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE,  [0.35, 0.35, 0.40, 1.0])
        glLightfv(GL_LIGHT1, GL_AMBIENT,  [0.00, 0.00, 0.00, 1.0])

        # Build display lists for all URDF links
        for link in ['base_link'] + urdf_loader.DRAW_ORDER:
            verts, faces, normals = urdf_loader.load_mesh(link)
            if verts is not None and len(faces):
                self._mesh_dl[link] = self._compile_mesh(verts, faces, normals)
            else:
                self._mesh_dl[link] = self._compile_fallback()

        self._gl_ready = True

    def _compile_mesh(self, verts, faces, normals):
        dl = glGenLists(1)
        glNewList(dl, GL_COMPILE)
        glBegin(GL_TRIANGLES)
        for tri in faces:
            for idx in tri:
                glNormal3fv(normals[idx].tolist())
                glVertex3fv(verts[idx].tolist())
        glEnd()
        glEndList()
        return dl

    def _compile_fallback(self):
        """Tiny box used when a mesh file can't be loaded."""
        s = 12.0
        quads = [
            ([ s, s,-s],[ s,-s,-s],[-s,-s,-s],[-s, s,-s]),
            ([ s, s, s],[-s, s, s],[-s,-s, s],[ s,-s, s]),
            ([-s, s, s],[-s, s,-s],[-s,-s,-s],[-s,-s, s]),
            ([ s, s,-s],[ s, s, s],[ s,-s, s],[ s,-s,-s]),
            ([ s,-s, s],[-s,-s, s],[-s,-s,-s],[ s,-s,-s]),
            ([ s, s, s],[ s, s,-s],[-s, s,-s],[-s, s, s]),
        ]
        dl = glGenLists(1)
        glNewList(dl, GL_COMPILE)
        glBegin(GL_QUADS)
        for quad in quads:
            v = [np.array(p, dtype=float) for p in quad]
            n = np.cross(v[1]-v[0], v[2]-v[0])
            nn = np.linalg.norm(n)
            if nn > 1e-9: n /= nn
            glNormal3fv(n.tolist())
            for vi in v: glVertex3fv(vi.tolist())
        glEnd()
        glEndList()
        return dl

    def resizeGL(self, w, h):
        h = max(h, 1)
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, w / h, 1.0, 5000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        if not self._gl_ready:
            return
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        cx, cy, cz = self._camera_pos()
        gluLookAt(cx, cy, cz,
                  self._cam_focus[0], self._cam_focus[1], self._cam_focus[2],
                  0, 0, 1)

        self._draw_grid()
        if self._ghost is not None:
            self._draw_robot(self._ghost, alpha=0.28)
        self._draw_robot(self._joints, alpha=1.0)
        self._draw_gizmo()

    # ── Camera ────────────────────────────────────────────────────────────────

    def _camera_pos(self):
        az = np.deg2rad(self._cam_azim)
        el = np.deg2rad(self._cam_elev)
        r  = self._cam_dist
        return (
            self._cam_focus[0] + r * np.cos(el) * np.cos(az),
            self._cam_focus[1] + r * np.cos(el) * np.sin(az),
            self._cam_focus[2] + r * np.sin(el),
        )

    # ── Scene drawing ─────────────────────────────────────────────────────────

    def _draw_grid(self):
        glDisable(GL_LIGHTING)
        N, S = 10, 30.0
        glLineWidth(1.0)
        glColor3f(0.22, 0.22, 0.28)
        glBegin(GL_LINES)
        for i in range(-N, N+1):
            glVertex3f(i*S, -N*S, 0); glVertex3f(i*S,  N*S, 0)
            glVertex3f(-N*S, i*S, 0); glVertex3f( N*S, i*S, 0)
        glEnd()
        glLineWidth(2.0)
        for col, end in [((0.8,0.15,0.15),(60,0,0)),
                         ((0.15,0.8,0.15),(0,60,0)),
                         ((0.15,0.15,0.9),(0,0,60))]:
            glColor3f(*col)
            glBegin(GL_LINES); glVertex3f(0,0,0); glVertex3f(*end); glEnd()
        glLineWidth(1.0)
        glEnable(GL_LIGHTING)

    def _draw_robot(self, joints_deg, alpha=1.0):
        """Draw all URDF links using precompiled display lists."""
        transforms = urdf_loader.compute_link_transforms(joints_deg)
        blend = alpha < 1.0
        if blend:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glDepthMask(GL_FALSE)
        for link, T in transforms.items():
            dl = self._mesh_dl.get(link)
            if dl is None:
                continue
            r, g, b = urdf_loader.LINK_COLORS.get(link, (0.5, 0.5, 0.5))
            glColor4f(r, g, b, alpha)
            glPushMatrix()
            # OpenGL is column-major → pass numpy row-major matrix transposed
            glMultMatrixf(T.astype(np.float32).T.flatten().tolist())
            glCallList(dl)
            glPopMatrix()
        if blend:
            glDepthMask(GL_TRUE)
            glDisable(GL_BLEND)

    # ── Gizmo drawing ─────────────────────────────────────────────────────────

    def _ee_pos(self):
        return urdf_loader.get_ee_pos(self._joints)

    def _draw_gizmo(self):
        ee = self._ee_pos()
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)  # Ensure gizmo is drawn on top of the robot
        for i, (d, c) in enumerate(zip(_AXIS_DIRS, _AXIS_COLS)):
            highlight = (i == self._drag_axis or i == self._hover_axis)
            glColor3f(1.0, 0.95, 0.2) if highlight else glColor3f(*c)
            tip = ee + d * GIZMO_LEN
            glLineWidth(3.5 if highlight else 2.0)
            glBegin(GL_LINES)
            glVertex3f(*ee.tolist()); glVertex3f(*tip.tolist())
            glEnd()
            # arrowhead fan
            perp1 = np.roll(d, 1)
            perp2 = np.cross(d, perp1)
            for p in [perp1, -perp1, perp2, -perp2]:
                glBegin(GL_LINES)
                glVertex3f(*(tip - d*9 + p*4).tolist())
                glVertex3f(*tip.tolist())
                glEnd()
        # EE sphere
        glEnable(GL_LIGHTING)
        glColor3f(1.0, 0.50, 0.10)
        glPushMatrix()
        glTranslatef(*ee.tolist())
        q = gluNewQuadric()
        gluSphere(q, EE_SPHERE, 18, 18)
        gluDeleteQuadric(q)
        glPopMatrix()
        glLineWidth(1.0)
        glEnable(GL_DEPTH_TEST)

    # ── Ray / picking helpers ─────────────────────────────────────────────────

    def _mouse_ray(self, mx, my):
        self.makeCurrent()
        vp   = glGetIntegerv(GL_VIEWPORT)
        
        # Robustly handle Retina/High-DPI displays by comparing GL viewport (physical) 
        # to Qt widget size (logical)
        ratio_x = vp[2] / float(self.width()) if self.width() > 0 else 1.0
        ratio_y = vp[3] / float(self.height()) if self.height() > 0 else 1.0
        
        mx_p = mx * ratio_x
        my_p = my * ratio_y
        
        mv   = glGetDoublev(GL_MODELVIEW_MATRIX)
        proj = glGetDoublev(GL_PROJECTION_MATRIX)
        wy   = vp[3] - my_p
        
        near = np.array(gluUnProject(mx_p, wy, 0.0, mv, proj, vp), dtype=float)
        far  = np.array(gluUnProject(mx_p, wy, 1.0, mv, proj, vp), dtype=float)
        d = far - near
        n = np.linalg.norm(d)
        return near, (d/n if n > 1e-9 else np.array([0.,0.,-1.]))

    def _ray_line_dist(self, ray_o, ray_d, lp, ld):
        """Min distance from ray to line segment 0..GIZMO_LEN along ld from lp."""
        w = ray_o - lp
        a = np.dot(ray_d, ray_d); b = np.dot(ray_d, ld)
        c = np.dot(ld, ld);       d = np.dot(ray_d, w); e = np.dot(ld, w)
        denom = a*c - b*b
        if abs(denom) < 1e-9:
            return float(np.linalg.norm(np.cross(w, ray_d)))
        sc = (b*e - c*d) / denom
        tc = max(0.0, min(GIZMO_LEN, (a*e - b*d) / denom))
        return float(np.linalg.norm((ray_o + sc*ray_d) - (lp + tc*ld)))

    def _closest_on_line(self, ray_o, ray_d, lp, ld):
        """Point on line ld through lp that is closest to the camera ray."""
        w = ray_o - lp
        a = np.dot(ray_d, ray_d); b = np.dot(ray_d, ld)
        c = np.dot(ld, ld);       d = np.dot(ray_d, w); e = np.dot(ld, w)
        denom = a*c - b*b
        if abs(denom) < 1e-9:
            return lp
        return lp + ((a*e - b*d) / denom) * ld

    def _closest_on_plane(self, ray_o, ray_d, plane_p, plane_n):
        """Intersection of ray with a plane."""
        denom = np.dot(ray_d, plane_n)
        if abs(denom) < 1e-9:
            return None
        t = np.dot(plane_p - ray_o, plane_n) / denom
        if t < 0:
            return None
        return ray_o + t * ray_d

    def _pick_axis(self, mx, my):
        ray_o, ray_d = self._mouse_ray(mx, my)
        ee = self._ee_pos()
        
        # Check sphere first
        oc = ray_o - ee
        b = np.dot(oc, ray_d)
        c = np.dot(oc, oc) - (EE_SPHERE * 1.5)**2 # Slightly larger hit area
        desc = b*b - c
        if desc > 0:
            return AXIS_SPHERE

        # Then check axes
        best, best_ax = GIZMO_HIT, AXIS_NONE
        for i, d in enumerate(_AXIS_DIRS):
            dist = self._ray_line_dist(ray_o, ray_d, ee, d)
            if dist < best:
                best, best_ax = dist, i
        return best_ax

    def _project_to_screen(self, p):
        self.makeCurrent()
        vp   = glGetIntegerv(GL_VIEWPORT)
        mv   = glGetDoublev(GL_MODELVIEW_MATRIX)
        proj = glGetDoublev(GL_PROJECTION_MATRIX)
        win_pos = gluProject(p[0], p[1], p[2], mv, proj, vp)
        return np.array([win_pos[0], win_pos[1]])

    def _get_screen_depth(self, pos3d):
        """Return the normalised window-Z [0..1] for a world-space point."""
        self.makeCurrent()
        vp   = glGetIntegerv(GL_VIEWPORT)
        mv   = glGetDoublev(GL_MODELVIEW_MATRIX)
        proj = glGetDoublev(GL_PROJECTION_MATRIX)
        win  = gluProject(pos3d[0], pos3d[1], pos3d[2], mv, proj, vp)
        return float(win[2])

    def _unproject_screen(self, mx, my, win_z):
        """Unproject logical-pixel (mx, my) at window depth win_z to world coords."""
        self.makeCurrent()
        vp      = glGetIntegerv(GL_VIEWPORT)
        ratio_x = vp[2] / float(self.width())  if self.width()  > 0 else 1.0
        ratio_y = vp[3] / float(self.height()) if self.height() > 0 else 1.0
        mx_p = mx * ratio_x
        my_p = my * ratio_y
        wy   = vp[3] - my_p
        mv   = glGetDoublev(GL_MODELVIEW_MATRIX)
        proj = glGetDoublev(GL_PROJECTION_MATRIX)
        return np.array(gluUnProject(mx_p, wy, win_z, mv, proj, vp), dtype=float)

    # ── Mouse events ─────────────────────────────────────────────────────────

    def mousePressEvent(self, event):
        self._last_pos = event.pos()
        mx, my = event.x(), event.y()
        if event.button() == Qt.LeftButton:
            ax = self._pick_axis(mx, my)
            if ax != AXIS_NONE:
                self._drag_axis     = ax
                self._mouse_mode    = 'gizmo'
                self._drag_start_ee = self._ee_pos().copy()
                self._drag_start_mx = mx
                self._drag_start_my = my

                self.makeCurrent()
                vp   = glGetIntegerv(GL_VIEWPORT)
                mv   = glGetDoublev(GL_MODELVIEW_MATRIX)
                proj = glGetDoublev(GL_PROJECTION_MATRIX)
                rx = vp[2] / float(self.width())  if self.width()  > 0 else 1.0
                ry = vp[3] / float(self.height()) if self.height() > 0 else 1.0
                self._drag_ratio_x = rx
                self._drag_ratio_y = ry

                ee   = self._drag_start_ee
                ee_s = gluProject(ee[0], ee[1], ee[2], mv, proj, vp)
                self._drag_depth = float(ee_s[2])

                if ax != AXIS_SPHERE:
                    # Project 3-D axis onto screen; compute pixels-per-mm
                    ax_d  = _AXIS_DIRS[ax]
                    tip   = ee + ax_d * GIZMO_LEN
                    tip_s = gluProject(tip[0], tip[1], tip[2], mv, proj, vp)
                    axis_2d = np.array([tip_s[0] - ee_s[0], tip_s[1] - ee_s[1]])
                    axis_len_px = float(np.linalg.norm(axis_2d))
                    if axis_len_px > 1.0:
                        self._drag_axis_screen_dir = axis_2d / axis_len_px
                        self._drag_mm_per_phys_px  = GIZMO_LEN / axis_len_px
                    else:
                        # Axis nearly perpendicular to screen — use unproject fallback
                        self._drag_axis_screen_dir = None
                        self._drag_mm_per_phys_px  = 1.0
                    self._drag_start_3d = self._unproject_screen(mx, my, self._drag_depth)
                else:
                    self._drag_axis_screen_dir = None
                    self._drag_start_3d = self._unproject_screen(mx, my, self._drag_depth)
            else:
                self._mouse_mode = 'orbit'
        elif event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._mouse_mode = 'pan'

    def mouseMoveEvent(self, event):
        mx, my = event.x(), event.y()
        if self._last_pos is None:
            self._last_pos = event.pos()
        dx = mx - self._last_pos.x()
        dy = my - self._last_pos.y()
        alt = bool(event.modifiers() & Qt.AltModifier)

        if self._mouse_mode == 'gizmo' and self._drag_axis != AXIS_NONE:
            new_ee = self._drag_start_ee.copy()

            if self._drag_axis != AXIS_SPHERE:
                ax_d = _AXIS_DIRS[self._drag_axis]
                if self._drag_axis_screen_dir is not None:
                    # Screen-projection: total mouse delta in physical pixels → world mm
                    dmx_phys = (mx - self._drag_start_mx) * self._drag_ratio_x
                    dmy_phys = -(my - self._drag_start_my) * self._drag_ratio_y  # flip Y (OpenGL up)
                    proj_px  = float(np.dot([dmx_phys, dmy_phys], self._drag_axis_screen_dir))
                    world_dist = proj_px * self._drag_mm_per_phys_px

                    dmx_phys = (mx - self._drag_start_mx) * self._drag_ratio_x
                    dmy_phys = -(my - self._drag_start_my) * self._drag_ratio_y
                    proj_px  = float(np.dot([dmx_phys, dmy_phys], self._drag_axis_screen_dir))
                    world_dist = proj_px * self._drag_mm_per_phys_px
                    new_ee = self._drag_start_ee + ax_d * world_dist
                    with open('drag_debug.log', 'a') as df:
                        df.write(f"DRAG {self._drag_axis}: mx={mx} start_mx={self._drag_start_mx} dmx={dmx_phys:.2f} dmy={dmy_phys:.2f} proj={proj_px:.2f} dist={world_dist:.2f}\n")

                else:
                    # Fallback: unproject then project onto axis direction
                    curr_3d     = self._unproject_screen(mx, my, self._drag_depth)
                    world_delta = curr_3d - self._drag_start_3d
                    proj_dist   = float(np.dot(world_delta, ax_d))
                    new_ee = self._drag_start_ee + ax_d * proj_dist
            else:
                # Sphere: free drag on camera-facing depth plane
                curr_3d     = self._unproject_screen(mx, my, self._drag_depth)
                world_delta = curr_3d - self._drag_start_3d
                if event.modifiers() & Qt.ShiftModifier:
                    proj_z = float(np.dot(world_delta, np.array([0., 0., 1.])))
                    new_ee = self._drag_start_ee + np.array([0., 0., 1.]) * proj_z
                else:
                    new_ee = self._drag_start_ee + world_delta

            result = inverse_kinematics(
                float(new_ee[0]), float(new_ee[1]), float(new_ee[2]),
                float(self._joints[3]), current_joints=self._joints)
            if result is not None:
                self._joints = result
                self.joints_changed.emit(self._joints.tolist())
                self.ee_dragged.emit(*self._ee_pos().tolist())

        elif self._mouse_mode == 'orbit' or (
                event.buttons() & Qt.LeftButton and not alt
                and self._mouse_mode not in ('pan', 'gizmo')):
            self._cam_azim -= dx * 0.4
            self._cam_elev += dy * 0.3
            self._cam_elev  = max(-89.0, min(89.0, self._cam_elev))

        elif self._mouse_mode == 'pan' or (
                event.buttons() & Qt.MiddleButton) or alt:
            right = np.array([ np.sin(np.deg2rad(self._cam_azim)),
                               -np.cos(np.deg2rad(self._cam_azim)), 0.0])
            scale = self._cam_dist * 0.001
            self._cam_focus += (-dx * scale * right
                                + dy * scale * np.array([0., 0., 1.]))
        else:
            # Hover highlight
            new_h = self._pick_axis(mx, my)
            if new_h != self._hover_axis:
                self._hover_axis = new_h

        self._last_pos = event.pos()
        self.update()

    def mouseReleaseEvent(self, event):
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
