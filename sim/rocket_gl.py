import math
import numpy as np
from collections import deque

from PyQt5.QtWidgets import QOpenGLWidget, QLabel, QHBoxLayout, QWidget
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QColor, QFont, QSurfaceFormat

from OpenGL.GL import *
from OpenGL.GLU import *

from model_loader import RocketModel, SURFACE_CHANNELS, SURFACE_COLORS


class RocketGLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        fmt = QSurfaceFormat()
        fmt.setDepthBufferSize(24)
        fmt.setSamples(4)
        fmt.setSwapInterval(1)
        QSurfaceFormat.setDefaultFormat(fmt)
        super().__init__(parent)
        self.setMinimumSize(400, 400)

        self.rocket_model: RocketModel | None = None
        self.servo_angles = [90.0] * 8
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.altitude = 0.0

        self.cam_distance = 8.0
        self.cam_azimuth = 45.0
        self.cam_elevation = 20.0
        self.cam_target = np.array([0.0, 0.0, 0.0])

        self._last_mouse_pos = None
        self._mouse_button = None

        self.trail_positions = deque(maxlen=500)

        self._body_dl = None
        self._surface_dls = {}
        self._needs_rebuild = True

        self.overlay_text = ""

    def set_rocket_model(self, model: RocketModel):
        self.rocket_model = model
        self._needs_rebuild = True
        self.update()

    def update_rocket(self, qx, qy, qz, qw, altitude, servo_angles=None):
        self.quaternion = np.array([qx, qy, qz, qw])
        self.altitude = altitude
        if servo_angles is not None:
            self.servo_angles = list(servo_angles)
        self.trail_positions.append(altitude)
        self.overlay_text = f"Alt: {altitude:.1f} m"
        self.update()

    def _rebuild_display_lists(self):
        if self.rocket_model is None or not self.rocket_model.valid:
            self._body_dl = None
            self._surface_dls = {}
            self._needs_rebuild = False
            return

        body_parts = self.rocket_model.get_body_parts()
        if body_parts:
            self._body_dl = glGenLists(1)
            glNewList(self._body_dl, GL_COMPILE)
            for name, mesh in body_parts.items():
                self._draw_mesh(mesh, (0.7, 0.7, 0.75, 1.0))
            glEndList()
        else:
            self._body_dl = None

        self._surface_dls = {}
        for ch_id, sa in self.rocket_model.surface_assignments.items():
            if sa.part_name and sa.part_name in self.rocket_model.all_meshes:
                dl = glGenLists(1)
                glNewList(dl, GL_COMPILE)
                color = SURFACE_COLORS[ch_id] if ch_id < len(SURFACE_COLORS) else (1, 1, 0)
                self._draw_mesh(self.rocket_model.all_meshes[sa.part_name],
                                (color[0], color[1], color[2], 1.0))
                glEndList()
                self._surface_dls[ch_id] = dl

        self._needs_rebuild = False

    def _draw_mesh(self, mesh, color):
        if not hasattr(mesh, 'faces') or len(mesh.faces) == 0:
            if hasattr(mesh, 'vertices'):
                glVertexPointer(3, GL_FLOAT, 0, mesh.vertices.astype(np.float32))
                glEnableClientState(GL_VERTEX_ARRAY)
                glColor4f(*color)
                glDrawArrays(GL_POINTS, 0, len(mesh.vertices))
                glDisableClientState(GL_VERTEX_ARRAY)
            return

        verts = mesh.vertices.astype(np.float32)
        faces = mesh.faces.astype(np.uint32)

        normals = mesh.vertex_normals.astype(np.float32) if hasattr(mesh, 'vertex_normals') and len(mesh.vertex_normals) > 0 else None

        scale = self.rocket_model.model_scale if self.rocket_model else 1.0
        center = self.rocket_model.model_center if self.rocket_model else np.zeros(3)

        glColor4f(*color)
        glBegin(GL_TRIANGLES)
        for face in faces:
            if normals is not None:
                for idx in face:
                    if idx < len(normals):
                        glNormal3fv(normals[idx])
                    v = verts[idx]
                    glVertex3f((v[0] - center[0]) * scale,
                               (v[1] - center[1]) * scale,
                               (v[2] - center[2]) * scale)
            else:
                for idx in face:
                    v = verts[idx]
                    glVertex3f((v[0] - center[0]) * scale,
                               (v[1] - center[1]) * scale,
                               (v[2] - center[2]) * scale)
        glEnd()

    def initializeGL(self):
        glClearColor(0.086, 0.102, 0.18, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_LIGHT1)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glLightfv(GL_LIGHT0, GL_POSITION, [5.0, 5.0, 10.0, 0.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.25, 1.0])

        glLightfv(GL_LIGHT1, GL_POSITION, [-5.0, -3.0, 5.0, 0.0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.3, 0.3, 0.35, 1.0])

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / max(h, 1)
        gluPerspective(45.0, aspect, 0.1, 5000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        cam_x = self.cam_target[0] + self.cam_distance * math.cos(math.radians(self.cam_elevation)) * math.cos(math.radians(self.cam_azimuth))
        cam_y = self.cam_target[1] + self.cam_distance * math.cos(math.radians(self.cam_elevation)) * math.sin(math.radians(self.cam_azimuth))
        cam_z = self.cam_target[2] + self.cam_distance * math.sin(math.radians(self.cam_elevation))

        gluLookAt(cam_x, cam_y, cam_z,
                   self.cam_target[0], self.cam_target[1], self.cam_target[2],
                   0, 0, 1)

        self._draw_ground_grid()

        if self.rocket_model and self.rocket_model.valid:
            if self._needs_rebuild:
                self._rebuild_display_lists()
            self._draw_rocket_model()
        else:
            self._draw_procedural_rocket()

        self._draw_trail()

        painter = QPainter(self)
        painter.setRenderHint(QPainter.TextAntialiasing)
        self._draw_overlay(painter)
        painter.end()

    def _draw_ground_grid(self):
        glDisable(GL_LIGHTING)
        glBegin(GL_LINES)
        glColor4f(0.2, 0.2, 0.3, 0.5)
        grid_size = 50
        step = 5
        for i in range(-grid_size, grid_size + 1, step):
            glVertex3f(i, -grid_size, 0)
            glVertex3f(i, grid_size, 0)
            glVertex3f(-grid_size, i, 0)
            glVertex3f(grid_size, i, 0)
        glEnd()
        glEnable(GL_LIGHTING)

    def _quaternion_to_rotation_matrix(self, q):
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]
        m = [0.0] * 16
        m[0] = 1 - 2*(qy*qy + qz*qz)
        m[1] = 2*(qx*qy + qw*qz)
        m[2] = 2*(qx*qz - qw*qy)
        m[3] = 0
        m[4] = 2*(qx*qy - qw*qz)
        m[5] = 1 - 2*(qx*qx + qz*qz)
        m[6] = 2*(qy*qz + qw*qx)
        m[7] = 0
        m[8] = 2*(qx*qz + qw*qy)
        m[9] = 2*(qy*qz - qw*qx)
        m[10] = 1 - 2*(qx*qx + qy*qy)
        m[11] = 0
        m[12] = 0
        m[13] = 0
        m[14] = 0
        m[15] = 1
        return m

    def _draw_procedural_rocket(self):
        glPushMatrix()

        alt_scaled = self.altitude * 0.1
        glTranslatef(0, 0, alt_scaled)

        rot = self._quaternion_to_rotation_matrix(self.quaternion)
        glMultMatrixf(self._ned_to_gl_rot(rot))

        body_len = 2.0
        body_r = 0.1
        glColor4f(0.85, 0.85, 0.9, 1.0)
        self._draw_cylinder(0, 0, 0, body_r, body_len)
        glColor4f(0.9, 0.3, 0.3, 1.0)
        self._draw_cone(0, 0, body_len, body_r, body_r * 2.5, body_len + body_r * 3)

        fin_colors_gl = SURFACE_COLORS
        fin_offsets = [
            (0, body_r, 0),
            (0, -body_r, 0),
            (body_r, 0, 0),
            (-body_r, 0, 0),
            (0, body_r, 0),
            (0, -body_r, 0),
            (body_r, 0, 0),
            (-body_r, 0, 0),
        ]

        for i in range(8):
            c = fin_colors_gl[i] if i < len(fin_colors_gl) else (1, 1, 0)
            glColor4f(c[0], c[1], c[2], 1.0)
            defl_deg = self.servo_angles[i] - 90.0 if i < len(self.servo_angles) else 0.0

            glPushMatrix()
            ox, oy, oz = fin_offsets[i]
            z_base = -body_len * 0.2 if i >= 4 else body_len * 0.3
            glTranslatef(ox * 2, oy * 2, z_base)

            if i < 4:
                glRotatef(defl_deg, 1, 0, 0)
            else:
                glRotatef(defl_deg, 1, 0, 0)

            fin_len = 0.4
            self._draw_fin(fin_len, body_r * 0.5, i < 4)
            glPopMatrix()

        glPopMatrix()

    def _draw_fin(self, length, width, is_canard):
        half = width
        sign = 1.0 if is_canard else -1.0
        glBegin(GL_TRIANGLES)
        glNormal3f(0, 1, 0)
        glVertex3f(-half * 0.3, 0, 0)
        glVertex3f(half * 0.3, 0, 0)
        glVertex3f(0, length, sign * length * 0.3)
        glEnd()
        glBegin(GL_TRIANGLES)
        glNormal3f(0, -1, 0)
        glVertex3f(half * 0.3, 0, 0)
        glVertex3f(-half * 0.3, 0, 0)
        glVertex3f(0, length, sign * length * 0.3)
        glEnd()

    def _draw_cylinder(self, cx, cy, cz, r, length, segments=16):
        glBegin(GL_QUAD_STRIP)
        for i in range(segments + 1):
            a = 2.0 * math.pi * i / segments
            nx, ny = math.cos(a), math.sin(a)
            glNormal3f(nx, ny, 0)
            glVertex3f(cx + r * nx, cy + r * ny, cz)
            glVertex3f(cx + r * nx, cy + r * ny, cz + length)
        glEnd()

    def _draw_cone(self, cx, cy, cz, r_base, r_tip, z_tip, segments=16):
        glBegin(GL_TRIANGLES)
        for i in range(segments):
            a0 = 2.0 * math.pi * i / segments
            a1 = 2.0 * math.pi * (i + 1) / segments
            nx0, ny0 = math.cos(a0), math.sin(a0)
            nx1, ny1 = math.cos(a1), math.sin(a1)
            nm = np.array([nx0 + nx1, ny0 + ny1, r_base / max(z_tip - cz, 0.01)])
            nm = nm / max(np.linalg.norm(nm), 1e-10)
            glNormal3f(*nm)
            glVertex3f(cx + r_base * nx0, cy + r_base * ny0, cz)
            glVertex3f(cx + r_base * nx1, cy + r_base * ny1, cz)
            glVertex3f(cx, cy, z_tip)
        glEnd()

    def _draw_rocket_model(self):
        if self._needs_rebuild:
            self._rebuild_display_lists()

        glPushMatrix()

        alt_scaled = self.altitude * 0.1
        glTranslatef(0, 0, alt_scaled)

        rot = self._quaternion_to_rotation_matrix(self.quaternion)
        glMultMatrixf(self._ned_to_gl_rot(rot))

        if self._body_dl is not None:
            glCallList(self._body_dl)

        for ch_id, dl in self._surface_dls.items():
            if ch_id < len(self.servo_angles):
                defl_deg = self.servo_angles[ch_id] - 90.0
            else:
                defl_deg = 0.0

            if self.rocket_model and ch_id in self.rocket_model.surface_assignments:
                sa = self.rocket_model.surface_assignments[ch_id]
                scale = self.rocket_model.model_scale
                center = self.rocket_model.model_center

                T = self.rocket_model.compute_deflection_transform(ch_id, defl_deg)

                T_scaled = np.eye(4, dtype=np.float64)
                S = np.diag([scale, scale, scale, 1.0])
                C_to = np.eye(4)
                C_to[:3, 3] = -center
                C_back = np.eye(4)
                C_back[:3, 3] = center

                T_scaled = S @ C_back @ T @ C_to @ np.linalg.inv(S)
                T_scaled = T_scaled.astype(np.float32)

                glPushMatrix()
                glMultMatrixf(T_scaled.T.flatten())
                glCallList(dl)
                glPopMatrix()

        glPopMatrix()

    def _ned_to_gl_rot(self, rot_ned):
        swap = [0.0] * 16
        swap[0] = 1; swap[1] = 0; swap[2] = 0; swap[3] = 0
        swap[4] = 0; swap[5] = 0; swap[6] = -1; swap[7] = 0
        swap[8] = 0; swap[9] = 1; swap[10] = 0; swap[11] = 0
        swap[12] = 0; swap[13] = 0; swap[14] = 0; swap[15] = 1
        return swap

    def _draw_trail(self):
        if len(self.trail_positions) < 2:
            return
        glDisable(GL_LIGHTING)
        glBegin(GL_LINE_STRIP)
        n = len(self.trail_positions)
        for i, alt in enumerate(self.trail_positions):
            alpha = (i + 1) / n
            glColor4f(0.0, 0.9, 0.46, alpha * 0.6)
            glVertex3f(0, 0, alt * 0.1)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_overlay(self, painter):
        font = QFont("Monospace", 10)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(220, 220, 220))

        x = 10
        y = 20
        painter.drawText(x, y, self.overlay_text)

        y += 18
        roll, pitch, yaw = self._quaternion_to_euler_deg()
        painter.drawText(x, y, f"R: {roll:+.1f}  P: {pitch:+.1f}  Y: {yaw:+.1f}")

        y += 16
        labels = ["C0", "C1", "C2", "C3", "F4", "F5", "F6", "F7"]
        for i in range(8):
            angle = self.servo_angles[i] if i < len(self.servo_angles) else 90.0
            defl = angle - 90.0
            c = SURFACE_COLORS[i] if i < len(SURFACE_COLORS) else (1, 1, 0)
            painter.setPen(QColor(int(c[0]*255), int(c[1]*255), int(c[2]*255)))
            painter.drawText(x, y, f"{labels[i]}: {defl:+.1f}")
            x += 85
            if (i + 1) % 4 == 0:
                x = 10
                y += 15

    def _quaternion_to_euler_deg(self):
        q = self.quaternion
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def mousePressEvent(self, event):
        self._last_mouse_pos = event.pos()
        self._mouse_button = event.button()

    def mouseMoveEvent(self, event):
        if self._last_mouse_pos is None:
            return
        dx = event.x() - self._last_mouse_pos.x()
        dy = event.y() - self._last_mouse_pos.y()

        if self._mouse_button == Qt.LeftButton:
            self.cam_azimuth += dx * 0.5
            self.cam_elevation = max(-89, min(89, self.cam_elevation + dy * 0.5))
        elif self._mouse_button == Qt.RightButton:
            self.cam_distance = max(1.0, self.cam_distance + dy * 0.05)
        elif self._mouse_button == Qt.MiddleButton:
            self.cam_target[2] += dy * 0.02

        self._last_mouse_pos = event.pos()
        self.update()

    def mouseReleaseEvent(self, event):
        self._last_mouse_pos = None
        self._mouse_button = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        self.cam_distance = max(1.0, self.cam_distance - delta * 0.005)
        self.update()
