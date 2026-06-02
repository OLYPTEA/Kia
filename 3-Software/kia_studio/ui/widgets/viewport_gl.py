"""High-fidelity moderngl viewport — studio look with soft shadows.

Parallel to viewport3d.py (pyqtgraph) during the engine upgrade; reaches feature parity
phase by phase. Z-up world (matches the kinematics). Current phases:
  P2 studio-lit per-link STL arm + light gradient background
  P3 ground plane + soft shadow (shadow map + PCF) + discrete grid
"""
from __future__ import annotations

import os

import numpy as np
from PySide6.QtGui import QSurfaceFormat
from PySide6.QtOpenGLWidgets import QOpenGLWidget

import moderngl

from PySide6.QtCore import Qt, Signal

from ...core.kinematics import (Geometry, fk_frames, chain_transforms, chain_joint_points,
                                DEFAULT_PIVOTS)
from ...core.mesh_loader import load_stl
from ..mesh_config import LINK_META, LINK_ORDER, calib_matrix, hinge_matrix, load_calibration

_MESH_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "resources", "meshes")

_LINK_ALBEDO = {
    "bati":     (0.74, 0.75, 0.78),
    "base":     (0.58, 0.64, 0.72),
    "shoulder": (0.50, 0.66, 0.68),
    "elbow":    (0.56, 0.62, 0.74),
    "wrist":    (0.62, 0.58, 0.72),
    "grip1":    (0.70, 0.72, 0.78),
    "grip2":    (0.68, 0.70, 0.77),
}
_BG_TOP = (0.93, 0.94, 0.96)
_BG_BOTTOM = (0.82, 0.84, 0.88)
_GROUND = (0.86, 0.87, 0.90)
_SHADOW_SIZE = 2048
_LIGHT_DIR = np.array([0.45, 0.5, 1.05], "f4")     # surface -> light (Z-up)

_JOINT_COLOR = (0.20, 0.42, 0.62)
_LINK_COLOR = (0.32, 0.40, 0.50)
_TCP_COLOR = (0.92, 0.42, 0.18)
_AXIS_RGB = ((0.86, 0.25, 0.25), (0.30, 0.72, 0.32), (0.30, 0.45, 0.92))
_PIVOT_PARENT = {"base": "bati", "shoulder": "base", "elbow": "shoulder", "wrist": "elbow"}
_PIVOT_LINKS = ("base", "shoulder", "elbow", "wrist")
_GIZMO_LEN = 50.0
_GIZMO_PICK_PX = 22.0

_FLAT_VERT = """
#version 330
uniform mat4 mvp; uniform float psize;
in vec3 in_pos;
void main(){ gl_Position = mvp * vec4(in_pos,1.0); gl_PointSize = psize; }
"""
_FLAT_FRAG = "#version 330\nuniform vec3 color; out vec4 f; void main(){ f = vec4(color,1.0); }"


def ensure_gl_format():
    fmt = QSurfaceFormat()
    fmt.setVersion(3, 3)
    fmt.setProfile(QSurfaceFormat.CoreProfile)
    fmt.setDepthBufferSize(24)
    fmt.setSamples(4)
    QSurfaceFormat.setDefaultFormat(fmt)


def _perspective(fov, aspect, near, far):
    f = 1.0 / np.tan(np.radians(fov) / 2)
    m = np.zeros((4, 4), "f4")
    m[0, 0] = f / aspect; m[1, 1] = f
    m[2, 2] = (far + near) / (near - far); m[2, 3] = (2 * far * near) / (near - far)
    m[3, 2] = -1.0
    return m


def _ortho(half, near, far):
    m = np.eye(4, dtype="f4")
    m[0, 0] = 1.0 / half; m[1, 1] = 1.0 / half
    m[2, 2] = -2.0 / (far - near); m[2, 3] = -(far + near) / (far - near)
    return m


def _look_at(eye, tgt, up):
    eye = np.asarray(eye, "f4"); tgt = np.asarray(tgt, "f4"); up = np.asarray(up, "f4")
    f = tgt - eye; f /= (np.linalg.norm(f) or 1.0)
    s = np.cross(f, up); s /= (np.linalg.norm(s) or 1.0)
    u = np.cross(s, f)
    m = np.eye(4, dtype="f4")
    m[0, :3] = s; m[1, :3] = u; m[2, :3] = -f
    m[:3, 3] = -m[:3, :3] @ eye
    return m


def _vertex_normals(verts, faces):
    norm = np.zeros_like(verts)
    tris = verts[faces]
    fn = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    for k in range(3):
        np.add.at(norm, faces[:, k], fn)
    lens = np.linalg.norm(norm, axis=1, keepdims=True)
    lens[lens == 0] = 1.0
    return (norm / lens).astype("f4")


_VERT = """
#version 330
uniform mat4 mvp; uniform mat4 model;
in vec3 in_pos; in vec3 in_norm;
out vec3 v_norm; out vec3 v_world;
void main(){
    v_norm = normalize(mat3(model) * in_norm);
    v_world = vec3(model * vec4(in_pos, 1.0));
    gl_Position = mvp * model * vec4(in_pos, 1.0);
}
"""

# shared lighting + PCF shadow helpers (Z-up)
_LIGHT_GLSL = """
uniform vec3 light_dir; uniform vec3 cam_pos;
uniform mat4 light_vp; uniform sampler2D shadow_map;
float shadow_factor(vec3 world, float ndl){
    vec4 lc = light_vp * vec4(world, 1.0);
    vec3 p = lc.xyz / lc.w * 0.5 + 0.5;
    if (p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0||p.z>1.0) return 1.0;
    float bias = max(0.0015 * (1.0 - ndl), 0.0004);
    float sh = 0.0; vec2 tx = 1.0 / vec2(textureSize(shadow_map, 0));
    for (int x=-2;x<=2;x++) for (int y=-2;y<=2;y++){
        float d = texture(shadow_map, p.xy + vec2(x,y)*tx).r;
        sh += (p.z - bias > d) ? 0.0 : 1.0;
    }
    return sh / 25.0;
}
"""

_FRAG = "#version 330\nin vec3 v_norm; in vec3 v_world; uniform vec3 albedo;\nout vec4 f_color;\n" + _LIGHT_GLSL + """
vec3 lobe(vec3 N, vec3 V, vec3 L, vec3 col, float spec){
    float d = max(dot(N,L),0.0); vec3 H = normalize(L+V);
    return col * (d + pow(max(dot(N,H),0.0),32.0)*spec);
}
void main(){
    vec3 N = normalize(v_norm); vec3 V = normalize(cam_pos - v_world);
    vec3 L = normalize(light_dir);
    float hemi = 0.5 + 0.5*N.z;
    vec3 amb = mix(vec3(0.30,0.31,0.34), vec3(0.47,0.48,0.51), hemi);
    float ndl = max(dot(N,L),0.0);
    float sh = shadow_factor(v_world, ndl);
    vec3 c = albedo*amb;
    c += albedo*lobe(N,V,L,vec3(0.95),0.25)*sh;                       // key (shadowed)
    c += albedo*lobe(N,V,normalize(vec3(-0.7,0.3,0.35)),vec3(0.22),0.0);  // fill
    c += vec3(pow(1.0-max(dot(N,V),0.0),3.0)*0.10);                   // edge sheen
    c = pow(clamp(c,0.0,1.0), vec3(1.0/2.2));
    f_color = vec4(c,1.0);
}
"""

_DEPTH_VERT = """
#version 330
uniform mat4 light_vp; uniform mat4 model;
in vec3 in_pos;
void main(){ gl_Position = light_vp * model * vec4(in_pos,1.0); }
"""
_DEPTH_FRAG = "#version 330\nvoid main(){}"

# ground: studio plane + faded grid + receives shadow
_GROUND_VERT = """
#version 330
uniform mat4 mvp; in vec3 in_pos; out vec3 v_world;
void main(){ v_world = in_pos; gl_Position = mvp * vec4(in_pos,1.0); }
"""
_GROUND_FRAG = "#version 330\nin vec3 v_world; uniform vec3 base; uniform vec3 center; uniform float radius;\nout vec4 f_color;\n" + _LIGHT_GLSL + """
float grid(vec2 p, float step){
    vec2 g = abs(fract(p/step - 0.5) - 0.5) / fwidth(p/step);
    return 1.0 - min(min(g.x,g.y),1.0);
}
void main(){
    float ndl = 1.0;
    float sh = shadow_factor(v_world + vec3(0,0,0.05), ndl);
    vec3 col = base;
    float gl = grid(v_world.xy, 20.0)*0.10 + grid(v_world.xy, 100.0)*0.16;   // fine + coarse
    col = mix(col, col*0.78, gl);
    float fade = clamp(1.0 - length(v_world.xy - center.xy)/(radius*3.0), 0.0, 1.0);
    col = mix(_BG, col, fade);
    col *= mix(0.55, 1.0, sh);                       // soft contact shadow
    f_color = vec4(col, 1.0);
}
""".replace("_BG", "vec3(0.86,0.875,0.905)")


class GLViewport(QOpenGLWidget):
    pivotMoved = Signal(str)   # emitted while dragging a pivot gizmo (link name)

    def __init__(self, geo: Geometry | None = None, parent=None):
        super().__init__(parent)
        self.geo = geo or Geometry()
        self.calib = load_calibration()
        self._pose = (0.0, 90.0, 0.0, 0.0)
        self._grip = 0.0
        self._ignore_telemetry = False
        self.azimuth, self.elevation, self.distance = -55.0, 22.0, 1.0
        self.target = np.zeros(3, "f4")
        self.scene_radius = 150.0
        self._home_frames = self._compute_home_frames()
        self._meshes: dict[str, dict] = {}
        self._gpu: dict[str, dict] = {}
        self._chain = chain_transforms(0.0, 90.0, 0.0, 0.0, self._pivots())
        self._last_mouse = None
        # overlays (skeleton + pivot gizmo) — shown in calibration mode
        self._show_skeleton = False
        self._pivot_link = None
        self._giz_center = np.zeros(3); self._giz_dirs = np.eye(3); self._giz_handles = np.zeros((3, 3))
        self._drag_axis = None; self._drag_pv0 = np.zeros(3); self._drag_s0 = 0.0
        self._load_meshes()
        self._frame_camera()

    def _pivots(self):
        return {n: (self.calib[n].get("px", DEFAULT_PIVOTS[n][0]),
                    self.calib[n].get("py", DEFAULT_PIVOTS[n][1]),
                    self.calib[n].get("pz", DEFAULT_PIVOTS[n][2])) for n in DEFAULT_PIVOTS}

    def _model(self, name):
        return (self._chain[name] @ self._home_frames[name]
                @ hinge_matrix(self.calib[name], self._grip)
                @ calib_matrix(self._meshes[name]["center"], self.calib[name]))

    def _tcp_home(self):
        """Tool tip in world-home coords: centroid of the gripper jaws, else wrist pivot."""
        cs = []
        for n in ("grip1", "grip2"):
            if n in self._meshes:
                c = self._meshes[n]["center"]
                P = self._home_frames[n] @ calib_matrix(c, self.calib[n])
                cs.append((P @ np.append(c, 1.0))[:3])
        if cs:
            return np.mean(cs, axis=0)
        return np.asarray(self._pivots()["wrist"], float)

    def _compute_home_frames(self):
        hf = fk_frames(0.0, 90.0, 0.0, 0.0, self.geo)
        return {"bati": np.eye(4), "base": hf[0], "shoulder": hf[1], "elbow": hf[2],
                "wrist": hf[3], "grip1": hf[3], "grip2": hf[3]}

    def _load_meshes(self):
        for name in LINK_ORDER:
            path = os.path.join(_MESH_DIR, LINK_META[name]["file"])
            if not os.path.exists(path):
                continue
            m = load_stl(path)
            self._meshes[name] = dict(verts=m.vertices.astype("f4"),
                                      norms=_vertex_normals(m.vertices, m.faces),
                                      faces=m.faces.astype("i4"), center=m.center)

    def _frame_camera(self):
        pts = []
        for name, m in self._meshes.items():
            P = self._model(name)
            pts.append((P @ np.c_[m["verts"], np.ones(len(m["verts"]))].T).T[:, :3])
        if pts:
            allv = np.vstack(pts)
            lo, hi = allv.min(0), allv.max(0)
            self.target = (((lo + hi) / 2)).astype("f4")
            self.target[2] = float((lo[2] + hi[2]) / 2)
            self.scene_radius = float(np.linalg.norm(hi - lo) / 2)
            self.distance = self.scene_radius * 2.6

    def _light_vp(self):
        c = self.target.copy()
        L = _LIGHT_DIR / np.linalg.norm(_LIGHT_DIR)
        eye = c + L * self.scene_radius * 3.0
        up = np.array([0, 1, 0], "f4") if abs(L[2]) > 0.95 else np.array([0, 0, 1], "f4")
        view = _look_at(eye, c, up)
        proj = _ortho(self.scene_radius * 1.6, 0.1, self.scene_radius * 6.0)
        return (proj @ view).astype("f4")

    def _ground_quad(self):
        r = self.scene_radius * 4.0
        cx, cy = float(self.target[0]), float(self.target[1])
        z = 0.0
        q = np.array([[cx-r, cy-r, z], [cx+r, cy-r, z], [cx+r, cy+r, z],
                      [cx-r, cy-r, z], [cx+r, cy+r, z], [cx-r, cy+r, z]], "f4")
        return q

    # ---- GL ----
    def initializeGL(self):
        self.ctx = moderngl.create_context()
        self.prog = self.ctx.program(vertex_shader=_VERT, fragment_shader=_FRAG)
        self.depth_prog = self.ctx.program(vertex_shader=_DEPTH_VERT, fragment_shader=_DEPTH_FRAG)
        self.ground_prog = self.ctx.program(vertex_shader=_GROUND_VERT, fragment_shader=_GROUND_FRAG)
        self.bg_prog = self.ctx.program(
            vertex_shader="#version 330\nin vec2 p; out vec2 uv; void main(){uv=p*0.5+0.5; gl_Position=vec4(p,0.999,1.0);}",
            fragment_shader="#version 330\nin vec2 uv; out vec4 c; uniform vec3 top; uniform vec3 bot;"
                            "void main(){ c=vec4(mix(bot,top,uv.y),1.0);} ")
        self.bg_vao = self.ctx.vertex_array(
            self.bg_prog, [(self.ctx.buffer(np.array([-1,-1,3,-1,-1,3],"f4").tobytes()), "2f", "p")])

        # shadow map
        self.shadow_tex = self.ctx.depth_texture((_SHADOW_SIZE, _SHADOW_SIZE))
        self.shadow_tex.repeat_x = self.shadow_tex.repeat_y = False
        self.shadow_fbo = self.ctx.framebuffer(depth_attachment=self.shadow_tex)

        # ground
        gq = self._ground_quad()
        self.ground_vao = self.ctx.vertex_array(
            self.ground_prog, [(self.ctx.buffer(gq.tobytes()), "3f", "in_pos")])

        # overlays: skeleton + pivot gizmo (flat lines/points)
        self.ctx.enable(moderngl.PROGRAM_POINT_SIZE)
        self.flat_prog = self.ctx.program(vertex_shader=_FLAT_VERT, fragment_shader=_FLAT_FRAG)
        self._skel_buf = self.ctx.buffer(reserve=5 * 3 * 4, dynamic=True)
        self._skel_vao = self.ctx.vertex_array(self.flat_prog, [(self._skel_buf, "3f", "in_pos")])
        self._giz_axis_buf = self.ctx.buffer(reserve=6 * 3 * 4, dynamic=True)
        self._giz_axis_vao = self.ctx.vertex_array(self.flat_prog, [(self._giz_axis_buf, "3f", "in_pos")])
        self._giz_hdl_buf = self.ctx.buffer(reserve=3 * 3 * 4, dynamic=True)
        self._giz_hdl_vao = self.ctx.vertex_array(self.flat_prog, [(self._giz_hdl_buf, "3f", "in_pos")])

        # meshes (position + normal, indexed)
        for name, m in self._meshes.items():
            inter = np.hstack([m["verts"], m["norms"]]).astype("f4")
            vbo = self.ctx.buffer(inter.tobytes())
            ibo = self.ctx.buffer(m["faces"].tobytes())
            depth_vao = self.ctx.vertex_array(self.depth_prog, [(vbo, "3f 3x4", "in_pos")], ibo)
            main_vao = self.ctx.vertex_array(self.prog, [(vbo, "3f 3f", "in_pos", "in_norm")], ibo)
            self._gpu[name] = dict(main=main_vao, depth=depth_vao)

    def _cam_eye(self):
        az, el = np.radians(self.azimuth), np.radians(self.elevation)
        d = self.distance
        return self.target + np.array(
            [d*np.cos(el)*np.cos(az), d*np.cos(el)*np.sin(az), d*np.sin(el)], "f4")

    def paintGL(self):
        light_vp = self._light_vp()
        # --- shadow depth pass ---
        self.shadow_fbo.use()
        self.shadow_fbo.clear(depth=1.0)
        self.ctx.enable(moderngl.DEPTH_TEST)
        self.depth_prog["light_vp"].write(light_vp.T.tobytes())
        for name, g in self._gpu.items():
            self.depth_prog["model"].write(self._model(name).T.astype("f4").tobytes())
            g["depth"].render(moderngl.TRIANGLES)

        # --- main pass ---
        fbo = self.ctx.detect_framebuffer(self.defaultFramebufferObject())
        fbo.use()
        dpr = self.devicePixelRatio()
        w, h = max(self.width(), 1), max(self.height(), 1)
        self.ctx.viewport = (0, 0, int(w*dpr), int(h*dpr))
        self.ctx.clear(*_BG_BOTTOM, depth=1.0)
        self.ctx.disable(moderngl.DEPTH_TEST)
        self.bg_prog["top"].value = _BG_TOP; self.bg_prog["bot"].value = _BG_BOTTOM
        self.bg_vao.render(moderngl.TRIANGLES)

        self.ctx.enable(moderngl.DEPTH_TEST)
        eye = self._cam_eye()
        view = _look_at(eye, self.target, (0, 0, 1))
        proj = _perspective(40.0, w / h, self.distance*0.02, self.distance*6 + 2000)
        vp = (proj @ view).T.astype("f4").tobytes()
        lvp = light_vp.T.tobytes()
        self.shadow_tex.use(0)

        # ground
        self.ground_prog["mvp"].write(vp)
        self.ground_prog["light_vp"].write(lvp)
        self.ground_prog["shadow_map"].value = 0
        self.ground_prog["base"].value = _GROUND
        self.ground_prog["center"].value = tuple(float(x) for x in self.target)
        self.ground_prog["radius"].value = self.scene_radius
        self.ground_vao.render(moderngl.TRIANGLES)

        # meshes
        self.prog["mvp"].write(vp)
        self.prog["light_vp"].write(lvp)
        self.prog["shadow_map"].value = 0
        self.prog["light_dir"].value = tuple(float(x) for x in _LIGHT_DIR)
        self.prog["cam_pos"].value = tuple(float(x) for x in eye)
        for name, g in self._gpu.items():
            self.prog["model"].write(self._model(name).T.astype("f4").tobytes())
            self.prog["albedo"].value = _LINK_ALBEDO.get(name, (0.7, 0.7, 0.7))
            g["main"].render(moderngl.TRIANGLES)

        # overlays (calibration): skeleton integrated with depth, gizmo always on top
        if self._show_skeleton or self._pivot_link:
            self.flat_prog["mvp"].write(vp)
            if self._show_skeleton:
                self._draw_skeleton()
            if self._pivot_link in _PIVOT_LINKS:
                self.ctx.disable(moderngl.DEPTH_TEST)
                self._draw_gizmo()

    def _draw_skeleton(self):
        pts = np.array(chain_joint_points(*self._pose, self._pivots()), "f4")
        self._skel_buf.write(pts.tobytes())
        self.flat_prog["psize"].value = 1.0
        self.flat_prog["color"].value = _LINK_COLOR
        self._skel_vao.render(moderngl.LINE_STRIP, vertices=5)
        self.flat_prog["psize"].value = 11.0
        self.flat_prog["color"].value = _JOINT_COLOR
        self._skel_vao.render(moderngl.POINTS, vertices=4)
        self.flat_prog["psize"].value = 14.0
        self.flat_prog["color"].value = _TCP_COLOR
        self._skel_vao.render(moderngl.POINTS, vertices=1, first=4)

    def _draw_gizmo(self):
        self._refresh_gizmo()
        seg = np.empty((6, 3), "f4")
        for i in range(3):
            seg[2 * i] = self._giz_center; seg[2 * i + 1] = self._giz_handles[i]
        self._giz_axis_buf.write(seg.tobytes())
        self.flat_prog["psize"].value = 1.0
        for i in range(3):
            self.flat_prog["color"].value = _AXIS_RGB[i]
            self._giz_axis_vao.render(moderngl.LINES, vertices=2, first=2 * i)
        self._giz_hdl_buf.write(self._giz_handles.astype("f4").tobytes())
        self.flat_prog["psize"].value = 15.0
        for i in range(3):
            self.flat_prog["color"].value = _AXIS_RGB[i]
            self._giz_hdl_vao.render(moderngl.POINTS, vertices=1, first=i)

    def _refresh_gizmo(self):
        Mp = self._chain[_PIVOT_PARENT[self._pivot_link]]
        pv = np.asarray(self._pivots()[self._pivot_link], float)
        self._giz_center = (Mp @ np.append(pv, 1.0))[:3]
        dirs = Mp[:3, :3].T
        self._giz_dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
        self._giz_handles = self._giz_center + _GIZMO_LEN * self._giz_dirs

    # ---- calibration API (parity with pyqtgraph viewport) ----
    def set_calib_frozen(self, frozen: bool):
        self._ignore_telemetry = frozen
        self._show_skeleton = frozen
        if frozen:
            self.set_pose(0.0, 90.0, 0.0, 0.0, 0.0)
        self.update()

    def show_pivot_marker(self, name):
        self._pivot_link = name if name in _PIVOT_LINKS else None
        self.update()

    def highlight_link(self, name):
        pass   # albedo dimming not used in the studio renderer

    # ---- 3D picking / drag for the pivot gizmo ----
    def _vp(self):
        w, h = max(self.width(), 1), max(self.height(), 1)
        view = _look_at(self._cam_eye(), self.target, (0, 0, 1))
        proj = _perspective(40.0, w / h, self.distance * 0.02, self.distance * 6 + 2000)
        return proj @ view

    def _project(self, M, p):
        clip = M @ np.array([p[0], p[1], p[2], 1.0])
        if clip[3] <= 1e-6:
            return None
        ndc = clip[:3] / clip[3]
        return ((ndc[0] + 1) * 0.5 * self.width(), (1 - ndc[1]) * 0.5 * self.height())

    def _ray_from_pixel(self, x, y):
        inv = np.linalg.inv(self._vp())
        nx, ny = 2 * x / self.width() - 1, 1 - 2 * y / self.height()
        near = inv @ np.array([nx, ny, -1.0, 1.0]); near = near[:3] / near[3]
        far = inv @ np.array([nx, ny, 1.0, 1.0]); far = far[:3] / far[3]
        d = far - near
        return near, d / (np.linalg.norm(d) or 1.0)

    def _axis_param_at_ray(self, axis_i, x, y):
        o, r = self._ray_from_pixel(x, y)
        u = self._giz_dirs[axis_i]
        w0 = self._giz_center - o
        b = float(u @ r)
        denom = 1.0 - b * b
        if abs(denom) < 1e-9:
            return self._drag_s0
        return float((b * (r @ w0) - (u @ w0)) / denom)

    def _pick_axis(self, x, y):
        M = self._vp()
        best, best_d = None, _GIZMO_PICK_PX
        for i in range(3):
            s = self._project(M, self._giz_handles[i])
            if s is None:
                continue
            d = ((s[0] - x) ** 2 + (s[1] - y) ** 2) ** 0.5
            if d < best_d:
                best, best_d = i, d
        return best

    def set_pose(self, base, shoulder, elbow, wrist, grip=None):
        self._pose = (base, shoulder, elbow, wrist)
        if grip is not None:
            self._grip = grip
        self._chain = chain_transforms(base, shoulder, elbow, wrist, self._pivots())
        self.update()

    def update_from_status(self, st):
        if self._ignore_telemetry:
            return
        j = st.joints
        self.set_pose(j[0], j[1], j[2], j[3], st.grip)

    def set_link_calib(self, name, cfg):
        """Replace one link's calibration and re-apply (parity with pyqtgraph viewport)."""
        self.calib[name].update(cfg)
        self.set_pose(*self._pose)

    def set_skeleton_visible(self, on: bool):
        self._show_skeleton = on; self.update()

    def set_meshes_visible(self, on: bool):
        pass

    def set_telemetry_frozen(self, frozen: bool):
        self._ignore_telemetry = frozen

    # ---- camera interaction ----
    def _cam_basis(self):
        eye = self._cam_eye()
        f = self.target - eye; f /= (np.linalg.norm(f) or 1.0)
        r = np.cross(f, np.array([0, 0, 1], "f4")); r /= (np.linalg.norm(r) or 1.0)
        u = np.cross(r, f)
        return r, u

    def mousePressEvent(self, ev):
        if self._pivot_link in _PIVOT_LINKS and ev.button() == Qt.LeftButton:
            p = ev.position()
            axis = self._pick_axis(p.x(), p.y())
            if axis is not None:
                cfg = self.calib[self._pivot_link]
                self._drag_axis = axis
                self._drag_pv0 = np.array([cfg["px"], cfg["py"], cfg["pz"]], float)
                self._drag_s0 = self._axis_param_at_ray(axis, p.x(), p.y())
                return
        self._last_mouse = ev.position()
        self._btn = ev.button()

    def mouseMoveEvent(self, ev):
        if self._drag_axis is not None:
            p = ev.position()
            s = self._axis_param_at_ray(self._drag_axis, p.x(), p.y())
            pv = self._drag_pv0.copy()
            pv[self._drag_axis] = self._drag_pv0[self._drag_axis] + (s - self._drag_s0)
            cfg = self.calib[self._pivot_link]
            cfg["px"], cfg["py"], cfg["pz"] = float(pv[0]), float(pv[1]), float(pv[2])
            self.set_pose(*self._pose)        # recompute chain with the new pivot
            self.pivotMoved.emit(self._pivot_link)
            return
        if self._last_mouse is None:
            return
        p = ev.position()
        dx, dy = p.x() - self._last_mouse.x(), p.y() - self._last_mouse.y()
        self._last_mouse = p
        pan = (ev.buttons() & Qt.RightButton) or (ev.modifiers() & Qt.ShiftModifier)
        if pan:
            r, u = self._cam_basis()
            k = self.distance * 0.0016
            self.target = self.target - r * (dx * k) + u * (dy * k)
        else:
            self.azimuth -= dx * 0.4
            self.elevation = float(np.clip(self.elevation + dy * 0.4, -89.0, 89.0))
        self.update()

    def mouseReleaseEvent(self, ev):
        self._drag_axis = None
        self._last_mouse = None

    def wheelEvent(self, ev):
        d = ev.angleDelta().y()
        self.distance *= (0.85 if d > 0 else 1.18)
        self.distance = float(np.clip(self.distance, self.scene_radius * 0.4, self.scene_radius * 12))
        self.update()
