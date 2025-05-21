import sys
import threading
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL.arrays import vbo

# Import your simulation
from swarm_server.swarmsim import SwarmSim, create_objects_Point_yaw
from pion.simulator import PointYaw

# Global state
sim = None
positions = None
colors = None
window_width = 800
window_height = 600
limit = 5.0
update_ms = 100

# VBOs
pos_vbo = None
col_vbo = None

# Camera params
cam_distance = 15.0
cam_elev = 30.0
cam_azim = -60.0

# Throttle redraw
last_positions = None
epsilon = 1e-3


def init_simulation(num_objects: int):
    global sim, positions, colors, pos_vbo, col_vbo, last_positions
    params = {
        "kp": np.ones((num_objects, 6)) * 0.2,
        "ki": np.zeros((num_objects, 6)),
        "kd": np.ones((num_objects, 6)),
        "attraction_weight": 1.0,
        "cohesion_weight": 1.0,
        "alignment_weight": 1.0,
        "repulsion_weight": 4.0,
        "unstable_weight": 1.0,
        "noise_weight": 1.0,
        "safety_radius": 0.5,
        "max_acceleration": 3,
        "max_speed": 1,
        "unstable_radius": 1.5,
    }
    points = create_objects_Point_yaw(int(np.sqrt(num_objects - 1)), [-limit, limit], [-limit, limit])
    zpoint = PointYaw(position=np.array([1, 1, 1, 0, 0, 0]))
    sim = SwarmSim(np.hstack([points, [zpoint]]), dt=0.24, logger=True, params=params)
    threading.Thread(target=sim.start_simulation_while, daemon=True).start()

    # initial copy
    states = sim.get_states()
    positions = states[:, :3].copy()
    last_positions = positions.copy()
    colors = np.random.rand(num_objects, 3).astype(np.float32)

    # create VBOs
    pos_vbo = vbo.VBO(positions.astype(np.float32))
    col_vbo = vbo.VBO(colors)


def build_display_lists():
    global grid_lists, cube_list
    grid_lists = {}
    step = 1.0
    # 2D grid display lists
    for view in ('top', 'front', 'side'):
        dl = glGenLists(1)
        glNewList(dl, GL_COMPILE)
        glColor3f(0.8, 0.8, 0.8)
        glBegin(GL_LINES)
        rng = np.arange(-limit, limit + step, step)
        if view == 'top':
            for i in rng:
                glVertex3f(-limit, i, 0); glVertex3f(limit, i, 0)
                glVertex3f(i, -limit, 0); glVertex3f(i, limit, 0)
        elif view == 'front':
            for i in rng:
                glVertex3f(0, -limit, i); glVertex3f(0, limit, i)
                glVertex3f(0, i, -limit); glVertex3f(0, i, limit)
        else:  # side
            for i in rng:
                glVertex3f(-limit, 0, i); glVertex3f(limit, 0, i)
                glVertex3f(i, 0, -limit); glVertex3f(i, 0, limit)
        glEnd()
        glEndList()
        grid_lists[view] = dl
    # 3D cube boundary list
    cube_list = glGenLists(1)
    glNewList(cube_list, GL_COMPILE)
    glLineWidth(2.0)
    glColor3f(0, 0, 0)
    glBegin(GL_LINES)
    coords = [-limit, limit]
    for x in coords:
        for y in coords:
            glVertex3f(x, y, -limit); glVertex3f(x, y, limit)
    for x in coords:
        for z in coords:
            glVertex3f(x, -limit, z); glVertex3f(x, limit, z)
    for y in coords:
        for z in coords:
            glVertex3f(-limit, y, z); glVertex3f(limit, y, z)
    glEnd()
    glLineWidth(1.0)
    glEndList()

def update_vbo(new_pos):
    global pos_vbo
    pos_vbo.set_array(new_pos.astype(np.float32))
    pos_vbo.bind()
    glBufferData(GL_ARRAY_BUFFER, new_pos.nbytes, pos_vbo, GL_DYNAMIC_DRAW)
    pos_vbo.unbind()


def display():
    global positions
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    half_w, half_h = window_width // 2, window_height // 2
    viewports = [
        ('top', 0, half_h, half_w, half_h),
        ('front', half_w, half_h, half_w, half_h),
        ('side', 0, 0, half_w, half_h),
        ('3d', half_w, 0, half_w, half_h),
    ]
    for view, x, y, w, h in viewports:
        glViewport(x, y, w, h)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        if view == '3d':
            gluPerspective(45.0, w/float(h or 1), 0.1, 100.0)
        else:
            glOrtho(-limit, limit, -limit, limit, -limit*2, limit*2)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        # camera
        if view == 'top': gluLookAt(0,0,10, 0,0,0, 0,1,0)
        elif view == 'front': gluLookAt(-10,0,0, 0,0,0, 0,0,1)
        elif view == 'side': gluLookAt(0,-10,0, 0,0,0, 0,0,1)
        else:
            gluLookAt(
                cam_distance*np.cos(np.radians(cam_azim))*np.cos(np.radians(cam_elev)),
                cam_distance*np.sin(np.radians(cam_azim))*np.cos(np.radians(cam_elev)),
                cam_distance*np.sin(np.radians(cam_elev)),
                0,0,0, 0,0,1
            )
        # draw grid & axes with depth test on but offset
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_POLYGON_OFFSET_LINE)
        glPolygonOffset(1.0, 1.0)
        glCallList(grid_lists.get(view, None) or 0)
        draw_axes(view)
        glCallList(cube_list)
        glDisable(GL_POLYGON_OFFSET_LINE)
        # draw points via VBO
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        pos_vbo.bind()
        glVertexPointer(3, GL_FLOAT, 0, pos_vbo)
        col_vbo.bind()
        glColorPointer(3, GL_FLOAT, 0, col_vbo)
        glPointSize(6.0)
        glDrawArrays(GL_POINTS, 0, len(positions))
        pos_vbo.unbind(); col_vbo.unbind()
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
    glutSwapBuffers()


def timer(value):
    global positions, last_positions
    if sim:
        raw = sim.get_states()[:, :3]
        # atomic copy
        new_pos = raw.copy()
        # throttle: redraw only if moved
        if np.linalg.norm(new_pos - last_positions) > epsilon:
            positions = new_pos
            update_vbo(positions)
            glutPostRedisplay()
            last_positions = new_pos
    glutTimerFunc(update_ms, timer, 0)


def main():
    init_simulation(5)
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(window_width, window_height)
    glutCreateWindow(b"Swarm Simulation - OpenGL Visualizer")
    glEnable(GL_DEPTH_TEST)
    glClearColor(1,1,1,1)
    build_display_lists()
    glutDisplayFunc(display)
    glutReshapeFunc(lambda w,h: setattr(sys.modules[__name__], 'window_width', w) or setattr(sys.modules[__name__], 'window_height', h))
    glutTimerFunc(update_ms, timer, 0)
    glutKeyboardFunc(lambda key,x,y: sim.stop() or sys.exit(0) if key==b'\x1b' else None)
    glutMainLoop()

if __name__ == '__main__':
    main()
