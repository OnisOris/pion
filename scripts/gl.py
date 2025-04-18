import sys
import threading
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

# Импорт вашего класса симуляции
from swarm_server.swarmsim import SwarmSim, create_objects_Point_yaw
from pion.simulator import PointYaw

# Глобальные переменные
sim = None
positions = None
colors = None
window_width = 800
window_height = 600
limit = 5.0
update_ms = 50  # интервал обновления в мс

# Параметры камеры (для 3D вида)
cam_distance = 15.0
cam_elev = 30.0
cam_azim = -60.0

# Display lists
grid_lists = {}
cube_list = None


def init_simulation(num_objects: int):
    global sim, positions, colors
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
        "max_acceleration": 10,
        "max_speed": 1,
        "unstable_radius": 1.5,
    }
    points = create_objects_Point_yaw(int(np.sqrt(num_objects - 1)), [-limit, limit], [-limit, limit])
    zpoint = PointYaw(position=np.array([1, 1, 1, 0, 0, 0]))
    sim = SwarmSim(np.hstack([points, [zpoint]]), dt=0.05, logger=True, params=params)
    threading.Thread(target=sim.start_simulation_while, daemon=True).start()
    # Инициалные позиции и цвета
    states = sim.get_states()
    positions = states[:, :3].copy()
    colors = np.random.rand(num_objects, 3)


def build_display_lists():
    global grid_lists, cube_list
    step = 1.0
    # Сетка для 2D видов
    for view in ('top', 'front', 'side'):
        dl = glGenLists(1)
        glNewList(dl, GL_COMPILE)
        glColor3f(0.8, 0.8, 0.8)
        glBegin(GL_LINES)
        if view == 'top':
            # XY plane grid
            for i in np.arange(-limit, limit + step, step):
                glVertex3f(-limit, i, 0); glVertex3f(limit, i, 0)
                glVertex3f(i, -limit, 0); glVertex3f(i, limit, 0)
        elif view == 'front':
            # YZ plane grid
            for i in np.arange(-limit, limit + step, step):
                glVertex3f(0, -limit, i); glVertex3f(0, limit, i)
                glVertex3f(0, i, -limit); glVertex3f(0, i, limit)
        else:
            # XZ plane grid
            for i in np.arange(-limit, limit + step, step):
                glVertex3f(-limit, 0, i); glVertex3f(limit, 0, i)
                glVertex3f(i, 0, -limit); glVertex3f(i, 0, limit)
        glEnd()
        glEndList()
        grid_lists[view] = dl
    # Границы рабочего объёма (куб)
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


def draw_axes(view):
    glLineWidth(2.0)
    glBegin(GL_LINES)
    if view == 'top':
        # X axis red
        glColor3f(1, 0, 0); glVertex3f(-limit, 0, 0); glVertex3f(limit, 0, 0)
        # Y axis green
        glColor3f(0, 1, 0); glVertex3f(0, -limit, 0); glVertex3f(0, limit, 0)
    elif view == 'front':
        # Y axis green
        glColor3f(0, 1, 0); glVertex3f(0, -limit, 0); glVertex3f(0, limit, 0)
        # Z axis blue
        glColor3f(0, 0, 1); glVertex3f(0, 0, -limit); glVertex3f(0, 0, limit)
    elif view == 'side':
        # X axis red
        glColor3f(1, 0, 0); glVertex3f(-limit, 0, 0); glVertex3f(limit, 0, 0)
        # Z axis blue
        glColor3f(0, 0, 1); glVertex3f(0, 0, -limit); glVertex3f(0, 0, limit)
    else:
        # 3D axes
        glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(limit, 0, 0)
        glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, limit, 0)
        glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, limit)
    glEnd()
    glLineWidth(1.0)


def draw_view_title(title, x0, y0, w, h):
    margin = 15
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, window_width, 0, window_height, -1, 1)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()
    glColor3f(0, 0, 0)
    glRasterPos2f(x0 + margin, y0 + h - margin)
    for ch in title:
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ord(ch))
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW)


def display():
    global positions
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    half_w, half_h = window_width // 2, window_height // 2
    viewports = [
        ('Top view (XY)', 0, half_h, half_w, half_h),
        ('Front view (YZ)', half_w, half_h, half_w, half_h),
        ('Side view (XZ)', 0, 0, half_w, half_h),
        ('3D view', half_w, 0, half_w, half_h),
    ]
    for title, x, y, w, h in viewports:
        view = title.split()[0].lower()
        glViewport(x, y, w, h)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        if view == '3d':
            gluPerspective(45.0, w / float(h or 1), 0.1, 100.0)
        else:
            glOrtho(-limit, limit, -limit, limit, -limit*2, limit*2)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        # Камеры для видов
        if view == 'top':
            gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0)
        elif view == 'front':
            gluLookAt(-10, 0, 0, 0, 0, 0, 0, 0, 1)
        elif view == 'side':
            gluLookAt(0, -10, 0, 0, 0, 0, 0, 0, 1)
        else:
            gluLookAt(
                cam_distance * np.cos(np.radians(cam_azim)) * np.cos(np.radians(cam_elev)),
                cam_distance * np.sin(np.radians(cam_azim)) * np.cos(np.radians(cam_elev)),
                cam_distance * np.sin(np.radians(cam_elev)),
                0, 0, 0,
                0, 0, 1,
            )
        # Рисуем сетку, оси, границы
        glDisable(GL_DEPTH_TEST)
        if view in grid_lists: glCallList(grid_lists[view])
        draw_axes(view)
        glCallList(cube_list)
        glEnable(GL_DEPTH_TEST)
        # Рисуем точки
        glPointSize(6.0)
        glBegin(GL_POINTS)
        for (x_, y_, z_), (r, g, b) in zip(positions, colors):
            glColor3f(r, g, b)
            glVertex3f(x_, y_, z_)
        glEnd()
        draw_view_title(title, x, y, w, h)
    glutSwapBuffers()


def timer(value):
    global positions
    if sim:
        positions = sim.get_states()[:, :3]
    glutPostRedisplay()
    glutTimerFunc(update_ms, timer, 0)


def reshape(width, height):
    global window_width, window_height
    window_width, window_height = width, height
    glViewport(0, 0, width, height)


def keyboard(key, x, y):
    if key == b'\x1b':
        sim.stop()
        sys.exit(0)


def main():
    init_simulation(17)
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(window_width, window_height)
    glutCreateWindow(b"Swarm Simulation - OpenGL Visualizer")
    glEnable(GL_DEPTH_TEST)
    glClearColor(1.0, 1.0, 1.0, 1.0)
    build_display_lists()
    glutDisplayFunc(display)
    glutReshapeFunc(reshape)
    glutTimerFunc(update_ms, timer, 0)
    glutKeyboardFunc(keyboard)
    glutMainLoop()


if __name__ == '__main__':
    main()
