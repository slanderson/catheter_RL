'''
This file runs the OpenGL loop using the global varialbes from IDM. This creates a 3D representation of
the end effector position and orientation (cone) or a sphere if only using position. It is run on a separate core 
than the robot IDM using the multiprocessing module.  
Jake Sganga
12/8/15
'''

import sys
import time
import numpy as np
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *



# Here, we've defined the location (x,y,z) of each vertex.
vertices= (
    (1, -.01, -.01),
    (1, .01, -.01),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -.01, .01),
    (1, .01, .01),
    (-1, -1, 1),
    (-1, 1, 1)
    )
vertices2= (
    (1, -.01, -.01),
    (1, .01, -.01),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -.01, .01),
    (1, .01, .01),
    (-1, -1, 1),
    (-1, 1, 1)
    )

# Each of the above tuples contains two numbers. Those numbers correspond to a vertex, 
# and the "edge" is going to be drawn between those two vertices
edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )
surfaces = (
    (0,1,2,3),
    (3,2,7,6),
    (6,7,5,4),
    (4,5,1,0),
    (1,5,7,2),
    (4,0,3,6)
    )
colors = (
    (1,0,0),
    (0,1,0),
    (0,0,1),
    (0,1,0),
    (1,1,1),
    (0,1,1),
    (1,0,0),
    (0,1,0),
    (0,0,1),
    (1,0,0),
    (1,1,1),
    (0,1,1),
    )
def Cone(vertices, color = True):
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()
    # now add colors
    if color:

        glBegin(GL_QUADS)
        for surface in surfaces:
            x = 0
            for vertex in surface:
                x+=1
                glColor3fv(colors[x])
                glVertex3fv(vertices[vertex])
        glEnd()


def Cylinder(use_fill = False, radius = 1, radius_tip = .01, height = 5,slices = 32, stacks = 16):
    quadric = gluNewQuadric()

    if use_fill:
        gluQuadricDrawStyle(quadric, GLU_FILL)
    else:
        gluQuadricDrawStyle(quadric, GLU_SILHOUETTE)
    gluCylinder(quadric, radius, radius_tip, height, slices, stacks)
    gluDeleteQuadric(quadric)

def Sphere(use_fill = False, radius = 1,slices = 32,stacks = 16):
    quadric = gluNewQuadric()
    if use_fill:
        gluQuadricDrawStyle(quadric, GLU_FILL)
    else:
        gluQuadricDrawStyle(quadric, GLU_SILHOUETTE)
    gluSphere(quadric, radius, slices, stacks)
    gluDeleteQuadric(quadric)

def coneMaterial( ):
    """Setup material for cone"""
    glMaterialfv(GL_FRONT, GL_AMBIENT, GLfloat_4(0.2, 0.2, 0.2, 1.0))
    glMaterialfv(GL_FRONT, GL_DIFFUSE, GLfloat_4(0.8, 0.8, 0.8, 1.0))
    glMaterialfv(GL_FRONT, GL_SPECULAR, GLfloat_4(1.0, 0.0, 1.0, 1.0))
    glMaterialfv(GL_FRONT, GL_SHININESS, GLfloat(50.0))
def light():
    """Setup light 0 and enable lighting"""
    glLightfv(GL_LIGHT0, GL_AMBIENT, GLfloat_4(0.0, 1.0, 0.0, 1.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, GLfloat_4(1.0, 1.0, 1.0, 1.0))
    glLightfv(GL_LIGHT0, GL_SPECULAR, GLfloat_4(1.0, 1.0, 1.0, 1.0))
    glLightfv(GL_LIGHT0, GL_POSITION, GLfloat_4(1.0, 1.0, 1.0, 0.0));   
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, GLfloat_4(0.2, 0.2, 0.2, 1.0))
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
def depth():
    """Setup depth testing"""
    glDepthFunc(GL_LESS)
    glEnable(GL_DEPTH_TEST)

def drawCone(x, use_fill = False, use_cone = True, radius = 1, radius_tip = .01, height = 5,slices = 32, stacks = 16):
    glPushMatrix()
    x_num = len(x)
    scale = 1
    offset = 5
    offset_elevation = 180

    glTranslatef(scale*x[1], -scale*x[2], -scale*x[0] - offset)
    if not use_cone:
        Sphere(use_fill, radius, slices, stacks)
    else:
        glRotatef(scale*x[3], 0.0, 0.0, 1.0)
        glRotatef(scale*x[4] + offset_elevation, 0.0, 1.0, 0.0)
        Cylinder(use_fill, radius, radius_tip, height, slices, stacks)
        
    glPopMatrix()

def OGLThread(global_variables):
    pygame.init()
    display = (800,600)
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
# The first value is the degree value of the field of view (fov). 
# The second value is the aspect ratio, which is the display width divided by the display height.
 # The next two values here are the znear and zfar, which are the near and far clipping planes.
    # gluPerspective(25, (display[0]/display[1]), 0.1, 75.0)
# this basically moves you, and the parameters are x, y and z
    glTranslatef(0.0,0.0, -25)
    # i = 0
    print('Started OpenGL thread')
    sys.stdout.flush()

    dx = global_variables[0]
    J  = global_variables[1]
    q  = global_variables[2]
    x_g  = global_variables[3]
    flags  = global_variables[4]
    x_desired = global_variables[5]

    x_num = len(x_g)
    x_num_des = len(x_desired)

    while True:
        now = time.perf_counter()
        for event in pygame.event.get():
            
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    quit()

        # ###added views
        # establish the projection matrix (perspective)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # x,y,width,height = glGetDoublev(GL_VIEWPORT)
        gluPerspective(
            120, # field of view in degrees
            (display[0]/display[1]), # aspect ratio
            .25, # near clipping plane
            500, # far clipping plane
        )

        # and then the model view matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(
            0,1,5, # eyepoint
            0,0,0, # center-of-view
            0,1,0, # up-vector
        )
        light()
        depth()

        # glRotatef multiplies the current matrix by a rotation matrix. 
        # The parameters here are angle, x, y and z.
        #push/pop with movements in between make sure the translation and rotations are done to the origin frame
        
        # glRotatef(-30, 0.0, 1.0, 0.0)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        # x_desired
        cone_flag = True if flags[4] > 0 else False
        drawCone(x_desired, use_fill = False, use_cone = cone_flag)
        # x
        drawCone(x_g, use_fill = True, use_cone = cone_flag)
        


        pygame.display.flip() # which updates our display, after drawing each object

        sys.stdout.flush()


