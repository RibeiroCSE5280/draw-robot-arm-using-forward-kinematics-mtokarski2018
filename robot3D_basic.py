#!/usr/bin/env python
# coding: utf-8


from vedo import *
import numpy as np
import time

def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)

    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)

    """
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1


    # x-axis as an arrow
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)

    originDot = Sphere(pos=[0,0,0],
                       c="black",
                       r=0.10)


    # Combine the axes together to form a frame as a single mesh object
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot

    return F


def getLocalFrameMatrix(R_ij, t_ij):
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i
      t_ij: translation of Frame j w.r.t. Frame i
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i.

    """
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])

    return T_ij
#handles the angle updates
def f_k(phi1, phi2, phi3):
    L1 = 5
    L2 = 8

    R_01 = RotationMatrix(phi1, axis_name='z')
    p1 = np.array([[3], [2], [0.0]])
    t_01 = p1
    T_01 = getLocalFrameMatrix(R_01, t_01)
    Frame1Arrows = createCoordinateFrameMesh()
    link1_mesh = Cylinder(r=0.4,
                          height=L1,
                          pos=(L1 / 2, 0, 0),
                          c="yellow",
                          alpha=.8,
                          axis=(1, 0, 0))
    r1 = 0.4
    sphere1 = Sphere(r=r1).pos(-r1, 0, 0).color("gray").alpha(.8)
    Frame1 = Frame1Arrows + link1_mesh + sphere1
    Frame1.apply_transform(T_01)

    R_12 = RotationMatrix(phi2, axis_name='z')
    p2 = np.array([[L1], [0.0], [0.0]])
    t_12 = p2
    T_12 = getLocalFrameMatrix(R_12, t_12)
    Frame2Arrows = createCoordinateFrameMesh()
    link2_mesh = Cylinder(r=0.4,
                          height=L2,
                          pos=(L2 / 2, 0, 0),
                          c="red",
                          alpha=.8,
                          axis=(1, 0, 0))
    Frame2 = Frame2Arrows + link2_mesh
    T_02 = T_01 @ T_12
    Frame2.apply_transform(T_02)

    R_23 = RotationMatrix(phi3, axis_name='z')
    p3 = np.array([[L2], [0.0], [0.0]])
    t_23 = p3
    T_23 = getLocalFrameMatrix(R_23, t_23)
    T_03 = T_01 @ T_12 @ T_23
    Frame3 = createCoordinateFrameMesh()
    Frame3.apply_transform(T_03)

    return Frame1, Frame2, Frame3

#handles the animation
def animate():
    axes = Axes(xrange=(0, 20), yrange=(-2, 10), zrange=(0, 6))
    #controls the amount of frames
    frames = 100
    phi1_arr = np.arange(0, 360, 360 / frames)
    phi2_arr = np.arange(0, 360, 360 / frames)
    phi3_arr = np.arange(0, 360, 360 / frames)

    for i in range(frames):
        phi1 = phi1_arr[i]
        phi2 = phi2_arr[i]
        phi3 = phi3_arr[i]

        Frame1, Frame2, Frame3 = f_k(phi1, phi2, phi3)

        show([Frame1, Frame2, Frame3], axes, viewup="z", interactive=False)
        #controls the speed of the animation
        time.sleep(0.3)

if __name__ == '__main__':
    animate()



