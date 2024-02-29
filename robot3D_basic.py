#!/usr/bin/env python
# coding: utf-8


from vedo import *
import time
import vedo


def forward_kinematics(Phi, L1, L2, L3, L4):

    R_01 = RotationMatrix(Phi[0], axis_name='z')   # Rotation matrix
    r1 = 0.4
    # Frame's origin (w.r.t. previous frame)
    p1 = np.array([[3], [2], [0.0]])
    t_01 = p1                                      # Translation vector

    # Matrix of Frame 1 w.r.t. Frame 0 (i.e., the world frame)
    T_01 = getLocalFrameMatrix(R_01, t_01)

    # Matrix of Frame 2 (written w.r.t. Frame 1, which is the previous frame)
    R_12 = RotationMatrix(Phi[1], axis_name='z')   # Rotation matrix
    # Frame's origin (w.r.t. previous frame)
    p2 = np.array([[L1+(2*r1)], [0.0], [0.0]])
    t_12 = p2                                      # Translation vector

    # Matrix of Frame 2 w.r.t. Frame 1
    T_12 = getLocalFrameMatrix(R_12, t_12)

    # Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
    T_02 = T_01 @ T_12

    # Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame)
    R_23 = RotationMatrix(Phi[2], axis_name='z')   # Rotation matrix
    # Frame's origin (w.r.t. previous frame)
    p3 = np.array([[L2+(2*r1)], [0.0], [0.0]])
    t_23 = p3                                      # Translation vector

    # Matrix of Frame 3 w.r.t. Frame 2
    T_23 = getLocalFrameMatrix(R_23, t_23)

    # Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
    T_03 = T_02 @ T_23

   # Matrix of Frame 4 (written w.r.t. Frame 3, which is the previous frame)
    R_34 = RotationMatrix(Phi[3], axis_name='z')   # Rotation matrix
    # Frame's origin (w.r.t. previous frame)
    p4 = np.array([[L3+r1], [0.0], [0.0]])
    t_34 = p4                                      # Translation vector

    # Matrix of Frame 4 w.r.t. Frame 3
    T_34 = getLocalFrameMatrix(R_34, t_34)

    # Matrix of Frame 4 w.r.t. Frame 0 (i.e., the world frame)
    T_04 = T_03 @ T_34

    end_effector = T_04[:, -1][:3]

    return T_01, T_02, T_03, T_04, end_effector


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

    if axis_name == 'x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name == 'y':
        rotation_matrix = np.array([[c,  0, s],
                                    [0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name == 'z':
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

    originDot = Sphere(pos=[0, 0, 0],
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
