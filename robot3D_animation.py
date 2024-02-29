#!/usr/bin/env python
# coding: utf-8


from vedo import *
import time
import vedo


def ForwardKinematics(Phi, L1, L2, L3):

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
    p2 = np.array([[L1+2*r1], [0.0], [0.0]])
    t_12 = p2                                      # Translation vector

    # Matrix of Frame 2 w.r.t. Frame 1
    T_12 = getLocalFrameMatrix(R_12, t_12)

    # Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
    T_02 = T_01 @ T_12

    # Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame)
    R_23 = RotationMatrix(Phi[2], axis_name='z')   # Rotation matrix
    # Frame's origin (w.r.t. previous frame)
    p3 = np.array([[L2+2*r1], [0.0], [0.0]])
    t_23 = p3                                      # Translation vector

    # Matrix of Frame 3 w.r.t. Frame 2
    T_23 = getLocalFrameMatrix(R_23, t_23)

    # Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
    T_03 = T_02 @ T_23

   # Matrix of Frame 4 (written w.r.t. Frame 3, which is the previous frame)
    R_34 = RotationMatrix(Phi[3], axis_name='z')   # Rotation matrix
    # Frame's origin (w.r.t. previous frame)
    p4 = np.array([[L3+2*r1], [0.0], [0.0]])
    t_34 = p4                                      # Translation vector

    # Matrix of Frame 4 w.r.t. Frame 3
    T_34 = getLocalFrameMatrix(R_34, t_34)

    # Matrix of Frame 4 w.r.t. Frame 0 (i.e., the world frame)
    T_04 = T_03 @ T_34

    end_effector = T_04[-1]

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


def main():

    # Set the limits of the graph x, y, and z ranges
    axes = Axes(xrange=(0, 30), yrange=(-5, 20), zrange=(0, 6))

    # Lengths of arm parts
    L1 = 5   # Length of link 1
    L2 = 8   # Length of link 2
    L3 = 3
    L4 = 0

    # Joint angles
    Phi = [30, -10, 20, 0]
    r1 = 0.4

    T_01, T_02, T_03, T_04, e = ForwardKinematics(Phi, L1, L2, L3)
    # Create the coordinate frame mesh and transform

    # Now, let's create a cylinder and add it to the local coordinate frame
    link1_mesh = Cylinder(r=0.4,
                          height=L1,
                          pos=(L1/2, 0, 0),
                          c="yellow",
                          alpha=.8,
                          axis=(1, 0, 0)
                          )

    # Also create a sphere to show as an example of a joint
    sphere1 = Sphere(r=r1).pos(L1+r1, 0, 0).color("gray").alpha(1)
    # Combine all parts into a single object
    Frame1 = link1_mesh + sphere1

    # Transform the part to position it at its correct location and orientation
    Frame1.apply_transform(T_01)

    link2_mesh = Cylinder(r=0.4,
                          height=L2,
                          pos=(L2/2, 0, 0),
                          c="red",
                          alpha=.8,
                          axis=(1, 0, 0)
                          )

    # Combine all parts into a single object
    sphere2 = Sphere(r=r1).pos(L2 + r1, 0, 0).color("gray").alpha(.8)
    Frame2 = link2_mesh + sphere2

    # Transform the part to position it at its correct location and orientation
    Frame2.apply_transform(T_02)

    # Create the coordinate frame mesh and transform. This point is the end-effector. So, I am
    # just creating the coordinate frame.
    # Frame3Arrows = createCoordinateFrameMesh()
    link3_mesh = Cylinder(r=0.4,
                          height=L3,
                          pos=(L3/2, 0, 0),
                          c="blue",
                          alpha=.8,
                          axis=(1, 0, 0)
                          )
    Frame3 = createCoordinateFrameMesh()
    Frame3 = link3_mesh

    # Transform the part to position it at its correct location and orientation
    Frame3.apply_transform(T_03)

    # Create the coordinate frame mesh and transform. This point is the end-effector. So, I am
    # just creating the coordinate frame.
    Frame4 = createCoordinateFrameMesh()
    sphere4 = Sphere(r=r1).pos(-r1, 0, 0).color("gray").alpha(1)
    Frame4 = sphere4

    # Transform the part to position it at its correct location and orientation
    Frame4.apply_transform(T_04)
    # Show everything
    show([Frame1, Frame2, Frame3, Frame4], axes, viewup="z", interactive=False)
    Frame1.apply_transform(np.linalg.inv(T_01))
    Frame2.apply_transform(np.linalg.inv(T_02))
    Frame3.apply_transform(np.linalg.inv(T_03))
    Frame4.apply_transform(np.linalg.inv(T_04))
    phi_increm = Phi

    for i in range(0, 25, 2):
        phi_increm[1] += i
        T_01, T_02, T_03, T_04, e = ForwardKinematics(phi_increm, L1, L2, L3)
        Frame1.apply_transform(T_01)
        Frame2.apply_transform(T_02)
        Frame3.apply_transform(T_03)
        Frame4.apply_transform(T_04)
        show([Frame1, Frame2, Frame3, Frame4],
             axes, viewup="z", interactive=False)
        time.sleep(0.2)
        Frame1.apply_transform(np.linalg.inv(T_01))
        Frame2.apply_transform(np.linalg.inv(T_02))
        Frame3.apply_transform(np.linalg.inv(T_03))
        Frame4.apply_transform(np.linalg.inv(T_04))
    for i in range(0, 25, 2):
        phi_increm[2] += i
        T_01, T_02, T_03, T_04, e = ForwardKinematics(phi_increm, L1, L2, L3)
        Frame1.apply_transform(T_01)
        Frame2.apply_transform(T_02)
        Frame3.apply_transform(T_03)
        Frame4.apply_transform(T_04)
        show([Frame1, Frame2, Frame3, Frame4],
             axes, viewup="z", interactive=False)
        time.sleep(0.2)
        Frame1.apply_transform(np.linalg.inv(T_01))
        Frame2.apply_transform(np.linalg.inv(T_02))
        Frame3.apply_transform(np.linalg.inv(T_03))
        Frame4.apply_transform(np.linalg.inv(T_04))
    for i in range(0, 25, 2):
        phi_increm[0] += i
        T_01, T_02, T_03, T_04, e = ForwardKinematics(phi_increm, L1, L2, L3)
        Frame1.apply_transform(T_01)
        Frame2.apply_transform(T_02)
        Frame3.apply_transform(T_03)
        Frame4.apply_transform(T_04)
        show([Frame1, Frame2, Frame3, Frame4],
             axes, viewup="z", interactive=False)
        time.sleep(0.2)
        Frame1.apply_transform(np.linalg.inv(T_01))
        Frame2.apply_transform(np.linalg.inv(T_02))
        Frame3.apply_transform(np.linalg.inv(T_03))
        Frame4.apply_transform(np.linalg.inv(T_04))


if __name__ == '__main__':
    main()
