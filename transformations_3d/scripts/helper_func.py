#!/usr/bin/env python

#--------------
# for 3D
#--------------

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
from matplotlib.text import Annotation
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def draw_3d_basis_vector(ax, head, text="", origin=[0,0,0], text_offset=[0,0,0], *args, **kwargs):
    text_global = origin + head + text_offset
    ax.quiver(origin[0], origin[1], origin[2], 
               head[0], head[1], head[2], length=1., normalize=True, *args, **kwargs)
    ax.text(text_global[0], text_global[1], text_global[2], text, size=20, *args, **kwargs)
    return

def draw_3d_frame(ax, origin = np.array([0,0,0]), 
                  x = np.array([1,0,0]), y = np.array([0,1,0]), z = np.array([0,0,1]),
                  text_x=r"$\vec{\mathscr{x}}$", text_y=r"$\vec{\mathscr{y}}$", text_z=r"$\vec{\mathscr{z}}$",
                  *args, **kwargs):
    draw_3d_basis_vector(ax, x, origin=origin, text=text_x, *args, **kwargs)
    draw_3d_basis_vector(ax, y, origin=origin, text=text_y, *args, **kwargs)
    draw_3d_basis_vector(ax, z, origin=origin, text=text_z, *args, **kwargs)
    return

def draw_3d_vector(ax, head=np.array([0,0,0]), text="", origin=[0,0,0], text_offset=[0,0,0], 
                   *args, **kwargs):
    arrow_handle = Arrow3D(head=head, origin=origin, mutation_scale=20, 
                           arrowstyle="->", *args, **kwargs)
    text_handle = Annotation3D(text, head, text_offset=text_offset,
                               size=20, *args, **kwargs)
    
    ax.add_artist(arrow_handle)
    ax.add_artist(text_handle)

    return [arrow_handle, text_handle]
    
def rotation_matrix_z(theta):
    return np.array([[ np.cos(theta),-np.sin(theta), 0],
                     [ np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

def rotation_matrix_y(theta):
    return np.array([[ np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def rotation_matrix_x(theta):
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

class Arrow3D(FancyArrowPatch):
    def __init__(self, head, origin=[0,0,0], *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        xs = [origin[0], head[0]]
        ys = [origin[1], head[1]]
        zs = [origin[2], head[2]]
        self._verts3d = xs, ys, zs
        self.init_style = self.get_arrowstyle()

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        FancyArrowPatch.set_positions(self,(xs[0],ys[0]),(xs[1],ys[1]))
        dx = np.abs(xs[0] - xs[1])
        dy = np.abs(ys[0] - ys[1])
        dz = np.abs(zs[0] - zs[1])

        thresh = 0.4
        if dx < thresh and dy < thresh:
            factor = np.max([dx,dy])*10.
            if factor > 0:
                self.set_arrowstyle("->",head_length=factor, head_width=factor/2.)
            else:
                self.set_arrowstyle("-")
        else:
            self.set_arrowstyle(self.init_style)
        FancyArrowPatch.draw(self, renderer)
        
    def set_positions(self, head, origin=[0,0,0]):
        xs = [origin[0], head[0]]
        ys = [origin[1], head[1]]
        zs = [origin[2], head[2]]
        self._verts3d = xs, ys, zs

        
class Annotation3D(Annotation):
    '''Annotate the point xyz with text s'''

    def __init__(self, s, xyz, text_offset=[0,0,0],*args, **kwargs):
        Annotation.__init__(self,s, xy=(0,0), *args, **kwargs)
        self._verts3d = xyz + text_offset
        self._offset = text_offset

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        Annotation.set_position(self, (xs,ys))
        Annotation.draw(self, renderer)
        
    def set_position(self, xyz):
        self._verts3d = xyz + self._offset
        
def build_parallelepiped(P):
    assert(P.shape[0] == 3), "Wrong number of dimensions"
    assert(P.shape[1] == 8), "Wrong number of points"
    return [[P[:,0],P[:,1],P[:,2],P[:,3]],
            [P[:,4],P[:,5],P[:,6],P[:,7]], 
            [P[:,0],P[:,1],P[:,5],P[:,4]], 
            [P[:,2],P[:,3],P[:,7],P[:,6]], 
            [P[:,1],P[:,2],P[:,6],P[:,5]],
            [P[:,4],P[:,7],P[:,3],P[:,0]]]

def draw_parallelepiped(ax, P, *args, **kwargs):
    if(P.shape[0] == 3):
        vertices = build_parallelepiped(P)
        ax.add_collection3d(Poly3DCollection(vertices, *args, **kwargs))
    else:
        print("The entered point cloud has invalid dimensions")
        
def generate_cube():
    return np.array([[-1,  1,  1, -1, -1,  1,  1, -1],
                     [-1, -1,  1,  1, -1, -1,  1,  1],
                     [-1, -1, -1, -1,  1,  1,  1,  1]])