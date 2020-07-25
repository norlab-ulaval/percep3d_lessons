import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

def pretty_print(a):
    if(a.ndim == 1):
        display(pd.DataFrame(a).T)
    else:
        display(pd.DataFrame(a))
        
    return

def pretty_ax(ax, title, ylable, xlable):
    ax.set_title(title)
    ax.set_ylabel(ylable)
    ax.set_xlabel(xlable)
    
    return

def draw_dartboard(ax):
    ring_color = 'dimgrey'
    back_color = 'black'
    ax.add_artist(plt.Circle( (0,0), 2.5, color=ring_color))
    ax.add_artist(plt.Circle( (0,0), 2.0, color=back_color))
    ax.add_artist(plt.Circle( (0,0), 1.5, color=ring_color))
    ax.add_artist(plt.Circle( (0,0), 1.0, color=back_color))
    ax.add_artist(plt.Circle( (0,0), 0.5, color=ring_color))
    ax.add_artist(plt.Circle( (0,0), 0.1, color=back_color))

    ax.axis('equal')
    ax.axis([-3, 3, -3, 3])
    
    return

def get_root_path():
    path = os.getcwd()
    root_parent_path = path[:path.rfind("percep3d")]
    return path[:path.find("/", len(root_parent_path))]

def normal_density(x, mu, cov):
    k = cov.size
    a = 1./(np.sqrt( ((2.*np.pi)**k) * np.linalg.det(cov) ))
    density = np.empty(x.shape[-1])
    
    for i,xi in enumerate(x.T):
        e = xi - mu
        density[i] = a * np.exp(-0.5 * e.T @ np.linalg.inv(cov) @ e)
    return density

def rot_2d(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]
                    ])

def scale_mat(params):
    return np.diagflat(params)

def build_sphere(radius, n):
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n)
    x = radius*np.outer(np.cos(u), np.sin(v))
    y = radius*np.outer(np.sin(u), np.sin(v))
    z = radius*np.outer(np.ones(np.size(u)), np.cos(v))
    
    return np.vstack([x.flatten(), y.flatten(), z.flatten()])