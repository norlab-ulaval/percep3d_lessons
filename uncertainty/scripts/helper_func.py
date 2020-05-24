import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

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