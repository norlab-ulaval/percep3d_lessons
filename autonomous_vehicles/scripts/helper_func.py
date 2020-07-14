import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import random

def get_root_path():
    path = os.getcwd()
    root_parent_path = path[:path.rfind("percep3d")]
    return path[:path.find("/", len(root_parent_path))]

def space_text_labels(dates, thresh = 3):
    for date in dates:
        if type(date[0]) is list:
            date.append(date[0][0] + (date[0][1] - date[0][0])/2.)
        else:
            date.append(date[0])


    collision = True

    while collision:
        collision = False
        random.shuffle(dates)
        for date in dates:
            min_dist = 20000
            for other_date in dates:
                if date == other_date:
                    pass
                else:
                    dist = date[2] - other_date[2]
                    if abs(min_dist) > abs(dist): min_dist = dist
            if abs(min_dist) < thresh:
                collision = True
                date[2] += np.copysign(0.5, min_dist)
                break
    

def plot_dates(ax, dates, x_coord = 0.5):
    
    for date in dates:
        if(x_coord >= 0):
            style = "arc,angleA=90,angleB=0,armA=0,armB=50,rad=0"
            relpos = (0., 0.5)
            ha='left'
            text = str(np.floor(date[0]).astype(int)) + ": " + str(date[1])
            color = "tab:red"
        else:
            style = "arc,angleA=90,angleB=180,armA=0,armB=50,rad=0"
            relpos = (1., 0.5)
            ha='right'
            text = str(date[1]) + ": " + str(np.floor(date[0]).astype(int))
            color = "yellow"

        if type(date[0]) is not list:
            ax.scatter(0, date[0], c=color)
            pos_x=0
            pos_y = date[0]
            arrowstyle="->"
        else:
            pos_x = x_coord/3.
            pos_y = np.mean(date[0])
            #rect = patches.Rectangle((-x,y),dx,dy,edgecolor='none',facecolor=color, alpha=0.3)
            #ax.add_patch(rect)
            poly = plt.Polygon([[0, date[0][0]],[0, date[0][1]],[pos_x, pos_y]],edgecolor='none',facecolor=color, alpha=0.3)
            ax.add_patch(poly)
            
            arrowstyle="-"
            
        ax.annotate(text,
                    xy=(pos_x, pos_y), xycoords='data', ha=ha,
                    xytext=(x_coord, date[2]), textcoords='data',
                    fontsize=15,
                    arrowprops=dict(arrowstyle=arrowstyle, color="0.5",
                                    shrinkA=5, shrinkB=5,
                                    patchA=None, patchB=None,
                                    connectionstyle=style,
                                    relpos=relpos,
                                    ),
                )