#!/usr/bin/env python

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, RegularPolygon
from numpy import radians as rad
from numpy import degrees as deg
from IPython.display import display, Markdown

def get_root_path():
    path = os.getcwd()
    root_parent_path = path[:path.rfind("percep3d")]
    return path[:path.find("/", len(root_parent_path))]

def draw_base_vector(ax, head, text="", origin=np.array([0., 0.]), text_offset=np.array([0., 0.]), color="tab:red", ha='right', va='top'):
    head_global = origin+head
    text_global = head_global+text_offset
    
    ax.annotate("", xy=head_global, xytext=origin,
                    arrowprops=dict(arrowstyle="->,head_width=0.6, head_length=1", color=color,  lw=2))
    ax.text(text_global[0], text_global[1], text, size=30, color=color, ha=ha, va=va)

    return

def draw_vector(ax, head, text="", origin=np.array([0., 0.]), text_offset=np.array([0., 0.]), color="white", ha='left', va='bottom'):
    text_global = head+text_offset
                
    ax.annotate("", xy=head, xytext=origin,
            arrowprops=dict(arrowstyle="-> ,head_width=0.4, head_length=0.8", color=color,  lw=1.))
    ax.text(text_global[0], text_global[1], text, size=20, color=color, ha=ha, va=va)
    
    return

def draw_point(ax, point, text="", text_offset=(0., 0.), color="white", ha='left', va='bottom'):
    text_global = point+text_offset
                
    ax.scatter(point[0], point[1], c=color)
    ax.text(text_global[0], text_global[1], text, size=20, color=color, ha=ha, va=va)
    
    return

def draw_frame(ax, origin=np.array([0., 0.]), x = np.array([1.,0.]), y = np.array([0.,1.]), color="white", 
               name="", text_x="", text_y=""):
    draw_base_vector(ax, origin=origin, head=x, text=text_x, text_offset=(0., -0.1), ha='right',va='top', color=color)
    draw_base_vector(ax, origin=origin, head=y, text=text_y, text_offset=(-0.1, 0.), ha='right',va='top', color=color)
    ax.text(origin[0], origin[1], name, color=color, size=30, ha='right',va='top')
    return

def clean_frame(ax, data, title, pad=1):
    min_graph = np.min([np.min(data)-pad,0])
    max_graph = np.max([np.max(data)+pad,1])

    ax.set_xlim((min_graph, max_graph))
    ax.set_ylim((min_graph, max_graph))
    ax.set_axis_off()
    ax.set_title(title);
    plt.tight_layout()
    return

def draw_arc_arrow(ax, angle_, theta2_, center=(0., 0.), text="", text_offset=(0., 0.), radius=1., color='white', is_rad=True, flip=False):

    if(is_rad):
        angle_ = deg(angle_)
        theta2_ = deg(theta2_)
    
    #========Line
    if(flip):
        head_angle = 180+angle_
        direction = 1.
        arc = Arc([center[0],center[1]],radius*2.,radius*2.,angle=angle_,
                  theta1=0., theta2=theta2_-angle_, capstyle='round', 
                  linestyle='-', lw=1, color=color)
    else:
        head_angle = angle_
        direction = -1.
        arc = Arc([center[0],center[1]],radius*2.,radius*2.,angle=theta2_,
                  theta1=0., theta2=angle_-theta2_, capstyle='round', 
                  linestyle='-', lw=1, color=color)
    ax.add_patch(arc)

    #========Create the arrow head
    arrow_scale = radius/30.
    delta_angle = arrow_scale/(radius)
    
    endX=center[0]+(radius)*(np.cos(rad(angle_) + direction*delta_angle) ) #Do trig to determine end position
    endY=center[1]+(radius)*(np.sin(rad(angle_) + direction*delta_angle) )

    ax.add_patch(                    #Create triangle as arrow head
        RegularPolygon(
            (endX, endY),            # (x,y)
            3,                       # number of vertices
            arrow_scale,                # radius
            rad(head_angle),     # orientation
            color=color
        )
    )
    #========text
    middle_angle = theta2_ - (theta2_ - angle_)/2.
    endX=center[0]+(radius*1.05)*(np.cos(rad(middle_angle)) )
    endY=center[1]+(radius*1.05)*(np.sin(rad(middle_angle)) )
    ax.text(endX+text_offset[0], endY+text_offset[1], text, size=20)
    return

def draw_angle_vectors(ax, v1, v2=np.array([1.,0]), text=r"$\theta$", text_offset=(0., 0.), color="white", radius=None, flip=False):
    lenght_v1 = np.linalg.norm(v1)
    theta_v1 = np.arctan2(v1[1],v1[0])
    theta_v2 = np.arctan2(v2[1],v2[0])
    if(radius==None):
        radius = lenght_v1
    draw_arc_arrow(ax, theta_v2, theta_v1, text=text, text_offset=text_offset, radius=radius, color=color, flip=flip)
    return

def draw_3d_basis_vector(ax, head, text="", origin=[0,0,0], text_offset=[0,0,0]):
    text_global = origin + head + text_offset
    ax.quiver(origin[0], origin[1], origin[2], 
               head[0], head[1], head[2], length=1., normalize=True)
    ax.text(text_global[0], text_global[1], text_global[2], text, size=30)
    return

def draw_3d_frame(ax, origin = np.array([0,0,0]), 
                 x = np.array([1,0,0]), y = np.array([0,1,0]), z = np.array([0,0,1]),
                 text_x=r"$\vec{\mathscr{x}}$", text_y=r"$\vec{\mathscr{y}}$", text_z=r"$\vec{\mathscr{z}}$"):
   draw_3d_basis_vector(ax, x, origin=origin, text=text_x)
   draw_3d_basis_vector(ax, y, origin=origin, text=text_y)
   draw_3d_basis_vector(ax, z, origin=origin, text=text_z)
   return

def display_geometric_quantities_2d(P, P_prime):
    p = P[0:2,0]
    q = P[0:2,1]
    r = P[0:2,2]
    p_prime = P_prime[0:2,0]
    q_prime = P_prime[0:2,1]
    r_prime = P_prime[0:2,2]
    
    # original coordinates
    l_p = np.linalg.norm(p)
    l_q = np.linalg.norm(q)
    u_p = p/l_p
    u_q = q/l_q
    dist = np.linalg.norm(p - q)
    theta = np.arccos(np.dot(p,q)/(l_p*l_q))
    ratio = l_p/l_q
    area = 0.5*np.linalg.det(np.vstack([q-p, r-p]).T)
    
    # modified coordinates
    l_p_prime = np.linalg.norm(p_prime)
    l_q_prime = np.linalg.norm(q_prime)
    u_p_prime = p_prime/l_p_prime
    u_q_prime = q_prime/l_q_prime
    dist_prime = np.linalg.norm(p_prime - q_prime)
    theta_prime = np.arccos(np.dot(p_prime,q_prime)/(l_p_prime*l_q_prime))
    ratio_prime = l_p_prime/l_q_prime
    area_prime = 0.5*np.linalg.det(np.vstack([q_prime-p_prime, r_prime-p_prime]).T)
    
    table = str("|      | before       | after        | equal"); table +="\n"
    table +=str("|---      |----      |---        |---"); table +="\n"
    table +=str("|p     |" +str(p) + "       | " +str(p_prime) + "    | " +str(np.allclose(p,p_prime))); table +="\n"
    table +=str("|q     |" +str(q) + "       | " +str(q_prime) + "    | " +str(np.allclose(q, q_prime))); table +="\n"
    table +=str("|lenght of p   | %.3f        | %.3f" %(l_p, l_p_prime) + "        | " +str(np.allclose(l_p, l_p_prime))); table +="\n"
    table +=str("|lenght of q   | %.3f        | %.3f" %(l_q, l_q_prime) + "        | " +str(np.allclose(l_q, l_q_prime))); table +="\n"
    table +=str("|direction of p   | "  + str(u_p) + "  | " +str(u_p_prime) + "  | " +str(np.allclose(u_p, u_p_prime))); table +="\n"
    table +=str("|direction of q   | "  + str(u_q) + "  | " +str(u_q_prime) + "  | " +str(np.allclose(u_q, u_q_prime))); table +="\n"
    table +=str("|dist p q| %.3f        | %.3f" %(dist, dist_prime) + "        | " +str(np.allclose(dist, dist_prime))); table +="\n"
    table +=str("|theta p q| %.3f        | %.3f" %(theta, theta_prime) + "        | " +str(np.allclose(theta, theta_prime))); table +="\n"
    table +=str("|ratio of lenght p q| %.3f        | %.3f" %(ratio, ratio_prime) + "        | " +str(np.allclose(ratio, ratio_prime))); table +="\n"
    table +=str("|area p q r | %.3f        | %.3f" %(area, area_prime) + "        | " +str(np.allclose(area, area_prime))); table +="\n"
    display(Markdown(table))
    
    return

def rigid_tranformation(params):
    """Returns a rigid transformation matrix

    :params: numpy array, params[0]=tx, params[1]=ty, params[2]=theta
    :returns: LaTeX bmatrix as a string
    """
    return np.array([[np.cos(params[2]), -np.sin(params[2]), params[0]],
                     [np.sin(params[2]),  np.cos(params[2]), params[1]],
                     [0,0,1]])

def scale_tranformation(params):
    return np.array([[params[0], 0, 0],
                     [0,  params[1], 0],
                     [0,0,1]])