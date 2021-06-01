import os
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


def get_root_path():
    path = os.getcwd()
    root_parent_path = path[:path.rfind("percep3d")]
    return path[:path.find("/", len(root_parent_path))]

def sorted_eig(A):
    eigenValues, eigenVectors = np.linalg.eig(A)
    idx = np.argsort(eigenValues)
    eigenValues = eigenValues[idx]
    eigenVectors = eigenVectors[:,idx]
    
    return (eigenValues, eigenVectors)

def rigid_tranformation(params):
    """Returns a rigid transformation matrix

    :params: numpy array, params[0]=tx, params[1]=ty, params[2]=theta
    :returns: LaTeX bmatrix as a string
    """
    return np.array([[np.cos(params[2]), -np.sin(params[2]), params[0]],
                     [np.sin(params[2]),  np.cos(params[2]), params[1]],
                     [0,0,1]])

def mode_beta(param):
    alpha = param[0]
    beta = param[1]
    return (alpha - 1)/(alpha + beta -2)

def build_room(param_v, param_h, angle = 0., wall_thickness = 0.01, nb_pts = 400):

    nb_pts = int(nb_pts/4)

    sensor_center = np.ones(3)

    wall_top = np.ones([3, nb_pts])
    wall_top[0] = np.random.beta(param_v[0], param_v[1], nb_pts)
    wall_top[1] = np.random.uniform(-wall_thickness/2., wall_thickness/2., nb_pts) + 1.

    wall_bottom = np.ones([3, nb_pts])
    wall_bottom[0] = np.random.beta(param_v[0], param_v[1], nb_pts)
    wall_bottom[1] = np.random.uniform(-wall_thickness/2., wall_thickness/2., nb_pts)
    sensor_center[0] = mode_beta(param_v)

    wall_left = np.ones([3, nb_pts])
    wall_left[1] = np.random.beta(param_h[0], param_h[1], nb_pts)
    wall_left[0] = np.random.uniform(-wall_thickness/2., wall_thickness/2., nb_pts)

    wall_right = np.ones([3, nb_pts])
    wall_right[1] = np.random.beta(param_h[0], param_h[1], nb_pts)
    wall_right[0] = np.random.uniform(-wall_thickness/2., wall_thickness/2., nb_pts) + 1.

    sensor_center[1] = mode_beta(param_h)

    T = rigid_tranformation([-sensor_center[0], -sensor_center[1], angle])
    P = np.hstack([wall_bottom, wall_top, wall_left, wall_right])
    
    return (T @ P)

def draw_base_vector(ax, head, text="", origin=np.array([0., 0.]), text_offset=np.array([0., 0.]), color="tab:red", ha='right', va='top'):
    head_global = origin+head
    text_global = head_global+text_offset
    
    ax.annotate("", xy=head_global, xytext=origin,
                    arrowprops=dict(arrowstyle="->,head_width=0.6, head_length=1", color=color,  lw=2))
    ax.text(text_global[0], text_global[1], text, size=30, color=color, ha=ha, va=va)

    return

def draw_frame(ax, origin=np.array([0., 0.]), x = np.array([1.,0.]), y = np.array([0.,1.]), color="white", 
               name="", text_x="", text_y=""):
    draw_base_vector(ax, origin=origin, head=x, text=text_x, text_offset=(0., -0.1), ha='right',va='top', color=color)
    draw_base_vector(ax, origin=origin, head=y, text=text_y, text_offset=(-0.1, 0.), ha='right',va='top', color=color)
    ax.text(origin[0], origin[1], name, color=color, size=30, ha='right',va='top')
    return

def draw_point_clouds(ax, P=None, Q=None, normals_P=None, normals_Q=None, errors=None, T=None, alpha=0.2):

    if P is not None:
        ax.scatter(P[0], P[1], alpha=alpha, color="tab:blue")
        if normals_P is not None:
            ax.quiver(P[0], P[1], normals_P[0], normals_P[1], color="yellow")
        if errors is not None:    
            ax.quiver(P[0], P[1], errors[0], errors[1], 
                      color="tab:red", alpha=0.4,
                      angles='xy', scale_units='xy', scale=1.)
    if Q is not None:
        ax.scatter(Q[0], Q[1], alpha=alpha, color="tab:green")
        if normals_Q is not None:
            ax.quiver(Q[0], Q[1], normals_Q[0], normals_Q[1], color="yellow")
    if T is not None:
        ax.quiver(0, 0, T[0,2], T[1,2], color="tab:red",
                  angles='xy', scale_units='xy', scale=1.)
    
    draw_frame(ax, x=[0.2, 0], y=[0, 0.2])
    ax.set_xlabel(r"$\vec{\mathscr{x}}$")
    ax.set_ylabel(r"$\vec{\mathscr{y}}$")
    ax.set_aspect('equal', adjustable='box')
    
class IcpInspector():
    def __init__(self, P=[], Q=[], T=[], I=[]):
        self.P = [P.copy()]
        self.Q = Q.copy()
        self.T = [np.copy(T)]
        self.I = [np.copy(I)]
        
    def append(self, P, T, I):
        self.P.append(P.copy())
        self.T.append(np.copy(T))
        self.I.append(np.copy(I))
        
    def draw_icp_frame(self, i):
        P_pts = self.P[i].features[:2,:]
        Q_pts = self.Q.features[:2,:]
        indices = self.I[i]
        errors = Q_pts[:, indices] - P_pts

        self.h_P.set_offsets(P_pts.T)
        self.h_err.set_offsets(P_pts.T)
        self.h_err.set_UVC(errors[0], errors[1])
        self.h_err_vec.set_UVC(errors[0], errors[1])
        self.time_text.set_text("iteration " + str(i))
        
        return (self.h_P, 
                self.h_err, 
                self.h_err_vec, 
                self.time_text)
    
    def build_animation(self):
        nb_frames = len(self.P)

        nb_pts = len(self.P[0].features[0])
        zeros = np.zeros(nb_pts)

        fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(12,6))
        ax = axs[0]
        ax.scatter(self.Q.features[0], self.Q.features[1], alpha=0.2, color="tab:green")
        self.h_P = ax.scatter(self.P[0].features[0], self.P[0].features[1], alpha=0.2, color="tab:blue")
        self.h_err = ax.quiver(zeros, zeros, 
                          zeros, zeros, 
                          color="tab:red", alpha=0.4,
                          angles='xy', scale_units='xy', scale=1.)

        ax.set_xlabel(r"$\vec{\mathscr{x}}$")
        ax.set_ylabel(r"$\vec{\mathscr{y}}$")
        ax.set_title("Point clouds")
        ax.set_aspect('equal', adjustable='box')
        draw_frame(ax, x=[0.2, 0], y=[0, 0.2])

        ax = axs[1]
        self.h_err_vec = ax.quiver(zeros, zeros, 
                              zeros, zeros, 
                              color="tab:red", alpha=0.2,
                              angles='xy', scale_units='xy', scale=1.)
        self.time_text = ax.text(0.05, 0.05, '', fontsize=20, transform=ax.transAxes)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel(r"Error on $\vec{\mathscr{x}}$")
        ax.set_ylabel(r"Error on $\vec{\mathscr{y}}$")
        ax.set_title("Residual errors")
        lim = [-0.25, 0.25]
        ax.set_xlim(lim)
        ax.set_ylim(lim)
        fig.tight_layout()
        
        anim = animation.FuncAnimation(fig, self.draw_icp_frame,
                                       frames=np.arange(nb_frames), interval=1000, 
                                       blit=True, repeat=True)
        plt.close(anim._fig)
        return anim
    
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
        
def draw_3d_point_clouds(ax, P, Q, errors):
    draw_parallelepiped(ax, Q[0:3], fc='white', lw=1, edgecolors='tab:green', alpha=.2)
    draw_parallelepiped(ax, P[0:3], fc='white', lw=1, ls=':', edgecolors='yellow', alpha=.2)

    ax.quiver(P[0], P[1], P[2], errors[0], errors[1], errors[2], color="tab:red", arrow_length_ratio = 0.05)

    # cosmetics
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax_lim = 3.
    ax.set_xlim(-ax_lim, ax_lim); ax.set_ylim(-ax_lim, ax_lim); ax.set_zlim(-ax_lim, ax_lim)
    pane_color = (1.0, 1.0, 1.0, 0.0)
    ax.xaxis.set_pane_color(pane_color); ax.yaxis.set_pane_color(pane_color); ax.zaxis.set_pane_color(pane_color)
    grid_color = (1.0, 1.0, 1.0, 0.2)
    ax.xaxis._axinfo["grid"]['color'] =  grid_color; ax.yaxis._axinfo["grid"]['color'] =  grid_color; ax.zaxis._axinfo["grid"]['color'] =  grid_color
