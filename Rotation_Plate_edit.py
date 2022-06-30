#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 19 14:54:32 2022

@author: dcolin
"""

import matplotlib.pyplot as plt
import numpy as np
import pickle
from enum import IntEnum
from sklearn.neighbors import BallTree

class HandleStatus(IntEnum):
    VALID = 1
    MISSING = 2
    DISABLED = 4
    
    
def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])

def plot(Mat,title):
    fig = plt.figure()                          #definiert plot
    plt.title(title)
    ax = fig.add_subplot(projection='3d')       #3d plot
    ax.set_zlim(-150,150)
    ax.scatter(*Mat[:,1:])
    ax.scatter(*Mat[:,0], c='red')
    
def plot_comp(A,B,title) :
    fig = plt.figure()                          #definiert plot
    plt.title(title)
    ax = fig.add_subplot(projection='3d')       #3d plot
    ax.set_zlim(-1,1)
    ax.scatter(*A, c='blue')
    ax.scatter(*B, c='red')
    ax.set_box_aspect([1,1,1])
    set_axes_equal(ax)

def align_points(A, B): #Kabsch Algorithmus
    mA = A.mean(0)
    mB = B.mean(0)
    
    H = (A - mA[None,:]).T @ (B - mB[None,:])
    u,s,vh = np.linalg.svd(H)
    v = vh.T
    
    R = v @ u.T
    
    if np.linalg.det(R) < 0:
        v[:,2] *= -1
        R = v @ u.T
    
    t = mB - R.dot(mA)
    
    err = np.sqrt(np.sum(np.square(A @ R.T + t[None,:] - B), 0)).mean()
    M = np.eye(4)
    M[:3,:3] = R
    M[:3, 3] = t
        
    return M, err

def icp(A, B, thresh=100.):
    tree = BallTree(B[:,:3])
    
    err = np.inf # numpy positive infinity 
    
    i = 0
    
    M = np.eye(4) # Return a 2-D array with ones on the diagonal and zeros elsewhere.
    
    while True:
        At = A @ M.T
        d, n = tree.query(At[:,:3], return_distance=True)
        d = d[:,0]
        n = n[:,0]
        
        mask = d <= thresh
        
        M_new, err_new = align_points(At[mask,:3], B[n[mask],:3])
        
        M = M_new @ M
        
        if err - err_new < 0.5:
            break
        
        err = err_new
        
        i += 1
        
        if i >= 50:
            break
        
    return M, err

def rotZ(phi): #phi in grad
    c = np.cos(np.deg2rad(phi))
    s = np.sin(np.deg2rad(phi))
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]])

def T_Matrix(t):
    if len(t) != 3 :
        Warning("T ungleich 3")
    return np.array([[1, 0, 0, t[0] ],
                     [0, 1, 0, t[1] ],
                     [0, 0, 1, t[2] ],
                     [0, 0, 0, 1]])

def load_reference():
    path = '/ceph/mri.meduniwien.ac.at/departments/physics/fmrilab/home/dcolin/solution_20211115-151419.pkl'
    with open(path, 'rb') as fp:
        B = pickle.load(fp)
    
    B = B['X']
    B = np.c_[B,np.zeros(18)]
    B = B*10  #Umwandlung in mm
    return B

def load_measurement():
    point_file_path = '/ceph/mri.meduniwien.ac.at/departments/physics/fmrilab/home/dcolin/test_04_07.pkl'
    with open(point_file_path, 'rb') as fp:
        data = pickle.load(fp)

    frame = 100
    x = np.array(data[frame]['stray_markers']['markers']) # waehlt die Marker aus (aus welchen Frame?)
    x = x[:,2:] # schaut das nur x,y,z da sind
    return x
    

def pca(x):
    #Kovarianz Matrix berechnen 
    xp = x - x.mean(0)       #Translationsschritt der gespeichert werden muss                   
    C = xp.T @ xp            
    #lösen des EW Problems                   
    w,v = np.linalg.eigh(C)
    # Koordinatenursprung für die PCA Komponenten (Orthonormalsystem Aufgbauen)
    md = np.median(x, 0) 
    # Abstand vom Urpsrung zu allen anderen Punkten
    d = np.sqrt(np.mean(np.square(x - md), 1))
    # Abstand zum eigenvektor v1     
    dp = np.abs(xp @ v[:,0]) 
    
    return v,d,dp
    
# Skript Anfang

def get_rot_matrix(x = load_measurement() ,B = load_reference()):
    dmax = 200
    dpmax = 30
    M_dict = {}
    phi = 0
    t = np.array([0.0,0.0,0.0])
    
    while x.shape != (18,3):
        v,d,dp = pca(x)
        t += -x.mean(0)
        print (t)
        mask = d < dmax    
        xm = x[mask]
        v,d,dp = pca(xm)
        mask2 = dp < dpmax                       
        xm = x[mask][mask2]
        x = xm - xm.mean(0)
        t += -xm.mean(0)
        print (t)
    
    B_hom = np.hstack((B, np.ones((B.shape[0], 1))))
    X_hom = np.hstack((x, np.ones((x.shape[0], 1))))
    
    while phi < 360:
        X_zrot = rotZ(phi) @ X_hom.T
        M,err = icp (X_zrot.T,B_hom)
        M_dict[phi] = err
        phi += 0.5
    
    T = T_Matrix(t)
    phi_min = min(M_dict.keys(), key=(lambda k: M_dict[k]))
    X_zrot = rotZ(phi_min) @ X_hom.T
    M,err = icp (X_zrot.T,B_hom)
    X = (M @ X_zrot)[:3]
    
    plot_comp(B.T,X,"Vergleich") 
    M_fin = T @ rotZ(phi_min) @ M 
    return (M_fin)

M = get_rot_matrix()
print (M)
x = load_measurement()
X_hom = np.hstack((x, np.ones((x.shape[0], 1))))
X = M @ X_hom.T

B = load_reference()
plot_comp(B.T,X,"Vergleich") 
