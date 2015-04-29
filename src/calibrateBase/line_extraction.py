#!/usr/bin/python

""" Line extration implementation """

import numpy as np
from numpy import linalg as LA

import matplotlib.pyplot as plt

def get_distance(m, i0, v0, x,):
        t = m[x, :] - m[i0, :];
        t_v0 = v0.dot(t);
        v1 = v0 * t_v0;
        return v1;
    
def get_max_residual(m, i0, i1):
    
    v0 = m[i1, :] - m[i0, :]; # Line vector
    v0_ = v0;
    
    v0 = v0 / np.linalg.norm(v0);
 
    res = list();
    for x in range(i0, i1+1):
        t = m[x, :] - m[i0, :];
        #plt.plot([m[i0, 0], m[i0, 0] + t[0]], [m[i0, 1], m[i0, 1]+ t[1]], '-b')
        t_v0 = v0.dot(t);
        v1 = v0 * t_v0;
        #plt.plot([m[x, 0], m[i0, 0] + v1[0]], [m[x, 1], m[i0, 1] + v1[1]], '-b')
        #plt.plot([m[x, 0], m[i0, 0] + get_distance(m, i0, v0, x)[0]], [m[x, 1], m[i0, 1] + get_distance(m, i0, v0, x)[1]], '-b')
        
        dist = np.linalg.norm(m[x] - (m[i0] + get_distance(m, i0, v0, x)));
        res.extend([dist]);
        #plt.plot([m[x, 0], m[i0, 0] + v0[0] * t_v0], [m[x, 1], m[i0, 1] + v0[1] * t_v0], '-b')
    
    #print res
    return np.max(res), np.argmax(res);

def split(m, i0, i1, threshold):
    r = [];
    if i0 < i1:
        # Get the residual
        plt.clf()
        
        v, j = get_max_residual(m, i0, i1)
        r=[];
        
        if v > threshold:
            # Split required
            r.append(i0+j);
            r.extend(split(m, i0, i0+j, threshold))
            r.extend(split(m, i0+j, i1, threshold))
    return r;

def merge(m, r, threshold):
    changed = True
    
    while changed:
        changed = False
        for x in range(0, len(r)-2):
            y = [r[x], r[x+1]]
            vy = m[y[1], :] - m[y[0], :];
            vy = vy / np.linalg.norm(vy);
        
            z = [r[x+1], r[x+2]]
            vz = m[z[1], :] - m[z[0], :];
            vz = vz / np.linalg.norm(vz);
            if abs(vz.dot(vy)) > 0.99:
                r.remove(y[1])
                changed = True;
                break
        
        
def split_and_merge(m, threshold):
    """ Function to extract lines from a laser scan using split and merge algorithm [Castellanos 1998]
    
    m is a numpy matrix N x 2
    """
    
    r = []
    # Split
    
    r = [0, m.shape[0]-1]
    r.extend(split(m, 0, m.shape[0]-1, threshold))
    r = sorted(r)
    
    # Merge
    merge(m, r, threshold)
    
    return r;
    
    
if __name__ == "__main__":
    print("Line extracion main")
    
    # Load the laser points in cartesian N x 2
    m = np.loadtxt(open("laser_scan_1.dat"))
    
    # Plot the laser points
    #plt.plot(m[:, 0], m[:, 1], '.r')
    #plt.axis('equal')
    #plt.title("Laser scan")
    #plt.show()
    
    # Extract line features
    r = split_and_merge(m, 0.25)
    
    # Plot found lines
    plt.plot(m[:, 0], m[:, 1], '.r')
    plt.plot(m[r, 0], m[r, 1], '-')
    plt.plot(m[r, 0], m[r, 1], '+')
    plt.axis('equal')
    plt.title("Laser scan")
    plt.show()
    
    # Get line which matches 