from math import *
from time import *
from matplotlib.pyplot import *
from numpy import * 

def loadPoints(file):
    #Load file
    X, Y, Z = [], [], [] ;
    
    with open(file) as f:
        
        for line in f:
            x, y, z = line.split('\t')
            
            X.append(float(x))
            Y.append(float(y))
            Z.append(float(z))
    
    return X, Y, Z

def  getNN(xq, yq, zq, X, Y, Z):
    #Find nearest point and its distance
    dmin = inf
    xn, yn, zn = X[0], Y[0], Z[0]
    
    for i in range(len(X)):
        #Compute distance
        dx, dy, dz = xq - X[i], yq - Y[i], zq - Z[i]
        d = (dx*dx + dy*dy + dz * dz)**0.5
        
        #Actualize minimum: distance + coordinates
        if d < dmin:
            dmin = d
            xn, yn, zn = X[i], Y[i], Z[i]
    return xn, yn, zn, dmin

def drawPoints(X, Y, Z, bx):
    # Create figure
    fig = figure()
    ax = axes(projection = '3d')
    ax.set_aspect('equal')

    #Compute sphere scale: 1 pix = 25.4 mm
    scale = 1
    if bx > 0:
        scale = int(bx * bx * 40 * 40)
        
    #Plot points
    ax.scatter(X, Y, Z, s=scale, alpha = 0.05)

    show()


def init_index(X, Y, Z, nr):
    # initialize 3D index
    Xmax = max(X)
    Xmin = min(X)
    dx = (Xmax - Xmin)
    bx = dx / nr

    Ymax = max(Y)
    Ymin = min(Y)
    dy = (Ymax - Ymin)
    by = dy / nr

    Zmax = max(Z)
    Zmin = min(Z)
    dz = (Zmax - Zmin)
    bz = dz / nr
    return Xmin, Ymin, Zmin, dx, dy, dz, bx, by, bz


def get_3d_index(x,y,z,Xmin,Ymin,Zmin,dx,dy,dz,nr):
    # Get 3D index of point
    xr = (x - Xmin) / dx
    yr = (y - Ymin) / dy
    zr = (z - Zmin) / dz


    jx = int(xr*nr*0.999)
    jy = int(yr*nr*0.999)
    jz = int(zr*nr*0.999)

    # Query point outside of grid
    if (xr < 0 or xr > 1) or (yr < 0 or yr > 1) or (zr < 0 or zr > 1):
        print("Warning: get_3d_index: Point outside of grid")
        return -1,-1,-1

    return jx, jy, jz


def hash(jx,jy,jz,nr):
    # Convert 3D index to 1D
    return jx+jy*nr+jz*nr**2


def voxelize(X, Y, Z, Xmin, Ymin, Zmin, dx, dy, dz, bx, by, bz, nr):
    # Voxelize point cloud
    VI = zeros((nr,nr,nr))
    # Browse all points
    for i in range(len(X)):
        # Get 3D index
        jx, jy, jz = get_3d_index(X[i], Y[i], Z[i], Xmin, Ymin, Zmin, dx, dy, dz, nr)
        # Mark voxel as used
        VI[jx][jy][jz] = 1
    # list of voxel centroids
    XV, YV, ZV = [], [], []
    for i in range(nr):
        for j in range(nr):
            for k in range(nr):
                # is voxel used
                if VI[i][j][k]:
                    XV.append(Xmin+i*bx+bx/2)
                    YV.append(Ymin+j*by+by/2)
                    ZV.append(Zmin+k*bz+bz/2)

    return XV, YV, ZV


# load points
X,Y,Z = loadPoints("tree_18.txt")

# draw points
drawPoints(X,Y,Z,0)

# initialization
# number of points
n = len(X)
# number of bins
nbin = n**(1/3)
# number of bins in row
nr = int(nbin**(1/3))
nr = 100

# initialize 3D index
Xmin, Ymin, Zmin, dx, dy, dz, bx, by, bz = init_index(X,Y,Z,nr)
print(bx, by, bz)

# Get 3D index
# Testing data
xq,yq,zq = 55,570,470
jx,jy,jz = get_3d_index(xq,yq,zq,Xmin,Ymin,Zmin,dx,dy,dz,nr)
print(jx,jy,jz)

# Convert 3D index to 1D
h = hash(jx,jy,jz,nr)
print(h)

# Voxelization
XV, YV, ZV = voxelize(X, Y, Z, Xmin, Ymin, Zmin, dx, dy, dz, bx, by, bz, nr)
drawPoints(XV,YV,ZV,(bx+by+bz)/3)