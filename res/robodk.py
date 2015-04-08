# DESCRIPTION:
# This is a simple robotics toolbox for Python, inspired from Peter Corke's Robotics Toolbox:
# http://petercorke.com/Robotics_Toolbox.html
#
# In this document: pose = transformation matrix = homogeneous matrix
# Visit: http://www.j3d.org/matrix_faq/matrfaq_latest.html
# for some help on matrix operations

import math
import operator
import sys
import unittest


#----------------------------------------------------
#--------      Generic math usage     ---------------

pi = math.pi

def atan2(y,x):
    return math.atan2(y,x)

def sqrt(value):
    return math.sqrt(value)

def sin(value):
    return math.sin(value)

def cos(value):
    return math.cos(value)

#----------------------------------------------------
#--------     Generic matrix usage    ---------------

def rotx(rx):
    """Return a X-axis rotation matrix
          |  1  0       0       0 |
rotx(A) = |  0  cos(A) -sin(A)  0 |
          |  0  sin(A)  cos(A)  0 |
          |  0  0       0       1 |"""
    ct = math.cos(rx)
    st = math.sin(rx)
    return Mat([[1,0,0,0],[0,ct,-st,0],[0,st,ct,0],[0,0,0,1]])

def roty(ry):
    """Return a Y-axis rotation matrix
          |  cos(A)  0   sin(A)  0 |
roty(A) = |  0       1   0       0 |
          | -sin(A)  0   cos(A)  0 |
          |  0       0   0       1 |"""
    ct = math.cos(ry)
    st = math.sin(ry)
    return Mat([[ct,0,st,0],[0,1,0,0],[-st,0,ct,0],[0,0,0,1]])

def rotz(rz):
    """Return a Z-axis rotation matrix
          |  cos(A)  -sin(A)   0   0 |
rotz(A) = |  sin(A)   cos(A)   0   0 |
          |  0        0        1   0 |
          |  0        0        0   1 |"""
    ct = math.cos(rz)
    st = math.sin(rz)
    return Mat([[ct,-st,0,0],[st,ct,0,0],[0,0,1,0],[0,0,0,1]])

def transl(x,y=None,z=None):
    """Return a translation matrix
                |  1   0   0   X |
transl(X,Y,Z) = |  0   1   0   Y |
                |  0   0   1   Z |
                |  0   0   0   1 |"""
    if y is None:
        xx = x[0]
        yy = x[1]
        zz = x[2]
    else:
        xx = x
        yy = y
        zz = z    
    return Mat([[1,0,0,xx],[0,1,0,yy],[0,0,1,zz],[0,0,0,1]])

def eye(size=4):
    """Return the identity matrix
        |  1   0   0   0 |
eye() = |  0   1   0   0 |
        |  0   0   1   0 |
        |  0   0   0   1 |"""
    return Mat.eye(size)

def size(matrix,dim=None):
    """Returns the size of a matrix (m,n).
    Dim can be set to 1 to return m (rows) or 2 to return n (columns)"""
    return matrix.size(dim)

def tr(matrix):
    """Calculates the transpose of the matrix"""
    return matrix.tr()

def invH(matrix):
    """Calculates the inverse of a homogeneous matrix"""
    return matrix.invH()

def catV(mat1, mat2):
    """Concatenate 2 matrix (vertical concatenation)"""
    return mat1.catV(mat2)

def catH(mat1, mat2):
    """Concatenate 2 matrix (horizontal concatenation)"""
    return mat1.catH(mat2)

def tic():
    """Start a stopwatch timer"""
    import time
    global TICTOC_START_TIME
    TICTOC_START_TIME = time.time()

def toc():
    """Read the stopwatch timer"""
    import time
    if 'TICTOC_START_TIME' in globals():
        elapsed = time.time() - TICTOC_START_TIME
        print("Elapsed time is " + str(elapsed) + " seconds.")
        return elapsed
    else:
        print("Toc: start time not set")
        return -1

#----------------------------------------------------
#------ Pose to xyzrpw and xyzrpw to pose------------
def pose_2_xyzrpw(H):
    """Calculates the equivalent position and euler angles ([x,y,z,r,p,w] vector) of the given pose 
    Note: transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)
    See also: xyzrpw_2_pose()"""
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    if (H[2,0] > (1.0 - 1e-6)):
        p = -pi/2
        r = 0
        w = math.atan2(-H[1,2],H[1,1])
    elif H[2,0] < -1.0 + 1e-6:
        p = pi/2
        r = 0
        w = math.atan2(H[1,2],H[1,1])
    else:
        p = math.atan2(-H[2,0],sqrt(H[0,0]*H[0,0]+H[1,0]*H[1,0]))
        w = math.atan2(H[1,0],H[0,0])
        r = math.atan2(H[2,1],H[2,2])    
    return [x, y, z, r*180/pi, p*180/pi, w*180/pi]

def xyzrpw_2_pose(xyzrpw):
    """Calculates the pose from the position and euler angles ([x,y,z,r,p,w] vector)
    The result is the same as calling: H = transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)
    See also: pose_2_xyzrpw()"""
    [x,y,z,r,p,w] = xyzrpw
    a = r*pi/180
    b = p*pi/180
    c = w*pi/180
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)    
    H = Mat([[cb*cc, cc*sa*sb - ca*sc, sa*sc + ca*cc*sb, x],
             [cb*sc, ca*cc + sa*sb*sc, ca*sb*sc - cc*sa, y],
             [  -sb,            cb*sa,            ca*cb, z],
             [    0,                0,                0, 1]])
    return H

def pose_2_quaternion(Ti):
    a=(Ti[0,0])
    b=(Ti[1,1])
    c=(Ti[2,2])
    sign2=1
    sign3=1
    sign4=1
    if (Ti[2,1]-Ti[1,2])<0:
        sign2=-1;
    if (Ti[0,2]-Ti[2,0])<0:
        sign3=-1;
    if (Ti[1,0]-Ti[0,1])<0:
        sign4=-1
    q1=sqrt(a+b+c+1)/2
    q2=sign2*sqrt(a-b-c+1)/2
    q3=sign3*sqrt(-a+b-c+1)/2
    q4=sign4*sqrt(-a-b+c+1)/2    
    return [q1, q2, q3, q4]

def quaternion_2_pose(qin):
    qnorm = sqrt(qin[0]*qin[0]+qin[1]*qin[1]+qin[2]*qin[2]+qin[3]*qin[3])
    q = qin
    q[0] = q[0]/qnorm
    q[1] = q[1]/qnorm
    q[2] = q[2]/qnorm
    q[3] = q[3]/qnorm
    pose = Mat([[ 1 - 2*q[2]*q[2] - 2*q[3]*q[3]  ,  2*q[1]*q[2] - 2*q[3]*q[0]  ,  2*q[1]*q[3] + 2*q[2]*q[0]   ,  0],
          [2*q[1]*q[2] + 2*q[3]*q[0]       ,  1 - 2*q[1]*q[1] - 2*q[3]*q[3] , 2*q[2]*q[3] - 2*q[1]*q[0] ,  0],
          [2*q[1]*q[3] - 2*q[2]*q[0]       ,  2*q[2]*q[3] + 2*q[1]*q[0]   ,   1 - 2*q[1]*q[1] - 2*q[2]*q[2], 0],
          [0 , 0 , 0 , 1]])
    return pose

def print_pose_ABB(pose):
    q = pose_2_quaternion(pose)
    print('[[%.3f,%.3f,%.3f],[%.6f,%.6f,%.6f,%.6f]]'%(pose[0,3],pose[1,3],pose[2,3],q[0],q[1],q[2],q[3]))
    
#----------------------------------------------------
#-------- ROBOT MODEL (D-H and D-H M) ---------------

def dh(rz,tx=None,tz=None,rx=None):
    """Returns the Denavit-Hartenberg 4x4 matrix for a robot link.
    calling dh(rz,tx,tz,rx) is the same as using rotz(rz)*transl(tx,0,tx)*rotx(rx)
    calling dh(rz,tx,tz,rx) is the same as calling dh([rz,tx,tz,rx])
    """
    if tx is None: [rz,tx,tz,rx] = rz
        
    crx = math.cos(rx)
    srx = math.sin(rx)
    crz = math.cos(rz)
    srz = math.sin(rz)    
    return Mat( [[crz, -srz*crx,  srz*srx, tx*crz],
                 [srz,  crz*crx, -crz*srx, tx*srz],
                 [  0,      srx,      crx,     tz],
                 [  0,        0,        0,      1]]);

def dhm(rx, tx=None, tz=None, rz=None):
    """Returns the Denavit-Hartenberg Modified 4x4 matrix for a robot link (Craig 1986).
    calling dhm(rx,tx,tz,rz) is the same as using rotx(rx)*transl(tx,0,tx)*rotz(rz)
    calling dhm(rx,tx,tz,rz) is the same as calling dhm([rx,tx,tz,rz])
    """
    if tx is None: [rx,tx,tz,rz] = rx
        
    crx = math.cos(rx)
    srx = math.sin(rx)
    crz = math.cos(rz)
    srz = math.sin(rz)    
    return Mat([[crz,        -srz,    0,      tx],
                [crx*srz, crx*crz, -srx, -tz*srx],
                [srx*srz, crz*srx,  crx,  tz*crx],
                [      0,       0,    0,       1]]);

#----------------------------------------------------
#-------- Useful geometric tools ---------------                   

def norm(p):
    return sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2])

def cross(a, b):
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]
    return c

def dot(a,b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def mult3(v,d):
    return [v[0]*d, v[1]*d, v[2]*d]

def subs3(a,b):
    return [a[0]-b[0],a[1]-b[1],a[2]-b[2]]

def add3(a,b):
    return [a[0]+b[0],a[1]+b[1],a[2]+b[2]]


def intersect_line_2_plane(pline,vline,pplane,vplane):
    D = -dot(vplane,pplane)
    k = -(D+dot(vplane,pline))/dot(vplane,vline)
    p = add3(pline,mult3(vline,k))
    return p

def proj_pt_2_plane(point,planepoint,planeABC):
    return intersect_line_2_plane(point,planeABC,planepoint,planeABC);

def proj_pt_2_line(point, paxe, vaxe):
    vpaxe2point = subs3(point,paxe)
    dist = dot(vaxe,vpaxe2point)/dot(vaxe,vaxe)
    return add3(paxe,mult3(vaxe,dist))

def fitPlane(points):
    import numpy as np
    XYZ = np.array(points)    
    [rows,cols] = XYZ.shape
    # Set up constraint equations of the form  AB = 0,
    # where B is a column vector of the plane coefficients
    # in the form b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0.
    p = (np.ones((rows,1)))
    AB = np.hstack([XYZ,p])
    [u, d, v] = np.linalg.svd(AB,0)        
    B = v[3,:]                  # Solution is last column of v.
    nn = np.linalg.norm(B[0:3])
    B = B / nn
    pplane = [0,0,-(B[3]/B[2])]
    vplane = B[0:3].tolist()
    return pplane, vplane  

                
#----------------------------------------------------
#--------       Mat matrix class      ---------------

class MatrixError(Exception):
    """ An exception class for Matrix """
    pass

class Mat(object):
    """A simple Python matrix class with basic
    operations and operator overloading."""
    
    def __init__(self, rows=None, ncols=None):
        if ncols is None:
            if rows is None:
                m = 4
                n = 4
                self.rows = [[0]*n for x in range(m)]
            else:
                if isinstance(rows,Mat):
                    rows = rows.copy().rows
                m = len(rows)
                transpose = 0
                if not isinstance(rows[0],list):
                    rows = [rows]
                    transpose = 1
                n = len(rows[0])
                if any([len(row) != n for row in rows[1:]]):# Validity check
                    raise Exception(MatrixError, "inconsistent row length")
                self.rows = rows
                if transpose:
                    self.rows = [list(item) for item in zip(*self.rows)]
        else:
            m = rows
            n = ncols
            self.rows = [[0]*n for x in range(m)]
    def copy(self):
        sz = self.size()
        newmat = Mat(sz[0],sz[1])
        for i in range(sz[0]):
            for j in range(sz[1]):
                newmat[i,j] = self[i,j]
        return newmat
        
    def __getitem__(self, idx):
        if isinstance(idx,int):#integer A[1]
            return tr(Mat(self.rows[idx]))
        elif isinstance(idx,slice):#one slice: A[1:3]
            return Mat(self.rows[idx])
        else:#two slices: A[1:3,1:3]
            idx1 = idx[0]
            idx2 = idx[1]
            if isinstance(idx1,int) and isinstance(idx2,int):
                return self.rows[idx1][idx2]
            matsize =self.size();
            if isinstance(idx1,slice):
                indices1 = idx1.indices(matsize[0])
                rg1 = range(*indices1)
            else: #is int
                rg1 = range(idx1,idx1+1)
            if isinstance(idx2,slice):
                indices2 = idx2.indices(matsize[1])
                rg2 = range(*indices2)
            else: #is int
                rg2 = range(idx2,idx2+1)                    
            newm = int(abs((rg1.stop-rg1.start)/rg1.step))
            newn = int(abs((rg2.stop-rg2.start)/rg2.step))
            newmat = Mat(newm,newn)
            cm = 0
            for i in rg1:
                cn = 0
                for j in rg2:
                    newmat.rows[cm][cn] = self.rows[i][j]
                    cn = cn + 1
                cm = cm + 1
            return newmat
    def __setitem__(self, idx, item):
        if isinstance(item,float) or isinstance(item,int):
            item = Mat([[item]])
        if isinstance(idx,int):#integer A[1]
            raise Exception(MatrixError, "Cannot set item. Use [i,:] instead.")
            #self.rows[idx] = item
        elif isinstance(idx,slice):#one slice: A[1:3]
            raise Exception(MatrixError, "Cannot set item. Use [a:b,:] instead.")
        else:#two slices: A[1:3,1:3]
            matsize =self.size();
            idx1 = idx[0]
            idx2 = idx[1]
            if isinstance(idx1,slice):
                indices1 = idx1.indices(matsize[0])
                rg1 = range(*indices1)
            else: #is int
                rg1 = range(idx1,idx1+1)
            if isinstance(idx2,slice):
                indices2 = idx2.indices(matsize[1])
                rg2 = range(*indices2)
            else: #is int
                rg2 = range(idx2,idx2+1)
            newm = int(abs((rg1.stop-rg1.start)/rg1.step))
            newn = int(abs((rg2.stop-rg2.start)/rg2.step))
            itmsz = item.size();
            if newm != itmsz[0] or newn != itmsz[1]:
                raise Exception(MatrixError, "Submatrix indices does not match the new matrix sizes",itmsz[0],"x",itmsz[1],"<-",newm,"x",newn)
            #newmat = Mat(newm,newn)
            cm = 0
            for i in rg1:
                cn = 0
                for j in rg2:
                    self.rows[i][j] = item.rows[cm][cn]
                    cn = cn + 1
                cm = cm + 1        
        
    def __str__(self):
        s='\n [ '.join([(', '.join([str(item) for item in row])+' ],') for row in self.rows])
        return '[[ ' + s[:-1] + ']\n'

    def __repr__(self):
        s=str(self)
        rank = str(self.size())
        rep="Matrix: %s\n%s" % (rank,s)
        return rep
                         
    def tr(self):
        """Returns a transpose of the matrix"""
        mat = Mat([list(item) for item in zip(*self.rows)])      
        return mat

    def size(self,dim=None):
        """Returns the size of a matrix (m,n).
        Dim can be set to 1 to return m (rows) or 2 to return n (columns)"""
        m = len(self.rows)
        n = len(self.rows[0])
        if dim is None:
            return (m, n)
        elif dim==0:
            return m
        elif dim==1:
            return n
        else:
            raise Exception(MatrixError, "Invalid dimension!")
        
    def catV(self,mat2):
        """Concatenate with another matrix (vertical concatenation)"""
        if not isinstance(mat2, Mat):
            raise Exception(MatrixError, "Concatenation must be performed with 2 matrices")
        sz1 = self.size()
        sz2 = mat2.size()
        if sz1[1] != sz2[1]:
            raise Exception(MatrixError, "Horizontal size of matrices does not match")
        newmat = Mat(sz1[0]+sz2[0],sz1[1])
        newmat[0:sz1[0],:] = self
        newmat[sz1[0]:,:] = mat2        
        return newmat
    
    def catH(self,mat2):
        """Concatenate with another matrix (horizontal concatenation)"""
        if not isinstance(mat2, Mat):
            raise Exception(MatrixError, "Concatenation must be performed with 2 matrices")
        sz1 = self.size()
        sz2 = mat2.size()
        if sz1[0] != sz2[0]:
            raise Exception(MatrixError, "Horizontal size of matrices does not match")
        newmat = Mat(sz1[0],sz1[1]+sz2[1])
        newmat[:,:sz1[1]] = self
        newmat[:,sz1[1]:] = mat2   
        return newmat
    def __eq__(self, mat):
        """Test equality"""
        return (mat.rows == self.rows)
        
    def __add__(self, mat):
        """Add a matrix to this matrix and
        return the new matrix. It doesn't modify
        the current matrix"""
        if isinstance(mat,int) or isinstance(mat,float):
            m, n = self.size()     
            result = Mat(m, n)        
            for x in range(m):
                for y in range(n):
                    result.rows[x][y] = self.rows[x][y] + mat
            return result
        sz = self.size()
        m = sz[0]
        n = sz[1]
        ret = Mat(m,n)
        if sz != mat.size():
            raise Exception(MatrixError, "Trying to add matrixes of varying size!")   
        for x in range(m):
            row = [sum(item) for item in zip(self.rows[x], mat.rows[x])]
            ret.rows[x] = row
        return ret

    def __sub__(self, mat):
        """Subtract a matrix from this matrix and
        return the new matrix. It doesn't modify
        the current matrix"""
        if isinstance(mat,int) or isinstance(mat,float):
            m, n = self.size()     
            result = Mat(m, n)        
            for x in range(m):
                for y in range(n):
                    result.rows[x][y] = self.rows[x][y] - mat
            return result
        sz = self.size()
        m = sz[0]
        n = sz[1]
        ret = Mat(m,n)
        if sz != mat.size():
            raise Exception(MatrixError, "Trying to add matrixes of varying size!")    
        for x in range(m):
            row = [item[0]-item[1] for item in zip(self.rows[x], mat.rows[x])]
            ret.rows[x] = row
        return ret

    def __mul__(self, mat):
        """Multiply a matrix with this matrix and
        return the new matrix. It doesn't modify
        the current matrix"""
        if isinstance(mat,int) or isinstance(mat,float):
            m, n = self.size()     
            mulmat = Mat(m, n)        
            for x in range(m):
                for y in range(n):
                    mulmat.rows[x][y] = self.rows[x][y]*mat
            return mulmat
        if isinstance(mat,list):#case of a matrix times a vector            
            szvect = len(mat)
            m = self.size(0);
            matvect = Mat(mat)            
            if szvect + 1 == m:
                vectok = catV(matvect,Mat([[1]]))
                result = self*vectok
                return (result[:-1,:]).tr().rows[0]
            elif szvect == m:
                result = self*Mat(matvect)
                return result.tr().rows[0]
            else:
                raise Exception(MatrixError, "Invalid product")       
        else:
            matm, matn = mat.size()
            m, n = self.size()
            if (n != matm):
                raise Exception(MatrixError, "Matrices cannot be multipled!")        
            mat_t = mat.tr()
            mulmat = Mat(m, matn)        
            for x in range(m):
                for y in range(mat_t.size(0)):
                    mulmat.rows[x][y] = sum([item[0]*item[1] for item in zip(self.rows[x], mat_t.rows[y])])
            return mulmat
    
    def eye(self, m=4):
        """Make identity matrix of size (mxm)"""
        rows = [[0]*m for x in range(m)]
        idx = 0        
        for row in rows:
            row[idx] = 1
            idx += 1
        return Mat(rows)

    def isHomogeneous(self):
        """returns 1 if it is a Homogeneous matrix"""
        m,n = self.size()
        if m != 4 or n != 4:
            return 0
        if self[3,:] != Mat([[0,0,0,1]]):
            return 0
        test = self[0:3,0:3];
        test = test*test.tr()
        test[0,0] = test[0,0] - 1
        test[1,1] = test[1,1] - 1
        test[2,2] = test[2,2] - 1
        zero = 0.0
        for x in range(3):
            for y in range(3):
                zero = zero + abs(test[x,y])
        if zero > 1e-4:
            return 0
        return 1
    
    def invH(self):
        """Calculates the inverse of a homogeneous matrix"""
        if not self.isHomogeneous():
            raise Exception(MatrixError, "Matrix is not homogeneous. invH() can only compute the inverse of a homogeneous matrix")
        Hout = self.tr();
        Hout[3,0:3] = Mat([[0,0,0]]);
        Hout[0:3,3] = (Hout[0:3,0:3]*self[0:3,3])*(-1);
        return Hout
