## Vector Transformer with RPY Angles

import math

# If this file is to be used by another Python file in a different folder,
# check out https://www.geeksforgeeks.org/python-import-module-from-different-directory/

## Helper functions

# Quaternions here are [qw, qx, qy, qz], where qw is the real component

'''
  * @brief  Alias for math.sin() function
  * @param  x : angle in radians
  * @retval  float : the result
'''
def sin(x):
    return math.sin(x);


'''
  * @brief  Alias for math.cos() function
  * @param  x : angle in radians
  * @retval  float : the result
'''
def cos(x):
    return math.cos(x);


'''
  * @brief  Performs a dot product between two 3D vectors
  * @param  v1 : the first 3D vector
  * @param  v2 : the second 3D vector
  * @retval  float : the result
'''
def dotProd3D(v1, v2):
    return (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]);


'''
  * @brief  Performs a cross product between two 3D vectors
  * @param  v1 : the first 3D vector
  * @param  v2 : the second 3D vector
  * @retval  float[3] : the resultant 3D vector
'''
def crossProd3D(v1, v2):
    # axb = <a2b3 - a3b2, a3b1 - a1b3, a1b2 - a2b1>
    res = [0,0,0];
    res[0] = v1[1]*v2[2] - v1[2]*v2[1];
    res[1] = v1[2]*v2[0] - v1[0]*v2[2];
    res[2] = v1[0]*v2[1] - v1[1]*v2[0];
    return res;


'''
  * @brief  Multiplies a 3D vector by a scalar
  * @param  v : the 3D vector
  * @param  s : the scalar
  * @retval  float[3] : the resultant 3D vector
'''
def multByScalar3D(v, s):
    res = [0,0,0];
    res[0] = v[0]*s;
    res[1] = v[1]*s;
    res[2] = v[2]*s;
    return res;


'''
  * @brief  Multiplies two quaternions
  * @param  q1 : the first quaternion
  * @param  q2 : the second quaternion
  * @retval  float[4] : the resultant quaternion
'''
def multQuats(q1, q2):
    # (r1,v1)*(r2,v2) = (r1*r2 - (v1•v2), r1*v2 + r2*v1 + (v1xv2))
    r1 = q1[0];
    v1 = [q1[1], q1[2], q1[3]];
    r2 = q2[0];
    v2 = [q2[1], q2[2], q2[3]];
    
    pxq = crossProd3D(v1, v2);
    p0q = multByScalar3D(v2, r1);
    q0p = multByScalar3D(v1, r2);
    
    res = [0,0,0,0]
    res[0] = r1*r2 - dotProd3D(v1, v2);
    res[1] = p0q[0] + q0p[0] + pxq[0];
    res[2] = p0q[1] + q0p[1] + pxq[1];
    res[3] = p0q[2] + q0p[2] + pxq[2];
    return res;


'''
  * @brief  Rotates a 3D vector using a quaternion
  * @param  v : the 3D vector
  * @param  q : the second quaternion
  * @retval  float[3] : the resultant vector
'''
def rotate3DVector(v, q):
    # Find conjugate of q
    q_ = [q[0], -1*q[1], -1*q[2], -1*q[3]];
    
    # Turn the vector into a quaternion
    p = [0, v[0], v[1], v[2]];
    
    # Transformation: p' = q*p*q_
    res0 = multQuats(q, p);
    res0 = multQuats(res0, q_);
    
    # Turn quaternion result into vector (the real part is 0)
    return res0[1:4];


'''
  * @brief  Converts roll-pitch-yaw to a quaternion
  * @param  roll : the roll angle in degrees
  * @param  pitch : the pitch angle in degrees
  * @param  yaw : the yaw angle in degrees
  * @retval  float[4] : the quaternion representing the rotation
'''
def rpy2quat(roll, pitch, yaw):
    # Convert rpy to radians
    r = (math.pi/180)*roll;
    p = (math.pi/180)*pitch;
    y = (math.pi/180)*yaw;
    
    # Convert rpy to quaternion
    q = [0,0,0,0];
    q[0] = (cos(r/2))*(cos(p/2))*(cos(y/2)) + (sin(r/2))*(sin(p/2))*(sin(y/2));
    q[1] = (sin(r/2))*(cos(p/2))*(cos(y/2)) - (cos(r/2))*(sin(p/2))*(sin(y/2));
    q[2] = (cos(r/2))*(sin(p/2))*(cos(y/2)) + (sin(r/2))*(cos(p/2))*(sin(y/2));
    q[3] = (cos(r/2))*(cos(p/2))*(sin(y/2)) - (sin(r/2))*(sin(p/2))*(cos(y/2));
    return q;


'''
  * @brief  Gets the rotated version of a 3D vector in the initial coordinate axes
  * @param  v : the original 3D vector
  * @param  roll : the roll angle in degrees
  * @param  pitch : the pitch angle in degrees
  * @param  yaw : the yaw angle in degrees
  * @retval  float[3] : the rotated 3D vector
'''
def getRotVect(v, roll, pitch, yaw):
    q = rpy2quat(roll, pitch, yaw);
    return rotate3DVector(v, q);


'''
  * @brief  Finds the magnitude of an n-dimensional vector
  * @param  v : the vector
  * @retval  float : the magnitude
'''
def getMag(v):
    mag = 0;
    for f in v:
        mag += math.pow(f, 2);
    return math.sqrt(mag);


'''
  * @brief  Normalizes an n-dimensional vector
  * @param  v : the vector
  * @retval  float[n] : the normalized vector
'''
def normVect(v):
    mag = getMag(v);
    v_n = v.copy();
    for i in range(len(v)):
        v_n[i] = v[i]/mag;
    return v_n;


# The commented-out functions were an attempt to find the initial orientation using the gravity measurements
# from the IMU in the first few seconds before the vehicle starts moving, but they didn't work.
'''
  * @brief  Converts a quaternion to roll-pitch-yaw
  * @param  q : the original quaternion
  * @retval  float[3] : the roll-pitch-yaw representing the rotation
'''
'''
def quat2rpy(q):
    r = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(math.pow(q[1],2) + math.pow(q[2],2))) * 180/math.pi;
    sinArg = 2*(q[0]*q[2] - q[1]*q[3]);
    if round(sinArg,2) >= 1:
        p = 90.0;
    elif round(sinArg,2) <= 1:
        p = -90.0;
    else:
        p = math.asin(sinArg) * 180/math.pi;
    y = math.atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(math.pow(q[2],2) + math.pow(q[3],2))) * 180/math.pi;
    return [r, p, y];
'''


'''
  * @brief  Gets the quaternion required to rotate vector 1 to vector 2
  * @param  v1 : the first 3D vector
  * @param  v2 : the second 3D vector
  * @retval  float[4] : the quaternion representing the rotation
'''
'''
def getQuatRot(v1, v2):
    # From https://www.xarg.org/proof/quaternion-from-two-vectors/
    qXYZ = crossProd3D(v1, v2);
    qW = getMag(v1)*getMag(v2) + dotProd3D(v1, v2);
    return normVect([qW, qXYZ[0], qXYZ[1], qXYZ[2]]);
'''


## Main (Comment this out if not troubleshooting)

'''
  * @brief  Sets up an example of the rotation
  * @retval  None
'''
'''
def main():
    v = [0, 0, -9.81];
    rpy = [10, 20, 0];
    v_ = getRotVect(v, rpy[0], rpy[1], rpy[2]);
    print("Original:", v);
    print("RPY (º):", rpy);
    print("q:", rpy2quat(rpy[0], rpy[1], rpy[2]))
    print("Rotated:", v_);

main();
#'''
