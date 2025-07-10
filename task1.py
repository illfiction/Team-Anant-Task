import numpy as np

def cross_product_matrix(v):
    return np.array(
        [
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ]
    )

def attitude_matrix_from_euler_axis_angle(e,v):
    return np.eye(3) - np.sin(e)*cross_product_matrix(v) + (1-np.cos(e))*np.dot(cross_product_matrix(v),cross_product_matrix(v))

def euler_axis_angle_from_attitude_matrix(A):
    v = np.arccos((A[0,0] + A[1,1] + A[2,2] - 1) / 2)
    e = np.array([A[2,1] - A[1,2], A[2,0] - A[0,2], A[0,1] - A[1,0]])/(2*np.sin(v))
    return e,v

def quaternion_from_euler_axis_angle(e,v):
    return np.array([e[0]*np.sin(v/2), e[1]*np.sin(v/2), e[2]*np.sin(v/2), np.cos(v/2)])

def quaternion_cross_product_matrix(q):
    return np.array(
        [
            [q[3], q[2], -q[1], q[0]],
            [-q[2], q[3], q[0], q[1]],
            [q[1], -q[0], q[3], q[2]],
            [-q[0], -q[1], -q[2], q[3]]
        ]
    )

def quaternion_dot_product_matrix(q):
    return np.array(
        [
            [q[3], -q[2], q[1], q[0]],
            [q[2], q[3], -q[0], q[1]],
            [-q[1], q[0], q[3], q[2]],
            [-q[0], -q[1], -q[2], q[3]]
        ]
    )

def inverse_quaternion(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])/np.dot(q,q)

def attitude_matrix_from_quaternion(q):
    return np.eye(3)*(q[3]**2 - np.dot(q[0:3],q[0:3])) - 2*q[3]*cross_product_matrix(q[0:3]) + 2*np.dot(q[0:3],np.transpose(q[0:3]))

def quaternion_from_attitude_matrix(A):
    return quaternion_from_euler_axis_angle(*euler_axis_angle_from_attitude_matrix(A))

def quaternion_from_euler_angles(roll,pitch,yaw):
    return quaternion_from_attitude_matrix(attitude_matrix_from_euler_angles(roll,pitch,yaw))

def attitude_matrix_from_euler_angles(roll,pitch,yaw):
    A321 = np.array(
        [
            [np.cos(pitch)*np.cos(roll), np.cos(pitch)*np.sin(roll), -np.sin(pitch)],
            [-np.cos(yaw)*np.sin(roll)+np.sin(yaw)*np.sin(pitch)*np.cos(roll), np.cos(yaw)*np.cos(roll)+np.sin(yaw)*np.sin(pitch)*np.sin(roll), np.sin(yaw)*np.cos(pitch)],
            [np.sin(yaw)*np.sin(roll)+np.cos(yaw)*np.sin(pitch)*np.cos(roll), -np.sin(yaw)*np.cos(roll)+np.cos(yaw)*np.sin(pitch)*np.sin(roll), np.cos(yaw)*np.cos(pitch)]
        ]
    )
    return A321
