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

def