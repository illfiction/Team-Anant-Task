from task1 import *

# # Test 1
# v = np.array([1,2,3])
# print(cross_product_matrix(v))

# # Test 2
# e = np.array([0,0,1])
# v = np.pi/2
# print(attitude_matrix_from_euler_axis_angle(e,v))
#
# # Test 3
# A = np.array([[0,-1,0],[1,0,0],[0,0,1]])
# print(euler_axis_angle_from_attitude_matrix(A))

# # Test 4
# e = [1,0,0]
# v = np.pi
# print(quaternion_from_euler_axis_angle(e,v))

# # Test 5
# q = [1,2,3,4]
# print(quaternion_cross_product_matrix(q))
# print(quaternion_dot_product_matrix(q))

# # Test 7
# q = [0,0.38268,0,0.92388]
# print(attitude_matrix_from_quaternion(q))

# Test 8
# A = np.array([[0,0,1],[0,1,0],[-1,0,0]])
#
# print(quaternion_from_attitude_matrix(A))

# Test 9
roll, pitch, yaw = 0 , np.pi/2, 0
print(attitude_matrix_from_euler_angles(roll, pitch, yaw))
print(quaternion_from_euler_angles(roll, pitch, yaw))
