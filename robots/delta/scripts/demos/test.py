import numpy as np
import tf

a = np.array([1, 2, 3])
b = np.array([[1,2,3], [4,5,6], [7,8,9]])

print(np.dot(b, a))

q = tf.transformations.quaternion_from_euler(0, 0, 1.57)
print(q)
R = tf.transformations.quaternion_matrix(q)
R_np = np.array(R)
print(R_np.shape)
print(R_np[0:3, 0:3])


