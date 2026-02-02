import math
from scipy.spatial.transform import Rotation as R

def dist(pos1, pos2, plane = False):
            if plane:
                return math.sqrt((pos1.x - pos2.x)**2 +
                                 (pos1.y - pos2.y)**2)
            return math.sqrt((pos1.x - pos2.x)**2 +
                             (pos1.y - pos2.y)**2 +
                             (pos1.z - pos2.z)**2)
def extract_yaw(quat):
        quat = [quat.x, quat.y, quat.z, quat.w]
        r = R.from_quat(quat)   
        return r.as_euler('xyz')[2]  # [Roll, Pitch, Yaw] in radians
