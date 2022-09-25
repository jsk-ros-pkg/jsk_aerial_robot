#!/usr/bin/env python

import sys 

argvs = sys.argv 
argc = len(argvs)

msg = """
Argvs:
---------------------------
1: cog_x
2: cog_y
3: cog_z
4: i_xx
5: i_yy
6: i_zz
7: i_xy
8: i_xz
9: i_yz
10: mass
"""

print(msg)

#print argvs

if (argc != 11):
    print('Argvs is not enough')
    quit()

print(argvs)

cog_x = float(argvs[1]) * .001
cog_y = float(argvs[2]) * .001
cog_z = float(argvs[3]) * .001
i_xx = float(argvs[4])
i_yy = float(argvs[5])
i_zz = float(argvs[6])
i_xy = -float(argvs[7])
i_xz = -float(argvs[8])
i_yz = -float(argvs[9])
mass = float(argvs[10])

cog_i_xx = i_xx - (cog_y * cog_y  + cog_z * cog_z) * mass
cog_i_yy = i_yy - (cog_x * cog_x  + cog_z * cog_z) * mass
cog_i_zz = i_zz - (cog_x * cog_x  + cog_y * cog_y) * mass
cog_i_xy = i_xy + cog_x * cog_y * mass
cog_i_xz = i_xz + cog_x * cog_z * mass
cog_i_yz = i_yz + cog_y * cog_z * mass

print("<inertial>\n \t<origin xyz=\"%f %f %f\" rpy=\"0 0 0\"/>\n \t<mass value=\"%f\"/>\n \t<inertia\n \t ixx=\"%f\" iyy=\"%f\" izz=\"%f\"\n \t ixy=\"%f\" ixz=\"%f\" iyz=\"%f\"/> \n</inertial>" % (cog_x, cog_y, cog_z, mass, cog_i_xx, cog_i_yy, cog_i_zz, cog_i_xy, cog_i_xz, cog_i_yz))
