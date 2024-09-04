# 2R inverse kinematics model
import math
import numpy

L4 = 1
L5 = 1
L6 = 1

# Desired Coordinates
xd = 2
yd = 0
zd = 1
phi = 0

# Translated coordinates to radius and height
rad_des = math.sqrt((math.pow(xd,2)+math.pow(yd,2)))
z_des = zd

#Make 2R offset changes
rad = rad_des - L6*math.cos(phi)
z = z_des - L6*math.sin(phi)
rad_2r = math.sqrt((math.pow(rad,2)+math.pow(z,2)))




if ((L4+L5) >= rad_2r):
    print('Valid desired position')
    theta1 = math.atan2(yd,xd)

    #Elbow up/down variable
    sigma = 1

   
    ctheta3 = (math.pow(rad,2)+math.pow(z,2)-math.pow(L4,2)-math.pow(L5,2))/(2*L4*L5)

    #theta3 = sigma*math.acos(ctheta3)
    theta3 = math.atan2((sigma*math.sqrt((1-math.pow(ctheta3,2)))),ctheta3)  # math.atan2 passes y first then x as arguements, may need to switch

    theta2 = math.atan2(z,rad) - math.atan2((L5*math.sin(theta3)),(L4+L5*math.cos(theta3)))

    theta4 = phi-(theta2+theta3)

    print(theta1,theta2,theta3,theta4)
else:
    print('not in taskspace')

# this is an extra comment to make the file neater

#updating script for branch







