# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย

'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.Napat_6520
2.Tadtawan_6531
'''

#import library
import numpy as np
from math import pi
import roboticstoolbox as rtb
import matplotlib
matplotlib.use('TkAgg')  # บังคับใช้ Tkinter backend
import matplotlib.pyplot as plt

from HW3_utils import FKHW3
from FRA333_HW3_6520_6531 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3
from spatialmath import SE3

#define variable
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
q_initial = [0, 0, 0]
q_singulality = [0, 0, pi]
w = np.array([0, 0, 0, 10, 0, 0])

#Find MDH parameter
robot = rtb.DHRobot([
                rtb.RevoluteMDH(alpha=0, a=0, d=d_1, offset=pi), 
                rtb.RevoluteMDH(alpha=pi/2, a=0, d=0, offset=0),
                rtb.RevoluteMDH(alpha=0, a=-a_2,  d=0, offset=0)], 
                    tool = SE3([
                    [0, 0, -1, -(a_3 + d_6)],
                    [0, 1, 0, -d_5],
                    [1, 0, 0, d_4],
                    [0, 0, 0, 1]]),
                    name = "3DOF_Robot")

