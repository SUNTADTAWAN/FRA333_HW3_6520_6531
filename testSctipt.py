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
from FRA333_HW3_6520_6531 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3
from spatialmath import SE3

#define variable
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
q_initial = [0, -pi/2, pi/2]
q_singulality = [0, -pi/2, -0.1]
w = np.array([0, 5, 0, 10, 0, 5])

#MDH parameter
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
print("========================================================================================================= \n")

#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
def check_jacobian(q: list[float]):
    # Assign Jacobian matrix from our Jacobian compute and "jacob0" from robotics toolsbox
    jacob_our = endEffectorJacobianHW3(q)
    jacob_rtb = robot.jacob0(q)

    # Find difference 
    diff_Jacobian = jacob_rtb - jacob_our

    # Create Threshold 
    threshold = 1e-15
    error = np.where(np.abs(diff_Jacobian) < threshold, 0, diff_Jacobian)
    
    print("#Number 1")
    print(f"Input q : {q}")
    print(f"Our Jacobian : \n {jacob_our} \n")
    print(f"Robotics toolsbox Jacobian : \n {jacob_rtb} \n")
    print(f"Difference of our and toolsbox : \n {error}")
    print("--------------------------------------------------------------------------------------------- \n")
    
    return error
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def check_singularity(q: list[float]):
    # Assign Singularity check output from our function
    our_sin = checkSingularityHW3(q)
    
    # Define jacobian and compute Singularity by determinant
    J = robot.jacob0(q)
    J_e_reduced = J[:3, :3] 
    det_J_e = np.linalg.det(J_e_reduced)
    
    # Condition check Singularity from robotics toolbox
    if (abs(det_J_e) < 0.001):
        sin = 1
    else:
        sin = 0 
        
    print("#Number 2")
    print(f"Input q : {q}")
    print(f"Singularity rtb : {sin}, our : {our_sin}")
    print("Near Singularity" if sin and our_sin else "Not Singularity")
    print("--------------------------------------------------------------------------------------------- \n")

    
    return sin and our_sin
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def check_effort(q: list[float],w: list[float]):
    # Effort from our by J_transpose * w
    tau_our = computeEffortHW3(q,w)
    
    # Effort from robotics-toolsbox
    J = robot.jacob0(q)
    tau_rtb = robot.pay(w, J=J, frame=0)
    
    # Finding error
    error = tau_rtb - tau_our
    
    # Create Threshold 
    threshold = 1e-15
    error = np.where(np.abs(error) < threshold, 0, error)
    print("#Number 3")
    print(f"Input q: {q}, Input w: {w}")
    print(f"Our Effort : \n {tau_our} ")
    print(f"Robotics toolsbox Effort : \n {tau_rtb}")
    print(f"Difference of our and toolsbox : \n {error}")
    print("--------------------------------------------------------------------------------------------- \n")
    return error

print(robot)

jacob_checker = check_jacobian(q_initial)
# sing_checker = check_singularity(q_singulality)
# effort_checker = check_effort(q_initial,w)
# print("========================================================================================================= \n")


# print(f"Output Jacobian : \n {jacob_checker} , \n Singularity : {sing_checker} , \n Effort : {effort_checker}")

