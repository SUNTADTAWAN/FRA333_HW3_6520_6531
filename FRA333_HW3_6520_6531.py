# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.Napat_6520
2.Tadtawan_6531
'''
#import library
from HW3_utils import FKHW3
import numpy as np

#=============================================<คำตอบข้อ 1>======================================================#

#code here
def endEffectorJacobianHW3(q: list[float]) -> np.ndarray:

    R, P, R_e, p_e = FKHW3(q) # Forward_kinematics 

    J_e = np.zeros((6, 3)) # Jacobian 

    p_i = P[:, 0] # Position of Joint 0 from base frame
    z_i = R[:, 2, 0] # Z position of frame 0
    delta_position = p_e - p_i # find diff position of frame0 to end_effector
    J_e[:3, 0] = np.cross(z_i, delta_position) # cross z_i x delta_position (Jv)
    Jw = z_i # Rotation Jacobian
    J_e[3:, 0] =  Jw # Rotation velocity

    p_i = P[:, 1] # Position of Joint 1 from base frame
    z_i = R[:, 2, 1] # Z position of frame 1
    delta_position = p_e - p_i # find diff position of frame1 to end_effector
    J_e[:3, 1] = np.cross(z_i, delta_position) # cross z_i x delta_position (Jv)
    Jw = z_i # Rotation Jacobian
    J_e[3:, 1] = Jw # Rotation velocity

    p_i = P[:, 2] # Position of Joint 2 from base frame
    z_i = R[:, 2, 2] # Z position of frame 2
    delta_position = p_e - p_i # find diff position of frame2 to end_effector
    J_e[:3, 2] = np.cross(z_i, delta_position) # cross z_i x delta_position (Jv)
    Jw = z_i # Rotation Jacobian
    J_e[3:, 2] = Jw # Rotation velocity

    J_e_translational = np.dot(R_e.T, J_e[:3, :]) # Translation
    J_e_rotational = np.dot(R_e.T, J_e[3:, :]) # Rotation
    J_e = np.vstack((J_e_translational, J_e_rotational)) 
    return J_e
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J_e = endEffectorJacobianHW3(q)
    
    J_e_reduced = J_e[:3, :3] 
    
    det_J_e = np.linalg.det(J_e_reduced)
    
    if (abs(det_J_e) < 0.01):
        return 1 
    else:
        return 0 

#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J_e = endEffectorJacobianHW3(q)
    
    J_e_Transpose = J_e.T # Transpose Jacobian

    tau = np.dot(J_e_Transpose, w) 
    
    return tau
#==============================================================================================================#