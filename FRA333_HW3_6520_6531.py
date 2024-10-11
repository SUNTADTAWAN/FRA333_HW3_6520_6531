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

    # Assign variable from Forward kinematics
    R, P, R_e, p_e = FKHW3(q) 

    # Assign position and rotation matrix
    p_01 = P[:,0]
    p_02 = P[:,1]
    p_03 = P[:,2]
    p_0e = p_e
    
    R_01 = R[:,:,0]
    R_02 = R[:,:,1]
    R_03 = R[:,:,2]
    R_0e = R_e
    
    # Crate array vector [0,0,1] Rotation around Z axix
    z = np.array([0.0, 0.0, 1.0]).reshape(3,1)
    
    # Create rotation axis frame
    z_01 = R_01 @ z
    z_02 = R_02 @ z
    z_03 = R_03 @ z
    # z_0e = R_0e @ z
    
    # Compute linear velocity Jacobian components
    Jv_1 = np.cross(z_01.T, (p_0e - p_01).T).T
    Jv_2 = np.cross(z_02.T, (p_0e - p_02).T).T
    Jv_3 = np.cross(z_03.T, (p_0e - p_03).T).T

    # Combine linear velocity components into J_v 
    J_v = np.hstack([Jv_1, Jv_2, Jv_3])
    
    # The angular velocity Jacobian components in J_w
    J_w = np.hstack([z_01, z_02, z_03])
    
    # Construct full Jacobian Matrix with Linear and Angular velocity
    J = np.vstack([J_v, J_w])
    
    return J
    
    
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    # Find Jacobian matrix
    J_e = endEffectorJacobianHW3(q)
    
    # Only linear velocity effect to singularity
    J_e_reduced = J_e[:3, :3] 
    
    # Find Determinant 
    det_J_e = np.linalg.det(J_e_reduced)

    # Check Singularity with threshold 0.01
    if (abs(det_J_e) < 0.001):
        return 1 
    else:
        return 0 

#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    """
    Use -J_3*w because the joint torques represent reaction forces opposing 
    the applied wrench on the end-effector, following Newton's third law.

    """
    
    # Find Jacobian Matrix
    J_e = endEffectorJacobianHW3(q)
    
    # Transpose Jacobian
    J_e_Transpose = J_e.T 

    # Find Effort from tau = J_e_transpose * w
    tau = np.dot(-J_e_Transpose, w) 
    
    return tau
#==============================================================================================================#


