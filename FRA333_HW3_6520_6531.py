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
    R, P, R_e, p_e = FKHW3(q) 

    n = len(q) 

    J = np.zeros((6, n))
    
    for i in range(n):
        p_i = P[:, i] 
        
        z_i = R[:, 2, i] 
        
        J[:3, i] = np.cross(z_i, (p_e - p_i))  
        
        J[3:, i] = z_i  

    J_e_translational = np.dot(R_e.T, J[:3, :])  
    J_e_rotational = np.dot(R_e.T, J[3:, :])    
    J_e = np.vstack((J_e_translational, J_e_rotational))
    return J_e
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J = endEffectorJacobianHW3(q)
    
    J_reduced = J[:3, :3] 
    
    det_J = np.linalg.det(J_reduced)
    
    if (abs(det_J) < 0.001):
        return 1 
    else:
        return 0 

#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J = endEffectorJacobianHW3(q)
    
    tau = np.dot(J.T, w)
    
    return tau
#==============================================================================================================#