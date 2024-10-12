#================================================<Import>======================================================#
import numpy as np
import math
from HW3_utils import FKHW3
#==============================================================================================================#
# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1. ไกรวิชญ์_6504
2. ธเนศพล_6527
'''
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    # คำนวณ ForwordKinematic เพื่อหา R, P, R_e,P_e
    R,P,R_e,p_e = FKHW3(q)

    # สร้างตัวแปรขนาด 6x3 ของ Linear Jacobian และ Angular Jacobian
    J = np.empty((6,3)) 

    #หาจำนวณของ  Joint โดยเอาจาก q ที่ Input เข้ามา
    num_joint = len(q)
    
    for i in range(num_joint):
        # คำนวณค่า rotation joint ที่ i
        if i == 0:
            rotation = [0,0,1] # คำนวณค่า rotation joint ที่ 1
        else:
            rotation = [-np.sin(q[0]),np.cos(q[0]),0] # คำนวณค่า rotation joint ที่ 2,3 มีการแปลงเพราะแกนหมุนอยู่คนละทิศทางกันกับเฟรมเริ่มต้น
        # ระยะห่างระหว่าง p_e และ p joint ที่ i
        diffrent = p_e - P[:,i] #

        # หา Linear velocity สำหรับ Jacobian แบบข้อต่อ Revolute
        J_v_i = np.cross(rotation,diffrent)
        J[0][i] = J_v_i[0]
        J[1][i] = J_v_i[1]
        J[2][i] = J_v_i[2]
        # หา Angular velocity สำหรับ Jacobian แบบข้อต่อ Revolute
        J_w_i = rotation
        J[3][i] = J_w_i[0]
        J[4][i] = J_w_i[1]
        J[5][i] = J_w_i[2]
    return J
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    # คำนวณ Jacobian
    J = endEffectorJacobianHW3(q)
    # ลดรูป Jacobian
    J_reduce = J[:3,:3]
    # คำนวณหาค่า det ของ Jacobian ที่ลดรูปแล้ว
    check = np.linalg.det(J_reduce)
    # กำหนดค่าและเงื่อนไขการเข้าสู่สภาวะ Singularity
    epsilon = 0.001
    if check < epsilon:
        return True
    else:
        return False
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    # คำนวณ Jacobian
    J = endEffectorJacobianHW3(q)
    # คำนวณ effort ของแต่ละข้อต่อ
    tau = np.dot(np.transpose(J),w)
    return tau
#==============================================================================================================#