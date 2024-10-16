# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1. ไกรวิชญ์_6504
2. ธเนศพล_6527
'''
import roboticstoolbox as rtb
import numpy as np
from math import pi
from spatialmath import SE3
from spatialmath.base import *
#===========================================<กำหนด q กับ w>====================================================#
q = [0,0,0] #Input เป็น Configulation space
#Input เป็น แรงได้จาก Sensor
#โดย w[0] ถึง w[2] เป็น force ของแต่ละ joint ตามลำดับ
#w[3] ถึง w[5] เป็น moment ของแต่ละ joint ตามลำดับ
w = np.array([1,2,3,4,5,6])
w = w.reshape(6,1)
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here

#สร้าง Robot RRR โดยใช้ Modify DH-Parameter
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d = 0.0892, offset = pi), # frame 0 ไป 1
        rtb.RevoluteMDH(alpha = pi/2), # frame 1 ไป 2
        rtb.RevoluteMDH(a = -0.425), # frame 2 ไป 3
    ],tool = SE3(-0.47443, -0.093, 0.109) * SE3.RPY(0, -pi/2, 0, order='zyx')# frame 3 ไป e
)
#คำนวณ Jacobian และแสดงค่า
J_sol = robot.jacob0(q)
print(f'Jacobian :\n {J_sol.round(5)}') # round(5) คือ แสดงทศนิยม 5 ตำแหน่ง

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
# คำนวณ Jacobian แบบลดรูป
J_sol_redece = robot.jacob0(q)[:3,:3]
# คำนวณหาค่า det ของ Jacobian ที่ลดรูปแล้ว
check = np.linalg.det(J_sol_redece)
# กำหนดค่าและเงื่อนไขการเข้าสู่สภาวะ Singularity
epsilon = 0.001
if check < epsilon:
    print(f'Singularity flag = {True}')
else:
    print(f'Singularity flag = {False}')
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
# คำนวณ Jacobian
J_sol = robot.jacob0(q)
# คำนวณ effort ของแต่ละข้อต่อ
tau = np.dot(np.transpose(J_sol),w)
print(f'effort ของแต่ละข้อต่อเรียงลำดับจากข้อต่อที่ 1-3 : \n {tau} ')
#==============================================================================================================#