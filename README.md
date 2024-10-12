# FRA333 Homework Assignment 3: Static Force

โปรเจคนี้จัดทำเพื่อประยุกต์ใช้ องค์ความรู้เกี่ยวกับการหาจลนศาสตร์เชิงอนุพันธ์ (Differential kinematics) ของหุ่นยนต์แขนกล 3 แกน (3-DOF Manipulator)

![Robot](Image\pic1.png)


## Table of Contents (TOC)
- [FRA333 Homework Assignment 3: Static Force](#fra333-homework-assignment-3-static-force)
  - [Table of Contents (TOC)](#table-of-contents-toc)
  - [การติดตั้ง](#การติดตั้ง)
  - [การเรียกใช้งาน](#การเรียกใช้งาน)
    - [ขั้นตอนการโหลด](#ขั้นตอนการโหลด)
    - [ขึ้นตอนการเรียกใช้ Function](#ขึ้นตอนการเรียกใช้-function)
    - [ขึ้นตอนการเรียกใข้ไฟล์การตรวจสอบ](#ขึ้นตอนการเรียกใข้ไฟล์การตรวจสอบ)
  - [ข้อ 1 คำนวณ Jacobian](#ข้อ-1-คำนวณ-jacobian)
    - [ขั้นตอนการทำงาน](#ขั้นตอนการทำงาน)
    - [ขั้นตอนการตรวจสอบ](#ขั้นตอนการตรวจสอบ)
    - [Output](#output)
  - [ข้อ 2 คำนวณหาสภาวะ Singularity](#ข้อ-2-คำนวณหาสภาวะ-singularity)
    - [ขั้นตอนการทำงาน](#ขั้นตอนการทำงาน-1)
    - [ขั้นตอนการตรวจสอบ](#ขั้นตอนการตรวจสอบ-1)
    - [Output](#output-1)
  - [ข้อ 3 คำนวณหา Effort ของแต่ละข้อต่อ](#ข้อ-3-คำนวณหา-effort-ของแต่ละข้อต่อ)
    - [ขั้นตอนการทำงาน](#ขั้นตอนการทำงาน-2)
    - [ขั้นตอนการตรวจสอบ](#ขั้นตอนการตรวจสอบ-2)
    - [Output](#output-2)
  - [ผู้จัดทำ](#ผู้จัดทำ)

## การติดตั้ง
ก่อนที่จะเรียกใช้งานได้นั้นเราต้องติดตััง Libary ของ [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) เนื่องจากเราใช้ในการตรวจคำตอบของ Func ที่เราทำขึ้่นมา


การติดตั้งจะมี Recommend ดังนี้
```Recommend
Python >= 3.6
numpy <2
และลง visual studio c++ compiler
```
เมื่อเรามีครบทั้งหมดแล้วเราสามารถลง Robotics Toolbox for python ได้ตามคำสั่งนี้
```bash
pip3 install roboticstoolbox-python
```

## การเรียกใช้งาน 
### ขั้นตอนการโหลด

เราต้อง Clone repository [FRA333_HW3_6504_6527](https://github.com/ThepokKung/FRA333_HW3_6504_6527) โดยการใช้คำสั่ง
```bash
git clone https://github.com/ThepokKung/FRA333_HW3_6504_6527
```
หรือสามารถโหลดจาก Website ได้เลยดังรูป

![แสดงการเรียกใช้งาน-1](Image\2-1.png)

### ขึ้นตอนการเรียกใช้ Function
โดยวิธีการใช้งานเราสามารถ Import ไฟล์เราไปใช้ได้เลยเหมือนใช้ Libary แต่ต้องลากไปยัง Folder รวมกับไฟล์ที่จะเรียกใช้งาน โดยเมื่ออยู่ที่ Foler เดี่ยวกันแล้วสามารถเรียกใช้คำสั่ง
```python
from FRA333_HW3_6504_6527 import *
```
เพื่อเป็นการใช้งาน Function ที่ผู้จัดทำสร้างขึ้น และเราสามารถนำไปใช้ได้เลยดังตัวอย่าง
```python
from FRA333_HW3_6504_6527 import *
J = endEffectorJacobianHW3(q)
S = checkSingularityHW3(q)
tau = computeEffortHW3(q,w)
```

### ขึ้นตอนการเรียกใข้ไฟล์การตรวจสอบ
หลังจากที่ผู้จัดทำได้เขียน Function ขึ้นมาแล้วต่อมาเราได้ทำงานตรวจสอบคำตอบของเรา สามารถเรียกใช้ไฟล์ testScript.py ได้เลยเพื่อตรวจสอบว่า Function ที่เราเขียนมานั่นถูกต้อง เมื่อเทียบกับ Libary Robotic Toolbox for python สามารถเรียกคำสั่ง

```python
python testScript.py
```

## ข้อ 1 คำนวณ Jacobian
### ขั้นตอนการทำงาน
ฟังก์ชันนี้จะคำนวณเมทริกซ์ Jacobian ที่ตำแหน่ง end-effector โดยมีการรับค่ามุมแต่ละข้อต่อ `q`

**Function**
    - endEffectorJacobianHW3(q: list[float]) -> list[float]) 
  - **Input**:
    ค่ามุมแต่ละข้อต่อ
    - `q`: มุมแต่ละข้อต่อในรูปแแบบ array (จำนวน 3 ข้อต่อสำหรับหุ่นยนต์ 3-DOF)

  - **Output**:
    ความสัมพันธ์ระหว่างความเร็วเชิงเส้นและเชิงมุม
    - เมทริกซ์ Jacobian ขนาด 6 x i โดยที่ i คือจำนวนข้อต่อ (จำนวน 3 ข้อต่อสำหรับหุ่นยนต์ 3-DOF)

  - **อธิบายโค้ดโดยละเอียด**
    
    คำนวณ ForwordKinematic จาก `FKHW3` เพื่อหาการ Rotation และ Position ของเฟรม 0 ถึงแต่ละข้อต่อ
    ```
    R,P,R_e,p_e = FKHW3(q)
    ```
    สร้างตัวแปรขนาด 6x3 สำหรับเก็บ Linear Jacobian และ Angular Jacobian ของหุ่นยนต์ 3-DOF
    ```
    J = np.empty((6,3)) 
    ```
    คำนวณ Jacobian แต่ละข้อต่อ
    ```
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
    ```
    ### Output
    ทดลอง Input `q = [0,0,0]`
    ```
    [[-0.109   -0.093   -0.093  ]
    [ 0.89943  0.       0.     ]
    [ 0.      -0.89943 -0.47443]
    [ 0.      -0.      -0.     ]
    [ 0.       1.       1.     ]
    [ 1.       0.       0.     ]]
    ```
### ขั้นตอนการตรวจสอบ
สร้าง Robot RRR โดยใช้ Modify DH-Parameter ของ roboticstoolbox
```
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d = 0.0892, offset = pi), # frame 0 ไป 1
        rtb.RevoluteMDH(alpha = pi/2), # frame 1 ไป 2
        rtb.RevoluteMDH(a = -0.425), # frame 2 ไป 3
    ],tool = SE3(-0.47443, -0.093, 0.109) * SE3.RPY(0, -pi/2, 0, order='zyx')# frame 3 ไป e
)
```
ใช้ฟังก์ชันการคำนวณ Jacobian ของ roboticstoolbox และแสดงค่า โดยเทียบจากเฟรม 0
```
J_sol = robot.jacob0(q)
print(f'Jacobian :\n {J_sol.round(5)}') # round(5) คือ แสดงทศนิยม 5 ตำแหน่ง
```
### Output
ทดลอง Input `q = [0,0,0]` แล้วเปรียบเทียบผลลัพธ์ที่ได้กับของเรา
```
Jacobian :
 [[-0.109   -0.093   -0.093  ]
 [ 0.89943  0.       0.     ]
 [-0.      -0.89943 -0.47443]
 [ 0.       0.       0.     ]
 [-0.       1.       1.     ]
 [ 1.       0.       0.     ]]
```
ผลการเปรียบเทียบ คือ ได้ผลลัพธ์ที่ตรงกัน


## ข้อ 2 คำนวณหาสภาวะ Singularity
### ขั้นตอนการทำงาน
ฟังก์ชันนี้จะนำเมทริกซ์ Jacobian ที่ตำแหน่ง end-effector มาหาสภาวะ Singularity โดยมีการรับค่ามุมแต่ละข้อต่อ `q`

**Function**
    - checkSingularityHW3(q:list[float])->bool
  - **Input**:
    ค่ามุมแต่ละข้อต่อ
    - `q`: มุมแต่ละข้อต่อในรูปแแบบ array (จำนวน 3 ข้อต่อสำหรับหุ่นยนต์ 3-DOF)

  - **Output**:
    สภาวะ Singularity
    - Boolean โดยที่มีค่าเป็น True ก็ต่อเมื่ออยู่ตำแหน่งใกล้สภาวะ Singularity     
            มีค่าเป็น False เมื่อแขนกลอยู่ในสภาวะปกติ

  - **อธิบายโค้ดโดยละเอียด**
    
    คำนวณ Jacobian จากฟังก์ชันในข้อ 1
    ```
    J = endEffectorJacobianHW3(q)
    ```
    ลดรูป Jacobian นำส่วนที่เราไม่สามารถ control ได้ออก
    ```
    J_reduce = J[:3,:3]
    ```
    คำนวณหาค่า det ของ Jacobian ที่ลดรูปแล้ว
    ```
    check = np.linalg.det(J_reduce)
    ```
    กำหนดค่าและเงื่อนไขการเข้าสู่สภาวะ Singularity
    ```
    epsilon = 0.001
    if check < epsilon:
        return True
    else:
        return False
    ```
    ### Output
    ทดลอง Input `q = [0,0,0]`
    ```
    False
    ```
### ขั้นตอนการตรวจสอบ
นำ Robot RRR ที่สร้างในข้อ 1 มาคำนวณ Jacobian แบบลดรูป
```
J_sol_redece = robot.jacob0(q)[:3,:3]
```
คำนวณหาค่า det ของ Jacobian ที่ลดรูปแล้ว
```
check = np.linalg.det(J_sol_redece)
```
กำหนดค่าและเงื่อนไขการเข้าสู่สภาวะ Singularity
```
epsilon = 0.001
if check < epsilon:
    print(f'Singularity flag = {True}')
else:
    print(f'Singularity flag = {False}')
```
### Output
ทดลอง Input `q = [0,0,0]` แล้วเปรียบเทียบผลลัพธ์ที่ได้กับของเรา
```
Singularity flag = False
```
ผลการเปรียบเทียบ คือ ได้ผลลัพธ์ที่ตรงกัน


## ข้อ 3 คำนวณหา Effort ของแต่ละข้อต่อ
### ขั้นตอนการทำงาน
ฟังก์ชันนี้จะนำเมทริกซ์ Jacobian ที่ตำแหน่ง end-effector และแรงที่สามารถอ่านค่าแรงได้จาก Sensor มาคำนวณหา Effort ของแต่ละข้อต่อ

โดยมีการรับค่ามุมแต่ละข้อต่อ `q` กับเวกเตอร์ที่ประกอบด้วยแรงและโมเมนต์ `w`

**Function**
    - computeEffortHW3(q:list[float], w:list[float])->list[float]
  - **Input**:
    ค่ามุมแต่ละข้อต่อ กับเวกเตอร์ที่ประกอบด้วยแรงและโมเมนต์
    - `q`: มุมแต่ละข้อต่อในรูปแแบบ array (จำนวน 3 ข้อต่อสำหรับหุ่นยนต์ 3-DOF)
    - `w`: เวกเตอร์ที่ประกอบด้วยแรงและโมเมนต์ มีขนาด 2i x 1 โดยที่ i คือจำนวนข้อต่อ (จำนวน 3 ข้อต่อสำหรับหุ่นยนต์ 3-DOF)

  - **Output**:
    Effort ของแต่ละข้อต่อ
    - เมทริกซ์ ขนาด i x 1 โดยที่ i คือจำนวนข้อต่อ (จำนวน 3 ข้อต่อสำหรับหุ่นยนต์ 3-DOF)

  - **อธิบายโค้ดโดยละเอียด**
    
    คำนวณ Jacobian จากฟังก์ชันในข้อ 1
    ```
    J = endEffectorJacobianHW3(q)
    ```
    นำ Jacobian มา transpose แล้วนำไป dot กับเวกเตอร์ที่ประกอบด้วยแรงและโมเมนต์ จะได้ Effort ของแต่ละข้อต่อ
    ```
    tau = np.dot(np.transpose(J),w)
    return tau
    ```

    ### Output
    ทดลอง Input ดังนี้
    ```
    q = [0,0,0]
    w = np.array([1,2,3,4,5,6])
    w = w.reshape(6,1)
    ```
    โดย w[0] ถึง w[2] เป็น force ของแต่ละ joint ตามลำดับ

    w[3] ถึง w[5] เป็น moment ของแต่ละ joint ตามลำดับ

    ผลลัพธ์
    ```
    [[7.68986]
    [2.20871]
    [3.48371]]
    ```
    
### ขั้นตอนการตรวจสอบ
นำ Robot RRR ที่สร้างในข้อ 1 มาคำนวณ Jacobian
```
J_sol = robot.jacob0(q)
```
นำ Jacobian มา transpose แล้วนำไป dot กับเวกเตอร์ที่ประกอบด้วยแรงและโมเมนต์ จะได้ Effort ของแต่ละข้อต่อ และแสดงผลลัพธ์ที่ได้นั้น
```
tau = np.dot(np.transpose(J_sol),w)
print(f'effort ของแต่ละข้อต่อเรียงลำดับจากข้อต่อที่ 1-3 : \n {tau} ')
```
### Output
ทดลอง Input ดังนี้
```
q = [0,0,0]
w = np.array([1,2,3,4,5,6])
w = w.reshape(6,1)
```
โดย w[0] ถึง w[2] เป็น force ของแต่ละ joint ตามลำดับ

w[3] ถึง w[5] เป็น moment ของแต่ละ joint ตามลำดับ

ผลลัพธ์
```
effort ของแต่ละข้อต่อเรียงลำดับจากข้อต่อที่ 1-3 :
 [[7.68986]
 [2.20871]
 [3.48371]]
```

## ผู้จัดทำ
* ไกรวิชญ์ วิชาโคตร   65340500004
* ธเนศพล หีบแก้ว    65340500027