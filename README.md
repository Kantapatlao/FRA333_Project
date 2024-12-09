## **FRA333 Class Project: Path finding and position trajectory generation to avoid collision in 3 DOF Planar Revolute robot**

โครงงานนี้ถูกจัดทำขึ้นเพื่อใช้ในการศึกษาและพัฒนาการสร้างวิถีการเคลื่อนที่ (Trajectory Generation) สำหรับหุ่นยนต์ 3 DOF Planar revolute robot ที่สามารถหลบหลีกสิ่งกีดขวางภายในพื้นที่ทำงาน (Task Space) ได้ โดยการจำลองการเคลื่อนที่และลักษณะของหุ่น ผ่านการใช้ Python และอัลกอริทึมที่ใช้ในการค้นหาเส้นทาง (Path finding) ผลลัพท์ที่คาดหวังคือโปรแกรมจำลองแขนกล 3 DOF ที่สามารถเคลื่อนที่ไปยังเป้าหมายได้โดยหลีกเลี่ยงการชนสิ่งกีดขวางตามข้อกำหนดในขั้นตอนการวางแผนวิถี (Trajectory planning)
การทดสอบจะดำเนินการผ่านการจำลองด้วย Pygame ที่จะแสดงให้เห็นถึงการเคลื่อนที่ในพื่นที่ทำงานที่มีสิ่งกีดขวางหลากหลายรูปแบบ มี่ใช้เป็นเกฯฑ์ให้เห็นภาพในการทดสอบ

## Table of Contents
- [จุดประสงค์โครงการ](#จุดประสงค์โครงการ)
- [System Overview](#system-overview)
- [การใช้งานโปรแกรม](#การใช้งานโปรแกรม)
- [ผลการทดลอง](#ผลการทดลอง)
- [สรุปและวิเคราะห์ผล](#สรุปและวิเคราะห์ผล)
- [เอกสารอ้างอิง](#เอกสารอ้างอิง)

## **จุดประสงค์โครงการ**
1)	เพื่อศึกษาการเคลื่อนที่ของแขนกลแบบ Revolute ในปริภูมิ 2 มิติ  
2)	เพื่อศึกษาวิธีการสร้างวิถีการโคจรให้แขนกลเคลื่อนที่ไปยังเป้าหมายในขณะที่มีสิ่งกีดขวาง
3)	เพื่อศึกษาการทำ Path finding และ Trajectory planning สำหรับการสร้างวิถีโคจรที่หลบหลีกสิ่งกีดขวางและไปยังเป้าหมาย

**ขอบเขต**
1)  ตัวหุ่นยนต์
  - กำหนดให้ทุก ๆ ก้านของแขนกลเป็น Rigid body ที่มีเพียงความยาวเท่านั้น ไม่มีความกว้างและลึก
  - แขนกลเป็น Joint แบบ Revolute จำนวน 3 Joints (RRR Robot)
  - แต่ละก้านของแขนกลมีความยาวเท่ากัน
  - ระบบรู้สภาพแวดล้อมทั้งหมดก่อนเริ่มทำ Path planning
  - ตำแหน่งเริ่มต้น(Home)เหมือนกันทุกการทำงาน
  - พิจารณาเฉพาะ Kinematic (ไม่มีการจำลองหรือพิจารณาแรงภายนอก)
2) แผนที่และสิ่งกีดขวาง
  - แผนที่มีขนาดเท่ากันทุกแผนที่
  - ขอบของแผนที่นับเป็นสิ่งกีดขวาง
  - สิ่งกีดขวางและเป้าหมายจะไม่มีการย้ายตำแหน่งขณะกำลังแสดงผล
3) การแสดงผล
  - ใช้ Pygame ในการแสดงผลการเคลื่อนที่
  - สิ่งกีดขวางจะแสดงผลเป็นพื้นที่สีดำบนแผนที่
  - ตำแหน่งเป้าหมายจะอยู่ในพื่นที่แผนที่และไม่อยู่บนสิ่งกีดขวาง

8)	สร้างวิธีโคจรโดยควบคุมตำแหน่งเท่านั้น
9)	Simulation รับ input เป็น ตำแหน่งฐานปัจจุบันของ Robot arm ,ตำแหน่งของ End-effector ณ จุดเริ่มต้น ,จุดพิกัดที่ต้องการให้ End effector ,แผนที่สภาพแวดล้อมที่ระบุสิ่งกีดขวาง 
10)	Output ของ Simulation เป็น พิกัด via point และออกมาเป็นการจำลองการเคลื่อนที่ผ่าน PyGame
11)	จำลองการเคลื่อนที่ภายใน Simulation ด้วยภาษา python 

## **System Overview**




## **โปรดลง Library พวกนี้ก่อนรัน**
  - numpy
  - math
  - roboticstoolbox

**ลงโดยใช้ Command พวกนี้**
```bash
  - pip install numpy==1.24.4
  - pip install math
  - pip install pygame
```
**Clone Project**
ใช้ Command นี้
```bash
  - gh repo clone Kantapatlao/FRA333_Project
```
## **การใช้งานโปรแกรม**
**ไฟล์ map_generator.ipynb**

make_map(): สร้างแผนที่แบบกริดในรูปแบบ ndarray ของ numpy จากขนาดแผนที่และจำนวนสิ่งกีดขวางที่กำหนด โดยที่สิ่งกีดขวางจะมีค่าเป็น 1 และเส้นทางที่สามารถเดินได้จะมีค่าเป็น 0

**ไฟล์ map_optimizer.py**

Discrete map: เป็นคลาสที่ใช้เก็บข้อมูลของแผนที่ในรูปแบบของตำแหน่ง X,Y ขนาดของแผนที่และสถานะของตำแน่งนั้นว่าเป็นสิ่งกีดขวางหรือไม่ประกอบไปด้วย
  - get_center_pos(): ส่งคืนค่าตำแหน่งศูนย์กลางของ Discrete map
  - get-bottonm_right_pos(): ส่งคืนค่าตำแหน่งมุมขวาล่างของ Discrete map
  - scale_discrete_map: ส่งคือ discrete map ใไม่ที่ปรับขนาดตามค่าที่กำหนด
Map: เป็นคลาสที่นำข้อมูลแผนที่แบบกริดมาแปลงเป็น Discrete map ประกอบไปด้วย
  - find_adjacent_node():รับข้อมูลจาก Discrete map มาแล้วส่งคืนแมพที่ ขอบบน, ล่าง, ขวา, ซ้ายติดกัน 
  - find_nearest_node(): 
  - list_obstacle():
  - show_graph():

**ไฟล์ robot.py**

Robot: คลาสที่เป็บข้อมูลของข้อต่อ, ความยาวของก้าน และตำแหน่งฐาน ประกอบไปด้วย
  - set_base_position():
  - forward_kinematic():
  - sequencial_IK_3():
  - check_wall_collision():
  - check_object_collision():
  - draw_robot():

**ไฟล์ A_Star.py**
_A_Star_Node: เป็นคลาสสำหรับใช้คำนวน A* ที่ใช้สำหรับประกาศก่อนใช้งาน

## **ผลการทดลอง**
**Examples : Map1.npy**
[Watch the video](D:\ปี3\MLwithju\ML_project\20241209-0339-49.7498429.mp4)
The simulation begins with the robot arm in its home position and a known target point. The inverse kinematics approach is used to calculate the joint (q) values required to move the robot arm to the target position.

**Examples : Out of Reach**

pic ture 2"D:ปี3\MLwithju\results\2 out of reach.png"

**Examples : Spawn On Obstacle**
pic ture 3"D:ปี3\MLwithju\results\3"

**การตรวจสอบความถูกต้อง : Map1.npy**
สำหรับการตรวจสอบความถูกต้อง เราสามารถแสดงผลลัพธ์ได้โดยใช้ทั้ Forward kinematic และ Inverse kinematics ของแขนหุ่นยนต์ เพื่อคำนวณและตรวจสอบตำแหน่งเป้าหมาย

**การตรวจสอบความถูกต้อง : Calculate About Target**
สำหรับการตรวจสอบความถูกต้อง เราสามารถแสดงผลลัพธ์โดยใช้ทั้งการคำนวณ Forward kinematic และ Inverse kinematics ของแขนหุ่นยนต์เพื่อคำนวณและยืนยันตำแหน่งเป้าหมาย

## **สรุปและวิเคราะห์ผล**
จากการศึกษาและลองทำแบบจำลองการเคลื่อนที่ของแขนกลแบบ 3 DOF Relolue ในสองมิติและประยุกต์ใช้ร่วมกับการทำ Path finding และ Trajectory planning โดยใช้ A* Search algorithm พบว่าจากผลลัพธ์ที่ได้แขนของหุ่นยนต์สามารถเคลื่อนที่ไปยังเป้าหมายโดยไม่โดนสิ่งกีดขวางและเมื่อตำแหน่งเป้าหมายอยู่เกินระยะแแขนหรือไม่มีเส้นทางที่ไปได้โดยไม่ชนก็สามารถส่ง Feedback กลับมาได้  แต่ในบางแผนที่และบางตำแหน่งจะยังไม่สามารถเคลื่อนที่ไปได้ เนื่องจากสมการ Inverse kinematic ที่ใช้ในการคิดกลับมาหาตำแหน่ง joint จึงทำให้ Logic ที่เช็คระยะมีปัญหาไปด้วย ในส่วนของ A* Search algorithm ยังพิสูจน์ไม่ได้ว่า Path ที่สร้างข้นมานั้น Optimal

## **เอกสารอ้างอิง**
Recent advance in Rapidly – Exploring random tree: A review
https://www.sciencedirect.com/science/article/pii/S2405844024084822

A Systematic Literature Review of A* Pathfinding
https://www.sciencedirect.com/science/article/pii/S1877050921000399



