# **FRA333 Class Project: Pathfinding and Position Trajectory Generation for Collision Avoidance in a 3-DOF Planar Revolute Robot**

โครงงานนี้ถูกจัดทำขึ้นเพื่อใช้ในการศึกษาและพัฒนาการสร้างวิถีการเคลื่อนที่ (Trajectory Generation) สำหรับหุ่นยนต์ 3 DOF Planar revolute robot ที่สามารถหลบหลีกสิ่งกีดขวางภายในพื้นที่ทำงาน (Task Space) ได้ โดยการจำลองการเคลื่อนที่และลักษณะของหุ่น ผ่านการใช้ Python และอัลกอริทึมที่ใช้ในการค้นหาเส้นทาง (Path finding) ผลลัพธ์ที่คาดหวังคือโปรแกรมจำลองแขนกล 3 DOF ที่สามารถเคลื่อนที่ไปยังเป้าหมายได้โดยหลีกเลี่ยงการชนสิ่งกีดขวางตามข้อกำหนดในขั้นตอนการวางแผนวิถี (Trajectory planning)
การทดสอบจะดำเนินการผ่านการจำลองด้วย Pygame ที่จะแสดงให้เห็นถึงการเคลื่อนที่ในพื้นที่ทำงานที่มีสิ่งกีดขวางหลากหลายรูปแบบ ที่ใช้เป็นเกณฑ์ให้เห็นภาพในการทดสอบ

# Table of Contents
- [จุดประสงค์โครงการ](#จุดประสงค์โครงการ)
- [System Overview](#system-overview)
- [หลักการทำงานของโปรแกรม](#หลักการทำงานของโปรแกรม)
  - [map_generator.ipynb](#map_generatoripynb)
  - [map_optimizer.py](#map_optimizerpy)
  - [robot.py](#robotpy)
  - [A_Star.py](#a_starpy)
- [การใช้งานโปรแกรม](#การใช้งานโปรแกรม)
- [ผลการทดลอง](#ผลการทดลอง)
- [สรุปและวิเคราะห์ผล](#สรุปและวิเคราะห์ผล)
- [แนวทางการแก้ไขและพัฒนาต่อ](#แนวทางการแก้ไขและพัฒนาต่อ)
- [เอกสารอ้างอิง](#เอกสารอ้างอิง)

# **จุดประสงค์โครงการ**
1) ศึกษาการเคลื่อนที่ของแขนกลแบบ Revolute ในพื้นที่สองมิติ (2D Space)  
2) ศึกษาวิธีการสร้างวิถีการเคลื่อนที่ (Trajectory Planning) ให้แขนกลสามารถเคลื่อนที่ไปยังเป้าหมายในขณะที่หลีกเลี่ยงสิ่งกีดขวาง  
3) พัฒนาการค้นหาเส้นทาง (Pathfinding) และการวางแผนวิถีการเคลื่อนที่ (Trajectory Planning) ที่เหมาะสมสำหรับการหลบหลีกสิ่งกีดขวางเพื่อไปยังเป้าหมาย


**ขอบเขต**
1)  ตัวหุ่นยนต์
  - กำหนดให้ทุก ๆ ก้านของแขนกลเป็น Rigid body ที่มีเพียงความยาวเท่านั้น ไม่มีความกว้างและลึก
  - แขนกลเป็น Joint แบบ Revolute จำนวน 3 Joints (RRR Robot)
  - แต่ละก้านของแขนกลมีความยาวเท่ากัน
  - ตำแหน่งเริ่มต้น (Home) เหมือนกันทุกการทำงาน
  - พิจารณาเฉพาะ Kinematic (ไม่มีการจำลองหรือพิจารณาแรงภายนอก)
  - Simulation รับ input เป็น:
    - ตำแหน่งฐานปัจจุบันของ Robot arm
    - ตำแหน่งของ End-effector ณ จุดเริ่มต้น
    - จุดพิกัดที่ต้องการให้ End-effector
2) แผนที่และสิ่งกีดขวาง
  - ระบบรู้สภาพแวดล้อมทั้งหมดก่อนเริ่มทำ Path planning
  - แผนที่มีขนาดเท่ากันทุกแผนที่
  - ขอบของแผนที่นับเป็นสิ่งกีดขวาง
  - สิ่งกีดขวางและเป้าหมายจะไม่มีการย้ายตำแหน่งขณะกำลังแสดงผล
  - สิ่งกีดขวางจะแสดงผลเป็นพื้นที่สีดำบนแผนที่
  - ตำแหน่งเป้าหมายจะอยู่ในพื้นที่แผนที่และไม่อยู่บนสิ่งกีดขวาง
  - Simulation รับ input เป็นแผนที่สภาพแวดล้อมที่ระบุสิ่งกีดขวาง
3) การแสดงผล
  - ใช้ Pygame ในการแสดงผลการเคลื่อนที่
  - Output ของ Simulation เป็น:
    - พิกัด via point
    - การจำลองการเคลื่อนที่ผ่าน Pygame
    - จำลองการเคลื่อนที่ภายใน Simulation ด้วยภาษา Python


# **System Overview**

![system_map](image%20for%20read%20me/SystemFinal2.png)



# **โปรดลง Library พวกนี้ก่อน**
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
# **หลักการทำงานของโปรแกรม**
## map_generator.ipynb

คือโปรแกรมที่ใช้ในการสร้างแผนที่ โดย

**make_map()** \
เป็นฟังค์ชั่นสร้างแผนที่แบบกริด (Grid map) โดยกำหนดขนาดใน sizeX และ sizeY จากนั้นเพิ่มสิ่งกีดขวางตามจำนวนที่ระบุใน obstacles_count หากไม่ได้ระบุ obstacles_count ฟังก์ชันจะสุ่มจำนวนสิ่งกีดขวางระหว่าง 1 ถึง 5 และวางไว้ในแผนที่ สิ่งกีดขวางจะถูกวางในตำแหน่งสุ่มภายในแผนที่ พร้อมขนาดและรูปร่างแบบสุ่ม
- **รูปแบบคำสั่ง**
```
make_map(sizeX, sizeY, obstacles_count)
```
- **พารามิเตอร์**
    - sizeX, sizeY: ขนาดของกริดแผนที่ ทั้งสองค่าต้องเป็น จำนวนเต็มบวก (unsigned integer) เท่านั้น ฟังก์ชันนี้ไม่แปลงชนิดตัวแปรให้อัตโนมัติ
    - obstacles_count: จำนวนสิ่งกีดขวางที่วางในแผนที่ ต้องเป็น จำนวนเต็มบวก (unsigned integer) เท่านั้น ฟังก์ชันนี้ไม่แปลงชนิดตัวแปรให้อัตโนมัติ โดย obstacles_count สามารถละเว้นได้ และฟังก์ชันจะสุ่มทำการจำนวนสิ่งกีดขวางตั้งแต่ 1 ถึง 5 เพื่อวางภายในแผนที่

- **ค่าที่ส่งกลับมา**\
    จะคืนค่าแผนที่ 2 มิติในรูปแบบของ numpy.ndarray ขนาดตามที่ระบุใน sizeX และ sizeY โดยเส้นทางที่สามารถเดินได้จะถูกแทนด้วยค่า 0 และสิ่งกีดขวางจะแทนด้วยค่า 1

- **ตัวอย่างแผนที่ที่ได้ออกมา**\
ภาพแผนที่ได้ออกมา\
![map1](image%20for%20read%20me/map_from_map_gen.png)\
![map2](image%20for%20read%20me/map_gen_2.png)\
![map3](image%20for%20read%20me/map_gen_3.png)

เมื่อไม่สามารถ generate แผนที่ออกมาได้\
![map4](image%20for%20read%20me/map_gen_4.png)

## map_optimizer.py
ใช้ปรับแต่งแผนที่ที่ได้มาจาก map_generator โดย\
**BT_Node()**\
เป็นออบเจ็กต์ที่ใช้สำหรับแบ่งแผนที่กริดที่ได้มาเป็นส่วนย่อยๆ
- **รูปแบบคำสั่ง**
```
BT_Node(self, data_in, posX_in, posY_in)
```
- **คุณลักษณะ**
  - data: ข้อมูลของโหนด (อาจเก็บข้อมูลในรูปแบบ ndarray สำหรับแผนที่กริด หรือ Discrete_map)
  - posX: พิกัด X ของโหนด (ใช้เฉพาะในระหว่างการแบ่งแผนที่ เมื่อการแบ่งแผนที่เสร็จสมบูรณ์แล้ว คุณสมบัตินี้จะไม่ได้ใช้งาน)
  - posY: พิกัด Y ของโหนด (ใช้เฉพาะในระหว่างการแบ่งแผนที่ เมื่อการแบ่งแผนที่เสร็จสมบูรณ์แล้ว คุณสมบัตินี้จะไม่ได้ใช้งาน)
  - childA, childB: โหนดลูกของ Binary Tree Node ปัจจุบัน แต่ละโหนดจะเก็บออบเจ็กต์ BT_Node เพียงหนึ่งตัว
- **ตัวอย่างการใช้งาน**
```
A = BT_Node(np.zeros((5,5)), 10, 10)
```
- **ค่าที่ส่งกลับมา**\
เป็น Binary Tree Node คือออบเจ็กต์ที่ใช้จัดการข้อมูลแบบลำดับชั้น

**Discrete map()**\
เป็นออบเจ็กต์ที่ใช้แทนส่วนหนึ่งของแผนที่ที่ไม่ต่อเนื่องกัน(Discrete Map)โดยแทนที่การอธิบายแต่ละพิกัดว่าเป็นสิ่งกีดขวางหรือไม่ด้วยการรวมพิกัดที่มีสถานะเดียวกันเข้าด้วยกันในรูปของสี่เหลี่ยมที่ทราบตำแหน่งและขนาด
- **รูปแบบคำสั่ง**
```
Discrete_map(self, value_in, posX_in, posY_in, sizeX_in, sizeY_in)
```
- **คุณลักษณะ**
  - value: เก็บค่าที่ระบุว่า Discrete Map ปัจจุบันเป็นสิ่งกีดขวาง (1) หรือเส้นทางว่าง (0)
  - posX, posY: เก็บพิกัดมุมซ้ายบนของ Discrete Map ปัจจุบัน
  - sizeX, sizeY: เก็บความกว้างและความสูงของ Discrete Map ปัจจุบัน
- **ค่าที่ส่งกลับมา**\
  เป็น Node ของแผนที่ส่วนที่ไม่ต่อเนื่องกัน
- **ตัวอย่างการใช้งาน**
```
DM = Discrete_map(1,10,20,15,25)
```
ภายในออบเจ็กต์ Discrete map() ประกอบไปด้วยฟังค์ชั่น ดังนี้
```
get_center_pos(self)
get-bottonm_right_pos(self)
scale_discrete_map(self, scale, x_offset, y_offset)
```
  - get_center_pos(self): ส่งคืนค่าตำแหน่งศูนย์กลางของ Discrete map
  - get-bottonm_right_pos(self): ส่งคืนค่าตำแหน่งมุมขวาล่างของ Discrete map
  - scale_discrete_map(self): ส่งคือ discrete map ใไม่ที่ปรับขนาดตามค่าที่กำหนด

**Map(self, in_map)**\
เป็นออบเจ็กต์ที่ใช้สำหรับเก็บและคำนวณแผนที่กริดปกติ (Normal Grid Map) ให้เป็นแผนที่กริดแบบไม่ต่อเนื่อง (Discrete Grid Map) โดยในขั้นตอนการเริ่มต้น (Initialization) จะรับแผนที่กริดในรูปแบบ numpy.ndarray จากนั้นจะคำนวณให้เป็นแผนที่แบบไม่ต่อเนื่อง และจัดเก็บผลลัพธ์ไว้ในออบเจ็กต์
- **รูปแบบคำสั่ง**
```
M = Map(np.zeros((10,10)))
```
- **คุณลักษณะ**
  - full_map: เก็บแผนที่กริดทั้งหมดในรูปแบบ numpy.ndarray ซึ่งถูกป้อนเข้ามาในขั้นตอนการเริ่มต้นออบเจ็กต์
  - tree_map: ตัวเก็บข้อมูลชั่วคราวสำหรับ Binary Tree ที่ใช้ในการเปลี่ยนแผนที่กริดให้เป็นแผนที่แบบไม่ต่อเนื่อง (Discretized Map) คุณสมบัตินี้ไม่ควรถูกเรียกใช้งานจากภายนอกเมธอดของออบเจ็กต์
  - optimized_map: รายการของ discrete_map ที่แสดงถึงแผนที่กริดที่ป้อนเข้าไป ลำดับของ discrete_map อาจไม่เรียงตามลำดับ
  - obstacle_list: เก็บเฉพาะ discrete_map ที่มีค่าเป็น 1 (สิ่งกีดขวาง) ซึ่งซ้ำซ้อนกับฟังก์ชัน list_obstacle
- **พารามิเตอร์**
  - in_map: แผนที่กริดที่ป้อนเข้ามาในรูปแบบ numpy.ndarray โดยค่า 1 แสดงถึงสิ่งกีดขวาง (obstacle) และค่า 0 แสดงถึงเส้นทางว่าง (free path)

ภายในออบเจ็กต์ Map() ประกอบไปด้วยฟังก์ชั่น ดังนี้
```
find_adjacent_node(self, input_node) 
find_nearest_node(self, x, y)
list_obstacle(self)
show_graph(self)
```
  - find_adjacent_node(self, input_node):รับข้อมูลจาก Discrete map มาแล้วส่งคืนแมพที่ ขอบบน, ล่าง, ขวา, ซ้ายติดกัน 
  - find_nearest_node(self, x, y): จากพิกัดที่กำหนด ส่งคืน Discrete map ที่มีพิกัดที่กำหนดอยู่ภายใน
  - list_obstacle(self): ใน Map ปัจจุบัน ส่งคืนรายการของ Discrete map ที่มีค่าเป็น 1 (คือสิ่งกีดขวาง)
  - show_graph(self): จาก Map ปัจจุบัน พิมพ์ข้อมูลของแต่ละโหนด (Discrete map) และคุณสมบัติออกมา

**ผลลัพท์หลังจากผ่าน map_optimizer**\
ข้อมูลจำนวน Node ที่เหลืออยู่ของแผนที่หลังผ่านกาน Optimize จาก 50x50 node\
![map_op_data](image%20for%20read%20me/Map_optimizer.png)\
map1.npy\
![map_op_1](image%20for%20read%20me/OP1.png)\
map2.npy\
![map_op_2](image%20for%20read%20me/OP2.png)\
map3.npy\
![map_op_3](image%20for%20read%20me/OP3.png)\
map4.npy\
![map_op_4](image%20for%20read%20me/OP4.png)\
map5.npy\
![map_op_5](image%20for%20read%20me/OP5.png)

## robot.py
สร้างออบเจ็กต์ RobotArm  ที่ใช้เก็บออบเจ็กต์ลิงก์ของหุ่นยนต์ (robot link) และตำแหน่งฐาน (base position) โดยแต่ละลิงก์จะเป็น Private object ของ RobotArm ซึ่งค่าเริ่มต้นของมุมของแต่ละลิงก์จะถูกตั้งเป็น 0
- **รูปแบบคำสั่ง**
```
RobotArm()
```
- **คุณลักษณะ**
  - base_position: ทูเพิลที่เก็บตำแหน่งฐานของหุ่นยนต์ (Robot)
  - links: รายการของออบเจ็กต์ลิงก์ (link object) แต่ละลิงก์ประกอบด้วย:
    - LENGTH: ความยาวของลิงก์ที่เกี่ยวข้อง ค่านี้ควรเป็นค่าคงที่
    - angle: มุมปัจจุบัน (ในหน่วยเรเดียน) ที่จุดเริ่มต้นของข้อต่อ (joint) เมื่อเทียบกับข้อต่อก่อนหน้า/ฐาน
    - end_positionX: พิกัด X ของปลายลิงก์
    - end_positionY: พิกัด Y ของปลายลิงก์

- **พารามิเตอร์**
  -  link_len: ลิสของความยาวแขนแต่ละท่อน จาก base ถึง end effector
-**ตัวอย่างการใช้งาน**
```
R = RobotArm([200, 200, 200])
```
ภายในออบเจ็กต์ RobotArm() ประกอบไปด้วยฟังก์ชั่น ดังนี้
```
set_base_position(self, base_x, base_y)
forward_kinematic(self, joint)
sequencial_IK_3(self, in_x, in_y)
check_wall_collision(self, map_x, map_y, map_size_x, map_size_y)
check_object_collision(self, obstacle_list)
draw_robot(self, pygame_screen, base_x=None, base_y=None)
```
  - set_base_position(self, base_x, base_y): อัปเดตคุณสมบัติ base_position ของออบเจ็กต์ RobotArm ปัจจุบันเพื่อเปลี่ยนตำแหน่งฐานของหุ่นยนต์.
  - forward_kinematic(self, joint): คำนวณ Forward Kinematics ของออบเจ็กต์ RobotArm ตามมุมข้อต่อที่ได้รับเป็นข้อมูลนำเข้า จากนั้นทำการอัปเดตค่า angle และ end_positionX, end_positionY ของแต่ละลิงก์ พร้อมทั้งคืนค่า end_positionX, end_positionY ของแต่ละลิงก์กลับมา
    - ตัวอย่างการใช้งาน
```
Link_pos = R.forward_kinematic([math.pi/2, -math.pi/2, 0])

# Link_pos[0] = X1, Y1
# Link_pos[1] = X2, Y2
# ...
```
  - sequencial_IK_3(self, in_x, in_y): คำนวณการจัดวางข้อต่อ (joint configuration) ของหุ่นยนต์ 3 ข้อต่อ (3 joints robot arm) ที่ทำให้ปลายแขนกล (end effector) แตะพิกัดเป้าหมาย in_x และ in_y รับข้อมูลเป็นจำนวนเต็มเท่านั้น โดยฟังก์ชันนี้ใช้เพื่อการสาธิตเท่านั้น และขั้นตอนการคำนวณมีดังนี้
    - หมุนข้อต่อที่ 1 (Joint 1) ให้ชี้ไปยังเป้าหมาย
    - คำนวณ Inverse Kinematics แบบ 2 มิติ สำหรับข้อต่อที่ 2 และ 3 (Joint 2 และ Joint 3)
    - หากข้อต่อที่ 2 และ 3 สามารถไปถึงเป้าหมายได้โดยไม่ต้องหมุนข้อต่อที่ 1 ฟังก์ชันจะรวมวิธีแก้ปัญหานั้นเป็นอีกหนึ่งผลลัพธ์

    - ตัวอย่างการใช้งาน
```
solution = R.sequencial_IK_3(100, 600)
```
  - check_wall_collision(self, map_x, map_y, map_size_x, map_size_y): จากขนาดและตำแหน่งของแผนที่ ตรวจสอบว่าค่ามุมข้อต่อในปัจจุบันชนกับขอบของแผนที่หรือไม่
  - check_object_collision(self, obstacle_list): จากออบเจ็กต์  RobotArm ปัจจุบันตรวจสอบว่าชนกับสิ่งกีดขวางใดหรือไม่
  - draw_robot(self, pygame_screen, base_x=None, base_y=None): วาดหุ่นยนต์ลงในหน้าจอ pygame ตามค่ามุมข้อต่อปัจจุบัน สามารถกำหนดตำแหน่งฐานเพิ่มเติมได้ในฟังก์ชันนี้

## A_Star.py

สร้างออบเจ็กต์ A_Star object เป็นออบเจ็กต์ว่างเปล่าเพื่อใช้เก็บโครงสร้างข้อมูล (data structure) สำหรับการใช้งานในฟังก์ชัน compute_path ในภายหลัง โครงสร้างข้อมูลควรเริ่มต้นในสภาพว่างเปล่าก่อนที่จะเริ่มคำนวณเส้นทาง ไม่จำเป็นต้องกำหนดค่าใดๆ ให้กับคุณสมบัติของออบเจ็กต์ในขั้นตอนนี้
- **รูปแบบคำสั่ง**
```
SA = A_Star()
```
- ฟังก์ชั่นภายใน
```
compute_path(self, goal_x, goal_y, map_input, robot_input)
```
คำนวณเส้นทางที่เหมาะสมที่สุดในพื้นที่งาน (task-space path) ซึ่งจะส่งผลให้เกิดการเคลื่อนที่ของมุมข้อต่อน้อยที่สุด และไม่มีการชนกับสิ่งกีดขวาง เส้นทางจะถูกคำนวณโดยใช้ฟังก์ชัน sequencial_IK_3 อย่างไรก็ตาม เนื่องจากข้อจำกัดของฟังก์ชันดังกล่าวที่สามารถให้คำตอบได้สูงสุดเพียง 4 รูปแบบของข้อต่อ (joint solutions) แม้ว่าจะมีคำตอบที่เป็นไปได้อย่างไม่มีที่สิ้นสุด ฟังก์ชันนี้อาจล้มเหลวในการคำนวณเส้นทางหากคำตอบทั้ง 4 รูปแบบที่ได้จาก sequencial_IK_3 ชนกับสิ่งกีดขวางหรือขอบแผนที่

  - **พารามิเตอร์**
    - goal_x: พิกัด X เป้าหมายในกรอบอ้างอิงของแผนที่ (ปรับขนาดให้เข้ากับความละเอียดของ Pygame) รับเฉพาะค่าเป็นจำนวนเต็ม (integer)
    - goal_y: พิกัด Y เป้าหมายในกรอบอ้างอิงของแผนที่ (ปรับขนาดให้เข้ากับความละเอียดของ Pygame) รับเฉพาะค่าเป็นจำนวนเต็ม (integer)
    - map_input: ออบเจ็กต์แผนที่ (Map object) ที่หุ่นยนต์อยู่ใน รับเฉพาะออบเจ็กต์ประเภท Map เท่านั้น
    - robot_input: ออบเจ็กต์หุ่นยนต์ (Robot object) ที่ต้องเคลื่อนที่ในแผนที่โดยไม่ชนกับสิ่งกีดขวาง รับเฉพาะออบเจ็กต์ประเภท Robot เท่านั้น

  - **ตัวอย่างการใช้งาน**
```
path = foo.compute_path(100,100, map, robot)

# Example result in path variable.
# Parent array (path[i]) hold each path node.
# Array nested inside (path[i][j]) hold each joint angle.
# path => [[0,0,0],[pi/2,pi/2,pi/2], ...]
```
  - **ค่าที่ส่งกลับมา**\
  Nested list 2 มิติที่แสดงเส้นทางไปยังพิกัดเป้าหมาย แต่ละองค์ประกอบในเส้นทาง (path) จะเก็บค่าการจัดวางข้อต่อ (joint configuration) ของแต่ละโหนดในเส้นทางนั้น

# **การใช้งานโปรแกรม**
## Path_Finding_Robot.py
จะนำฟังก์ชั่นและออปเจ็กต์จากไฟล์ map_optimizer.py, robot.py และ A_.py มาทำการจำลอง robot arm บน pygame 

**สิ่งที่ต้อง import เข้ามา**
```
# นำเข้าโมดูล os เพื่อทำงานร่วมกับระบบปฏิบัติการ
import os

# นำเข้าโมดูล math สำหรับการคำนวณทางคณิตศาสตร์
import math

# นำเข้า numpy สำหรับการดำเนินการทางตัวเลข โดยเฉพาะการทำงานกับอาร์เรย์และเมทริกซ์
import numpy as np

# นำเข้า pygame สำหรับการพัฒนาเกม โดยเฉพาะการแสดงผลกราฟิก
import pygame

# นำเข้าคลาส Discrete_map และ Map จาก Path_Finding.map_optimizer สำหรับจัดการแผนที่
from Path_Finding.map_optimizer import Discrete_map, Map

# นำเข้าค่าคงที่จาก RobotARM.constant สำหรับการตั้งค่าหุ่นยนต์อาร์ม
import RobotARM.constant as R_const

# นำเข้าคลาส RobotArm จาก RobotARM.robot สำหรับควบคุมการทำงานของหุ่นยนต์อาร์ม
from RobotARM.robot import RobotArm

# นำเข้าฟังก์ชัน map2img จาก Map_Utils.visualize_map เพื่อแสดงผลแผนที่ในรูปแบบภาพ
from Map_Utils.visualize_map import map2img

# นำเข้าคลาส A_Star จาก A_Star.A_Star สำหรับการใช้งานอัลกอริทึมค้นหาเส้นทาง
from A_Star.A_Star import A_Star


```
**ตัวแปรที่กำหนด**
```
# ค่าคงที่สำหรับขนาดหน้าจอ
SCREEN_WIDTH = 1280 
SCREEN_HEIGHT = 720
PI = math.pi

# เลือกไฟล์แผนที่
MAP_PATH = os.path.join(os.path.abspath("Map"), 'map1.npy')

```
MAP_PATH ใช้เลือกแผนที่ที่ต้องการทำการจำลองเมื่อต้องการเปลี่ยนแผนที่ที่จะทดสอบให้เปลี่ยนที่ตัวเลขของ 'map1.npy' ได้ตั้งแต่ 1-5

**ภายใน main()**
```
    # การเริ่มต้นแต่ละโมดูล
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    # การเริ่มต้นระบบแผนที่
    np_map = np.load(MAP_PATH, allow_pickle=False)
    the_map = Map(np_map)

    # การเริ่มต้นอาร์มหุ่นยนต์
    Robot = RobotArm([180,180,180])
    Robot.forward_kinematic([PI/2, -PI, PI])
    Robot.set_base_position(R_const.ROBOT_COORDINATE_X, R_const.ROBOT_COORDINATE_Y)

    # แสดงแผนที่บนหน้าจอ
    map2img(screen, np_map, 100, 100)

    # จุดหมาย (target) ที่หุ่นยนต์ต้องไป
    target_x = 150
    target_y = 150

    # สร้างอ็อบเจกต์ A_Star เพื่อหาทาง
    A = A_Star()
    path = A.compute_path(target_x, target_y, the_map, Robot)
    running = True
    state = 0
    key_reset = True

    # วนลูปสำหรับการแสดงผลและรับอินพุตจากผู้ใช้
    while running:

        screen.fill((255,255,255))  # กรอกหน้าจอด้วยสีขาว
        map2img(screen, np_map, 100, 100)  # แสดงแผนที่

        for event in pygame.event.get():
            
            if event.type == pygame.QUIT:
                running = False  # ออกจากลูปเมื่อปิดหน้าต่าง
            
            elif event.type == pygame.KEYDOWN and key_reset:
                key = pygame.key.get_pressed()
                if key[pygame.K_LEFT]:
                    state = max(0, state - 1)  # ลดสถานะ (state) เมื่อกดปุ่มซ้าย
                    key_reset = False

                if key[pygame.K_RIGHT]:
                    state = min(len(path) - 1, state + 1)  # เพิ่มสถานะ (state) เมื่อกดปุ่มขวา
                    key_reset = False

            elif event.type == pygame.KEYUP:
                key_reset = True  # รีเซ็ตการกดปุ่มเมื่อปล่อยปุ่ม

        # คำนวณการเคลื่อนที่ของหุ่นยนต์
        Robot.forward_kinematic(path[state].joint_sol)
        Robot.draw_robot(screen, R_const.ROBOT_COORDINATE_X, R_const.ROBOT_COORDINATE_Y)
        
        # แสดงจุดหมาย (target)
        pygame.draw.circle(screen, (255,0,0), (target_x + 100, target_y + 100), radius=3)

        pygame.display.flip()  # อัพเดทหน้าจอ

    pygame.quit()  # ปิด Pygame
    return 0

if __name__ == "__main__":
    main()
```
**การทำงาน:**

1. เมื่อโปรแกรมเริ่มต้นขึ้น จะมีการโหลดแผนที่จากไฟล์ map[].npy
2. สามารถกำหนด target ได้ที่ตัวแปร target_x = () และ target_y = ()
3. หุ่นยนต์จะใช้ A Algorithm* ในการคำนวณเส้นทางจากตำแหน่งเริ่มต้นไปยังจุดหมายที่กำหนด
4. ผู้ใช้สามารถกดปุ่มลูกศรซ้าย (←) และลูกศรขวา (→) เพื่อเปลี่ยนสถานะ (state) และดูการเคลื่อนที่ของหุ่นยนต์
5. โปรแกรมจะมีการแสดงแผนที่และหุ่นยนต์บนหน้าจอ

**คำอธิบายฟังก์ชัน:**
  - main(): ฟังก์ชันหลักที่เริ่มต้นโปรแกรม, คำนวณเส้นทางและแสดงผลการเคลื่อนที่ของหุ่นยนต์
  - RobotArm: คลาสที่ใช้ควบคุมหุ่นยนต์อาร์ม, คำนวณการเคลื่อนที่และแสดงภาพ
  - A_Star: คลาสที่ใช้สำหรับคำนวณเส้นทางจากจุดเริ่มต้นไปยังจุดหมาย
  - map2img: ฟังก์ชันที่ใช้แสดงแผนที่บนหน้าจอ

## **ผลการทดลอง**
**Examples : Map1.npy**
[![Watch the video](image%20for%20read%20me/Path%20find1.png)](https://youtu.be/ng2P5IhppaE)

การจำลองเริ่มต้นด้วยแขนหุ่นยนต์อยู่ในตำแหน่งเริ่มต้น (Home Position) และมีจุดเป้าหมายที่กำหนดไว้ล่วงหน้า ใช้แนวทางคำนวณ Inverse Kinematics เพื่อหาค่ามุมข้อต่อ (q) ที่จำเป็นสำหรับการเคลื่อนย้ายแขนหุ่นยนต์ไปยังตำแหน่งเป้าหมาย

**Examples : Out of Reach**

![Out_of_reach](image%20for%20read%20me/4Out_of_lenght.png)

**Examples : Spawn On Obstacle**

![cant_go](image%20for%20read%20me/3%20spawn%20on%20wall.png)

**การตรวจสอบความถูกต้อง : Map1.npy**
สำหรับการตรวจสอบความถูกต้อง สามารถแสดงผลลัพธ์ได้โดยการใช้ทั้งการคำนวณ Forward Kinematics และ Inverse Kinematics ของแขนกล เพื่อคำนวณและตรวจสอบตำแหน่งเป้าหมายอย่างละเอียด

**การตรวจสอบความถูกต้อง: การคำนวณตำแหน่งเป้าหมาย (Calculate About Target)**
สำหรับการตรวจสอบความถูกต้อง สามารถแสดงผลลัพธ์ได้โดยการใช้ทั้ง Forward Kinematics และ Inverse Kinematics ของแขนกล เพื่อคำนวณและยืนยันตำแหน่งของเป้าหมายให้แม่นยำ

## **สรุปและวิเคราะห์ผล**
  จากการศึกษาและทดลองสร้างแบบจำลองการเคลื่อนที่ของแขนกลแบบ 3 DOF (Revolute) ในสองมิติ พร้อมทั้งประยุกต์ใช้ร่วมกับกระบวนการหาเส้นทาง (Path Finding) และการวางแผนการเคลื่อนที่ (Trajectory Planning) โดยใช้อัลกอริทึม A* Search ผลการทดลองแสดงให้เห็นว่า แขนกลสามารถเคลื่อนที่ไปยังตำแหน่งเป้าหมายได้โดยไม่ชนกับสิ่งกีดขวาง และในกรณีที่ตำแหน่งเป้าหมายอยู่นอกระยะของแขนกล หรือไม่มีเส้นทางที่ปลอดภัยไปถึงได้ ระบบสามารถส่ง Feedback กลับมาแจ้งสถานการณ์ดังกล่าวได้อย่างมีประสิทธิภาพ อย่างไรก็ตาม ในบางรูปแบบแผนที่และบางตำแหน่งของเป้าหมาย ระบบยังไม่สามารถนำแขนกลเคลื่อนที่ไปยังเป้าหมายได้ เนื่องจากข้อจำกัดของสมการ Inverse Kinematics ที่ใช้ในการคำนวณหาตำแหน่งข้อต่อ (Joint Position) ซึ่งส่งผลกระทบต่อกระบวนการตรวจสอบระยะทาง (Logic for Distance Checking) ให้เกิดความคลาดเคลื่อนได้ นอกจากนี้ยังไม่สามารถพิสูจน์ได้ว่าเส้นทางที่สร้างขึ้นโดย A* Search Algorithm นั้นเป็นเส้นทางที่มีความเหมาะสมที่สุด (Optimal Path)

## **แนวทางการแก้ไขและพัฒนาต่อ**
- ปรับปรุงสมการ Inverse Kinematics
  - ศึกษาและปรับปรุงสมการ Inverse Kinematics ให้มีความถูกต้องและครอบคลุมกรณีต่าง ๆ ได้มากขึ้น
- ปรับปรุง A* Search Algorithm
  - ประยุกต์ใช้ Heuristic Function ที่เหมาะสมมากขึ้น เพื่อให้การหาเส้นทางมีประสิทธิภาพ และลดโอกาสเกิดเส้นทางที่ไม่ Optimal
  - ทดลองเปรียบเทียบกับอัลกอริทึมอื่น เช่น Dijkstra หรือ RRT (Rapidly-exploring Random Tree) เพื่อวิเคราะห์ข้อดีและข้อเสีย
- เพิ่มการตรวจสอบประสิทธิภาพ
  - พัฒนาโมดูลสำหรับประเมินความถูกต้องของเส้นทาง (Path Accuracy) และเปรียบเทียบเส้นทางที่สร้างขึ้นกับเส้นทางที่ Optimal

## **เอกสารอ้างอิง**
Recent advance in Rapidly – Exploring random tree: A review
https://www.sciencedirect.com/science/article/pii/S2405844024084822

A Systematic Literature Review of A* Pathfinding
https://www.sciencedirect.com/science/article/pii/S1877050921000399



