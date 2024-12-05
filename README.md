## **FRA333 Class Project: Path finding and position trajectory generation to avoid collision in 3 DOF Planar Revolute robot**

โครงงานนี้ถูกจัดทำขึ้นเพื่อใช้ในการศึกษาและพัฒนาการสร้างวิถีการเคลื่อนที่ (Trajectory Generation) สำหรับหุ่นยนต์ 3 DOF Planar revolute robot ที่สามารถหลบหลีกสิ่งกีดขวางภายในพื้นที่ทำงาน (Task Space) ได้ โดยการจำลองการเคลื่อนที่และลักษณะของหุ่น ผ่านการใช้ Python และอัลกอริทึมที่ใช้ในการค้นหาเส้นทาง (Path finding) ผลลัพท์ที่คาดหวังคือโปรแกรมจำลองแขนกล 3 DOF ที่สามารถเคลื่อนที่ไปยังเป้าหมายได้โดยหลีกเลี่ยงการชนสิ่งกีดขวางตามข้อกำหนดในขั้นตอนการวางแผนวิถี (Trajectory planning)
การทดสอบจะดำเนินการผ่านการจำลองด้วย Pygame ที่จะแสดงให้เห็นถึงการเคลื่อนที่ในพื่นที่ทำงานที่มีสิ่งกีดขวางหลากหลายรูปแบบ มี่ใช้เป็นเกฯฑ์ให้เห็นภาพในการทดสอบ

**คำสำคัญ**: Rapidly exploring random tree, Informed search, Trajectory generation

## **จุดประสงค์โครงการ**[Headers](#headers)  
1)	เพื่อศึกษาการเคลื่อนที่ของแขนกลแบบ Revolute ในปริภูมิ 2 มิติ  
2)	เพื่อศึกษาวิธีการสร้างวิถีการโคจรให้แขนกลเคลื่อนที่ไปยังเป้าหมายในขณะที่มีสิ่งกีดขวาง
3)	เพื่อศึกษาการทำ Path finding และ Trajectory planning สำหรับการสร้างวิถีโคจรที่หลบหลีกสิ่งกีดขวางและไปยังเป้าหมาย

## **ขอบเขต**
1)	กำหนดให้ทุก ๆ ก้านของแขนกลเป็น Rigid body ที่มีเพียงความยาวเท่านั้น ไม่มีความกว้างและลึก
2)	แขนกลเป็น Joint แบบ Revolute จำนวน 3 Joint
3)	แต่ละก้านของแขนกลมีความยาวเท่ากัน
4)	ระบบรู้สภาพแวดล้อมทั้งหมดก่อนเริ่มทำ Path planning
5)	สภาพแวดล้อมที่หุ่นยนต์อยู่คงที่ ไม่มีการเปลี่ยนแปลง
6)	พิจารณาเฉพาะ Kinematic (ไม่พิจารณาแรงภายนอก)
7)	สร้างวิธีโคจรโดยควบคุมตำแหน่งเท่านั้น
8)	Simulation รับ input เป็น ตำแหน่งฐานปัจจุบันของ Robot arm ,ตำแหน่งของ End-effector ณ จุดเริ่มต้น ,จุดพิกัดที่ต้องการให้ End effector ,แผนที่สภาพแวดล้อมที่ระบุสิ่งกีดขวาง 
9)	Output ของ Simulation เป็น พิกัด via point และออกมาเป็นการจำลองการเคลื่อนที่ผ่าน PyGame
10)	จำลองการเคลื่อนที่ภายใน Simulation ด้วยภาษา python 

## **ทบทวนวรรณกรรมและทฤษฎีที่เกี่ยวข้อง**


## **วิธีการดำเนินงาน**
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

## **ผลการทดลอง**
## **สรุปและวิเคราะห์ผล**

## **เอกสารอ้างอิง**
Recent advance in Rapidly – Exploring random tree: A review
https://www.sciencedirect.com/science/article/pii/S2405844024084822

A Systematic Literature Review of A* Pathfinding
https://www.sciencedirect.com/science/article/pii/S1877050921000399
