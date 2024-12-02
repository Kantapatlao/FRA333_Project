## **FRA333 Class Project: Path finding and position trajectory generation to avoid collision in 3 DOF Planar Revolute robot**

โครงงานนี้ถูกจัดทำขึ้นเพื่อใช้ในการศึกษาและพัฒนาการสร้างวิถีการเคลื่อนที่ (Trajectory Generation) สำหรับหุ่นยนต์ 3 DOF Planar revolute robot ที่สามารถหลบหลีกสิ่งกีดขวางภายในพื้นที่ทำงาน (Task Space) ได้ โดยการจำลองการเคลื่อนที่และลักษณะของหุ่น โดยการใช้ Python และอัลกอริทึมที่ใช้ในการค้นหาเส้นทาง (Path finding) ผลลัพท์ที่คาดหวังคือโปรแกรมจำลองแขนกล 3 DOF ที่สามารถเคลื่อนที่ไปยังเป้าหมายได้โดยไม่ชนสิ่งกีดขวาง และหลีกเลี่ยงการชนตามข้อกำหนดในขั้นตอนการวางแผนวิถี
การทดสอบจะดำเนินการผ่านการจำลองด้วย Pygame ที่จะแสดงให้เห็นถึงการเคลื่อนที่ในพื่นที่ทำงานที่มีสิ่งกีดขวางหลากหลาย เพื่อให้ได้นำไปประยุกต์ใช้ในงานอุตสาหกรรมในอนาคต

**คำสำคัญ**: Rapidly exploring random tree, Informed search, Trajectory generation

## **จุดประสงค์โครงการ**
1)	เพื่อศึกษาการเคลื่อนที่ของแขนกลแบบ Revolute ในปริภูมิ 2 มิติ  
2)	เพื่อศึกษาวิธีการสร้างวิถีการโคจรให้แขนกลเคลื่อนที่ไปยังเป้าหมายในขณะที่มีสิ่งกีดขวาง
3)	เพื่อศึกษาการทำ Path finding และ Trajectory planning สำหรับการสร้างวิถีโคจรที่หลบหลีกสิ่งกีดขวางและไปยังเป้าหมาย

## **ขอบเขต**
1)	กำหนดให้ทุก ๆ ก้านของแขนกลเป็น Rigid body ที่มีเพียงความยาวเท่านั้น ไม่มีความกว้างและลึก
2)	แขนกลเป็น Joint แบบ Revolute จำนวน 3 Joint
3)	แต่ละก้านของแขนกลสามารถยืดได้สูงสุด 2ln เมื่อ ln คือความยาวของก้าน n
4)	ระบบรู้สภาพแวดล้อมทั้งหมดก่อนเริ่มทำ Path planning
5)	สภาพแวดล้อมที่หุ่นยนต์อยู่คงที่ ไม่มีการเปลี่ยนแปลง
6)	พิจารณาเฉพาะ Kinematic (ไม่พิจารณาแรงภายนอก)
7)	สร้างวิธีโคจรโดยควบคุมตำแหน่งเท่านั้น
8)	Simulation รับ input เป็น ตำแหน่งฐานปัจจุบันของ Robot arm ตำแหน่งของ End-effector ณ จุดเริ่มต้น จุดพิกัดที่ต้องการให้ End effecto แผนที่สภาพแวดล้อมที่ระบุสิ่งกีดขวาง
9)	Output ของ Simulation เป็น พิกัด via point และออกมาเป็นการจำลองการเคลื่อนที่ผ่าน PyGame
10)	จำลองการเคลื่อนที่ภายใน Simulation ด้วยภาษา python 