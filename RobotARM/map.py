import numpy as np
import cv2

# โหลดข้อมูล numpy array จากไฟล์
map_array = np.load("Map/map2.npy")  # ระบุชื่อไฟล์ให้ตรงกับที่ต้องการ

# ตรวจสอบขนาดของ map_array
map_height, map_width = map_array.shape

# คำนวณสเกลเพื่อให้ภาพใหญ่สุด 500 พิกเซล
scale = 500 / max(map_width, map_height)  # อัตราส่วนสำหรับขยาย
new_width = int(map_width * scale)
new_height = int(map_height * scale)

# ปรับขนาดภาพ
resized_map = cv2.resize(map_array, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

# บันทึกภาพที่ขยายแล้วเป็น PNG
cv2.imwrite("output_map_scaled.png", resized_map)

# แสดงผลภาพ (สำหรับตรวจสอบ)
cv2.imshow("Scaled Map", resized_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
