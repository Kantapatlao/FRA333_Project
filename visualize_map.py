import math
import os
import io
import numpy as np
import cv2
import pygame
import random
from pygame.locals import *
from Path_Finding.map_optimizer import Map

map_file_path = os.path.join(os.path.abspath("Map"), 'map1.npy')
# โหลดข้อมูล numpy array จากไฟล์
map_array = np.load(map_file_path)
map_array = map_array.astype(np.uint8)

# ตรวจสอบขนาดของ map_array
map_height, map_width = map_array.shape

# คำนวณสเกลเพื่อให้ภาพใหญ่สุด 500 พิกเซล
scale = 500 / max(map_width, map_height)  # อัตราส่วนสำหรับขยาย
new_width = int(map_width * scale)
new_height = int(map_height * scale)

# ปรับขนาดภาพ
resized_map = cv2.resize(map_array, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

# กลับสีภาพ
inv_resized_map = np.ones((new_height, new_width), dtype=np.uint8) * 255
inv_resized_map[resized_map == 1] = 0


# save ภาพเป็น file on memory
map_file = io.BytesIO(cv2.imencode(".png", inv_resized_map)[1])

pygame.init()

display = pygame.display.set_mode((1280, 720))
bg = pygame.image.load(map_file, "foo.png")

node_map = Map(map_array)


running = True
display.blit(bg, (0,0))

for n in node_map.optimized_map:
    if n.data.value == 1:
        pygame.draw.rect(display, (0,0,0) ,Rect(n.data.posX * 10 ,n.data.posY * 10,n.data.sizeX * 10,n.data.sizeY * 10))
    
    if n.data.value == 0:
        pygame.draw.rect(display, (255,255,255) ,Rect(n.data.posX * 10 ,n.data.posY * 10,n.data.sizeX * 10,n.data.sizeY * 10))
pygame.display.flip()

while running:
    
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    

pygame.quit()
