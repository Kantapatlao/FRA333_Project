import os
import io
import numpy as np
import cv2
import pygame
from pygame.locals import *
from Path_Finding.map_optimizer import Map

file_name = ["map1.npy","map2.npy","map3.npy","map4.npy","map5.npy"]
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
inv_resized_map = np.ones((new_height, new_width), dtype=np.uint8) * 255
inv_resized_map[resized_map == 1] = 0

# กลับสีภาพ


map_file = io.BytesIO(cv2.imencode(".png", inv_resized_map)[1])

pygame.init()
display = pygame.display.set_mode((1280, 720))
bg = pygame.image.load(map_file, "foo.png")


running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    
    display.blit(bg, (0,0))
    pygame.display.flip()

# map.py
import numpy as np

def load_map(filepath):
    """Load a map from a .npy file."""
    return np.load(filepath)

def process_map(map_array):
    """Process the map and return a scaled version."""
    scale = 500 / max(map_array.shape)
    new_size = (int(map_array.shape[1] * scale), int(map_array.shape[0] * scale))
    return cv2.resize(map_array, new_size, interpolation=cv2.INTER_NEAREST)

def save_map_as_image(map_array, output_path):
    """Save the processed map as a PNG image."""
    cv2.imwrite(output_path, map_array)

import tkinter as tk
from tkinter import filedialog



# Create a simple UI
root = tk.Tk()
root.title("Map Processor")

open_button = tk.Button(root, text="Open Map File", command=file_name)
open_button.pack(pady=20)

root.mainloop()


pygame.quit()
