import io
import numpy as np
import cv2
import pygame

def map2img(pygame_screen, map_array, in_pos_x, in_pos_y):

    SCALE_VALUE = 500

    # Check type
    if type(map_array) is not np.ndarray:
        raise TypeError("map_array only take numpy ndarray as input.")
    
    if type(in_pos_x) is not int:
        raise TypeError("in_pos_x only take integer as input.")
    
    if type(in_pos_y) is not int:
        raise TypeError("in_pos_y only take integer as input.")
    
    map_array = map_array.astype(np.uint8)

    # ตรวจสอบขนาดของ map_array
    map_height, map_width = map_array.shape

    # คำนวณสเกลเพื่อให้ภาพใหญ่สุด 500 พิกเซล
    scale = SCALE_VALUE / max(map_width, map_height)  # อัตราส่วนสำหรับขยาย
    new_width = int(map_width * scale)
    new_height = int(map_height * scale)

    # ปรับขนาดภาพ
    resized_map = cv2.resize(map_array, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

    # กลับสีภาพ
    inv_resized_map = np.ones((new_height, new_width), dtype=np.uint8) * 255
    inv_resized_map[resized_map == 1] = 0


    # save ภาพเป็น file on memory
    map_file = io.BytesIO(cv2.imencode(".png", inv_resized_map)[1])

    bg = pygame.image.load(map_file, "foo.png")
    
    pygame.draw.rect(pygame_screen, (0,0,0), (max(in_pos_x-5,0), max(in_pos_y-5,0), new_width+10, new_height+10))
    pygame_screen.blit(bg, (in_pos_x, in_pos_y))


