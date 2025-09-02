#!/usr/bin/env python3
# coding=utf-8

from ultralytics import YOLO
import os
import cv2
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory

size=320

def set_model_input():
    # Define o caminho absoluto para o seu arquivo de modelo
    model_path = '/home/ivan/best.pt'
    
    print(f"Tentando carregar o modelo de: {model_path}")
    if not os.path.exists(model_path):
        print(f"Atenção: O arquivo do modelo não foi encontrado em {model_path}.")
        return None

    # Carrega o modelo YOLOv8
    model = YOLO(model_path)
    
    return model

def detect_model(model, current_frame):
    if model is None:
        print("Modelo não carregado, pulando inferência.")
        # Retorna valores vazios para evitar erros
        return [], [], [], current_frame

    start_time = time.time()
    
    results = model.predict(source=current_frame,
                            conf=0.25,
                            imgsz=size,
                            max_det=10,
                            verbose=False)
    
    classes = results[0].boxes.cls.tolist()
    scores = results[0].boxes.conf.tolist()
    boxes = results[0].boxes.xywh.tolist()
    
    finish_time = time.time()
    fps_inf = 1/(finish_time - start_time)
    
    print(f"Classes: {classes}, Scores: {scores}")
    print(f"Boxes: {boxes}")
    print(f'FPS da inferência: {fps_inf:.2f}')

    inference_frame = results[0].plot()

    return classes, scores, boxes, inference_frame
