#!/usr/bin/env python3
# coding=utf-8

from ultralytics import YOLO
import os
import cv2
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory

size = 320

def set_model_input(is_simulation=False):
    """
    Carrega o modelo de IA no formato .pt (PyTorch).
    - Se is_simulation for True, carrega o modelo 'best.pt'.
    - Se is_simulation for False, carrega o modelo 'yolov8n-vision.pt' para o robô real.
    """
    try:
        package_share_path = get_package_share_directory('object_finder')
        model_base_path = os.path.join(package_share_path, 'modelo')

        if is_simulation:
            # --- MODO SIMULAÇÃO ---
            print("Carregando modelo para SIMULAÇÃO (.pt)...")
            model_path = os.path.join(model_base_path, 'best.pt')
        else:
            # --- MODO REAL (SEM OPENVINO) ---
            print("Carregando modelo PADRÃO para o Robô Real (.pt)...")
            model_path = os.path.join(model_base_path, 'yolov8n-vision.pt') # Carrega o modelo .pt padrão

        print(f"Tentando carregar o modelo de: {model_path}")
        if not os.path.exists(model_path):
            print(f"!!!!!! ATENÇÃO: O arquivo do modelo não foi encontrado em {model_path}. !!!!!!")
            return None

        model = YOLO(model_path)
        return model

    except Exception as e:
        print(f"Ocorreu um erro ao carregar o modelo: {e}")
        return None

def detect_model(model, current_frame):
    """
    Realiza a inferência em um frame de imagem usando o modelo carregado.
    """
    if model is None:
        print("Modelo não carregado, pulando inferência.")
        return [], [], [], current_frame

    start_time = time.time()
    
    results = model.predict(source=current_frame,
                            conf=0.45,
                            imgsz=size,
                            max_det=10,
                            verbose=False,
                            iou = 0.5)
    
    classes = results[0].boxes.cls.tolist()
    scores = results[0].boxes.conf.tolist()
    boxes = results[0].boxes.xywh.tolist()
    
    finish_time = time.time()
    fps_inf = 1 / (finish_time - start_time)
    
    inference_frame = results[0].plot()

    return classes, scores, boxes, inference_frame
