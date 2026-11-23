#!/usr/bin/env python3
# coding=utf-8

from ultralytics import YOLO
import time

size = 320

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