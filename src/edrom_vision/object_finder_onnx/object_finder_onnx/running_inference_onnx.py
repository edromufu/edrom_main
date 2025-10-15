# object_finder_onnx/running_inference_onnx.py
import cv2, os, numpy as np
from ament_index_python.packages import get_package_share_directory

INPUT_WIDTH, INPUT_HEIGHT = 320, 320
SCORE_THRESHOLD, NMS_THRESHOLD, CONFIDENCE_THRESHOLD = 0.25, 0.45, 0.3

def set_model_input():
    package_share_path = get_package_share_directory('object_finder_onnx')
    model_path = os.path.join(package_share_path, 'modelo', 'yolov8n-vision.onnx')

    print(f"Carregando modelo ONNX de: {model_path}")
    if not os.path.exists(model_path):
        print(f"!!!!!! ATENÇÃO: Modelo ONNX não encontrado em {model_path} !!!!!!"); return None
    return cv2.dnn.readNetFromONNX(model_path)

def detect_model(model, current_frame):
    if model is None: return [], [], [], current_frame

    blob = cv2.dnn.blobFromImage(current_frame, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
    model.setInput(blob)
    outputs = model.forward()
    outputs = np.transpose(outputs[0])

    boxes_dnn, scores_dnn, class_ids_dnn = [], [], []
    h_orig, w_orig = current_frame.shape[:2]
    x_scale, y_scale = w_orig / INPUT_WIDTH, h_orig / INPUT_HEIGHT

    for row in outputs:
        confidence = row[4]
        if confidence > CONFIDENCE_THRESHOLD:
            classes_scores = row[5:]
            class_id = np.argmax(classes_scores)
            if classes_scores[class_id] > SCORE_THRESHOLD:
                scores_dnn.append(confidence)
                class_ids_dnn.append(class_id)
                cx, cy, w, h = row[0:4]
                left, top = int((cx - w/2) * x_scale), int((cy - h/2) * y_scale)
                width, height = int(w * x_scale), int(h * y_scale)
                boxes_dnn.append([left, top, width, height])

    indices = cv2.dnn.NMSBoxes(boxes_dnn, np.array(scores_dnn), CONFIDENCE_THRESHOLD, NMS_THRESHOLD)

    final_classes, final_scores, final_boxes_xywh = [], [], []
    inference_frame = current_frame.copy()

    if len(indices) > 0:
        for i in indices.flatten():
            class_id, score, box = class_ids_dnn[i], scores_dnn[i], boxes_dnn[i]
            left, top, width, height = box

            final_classes.append(class_id)
            final_scores.append(score)
            final_boxes_xywh.append([left + width/2, top + height/2, width, height])

            cv2.rectangle(inference_frame, (left, top), (left + width, top + height), (0, 255, 0), 2)
            cv2.putText(inference_frame, f'{class_id}: {score:.2f}', (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return final_classes, final_scores, final_boxes_xywh, inference_frame
