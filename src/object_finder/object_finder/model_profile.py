#!/usr/bin/env python3
# coding=utf-8

import cv2
import time
import psutil
import numpy as np
import os

# --- IMPORT DO SEU MÓDULO DE INFERÊNCIA ---
# Certifique-se de que este script consiga encontrar o seu pacote.
# A forma como você estruturou o seu projeto deve permitir que isso funcione
# se você executar o script a partir do seu workspace ROS.
try:
    import object_finder.running_inference as ri
except ImportError:
    print("ERRO: Nao foi possivel importar 'object_finder.running_inference'.")
    print("Certifique-se de que voce esta executando este script no ambiente ROS 2 correto")
    print("ou que o caminho para 'object_finder' esta no seu PYTHONPATH.")
    exit()

# ==============================================================================
# --- CONFIGURAÇÕES ---
# ==============================================================================
CAMERA_INDEX = 0             # Índice da sua câmera (geralmente 0 ou 1)
IS_SIMULATION = False        # Força o uso do modelo do robô real
FONT = cv2.FONT_HERSHEY_SIMPLEX
# ==============================================================================


def main():
    # Obtém o processo atual para monitoramento de CPU e Memória
    process = psutil.Process(os.getpid())
    # Inicializa a medição de CPU
    process.cpu_percent(interval=None)

    # --- CARREGAMENTO DO MODELO ---
    # Esta linha foi pega diretamente do seu código `running_inference.py`
    print("Carregando o modelo de deteccao...")
    model = ri.set_model_input(is_simulation=IS_SIMULATION) # <-- SUA LÓGICA DE CARREGAMENTO VEM AQUI
    if model is None:
        print("Falha ao carregar o modelo. Encerrando o profiler.")
        return
    print("Modelo carregado.")
    # --------------------------------

    # Inicializa a câmera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Erro: Nao foi possivel abrir a camera no indice {CAMERA_INDEX}")
        return

    print("\nIniciando o monitoramento. Pressione 'q' na janela para sair.")

    while True:
        # Captura um frame da câmera
        ret, frame = cap.read()
        if not ret:
            print("Nao foi possivel capturar o frame. Encerrando.")
            break

        # Inicia o cronômetro para medir o tempo de inferência
        start_time = time.time()

        # --- EXECUÇÃO DA INFERÊNCIA ---
        # Esta linha foi pega diretamente do seu código `running_inference.py`
        classes, scores, boxes, inference_frame = ri.detect_model(model, frame) # <-- SUA LÓGICA DE INFERÊNCIA VEM AQUI
        # ---------------------------------

        # Para o cronômetro
        end_time = time.time()

        # --- CÁLCULO DAS MÉTRICAS ---
        # 1. FPS (Frames Per Second)
        duration = end_time - start_time
        fps = 1 / duration if duration > 0 else 0

        # 2. Uso de CPU (%)
        cpu_usage = process.cpu_percent(interval=None)

        # 3. Uso de Memória (MB)
        mem_info = process.memory_info()
        mem_usage_mb = mem_info.rss / (1024 * 1024)

        # --- EXIBIÇÃO DOS RESULTADOS ---
        # Cria a string de status
        stats_text = f"FPS: {fps:.1f} | CPU: {cpu_usage:.1f}% | Memoria: {mem_usage_mb:.1f} MB"

        # Desenha um fundo preto para o texto ficar mais legível
        cv2.rectangle(inference_frame, (0, 0), (700, 40), (0, 0, 0), -1)
        # Escreve o texto na imagem
        cv2.putText(inference_frame, stats_text, (10, 30), FONT, 1, (0, 255, 0), 2)

        # Mostra a imagem resultante
        cv2.imshow("Monitor de Performance do Modelo", inference_frame)

        # Verifica se a tecla 'q' foi pressionada para sair
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libera os recursos
    cap.release()
    cv2.destroyAllWindows()
    print("Monitoramento encerrado.")


if __name__ == '__main__':
    main()
