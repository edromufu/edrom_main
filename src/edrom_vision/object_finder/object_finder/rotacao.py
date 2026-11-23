"""
Conversão de coordenadas entre a imagem original da câmera e a imagem "em pé".

A câmera é montada girada. O object_finder gira o frame `rotacao` graus no
sentido horário antes do YOLO, mas publica os tópicos do soccer_interfaces
(*_in_image) nas coordenadas da imagem ORIGINAL (w x h), que é o referencial do
camera_info usado pelo IPM.

Quem trabalha em pixel e precisa da imagem em pé (behaviour, controle de
cabeça) usa `para_imagem_em_pe` para desfazer a rotação.

Sem dependências além do Python, para poder ser importado pelo behaviour.
"""

# Rotação física da câmera (graus, sentido horário). Deve bater com o
# argumento image_rotation do vision.launch.py.
ROTACAO_CAMERA = 180

ROTACOES_VALIDAS = (0, 90, 180, 270)


def para_imagem_original(cx, cy, bw, bh, rotacao, w, h):
    """
    Caixa [cx, cy, bw, bh] na imagem em pé -> coordenadas da imagem original
    da câmera (w x h).
    """
    if rotacao == 90:
        return cy, h - cx, bh, bw
    if rotacao == 180:
        return w - cx, h - cy, bw, bh
    if rotacao == 270:
        return w - cy, cx, bh, bw
    return cx, cy, bw, bh


def para_imagem_em_pe(x, y, bw, bh, rotacao, w, h):
    """
    Inversa de `para_imagem_original`: caixa [x, y, bw, bh] na imagem original
    da câmera (w x h) -> coordenadas da imagem em pé.
    """
    if rotacao == 90:
        return h - y, x, bh, bw
    if rotacao == 180:
        return w - x, h - y, bw, bh
    if rotacao == 270:
        return y, w - x, bh, bw
    return x, y, bw, bh


def melhor_bola_em_pe(balls, rotacao, w, h):
    """
    Recebe a lista `balls` de um soccer_vision_2d_msgs/BallArray e retorna a
    bola de maior confiança como (x, y, largura, altura) na imagem em pé,
    ou None se a lista estiver vazia.
    """
    if not balls:
        return None
    b = max(balls, key=lambda bola: bola.confidence.confidence)
    return para_imagem_em_pe(
        b.center.x, b.center.y, b.bb.size_x, b.bb.size_y, rotacao, w, h)
