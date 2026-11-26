"""
Seleção da bola nos tópicos do object_finder.

Sem dependências além do Python, para poder ser importado pelo behaviour.
"""


def melhor_bola(balls):
    """
    Recebe a lista `balls` de um soccer_vision_2d_msgs/BallArray e retorna a
    bola de maior confiança como (x, y, largura, altura) em pixels da imagem,
    ou None se a lista estiver vazia.
    """
    if not balls:
        return None
    b = max(balls, key=lambda bola: bola.confidence.confidence)
    return b.center.x, b.center.y, b.bb.size_x, b.bb.size_y
