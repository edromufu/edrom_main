#funções auxiliares
from math import sqrt, atan2

def subtrair(a, b):
    return (a[0] - b[0], a[1] - b[1])

def somar(a, b):
    return (a[0] + b[0], a[1] + b[1])

def multiplicar(v, escalar):
    return (v[0] * escalar, v[1] * escalar)

def norma(v):
    return sqrt(v[0]**2 + v[1]**2)

def normalizar(v):
    n = norma(v)
    if n < 1e-9:
        return (0.0, 0.0)
    return (v[0] / n, v[1] / n)

#função alvo chute
def calcular_alvo_chute(bola, gol, d_aprox):
    direcao = normalizar(subtrair(gol, bola))
    alvo = (
        bola[0] - (d_aprox * direcao[0]),
        bola[1] - (d_aprox * direcao[1])
    )
    theta = atan2(gol[1] - bola[1], gol[0] - bola[0])
    return alvo, theta

#função força de atração
def forca_atrativa(robo, alvo, k_atr):
    erro = subtrair(alvo, robo)
    return multiplicar(erro, k_atr)
#função força de repulsão
def forca_repulsiva(robo, obstaculo, raio_robo, k_rep, d0):
    pos_obs = (obstaculo["x"], obstaculo["y"])
    vetor = subtrair(robo, pos_obs) # Aponta do obstáculo para o robô
    dist_centros = norma(vetor)
    d = dist_centros - (raio_robo + obstaculo["raio"])
    
    if d >= d0:
        return (0.0, 0.0)
        
    d = max(d, 1e-3)
    direcao = normalizar(vetor)
    modulo = k_rep * (1.0 / d - 1.0 / d0) * (1.0 / (d * d))
    return multiplicar(direcao, modulo)

#função somando forças
def calcular_forca_total(robo, alvo, obstaculos, parametros):
    total = forca_atrativa(robo, alvo, parametros["k_atr"])
    for obstaculo in obstaculos:
        f_rep = forca_repulsiva(
            robo,
            obstaculo,
            parametros["raio_robo"],
            parametros["k_rep"],
            parametros["d0"]
        )
        total = somar(total, f_rep)
    return total

def limitar_velocidade(forca, velocidade_max):
    modulo = norma(forca)
    if modulo <= velocidade_max:
        return forca
    # Reduz o tamanho do vetor mantendo a sua direção
    return multiplicar(normalizar(forca), velocidade_max)

def atualizar_posicao(robo, velocidade, dt):
    # deslocamento = velocidade * tempo
    deslocamento = multiplicar(velocidade, dt)
    
    # nova_posição = posição_atual + deslocamento
    return somar(robo, deslocamento)

def normalizar_grid_por_media(pontos, tamanho_janela=5, tamanho_celula=1.0, origem=(0.0, 0.0)):
    if tamanho_celula <= 0 or tamanho_janela <= 0:
        raise ValueError("Resolução e janela devem ser positivas.")
    if not pontos:
        return []

    caminho_bruto = []
    
    for i in range(0, len(pontos), tamanho_janela):
        janela = pontos[i:i+tamanho_janela]
        
        # Calcula a média (centro de massa) dos pontos nessa janela
        media_x = sum(p[0] for p in janela) / len(janela)
        media_y = sum(p[1] for p in janela) / len(janela)
        
        # Arredonda para descobrir qual é a célula dominante
        celula_x = int(round((media_x - origem[0]) / tamanho_celula))
        celula_y = int(round((media_y - origem[1]) / tamanho_celula))
        nova_celula = (celula_x, celula_y)
        
        # Só adiciona se for uma célula diferente da última que registramos
        if not caminho_bruto or caminho_bruto[-1] != nova_celula:
            caminho_bruto.append(nova_celula)
            
    caminho_normalizado = []
    
    for i in range(len(caminho_bruto)):
        if i == 0:
            caminho_normalizado.append(caminho_bruto[i])
            continue
            
        atual = caminho_normalizado[-1]
        proximo = caminho_bruto[i]
        
        # Diferença em x e y (quantas células de distância)
        dx = proximo[0] - atual[0]
        dy = proximo[1] - atual[1]
        
        # Se a diferença for maior que 1 (pulou uma ou mais células)
        if abs(dx) > 1 or abs(dy) > 1:
            # Descobre quantos passos faltam para preencher o buraco
            passos_faltantes = max(abs(dx), abs(dy))
            
            # Preenche as células intermediárias
            for passo in range(1, passos_faltantes + 1):
                x_interp = atual[0] + int(round((dx * passo) / passos_faltantes))
                y_interp = atual[1] + int(round((dy * passo) / passos_faltantes))
                caminho_normalizado.append((x_interp, y_interp))
        else:
            # Se não pulou nada, só adiciona normalmente
            caminho_normalizado.append(proximo)
            
    return caminho_normalizado
