"""
PLANEJADOR HÍBRIDO: CAMPO VETORIAL (APF) + THETA*

Dependências:
- campoVetorial (apf)
- thtastart (ts)
"""

import math
from enum import Enum, auto
from typing import Any, Dict, List, Tuple, Optional

if __package__:
    from . import campoVetorial as apf
    from . import thtastart as ts
else:
    import behaviour.path_planning.path_planning.campoVetorial as apf
    import behaviour.path_planning.path_planning.thtastart as ts


# DADOS DEFINIDOS

class TypeRef(Enum):
    ME_POS = auto()
    BALL_POS = auto()
    GOL_POS = auto()
    OBSTACLES = auto()
    REFERENCE_POS = auto()


class Reference:
    """Representa uma coordenada espacial vinculada a um ID de identificação."""
    def __init__(self, x: float, y: float, ref_id: int):
        self.x = x
        self.y = y
        self.id = ref_id


class Blackboard:
    """Central de memória compartilhada para armazenar estados lidos do ambiente."""
    def __init__(self):
        self._data: Dict[TypeRef, Any] = {
            TypeRef.ME_POS: (0.0, 0.0),
            TypeRef.BALL_POS: (0.0, 0.0),
            TypeRef.OBSTACLES: [],
            TypeRef.REFERENCE_POS: []
        }

    def get(self, key: TypeRef) -> Any: 
        return self._data.get(key)
        
    def set(self, key: TypeRef, value: Any) -> None: 
        self._data[key] = value


# GERENCIAMENTO DE GRADE E MAPEAMENTO

def generate_costmap(referencias: List[Reference], tamanho_celula: float = 0.1, 
                     margem: float = 0.5, inicio: Optional[Tuple[float, float]] = None, 
                     alvo: Optional[Tuple[float, float]] = None, obstaculos: tuple = (), 
                     raio_robo: float = 0.25, margem_seguranca: float = 0.1, 
                     limites: Optional[Tuple[float, float, float, float]] = None,
                     custo_espaco_livre: float = 5.0) -> Optional[Dict]:
    """
    Gera um mapa de custos 2D. Obstáculos recebem -1 (bloqueado).
    Células livres recebem um peso base alto (custo_espaco_livre) para 
    permitir que o caminho do APF se torne o caminho "mais barato".
    """
    if not math.isfinite(tamanho_celula) or tamanho_celula <= 0:
        raise ValueError("Resolução deve ser positiva e finita.")
    if any(not math.isfinite(v) or v < 0 for v in (margem, raio_robo, margem_seguranca)):
        raise ValueError("Raios e margens devem ser não negativos e finitos.")
        
    pontos = [(r.x, r.y) for r in referencias]
    if inicio: pontos.append(inicio)
    if alvo: pontos.append(alvo)
        
    padding = raio_robo + margem_seguranca + tamanho_celula * math.sqrt(2)
    
    for obs in obstaculos:
        if not all(math.isfinite(obs[k]) for k in ('x', 'y', 'raio')) or obs['raio'] < 0:
            raise ValueError("Obstáculo inválido.")
        r = obs['raio'] + padding
        pontos.extend([(obs['x'] - r, obs['y'] - r), (obs['x'] + r, obs['y'] + r)])
        
    if not pontos:
        return None
    if not all(math.isfinite(v) for point in pontos for v in point):
        raise ValueError("Posição inválida nas referências.")

    if limites is None:
        min_x = min(p[0] for p in pontos) - margem
        max_x = max(p[0] for p in pontos) + margem
        min_y = min(p[1] for p in pontos) - margem
        max_y = max(p[1] for p in pontos) + margem
    else:
        min_x, max_x, min_y, max_y = limites
        if not all(math.isfinite(v) for v in limites) or min_x >= max_x or min_y >= max_y:
            raise ValueError("Limites da grade inválidos.")

    colunas = int(math.ceil((max_x - min_x) / tamanho_celula)) + 1
    linhas = int(math.ceil((max_y - min_y) / tamanho_celula)) + 1
    
    if linhas * colunas > 250000:
        raise ValueError("Grade excessiva: revise as unidades, limites ou resolução.")

    grid = []
    for row in range(linhas):
        y = min_y + row * tamanho_celula
        values = []
        for col in range(colunas):
            x = min_x + col * tamanho_celula
            blocked = any(math.hypot(x - o['x'], y - o['y']) <= o['raio'] + padding for o in obstaculos)
            
            if limites is not None:
                blocked |= not (min_x + padding <= x <= max_x - padding and
                                min_y + padding <= y <= max_y - padding)
                                
            # IMPORTANTE: Espaço livre ganha custo elevado; APF reduzirá isso depois
            values.append(-1 if blocked else custo_espaco_livre)
        grid.append(values)
        
    return {
        'grid': grid, 
        'linhas': linhas, 
        'colunas': colunas, 
        'tamanho_celula': tamanho_celula,
        'offset_x': min_x, 
        'offset_y': min_y
    }

def world_to_grid(pos: Tuple[float, float], costmap: Dict) -> Tuple[int, int]:
    """Converte coordenadas do mundo real (metros) para índices da matriz."""
    x = (pos[0] - costmap['offset_x']) / costmap['tamanho_celula']
    y = (pos[1] - costmap['offset_y']) / costmap['tamanho_celula']
    
    if not (0 <= x <= costmap['colunas'] - 1 and 0 <= y <= costmap['linhas'] - 1):
        raise ValueError(f"Posição {pos} fora dos limites da grade.")
    return round(x), round(y)

def grid_to_world(grid_pos: Tuple[int, int], costmap: Dict) -> Tuple[float, float]:
    """Converte índices da matriz para coordenadas do mundo real (metros)."""
    return (costmap['offset_x'] + grid_pos[0] * costmap['tamanho_celula'],
            costmap['offset_y'] + grid_pos[1] * costmap['tamanho_celula'])


# CINEMÁTICA E VALIDAÇÕES

def path_is_clear(path: List[Tuple[float, float]], obstaculos: List[Dict], 
                  clearance: float, limites: Optional[Tuple[float, float, float, float]] = None) -> bool:
    """Verifica se segmentos contínuos do caminho colidem com obstáculos."""
    if not path:
        return False
        
    if limites is not None:
        xmin, xmax, ymin, ymax = limites
        if any(not (xmin + clearance < x < xmax - clearance and
                    ymin + clearance < y < ymax - clearance) for x, y in path):
            return False
            
    for a, b in zip(path, path[1:] or path):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        
        for o in obstaculos:
            t = max(0, min(1, ((o['x'] - a[0]) * dx + (o['y'] - a[1]) * dy) / length2)) if length2 else 0
            if math.hypot(a[0] + t * dx - o['x'], a[1] + t * dy - o['y']) <= o['raio'] + clearance:
                return False
    return True

def calculate_cross_track_error(caminho_apf: List[Tuple[float, float]], 
                                caminho_theta: List[Tuple[float, float]], 
                                max_desvio: float = 1.0) -> float:
    """Calcula a divergência entre a rota gerada pelo APF e a do Theta*."""
    if not caminho_apf or len(caminho_theta) < 2: 
        return 0.0
        
    erro_total = 0.0
    for p in caminho_apf:
        menor_dist = float('inf')
        for i in range(len(caminho_theta) - 1):
            a, b = caminho_theta[i], caminho_theta[i+1]
            ab_x, ab_y = b[0] - a[0], b[1] - a[1]
            ap_x, ap_y = p[0] - a[0], p[1] - a[1]
            ab_len2 = ab_x**2 + ab_y**2
            
            if ab_len2 == 0: 
                dist = math.hypot(ap_x, ap_y)
            else:
                t = max(0.0, min(1.0, (ap_x * ab_x + ap_y * ab_y) / ab_len2))
                proj_x, proj_y = a[0] + t * ab_x, a[1] + t * ab_y
                dist = math.hypot(p[0] - proj_x, p[1] - proj_y)
                
            if dist < menor_dist: 
                menor_dist = dist
        erro_total += menor_dist
        
    return min(100.0, ((erro_total / len(caminho_apf)) / max_desvio) * 100.0)

def predict_apf_trajectory(inicio: Tuple[float, float], alvo: Tuple[float, float], 
                           obstaculos: List[Dict], params: Dict, 
                           max_passos: int = 500) -> List[Tuple[float, float]]:
    """Simula o trajeto gerado puramente pelo Campo Vetorial."""
    caminho = [inicio]
    pos = inicio
    
    for _ in range(max_passos):
        if apf.norma(apf.subtrair(alvo, pos)) < 0.2: 
            break
            
        forca = apf.calcular_forca_total(pos, alvo, obstaculos, params)
        vel = apf.limitar_velocidade(forca, params["velocidade_max"])
        
        if apf.norma(vel) < 0.01: 
            break 
            
        pos = apf.atualizar_posicao(pos, vel, params["dt"])
        caminho.append(pos)
        
    return caminho

# HÍBRID PLANNER

class HybridPlanner:
    """Núcleo de decisão que alterna e mescla as heurísticas do APF e Theta*."""
    def __init__(self, bb: Blackboard, params_apf: dict, limites=None,
                 tamanho_celula: float = 0.1, margem_seguranca: float = 0.1):
        self.bb = bb
        self.params_apf = params_apf
        self.limites = limites
        self.tamanho_celula = tamanho_celula
        self.margem_seguranca = margem_seguranca
        
        # Parâmetros de tolerância para recalcular rota
        self.MAX_DESVIO_M = 1.0
        self.THRESHOLD_VARIACAO = 15.0
        
        # Estados internos
        self.caminho_theta_world: List[Tuple[float, float]] = []
        self.erro_baseline: float = 0.0
        self.costmap: Optional[Dict] = None
        self._scene = None

    @property
    def clearance(self) -> float:
        """Distância segura obrigatória que o robô deve manter."""
        return self.params_apf['raio_robo'] + self.margem_seguranca

    def scene(self) -> Tuple:
        """Tira uma 'foto' hashable do cenário atual para detecção de mudanças."""
        return (tuple(self.bb.get(TypeRef.BALL_POS)),
                tuple(sorted((o['x'], o['y'], o['raio']) for o in self.bb.get(TypeRef.OBSTACLES))),
                tuple((r.x, r.y) for r in self.bb.get(TypeRef.REFERENCE_POS)))

    def gerar_caminho_theta(self) -> List[Tuple[float, float]]:
        """Processa a mesclagem APF -> Grid -> Theta*."""
        self.caminho_theta_world = []
        inicio = self.bb.get(TypeRef.ME_POS)
        alvo = self.bb.get(TypeRef.BALL_POS)
        obstaculos = self.bb.get(TypeRef.OBSTACLES)
        
        self.costmap = generate_costmap(
            self.bb.get(TypeRef.REFERENCE_POS), 
            self.tamanho_celula,
            inicio=inicio, 
            alvo=alvo, 
            obstaculos=obstaculos,
            raio_robo=self.params_apf['raio_robo'], 
            margem_seguranca=self.margem_seguranca,
            limites=self.limites,
            custo_espaco_livre=5.0  # Espaço neutro é custoso
        )
        self._scene = self.scene()
        
        if not self.costmap: return []
            
        try:
            start_idx = world_to_grid(inicio, self.costmap)
            goal_idx = world_to_grid(alvo, self.costmap)
        except ValueError:
            return []
            
        if not all(path_is_clear([p], obstaculos, self.clearance, self.limites) for p in (inicio, alvo)):
            return []

        self.costmap['grid'][start_idx[1]][start_idx[0]] = 1.0
        
        # Roda APF para descobrir o "Caminho Desejado"
        apf_world_path = predict_apf_trajectory(inicio, alvo, obstaculos, self.params_apf)
        apf_grid_path = apf.normalizar_grid_por_media(
            apf_world_path, 
            tamanho_janela=3, 
            tamanho_celula=self.tamanho_celula,
            origem=(self.costmap['offset_x'], self.costmap['offset_y'])
        )
        
        # Desconta o peso na grid para atrair o Theta*
        for col, row in apf_grid_path:
            if 0 <= row < self.costmap['linhas'] and 0 <= col < self.costmap['colunas']:
                if self.costmap['grid'][row][col] != -1:
                    # Rota do APF passa a custar menos que o espaço livre (1.0 vs 5.0)
                    self.costmap['grid'][row][col] = 1.0 
                    
        # Executa o Theta* na grid otimizada
        cells = ts.weighted_theta_star(self.costmap['grid'], start_idx, goal_idx)
        if not cells: return []
            
        path = [tuple(inicio)] + [grid_to_world(p, self.costmap) for p in cells[1:-1]] + [tuple(alvo)]
        
        if not path_is_clear(path, obstaculos, self.clearance, self.limites):
            return []
            
        self.caminho_theta_world = path
        self.erro_baseline = calculate_cross_track_error(apf_world_path, path, self.MAX_DESVIO_M)
        return path

    def processar_ciclo(self) -> List[Tuple[float, float]]:
        """Função chamada a cada iteração de controle (Tick) para validar o trajeto."""
        inicio = tuple(self.bb.get(TypeRef.ME_POS))
        obs = self.bb.get(TypeRef.OBSTACLES)
        
        # Se cenário mudou fundamentalmente ou não há caminho
        if not self.caminho_theta_world or self._scene != self.scene():
            return self.gerar_caminho_theta()
            
        remaining = list(self.caminho_theta_world[1:])
        while len(remaining) > 1 and math.dist(inicio, remaining[0]) < 0.06:
            remaining.pop(0)
            
        path = [inicio] + remaining
        
        # Verifica colisão imediata no trajeto guardado
        if not path_is_clear(path, obs, self.clearance, self.limites):
            return self.gerar_caminho_theta()
            
        # Avalia se a heurística APF divergiu muito da rota Theta* estática
        apf_path = predict_apf_trajectory(inicio, self.bb.get(TypeRef.BALL_POS), obs, self.params_apf)
        error = calculate_cross_track_error(apf_path, path, self.MAX_DESVIO_M)
        
        if abs(error - self.erro_baseline) > self.THRESHOLD_VARIACAO:
            return self.gerar_caminho_theta()
            
        self.caminho_theta_world = path
        return path