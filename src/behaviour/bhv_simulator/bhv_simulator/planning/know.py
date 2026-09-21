

''' IDEIA AAAAAAA
Oq pretendo fazer? irei rodar o campo vetorial e alterar a grid do theta estrala para priorizar 
as celulas que o campo veotorial passar, a partir desse ponto irei ver o qual distance o campo 
vetorial esta em relação ao theta estrela. irei guardar esse valor e a cada passo do robo irei rodar 
o campo vetorial novamente, caso o valor atual possua uma diferença maior que um determinado valor, irei recalcular o caminho do theta estrela, caso contrario irei seguir o caminho atual.

PECULIARIDADES DE CADA UM ->
Campo vetorial: Não precisa de uma grid para funcionar, apenas lhe dar a posição dos obstaculos, de onde está partindo e onde pretende chegar, alem de ser rapido para calcular o caminho

Theta estrela: Precisa de uma grid para funcionar, alem de ser mais lento para calcular o caminho, porem ele consegue achar caminhos mais otimizados e precisos.

Funcionamento do codigo.
1. informações externas: irei pressupar que sabemos a distancia ponta a ponta do campo(sabemos), que sabemos onde esta a bola(mais ou menos) e qualquer obstaculo, alem da posição do robo. 

2. campo vetorial: irei rodar o campo vetorial e gerar uma grid de pesos, onde as celulas que o campo vetorial passar terão um peso menor, e as celulas que estiverem longe do campo vetorial terão um peso maior.

3. theta estrela: irei converter as informações anteriores de forma a serem viaveis no theta estrela, em seguinda irei rodar o algorimito para ter o caminho mais otimizado.

4. comparação: irei armazenar o erro de ambos os caminhos.

5. loop: a cada passo do robo irei rodar o campo vetorial novamente, caso o erro atual seja maior que um determinado valor somado ao erro da primeira comparação, irei recalcular o caminho do theta estrela, caso contrario irei seguir o caminho atual.'''
import math
from enum import Enum, auto
from typing import Any, Dict, List, Tuple
if __package__:
    from . import campoVetorial as apf
    from . import thtastart_v2_p1inguin as ts
else:
    import campoVetorial as apf
    import thtastart_v2_p1inguin as ts
#-------- dados --------
class TypeRef(Enum):
    ME_POS = auto()
    BALL_POS = auto()
    GOL_POS = auto()
    OBSTACLES = auto()
    REFERENCE_POS = auto()

class Reference:
    def __init__(self, x: float, y: float, ref_id: int):
        self.x = x
        self.y = y
        self.id = ref_id

class Blackboard:
    def __init__(self):
        self._data: Dict[TypeRef, Any] = {
            TypeRef.ME_POS: (0.0, 0.0),
            TypeRef.BALL_POS: (0.0, 0.0),
            TypeRef.OBSTACLES: [],
            TypeRef.REFERENCE_POS: []
        }
    def get(self, key: TypeRef) -> Any: return self._data.get(key)
    def set(self, key: TypeRef, value: Any) -> None: self._data[key] = value

# -------- funcoes --------
def generate_costmap(referencias, tamanho_celula=0.1, margem=0.5,
                     inicio=None, alvo=None, obstaculos=(), raio_robo=0.25,
                     margem_seguranca=0.1, limites=None):
    """Grid nodes in metres; -1 is blocked, never a potential-field discount."""
    if not math.isfinite(tamanho_celula) or tamanho_celula <= 0:
        raise ValueError("Resolução deve ser positiva e finita.")
    if any(not math.isfinite(v) or v < 0 for v in (margem, raio_robo, margem_seguranca)):
        raise ValueError("Raios e margens devem ser não negativos e finitos.")
    pontos = [(r.x, r.y) for r in referencias]
    pontos += [p for p in (inicio, alvo) if p is not None]
    padding = raio_robo + margem_seguranca + tamanho_celula * math.sqrt(2)
    for obs in obstaculos:
        if not all(math.isfinite(obs[k]) for k in ('x', 'y', 'raio')) or obs['raio'] < 0:
            raise ValueError("Obstáculo inválido.")
        r = obs['raio'] + padding
        pontos.extend([(obs['x'] - r, obs['y'] - r), (obs['x'] + r, obs['y'] + r)])
    if not pontos:
        return None
    if not all(math.isfinite(v) for point in pontos for v in point):
        raise ValueError("Posição inválida.")
    if limites is None:
        min_x, max_x = min(p[0] for p in pontos) - margem, max(p[0] for p in pontos) + margem
        min_y, max_y = min(p[1] for p in pontos) - margem, max(p[1] for p in pontos) + margem
    else:
        min_x, max_x, min_y, max_y = limites
        if not all(math.isfinite(v) for v in limites) or min_x >= max_x or min_y >= max_y:
            raise ValueError("Limites inválidos.")
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
            blocked = any(math.hypot(x - o['x'], y - o['y']) <= o['raio'] + padding
                          for o in obstaculos)
            if limites is not None:
                blocked |= not (min_x + padding <= x <= max_x - padding and
                                min_y + padding <= y <= max_y - padding)
            values.append(-1 if blocked else 1.0)
        grid.append(values)
    return dict(grid=grid, linhas=linhas, colunas=colunas, tamanho_celula=tamanho_celula,
                offset_x=min_x, offset_y=min_y)


def world_to_grid(pos, costmap):
    # Reject outside points: clamping silently changes the requested goal.
    x = (pos[0] - costmap['offset_x']) / costmap['tamanho_celula']
    y = (pos[1] - costmap['offset_y']) / costmap['tamanho_celula']
    if not (0 <= x <= costmap['colunas'] - 1 and 0 <= y <= costmap['linhas'] - 1):
        raise ValueError("Posição fora da grade.")
    return round(x), round(y)


def grid_to_world(grid_pos, costmap):
    return (costmap['offset_x'] + grid_pos[0] * costmap['tamanho_celula'],
            costmap['offset_y'] + grid_pos[1] * costmap['tamanho_celula'])


def path_is_clear(path, obstaculos, clearance, limites=None):
    """Continuous segment-circle check, including exact (unrounded) endpoints."""
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

#-------- funcoes auxiliares --------
def calculate_cross_track_error(caminho_apf, caminho_theta, max_desvio=1.0):
    if not caminho_apf or len(caminho_theta) < 2: return 0.0
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
            if dist < menor_dist: menor_dist = dist
        erro_total += menor_dist
    return min(100.0, ((erro_total / len(caminho_apf)) / max_desvio) * 100.0)

def predict_apf_trajectory(inicio, alvo, obstaculos, params, max_passos=500):
    caminho = [inicio]
    pos = inicio
    for _ in range(max_passos):
        if apf.norma(apf.subtrair(alvo, pos)) < 0.2: break
        forca = apf.calcular_forca_total(pos, alvo, obstaculos, params)
        vel = apf.limitar_velocidade(forca, params["velocidade_max"])
        if apf.norma(vel) < 0.01: break 
        pos = apf.atualizar_posicao(pos, vel, params["dt"])
        caminho.append(pos)
    return caminho

# ---- Planner ----
class HybridPlanner:
    def __init__(self, bb: Blackboard, params_apf: dict, limites=None,
                 tamanho_celula=0.1, margem_seguranca=0.1):
        self.bb, self.params_apf = bb, params_apf
        self.limites, self.tamanho_celula = limites, tamanho_celula
        self.margem_seguranca = margem_seguranca
        self.MAX_DESVIO_M, self.THRESHOLD_VARIACAO = 1.0, 15.0
        self.caminho_theta_world, self.erro_baseline, self.costmap = [], 0.0, None
        self._scene = None

    @property
    def clearance(self):
        return self.params_apf['raio_robo'] + self.margem_seguranca

    def scene(self):
        return (tuple(self.bb.get(TypeRef.BALL_POS)),
                tuple(sorted((o['x'], o['y'], o['raio']) for o in self.bb.get(TypeRef.OBSTACLES))),
                tuple((r.x, r.y) for r in self.bb.get(TypeRef.REFERENCE_POS)))

    def gerar_caminho_theta(self):
        self.caminho_theta_world = []
        inicio, alvo = self.bb.get(TypeRef.ME_POS), self.bb.get(TypeRef.BALL_POS)
        obstaculos = self.bb.get(TypeRef.OBSTACLES)
        self.costmap = generate_costmap(
            self.bb.get(TypeRef.REFERENCE_POS), self.tamanho_celula,
            inicio=inicio, alvo=alvo, obstaculos=obstaculos,
            raio_robo=self.params_apf['raio_robo'], margem_seguranca=self.margem_seguranca,
            limites=self.limites)
        self._scene = self.scene()
        if not self.costmap:
            return []
        try:
            start_idx, goal_idx = (world_to_grid(p, self.costmap) for p in (inicio, alvo))
        except ValueError:
            return []
        if not all(path_is_clear([p], obstaculos, self.clearance, self.limites) for p in (inicio, alvo)):
            return []
        # Permit only the measured start in the extra discretization margin.
        # Every returned continuous segment is validated below.
        self.costmap['grid'][start_idx[1]][start_idx[0]] = 1.0
        apf_world_path = predict_apf_trajectory(inicio, alvo, obstaculos, self.params_apf)
        apf_grid_path = apf.normalizar_grid_por_media(
            apf_world_path, tamanho_janela=3, tamanho_celula=self.tamanho_celula,
            origem=(self.costmap['offset_x'], self.costmap['offset_y']))
        for col, row in apf_grid_path:
            if 0 <= row < self.costmap['linhas'] and 0 <= col < self.costmap['colunas']:
                if self.costmap['grid'][row][col] != -1:
                    self.costmap['grid'][row][col] = 1
        cells = ts.weighted_theta_star(self.costmap['grid'], start_idx, goal_idx)
        if not cells:
            return []
        path = [tuple(inicio)] + [grid_to_world(p, self.costmap) for p in cells[1:-1]] + [tuple(alvo)]
        if not path_is_clear(path, obstaculos, self.clearance, self.limites):
            return []
        self.caminho_theta_world = path
        self.erro_baseline = calculate_cross_track_error(apf_world_path, path, self.MAX_DESVIO_M)
        return path

    def processar_ciclo(self) -> List[Tuple[float, float]]:
        inicio = tuple(self.bb.get(TypeRef.ME_POS))
        obs = self.bb.get(TypeRef.OBSTACLES)
        if not self.caminho_theta_world or self._scene != self.scene():
            return self.gerar_caminho_theta()
        remaining = list(self.caminho_theta_world[1:])
        while len(remaining) > 1 and math.dist(inicio, remaining[0]) < 0.06:
            remaining.pop(0)
        path = [inicio] + remaining
        if not path_is_clear(path, obs, self.clearance, self.limites):
            return self.gerar_caminho_theta()
        apf_path = predict_apf_trajectory(inicio, self.bb.get(TypeRef.BALL_POS), obs, self.params_apf)
        error = calculate_cross_track_error(apf_path, path, self.MAX_DESVIO_M)
        if abs(error - self.erro_baseline) > self.THRESHOLD_VARIACAO:
            return self.gerar_caminho_theta()
        self.caminho_theta_world = path
        return path

# ---- ros? ----    
if __name__ == "__main__":
    # Inicializa os dados
    bb = Blackboard()
    params = {
        "k_atr": 1.5, "k_rep": 0.08, "d0": 0.35, "raio_robo": 0.01, 
        "velocidade_max": 0.15, "dt": 0.05
    }
    planner = HybridPlanner(bb, params)

    # .Seria bom colocar isso automatico no ros
    bb.set(TypeRef.ME_POS, (0.0, 0.0))
    bb.set(TypeRef.BALL_POS, (6.0, 3.0))
    obs = [{"x": 2.2, "y": 1.8, "raio": 0.3}, {"x": 2.8, "y": 3.0, "raio": 0.3}]
    bb.set(TypeRef.OBSTACLES, obs)
    bb.set(TypeRef.REFERENCE_POS, [Reference(o["x"], o["y"], i) for i, o in enumerate(obs)])

    # tambem seria bom limitar mais essse função ou, talvez, abrir ela ao inves de mante-la compacta
    caminho_para_seguir = planner.processar_ciclo() 

    # agora preciso ver como isso se encaixa no ros
    
    print("Caminho Ótimo Entregue ao Robô:")
    print(caminho_para_seguir)