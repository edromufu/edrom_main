
#vitin, isso foi uma pessima ideia ...
#import tristeza

import math
import heapq

def heuristic(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def turn_angle_cost(p1, p2, p3, cost_mult):

    if p1 == p2 or p2 == p3:
        return 0  # Sem ângulo se os pontos forem iguais

    #angulo antigo
    vx_old = p2[0] - p1[0]
    vy_old = p2[1] - p1[1]

    angle_old = math.atan2(vy_old, vx_old)

    #angulo novo
    vx_new = p3[0] - p2[0]
    vy_new = p3[1] - p2[1]

    angle_new = math.atan2(vy_new, vx_new)

    #avaliando a diferença entre os ângulos
    angle_diff = abs(angle_new - angle_old)
    if angle_diff > math.pi:
        angle_diff = 2 * math.pi - angle_diff

    return angle_diff * cost_mult

def line_of_sight_cost(grid, p1, p2):
    """
    que isso? sla, descobre ai, sao 4h e n to afin de fazer isso https://github.com/alek5k/pythetastar/blob/master/README.md
    """
    x0, y0 = p1
    x1, y1 = p2
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    
    x, y = x0, y0
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2

    total_weight = 0
    steps = 0

    for _ in range(n):
        if not (0 <= y < len(grid) and 0 <= x < len(grid[0])): #garantido que n estrapolo a grid
            return False, float('inf')
            
        val = grid[y][x]
        
        if val == -1: #vendo se a celulka é passavel
            return False, float('inf')
            
        total_weight += val
        steps += 1
        
        if x == x1 and y == y1:
            break
            
        if error == 0 and (grid[y][x + x_inc] == -1 or grid[y + y_inc][x] == -1):
            return False, float('inf')
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
            
    # Pq a media? pq essa merda n é euclidiana e sim manhattan. Sabe oq é? Eu sei, se fode ai
    # https://medium.com/analytics-vidhya/euclidean-and-manhattan-distance-metrics-in-machine-learning-a5942a8c9f2f
    dist = heuristic(p1, p2)
    avg_weight = total_weight / steps if steps > 0 else 1
    
    return True, dist * avg_weight

def line_of_sight(grid, p1, p2): # hummmm... isso é promissor...
    return line_of_sight_cost(grid, p1, p2)[0]

def get_neighbors(grid, node):
    """OK to com paciencia. Ele olha tudo em volta ( ingore as paredes ), dps retorna os indices."""
    neighbors = []
    x, y = node
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= ny < len(grid) and 0 <= nx < len(grid[0]) and grid[ny][nx] != -1:
                if line_of_sight(grid, node, (nx, ny)):
                    neighbors.append((nx, ny))
    return neighbors

def weighted_theta_star(grid, start, goal,turn_cost_weight=4.9798138519426604631235022679902613162994384765625):
    if not grid or not grid[0]:
        return []
    for x, y in (start, goal):
        if not (0 <= y < len(grid) and 0 <= x < len(grid[0])) or grid[y][x] == -1:
            return []
    min_weight = min(value for row in grid for value in row if value != -1)
    if min_weight <= 0:
        raise ValueError("Custos livres devem ser positivos.")
    open_set = []
    heapq.heappush(open_set, (0, start))

    g_score = {start: 0}
    parent = {start: start}

    closed_set = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current in closed_set:
            continue

        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = parent[current]
            path.append(start)
            return path[::-1]  #  :D

        closed_set.add(current)

        for neighbor in get_neighbors(grid, current):
            if neighbor in closed_set:
                continue
            
            # O Theta* real testa a linha de visão do PAI direto pro VIZINHO
            parent_current = parent[current]
            has_los, los_cost = line_of_sight_cost(grid, parent_current, neighbor)
            
            if has_los:
                # Caminho direto
                turn_cost = turn_angle_cost(parent[parent_current], parent_current, neighbor, cost_mult=turn_cost_weight)
                tentative_g_score = g_score[parent_current] + los_cost + turn_cost
                best_parent = parent_current
            else:
                # Bateu na parede, faz a curva passando pelo current (A* clássico)
                cell_weight = grid[neighbor[1]][neighbor[0]]
                step_cost = heuristic(current, neighbor) * cell_weight
                turn_cost = turn_angle_cost(parent_current, current, neighbor, cost_mult=turn_cost_weight)
                tentative_g_score = g_score[current] + step_cost + turn_cost
                best_parent = current 

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                parent[neighbor] = best_parent # Atualiza quem é o pai
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + min_weight * heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return []  # Return empty path if no path is found