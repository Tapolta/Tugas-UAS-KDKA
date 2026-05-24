import heapq
from typing import List, Tuple, Dict

class RouterService:
    EMPTY_PATH = 0
    WALL = 1
    MARKET = 2
    STATION = 3
    EVENT_MORNING = 4
    EVENT_NOON = 5
    EVENT_EVENING = 6
    START_NODE = 7
    GOAL_NODE = 8

    @classmethod
    def get_obstacle_weights(cls, hari: str, jam: str) -> Dict[int, int]:
        weights = {
            cls.MARKET: 5, cls.STATION: 1,
            cls.EVENT_MORNING: 2, cls.EVENT_NOON: 2, cls.EVENT_EVENING: 2
        }
        
        hari_clean = hari.strip().capitalize()
        
        if hari_clean in ['Senin', 'Selasa', 'Rabu', 'Kamis', 'Jumat']:
            if "Pagi" in jam: weights[cls.EVENT_MORNING] = 25
            elif "Siang" in jam:
                weights[cls.MARKET] = 30
                weights[cls.EVENT_NOON] = 25
            elif "Sore" in jam: weights[cls.EVENT_EVENING] = 25

        elif hari_clean in ['Sabtu', 'Minggu']:
            weights[cls.MARKET] = 20
            if "Siang" in jam: weights[cls.EVENT_NOON] = 15
            elif "Sore" in jam: weights[cls.EVENT_EVENING] = 20

        return weights

    @staticmethod
    def _calculate_heuristic(p1: Tuple[int, int], p2: Tuple[int, int]) -> int:
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    @classmethod
    def find_shortest_path(
        cls, 
        peta: List[List[int]], 
        start: Tuple[int, int], 
        goal: Tuple[int, int], 
        weights: Dict[int, int]
    ) -> List[Tuple[int, int]]:
        cols, rows = len(peta), len(peta[0])
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: cls._calculate_heuristic(start, goal)}
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
                
            for dc, dr in directions:
                neighbor = (current[0] + dc, current[1] + dr)
                
                if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows:
                    tile_type = peta[neighbor[0]][neighbor[1]]
                    if tile_type == cls.WALL:
                        continue
                    
                    step_cost = weights.get(tile_type, 1) if tile_type != cls.EMPTY_PATH else 1
                    tentative_g_score = g_score[current] + step_cost
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + cls._calculate_heuristic(neighbor, goal)
                        
                        if neighbor not in [item[1] for item in open_set]:
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
                            
        return []