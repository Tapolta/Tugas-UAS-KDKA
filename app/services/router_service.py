import heapq
from collections import deque
from typing import List, Tuple, Dict, Optional

class RouterService:
    # Definisi Konstanta Peta
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
        """Mengambil bobot dinamis berdasarkan hari dan waktu."""
        weights = {
            cls.MARKET: 5, 
            cls.STATION: 1,
            cls.EVENT_MORNING: 2, 
            cls.EVENT_NOON: 2, 
            cls.EVENT_EVENING: 2
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
    def _calculate_heuristic(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
        """Menghitung jarak Manhattan antara dua titik."""
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    @staticmethod
    def _reconstruct_path(came_from: Dict[Tuple[int, int], Tuple[int, int]], current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Membangun ulang jalur dari goal ke start."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # ==========================================
    # 1. WEIGHTED A* ALGORITHM
    # ==========================================
    @classmethod
    def find_path_weighted_astar(
        cls, 
        peta: List[List[int]], 
        start: Tuple[int, int], 
        goal: Tuple[int, int], 
        weights: Dict[int, int],
        epsilon: float = 1.5
    ) -> List[Tuple[int, int]]:
        cols, rows = len(peta), len(peta[0])
        
        # [OPTIMASI] Validasi awal: Cegah pencarian jika start/goal berada di luar nalar (misal: di atas tembok)
        if peta[start[0]][start[1]] == cls.WALL or peta[goal[0]][goal[1]] == cls.WALL:
            return []

        # Inisialisasi Priority Queue. Format tuple: (f_score, koordinat_node)
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        
        # came_from: Melacak dari node mana kita datang untuk membangun rute di akhir
        came_from = {}
        
        # g_score: Melacak total biaya aktual dari titik start ke node tertentu
        g_score = {start: 0.0}
        
        # [OPTIMASI] Gunakan Tuple alih-alih List untuk directions karena lebih cepat saat diiterasi
        directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
        
        while open_set:
            # Pop node dengan f_score terendah (prioritas tertinggi)
            current_f, current = heapq.heappop(open_set)
            
            # LOGIKA: Jika node saat ini adalah tujuan, bangun rute dan kembalikan hasilnya
            if current == goal:
                return cls._reconstruct_path(came_from, current)
            
            # [OPTIMASI] Lazy Deletion: 
            # Jika ada node duplikat di antrean dengan f_score yang lebih buruk (karena pembaruan sebelumnya), abaikan saja.
            current_g = g_score.get(current, float('inf'))
            if current_f > current_g + (epsilon * cls._calculate_heuristic(current, goal)):
                continue

            # LOGIKA: Cek semua tetangga di 4 arah
            for dc, dr in directions:
                neighbor = (current[0] + dc, current[1] + dr)
                
                # Pastikan tetangga tidak keluar dari grid array peta
                if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows:
                    tile_type = peta[neighbor[0]][neighbor[1]]
                    
                    # LOGIKA: Jika menabrak tembok, lewati (jangan diproses)
                    if tile_type == cls.WALL:
                        continue
                    
                    # LOGIKA BOBOT: Ambil nilai bobot (hambatan) dari tile tersebut. Default = 1 jika tidak ada di dictionary.
                    step_cost = weights.get(tile_type, 1) if tile_type != cls.EMPTY_PATH else 1
                    
                    # Hitung biaya sementara (g_score) jika kita bergerak ke tetangga ini melalui 'current' node
                    tentative_g_score = current_g + step_cost
                    
                    # LOGIKA: Jika jalur ke tetangga ini lebih murah daripada jalur yang pernah ditemukan sebelumnya...
                    if tentative_g_score < g_score.get(neighbor, float('inf')):
                        # ...maka catat jalur ini sebagai jalur terbaik sementara
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        
                        # LOGIKA WEIGHTED A*: f(n) = g(n) + (epsilon * h(n))
                        # Epsilon > 1 membuat algoritma mengutamakan heuristik (lebih cepat menuju goal, tapi sedikit mengorbankan akurasi jalur terpendek)
                        h_score = cls._calculate_heuristic(neighbor, goal)
                        f_score = tentative_g_score + (epsilon * h_score)
                        
                        # Masukkan tetangga ke antrean prioritas untuk dievaluasi nanti
                        heapq.heappush(open_set, (f_score, neighbor))
                            
        # Jika loop selesai tapi goal tidak ditemukan, kembalikan rute kosong (jalan buntu)
        return []

    # ==========================================
    # 2. BREADTH-FIRST SEARCH (BFS) ALGORITHM
    # ==========================================
    @classmethod
    def find_path_bfs(
        cls, 
        peta: List[List[int]], 
        start: Tuple[int, int], 
        goal: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        cols, rows = len(peta), len(peta[0])
        if peta[start[0]][start[1]] == cls.WALL or peta[goal[0]][goal[1]] == cls.WALL:
            return []

        # LOGIKA BFS: Gunakan Deque (Double Ended Queue) untuk antrean FIFO (First In, First Out)
        queue = deque([start])
        
        # Gunakan Set untuk pelacakan node yang sudah divisit (Pencarian di dalam Set = O(1), sangat cepat)
        visited = {start}
        came_from = {}
        directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
        
        while queue:
            # Ambil node paling pertama yang masuk ke antrean
            current = queue.popleft()
            
            if current == goal:
                return cls._reconstruct_path(came_from, current)
                
            for dc, dr in directions:
                neighbor = (current[0] + dc, current[1] + dr)
                
                if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows:
                    tile_type = peta[neighbor[0]][neighbor[1]]
                    
                    # LOGIKA: BFS tidak peduli bobot jalan. Selama bukan WALL dan belum divisit, masukkan ke antrean.
                    # Ini menjamin BFS akan menemukan jalur dengan *jumlah langkah paling sedikit*, bukan *bobot teringan*.
                    if tile_type != cls.WALL and neighbor not in visited:
                        visited.add(neighbor)
                        came_from[neighbor] = current
                        queue.append(neighbor)
                        
        return []

    # ==========================================
    # 3. DEPTH-FIRST SEARCH (DFS) ALGORITHM
    # ==========================================
    @classmethod
    def find_path_dfs(
        cls, 
        peta: List[List[int]], 
        start: Tuple[int, int], 
        goal: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        cols, rows = len(peta), len(peta[0])
        if peta[start[0]][start[1]] == cls.WALL or peta[goal[0]][goal[1]] == cls.WALL:
            return []

        # LOGIKA DFS: Gunakan List standar sebagai Stack untuk LIFO (Last In, First Out)
        stack = [start]
        visited = {start}
        came_from = {}
        directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
        
        while stack:
            # Ambil node yang PALING TERAKHIR dimasukkan ke stack (menyelam sedalam mungkin)
            current = stack.pop()
            
            if current == goal:
                return cls._reconstruct_path(came_from, current)
                
            for dc, dr in directions:
                neighbor = (current[0] + dc, current[1] + dr)
                
                if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows:
                    tile_type = peta[neighbor[0]][neighbor[1]]
                    
                    # LOGIKA: Menelusuri terus satu cabang sampai mentok (Wall/ujung peta/sudah divisit).
                    # Jika mentok, akan mundur (backtrack) dan mencoba cabang sebelumnya.
                    if tile_type != cls.WALL and neighbor not in visited:
                        visited.add(neighbor)
                        came_from[neighbor] = current
                        stack.append(neighbor)
                        
        return []