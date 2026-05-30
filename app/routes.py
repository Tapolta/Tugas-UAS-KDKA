from flask import Blueprint, render_template, request, jsonify
from app.services.router_service import RouterService

simulation_bp = Blueprint('simulation', __name__)

robot_physical_state = {
    "status": "idle",          
    "rute_penuh": [],          
    "posisi_sekarang": None,
    "jalan_dilewati": []
}

@simulation_bp.route('/')
def landing():
    return render_template('landing.html')

@simulation_bp.route('/main-map')
def main_map():
    return render_template('main_map.html')

@simulation_bp.route('/api/simulasi', methods=['POST'])
def simulasi_rute():
    data = request.get_json()
    if not data:
        return jsonify({"status": "error", "message": "Malformasi data JSON tidak valid."}), 400
        
    hari = data.get('hari')
    jam = data.get('jam')
    peta = data.get('peta') 
    
    algoritma_pilihan = data.get('algoritma', 'astar').lower()
    
    if not all([hari, jam, peta]):
        return jsonify({"status": "error", "message": "Parameter tidak lengkap."}), 400

    titik_mulai = None
    titik_selesai = None
    daftar_pemberhentian = [] 
    
    for col_idx, col in enumerate(peta):
        for row_idx, value in enumerate(col):
            if value == RouterService.START_NODE:
                titik_mulai = (col_idx, row_idx)
            elif value == RouterService.GOAL_NODE:
                titik_selesai = (col_idx, row_idx)
            elif value == RouterService.STATION:
                daftar_pemberhentian.append((col_idx, row_idx))

    if not titik_mulai or not titik_selesai:
        return jsonify({"status": "error", "message": "Peta wajib memiliki minimal 1 Titik Mulai dan 1 Titik Selesai!"}), 400

    try:
        active_weights = RouterService.get_obstacle_weights(hari, jam)
        
        def get_calculated_path(start_node, goal_node):
            if algoritma_pilihan == 'bfs':
                return RouterService.find_path_bfs(peta, start_node, goal_node)
            elif algoritma_pilihan == 'dfs':
                return RouterService.find_path_dfs(peta, start_node, goal_node)
            else:
                return RouterService.find_path_weighted_astar(peta, start_node, goal_node, active_weights)

        rute_penuh = []
        posisi_sekarang = titik_mulai

        while daftar_pemberhentian:
            terdekat = None
            jarak_terpendek = float('inf')
            rute_terpilih = []
            
            for stasiun in daftar_pemberhentian:
                rute_uji = get_calculated_path(posisi_sekarang, stasiun)
                
                if rute_uji and len(rute_uji) < jarak_terpendek:
                    jarak_terpendek = len(rute_uji)
                    terdekat = stasiun
                    rute_terpilih = rute_uji
            
            if not terdekat:
                return jsonify({"status": "error", "message": "Ada titik pemberhentian yang terblokir!"}), 422
                
            if rute_penuh:
                rute_penuh.extend(rute_terpilih[1:])
            else:
                rute_penuh.extend(rute_terpilih)
                
            posisi_sekarang = terdekat
            daftar_pemberhentian.remove(terdekat)

        rute_ke_finish = get_calculated_path(posisi_sekarang, titik_selesai)
        
        if not rute_ke_finish:
            return jsonify({"status": "error", "message": "Rute menuju titik selesai terblokir!"}), 422
            
        if rute_penuh:
            rute_penuh.extend(rute_ke_finish[1:])
        else:
            rute_penuh.extend(rute_ke_finish)

        return jsonify({
            "status": "success", 
            "rute": rute_penuh,
            "algoritma_digunakan": algoritma_pilihan
        }), 200

    except Exception as e:
        return jsonify({"status": "error", "message": f"Kegagalan sistem internal: {str(e)}"}), 500
    
@simulation_bp.route('/api/robot/siapkan-rute', methods=['POST'])
def siapkan_rute():
    data = request.get_json()
    
    if not data:
        return jsonify({"status": "error", "message": "Data JSON tidak ditemukan."}), 400
        
    rute_diterima = data.get('rute')
    
    if not rute_diterima or not isinstance(rute_diterima, list) or len(rute_diterima) == 0:
        return jsonify({"status": "error", "message": "Data rute tidak valid atau kosong."}), 400

    try:
        global robot_physical_state
        
        robot_physical_state["rute_penuh"] = rute_diterima
        robot_physical_state["posisi_sekarang"] = rute_diterima[0]
        robot_physical_state["jalan_dilewati"] = []
        robot_physical_state["status"] = "ready"
        
        return jsonify({
            "status": "success", 
            "message": "Rute berhasil disimpan dan siap dieksekusi oleh robot fisik."
        }), 200
        
    except Exception as e:
        return jsonify({
            "status": "error", 
            "message": f"Gagal menyiapkan rute: {str(e)}"
        }), 500
    
@simulation_bp.route('/api/robot/status', methods=['GET'])
def status():
    try:
        global robot_physical_state
        
        return jsonify(robot_physical_state), 200
        
    except Exception as e:
        return jsonify({
            "status": "error", 
            "message": f"Gagal mengambil status robot: {str(e)}"
        }), 500


@simulation_bp.route('/api/robot/update', methods=['POST'])
def update_robot():
    data = request.get_json()
    
    if not data:
        return jsonify({"status": "error", "message": "Data JSON tidak ditemukan."}), 400

    try:
        global robot_physical_state
        
        if 'status' in data:
            robot_physical_state['status'] = data['status']
            
        if 'posisi_sekarang' in data:
            posisi_baru = data['posisi_sekarang']
            
            if isinstance(posisi_baru, list) and len(posisi_baru) == 2:
                if robot_physical_state['posisi_sekarang'] is not None:
                    if robot_physical_state['posisi_sekarang'] != posisi_baru:
                        robot_physical_state['jalan_dilewati'].append(robot_physical_state['posisi_sekarang'])
                
                robot_physical_state['posisi_sekarang'] = posisi_baru

        if robot_physical_state['status'] == 'idle':
            robot_physical_state['rute_penuh'] = []
            
        return jsonify({"status": "success", "message": "State robot berhasil diperbarui."}), 200

    except Exception as e:
        return jsonify({"status": "error", "message": f"Gagal memperbarui state: {str(e)}"}), 500