from flask import Blueprint, render_template, request, jsonify
from app.services.router_service import RouterService

simulation_bp = Blueprint('simulation', __name__)

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
        rute_penuh = []
        posisi_sekarang = titik_mulai

        while daftar_pemberhentian:
            terdekat = None
            jarak_terpendek = float('inf')
            rute_terpilih = []
            
            for stasiun in daftar_pemberhentian:
                rute_uji = RouterService.find_shortest_path(peta, posisi_sekarang, stasiun, active_weights)
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

        rute_ke_finish = RouterService.find_shortest_path(peta, posisi_sekarang, titik_selesai, active_weights)
        if not rute_ke_finish:
            return jsonify({"status": "error", "message": "Rute menuju titik selesai terblokir!"}), 422
            
        if rute_penuh:
            rute_penuh.extend(rute_ke_finish[1:])
        else:
            rute_penuh.extend(rute_ke_finish)

        return jsonify({"status": "success", "rute": rute_penuh}), 200

    except Exception as e:
        return jsonify({"status": "error", "message": f"Kegagalan sistem internal: {str(e)}"}), 500