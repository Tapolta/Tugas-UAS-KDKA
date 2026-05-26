from flask import Blueprint, render_template, request, jsonify
from app.services.router_service import RouterService

simulation_bp = Blueprint('simulation', __name__)

# --- STATE MEMORI SEMENTARA UNTUK ROBOT FISIK ---
# Catatan: Pada aplikasi produksi, data ini idealnya disimpan di database/Redis
robot_physical_state = {
    "status": "idle",          # "idle" atau "running"
    "rute_penuh": [],          # Menyimpan koordinat rute aktif [[x,y], [x,y], ...]
    "posisi_sekarang": None,   # Koordinat [x, y] saat ini
    "jalan_dilewati": []       # Koordinat rute yang sudah berhasil dibersihkan/dilewati
}

@simulation_bp.route('/')
def landing():
    return render_template('landing.html')

@simulation_bp.route('/main-map')
def main_map():
    return render_template('main_map.html')

# 1. API GENERATE RUTE SIMULASI WEB (Kode Asli Kamu)
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


# 2. API BARU: MENERIMA/MENYIAPKAN RUTE UNTUK ROBOT FISIK
@simulation_bp.route('/api/robot/siapkan-rute', methods=['POST'])
def siapkan_rute_fisik():
    data = request.get_json()
    if not data or 'rute' not in data:
        return jsonify({"status": "error", "message": "Data rute tidak ditemukan."}), 400
    
    rute = data.get('rute')
    if not rute or len(rute) == 0:
        return jsonify({"status": "error", "message": "Rute kosong."}), 400
        
    # Reset dan set state robot fisik ke posisi awal rute
    robot_physical_state["rute_penuh"] = rute
    robot_physical_state["posisi_sekarang"] = rute[0]
    robot_physical_state["jalan_dilewati"] = [rute[0]]
    robot_physical_state["status"] = "running" # Otomatis aktif/bisa di-fetch ESP32
    
    return jsonify({"status": "success", "message": "Rute berhasil diunggah ke server backend!"}), 200


# 3. API BARU: POLLING STATUS UNTUK CANVAS JAVASCRIPT FRONTEND
@simulation_bp.route('/api/robot/status', methods=['GET'])
def ambil_status_robot():
    # Mengembalikan koordinat aktual robot fisik agar digambar sebagai lingkaran ungu di web
    return jsonify({
        "status": robot_physical_state["status"],
        "posisi_sekarang": robot_physical_state["posisi_sekarang"],
        "jalan_dilewati": robot_physical_state["jalan_dilewati"]
    }), 200


# 4. API TAMBAHAN (OPSIONAL): UNTUK ESP32/HARDWARE UPDATE KOORDINATNYA
@simulation_bp.route('/api/robot/update-posisi', methods=['POST'])
def update_posisi_dari_hardware():
    data = request.get_json()
    posisi = data.get('posisi') # Kirim format [col, row] dari ESP32
    
    if posisi:
        robot_physical_state["posisi_sekarang"] = posisi
        if posisi not in robot_physical_state["jalan_dilewati"]:
            robot_physical_state["jalan_dilewati"].append(posisi)
            
        # Jika koordinat saat ini sudah menyentuh titik koordinat terakhir rute
        if posisi == robot_physical_state["rute_penuh"][-1]:
            robot_physical_state["status"] = "idle"
            
        return jsonify({"status": "success"}), 200
    return jsonify({"status": "error", "message": "Koordinat tidak valid"}), 400