from flask import Flask

app = Flask(__name__)

from app.routes import simulation_bp

app.register_blueprint(simulation_bp)