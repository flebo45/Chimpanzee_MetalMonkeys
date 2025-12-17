import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__, template_folder='templates')
app.config['SECRET_KEY'] = 'hunter_secet_key'

socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# HTTP Routes
@app.route('/')
def index():
    return render_template('index.html')

# WebSocket Events
@socketio.on('robot_telemetry')
def handle_telemetry(data):
    """
    JSON Input Example:
    {
       'error_x',
       'cmd_vel_z',
       'distance',
       'visible',
       'battery_level'
    }
    """
    #print("Received telemetry data:", data)
    socketio.emit('update_gui', data)

@socketio.on('robot_video')
def handle_video(base64_string):
    """
    Frame video coding in base64 string
    """
    #print("Received video frame", len(base64_string), "characters")
    socketio.emit('update_video', base64_string)

@socketio.on('connect')
def handle_connect():
    print("Client connected")

if __name__ == '__main__':
    print("Starting Dashboard Server...")
    socketio.run(app, host='0.0.0.0', port=5000)