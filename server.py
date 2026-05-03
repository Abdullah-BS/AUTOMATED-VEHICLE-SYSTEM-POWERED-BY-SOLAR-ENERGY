#!/usr/bin/env python3
"""
Flask backend — run this in WSL:
  python3 server.py
Then open slam-control.html in your browser.
"""
from flask import Flask, jsonify, request
from flask_cors import CORS
import subprocess, datetime, os

app = Flask(__name__)
CORS(app)

@app.route('/save-map', methods=['POST'])
def save_map():
    data = request.get_json(silent=True) or {}
    map_name = data.get('mapName', 'my_map').strip() or 'my_map'
    # Sanitize: only allow alphanumeric, underscores, hyphens
    safe_name = ''.join(c for c in map_name if c.isalnum() or c in '_-')
    if not safe_name:
        safe_name = 'my_map'
    save_dir = os.path.expanduser('~/maps')
    os.makedirs(save_dir, exist_ok=True)
    map_path = os.path.join(save_dir, safe_name)
    cmd = f'bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/my_mapper/maps/{safe_name}"'
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30)
        success = result.returncode == 0
        return jsonify({
            'success': success,
            'mapName': safe_name,
            'mapPath': map_path,
            'stdout': result.stdout,
            'stderr': result.stderr,
            'timestamp': datetime.datetime.now().strftime('%H:%M:%S')
        })
    except subprocess.TimeoutExpired:
        return jsonify({'success': False, 'stderr': 'Command timed out after 30s', 'timestamp': datetime.datetime.now().strftime('%H:%M:%S')})
    except Exception as e:
        return jsonify({'success': False, 'stderr': str(e), 'timestamp': datetime.datetime.now().strftime('%H:%M:%S')})

@app.route('/status', methods=['GET'])
def status():
    return jsonify({'status': 'online', 'timestamp': datetime.datetime.now().strftime('%H:%M:%S')})

if __name__ == '__main__':
    print("🗺  SLAM Map Saver API running on http://localhost:5000")
    print("   Open slam-control.html in your browser to use the interface.")
    app.run(host='0.0.0.0', port=5000, debug=False)
