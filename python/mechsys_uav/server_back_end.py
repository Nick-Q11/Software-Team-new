from flask import Flask, render_template, jsonify
import requests
import threading
from mechsys_uav import UAV

app = Flask(__name__)
    
# Store the latest coordinates
current_location = {
    'latitude': 0.0,
    'longitude': 0.0,
}


def update_location(latitude=None, longitude=None):
    if latitude is not None:
        current_location['latitude'] = float(latitude)
    if longitude is not None:
        current_location['longitude'] = float(longitude)
        
def send_position(lat, long,  ip="127.0.0.1", port=5000):
    """Send the current position to the server"""
    url = f'http://{ip}:{port}/update_position'
    data = {
        'latitude': lat,
        'longitude': long
    }
    try:
        response = requests.post(url, json=data, timeout=1)
        if response.status_code == 200:
            print("Position sent successfully.")
        else:
            print(f"Failed to send position. Status code: {response.status_code}")
    except Exception as e:
        print(f"Error sending position: {e}")


@app.route('/')
def index():
    """Serve the main webpage"""
    return render_template('index.html')

@app.route('/get_location')
def get_location():
    
    """Endpoint for webpage to get current coordinates"""
    return jsonify(current_location)

@app.route('/update_position', methods=['POST'])
def handle_update_position():
    """Endpunkt, der die POST-Anfragen von send_position verarbeitet"""
    data = requests.get_json()
    if data and 'latitude' in data and 'longitude' in data:
        update_location(latitude=data['latitude'], longitude=data['longitude'])
        return jsonify({"status": "success", "current_location": current_location}), 200
    return jsonify({"status": "error", "message": "Invalid data"}), 400

def main():
    #threading.Thread(target=send_position, args=(0.0, 0.0), daemon=True).start()
    update_location(latitude=4.0, longitude=10.0)
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == '__main__':
    # Run on all network interfaces, port 5000
    main()
    