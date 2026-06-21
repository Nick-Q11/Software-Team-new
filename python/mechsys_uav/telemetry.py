from flask import Flask, render_template, request, jsonify
from mechsys_uav import UAV

app = Flask(__name__)

# Store the latest coordinates
current_location = {
    'latitude': 0.0,
    'longitude': 0.0,
}

@app.route('/')
def index():
    """Serve the main webpage"""
    return render_template('index.html')


"""def update_location(long =0.0, lat=0.0):
    
    global current_location
    data = request.json
    current_location['latitude'] = data.get('latitude', lat)
    current_location['longitude'] = data.get('longitude', long)

    print(f"Location updated: {current_location}")
    return jsonify({'status': 'success'})
"""
def update_location(uav: UAV):
    """Update the location based on the UAV's current position"""
    global current_location
    data = request.json
    lat, long, alt = uav.get_position()
        
    current_location['latitude'] = data.get('latitude', lat)
    current_location['longitude'] = data.get('longitude', long)

    print(f"Location updated: {current_location}")
    return jsonify({'status': 'success'})

@app.route('/get_location')
def get_location():
    """Endpoint for webpage to get current coordinates"""
    return jsonify(current_location)

if __name__ == '__main__':
    # Run on all network interfaces, port 5000
    app.run(host='0.0.0.0', port=5000, debug=False)