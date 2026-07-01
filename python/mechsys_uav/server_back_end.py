from flask import Flask, render_template, request, jsonify
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


@app.route('/')
def index():
    """Serve the main webpage"""
    return render_template('index.html')

@app.route('/get_location')
def get_location():
    
    """Endpoint for webpage to get current coordinates"""
    return jsonify(current_location)

def main():
    
    index()

def test():   
    long = 49.12
    lat = 11.12
    update_location(long, lat)
if __name__ == '__main__':
    # Run on all network interfaces, port 5000
    test()
    app.run(host='0.0.0.0', port=5000, debug=False)