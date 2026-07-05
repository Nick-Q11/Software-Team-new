from flask import Flask, render_template, jsonify, request

app = Flask(__name__)

current_location = {
    'latitude': 0.0,
    'longitude': 0.0,
}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/get_location')
def get_location():
    return jsonify(current_location)

@app.route('/update_position', methods=['POST'])
def handle_update_position():
    data = request.get_json()
    if data and 'latitude' in data and 'longitude' in data:
        current_location['latitude'] = float(data['latitude'])
        current_location['longitude'] = float(data['longitude'])
        return jsonify({"status": "success"}), 200
    return jsonify({"status": "error", "message": "Ungültige Daten"}), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)