# api_server.py
from flask import Flask, request, jsonify
import sqlite3

app = Flask(__name__)

# Initialize the database, creating a table to store sensor data
def init_db():
    conn = sqlite3.connect('sensor_data.db')
    c = conn.cursor()
    c.execute('''
        CREATE TABLE IF NOT EXISTS sensor_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            sensor_mode TEXT,
            data TEXT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    conn.commit()
    conn.close()

# Endpoint to store data sent by the base station
@app.route('/store_data', methods=['POST'])
def store_data():
    data = request.get_json()  # Get JSON payload
    sensor_mode = data.get('sensor_mode')
    sensor_data = data.get('data')

    # Insert data into the database
    conn = sqlite3.connect('sensor_data.db')
    c = conn.cursor()
    c.execute("INSERT INTO sensor_data (sensor_mode, data) VALUES (?, ?)", (sensor_mode, sensor_data))
    conn.commit()
    conn.close()

    return jsonify({"status": "success"}), 200

if __name__ == '__main__':
    init_db()  # Initialize database on first run
    app.run(host='0.0.0.0', port=5000)  # Run the API on port 5000
