from flask import Flask, json, render_template, request, jsonify
from pymongo.mongo_client import MongoClient
from pymongo.server_api import ServerApi
import pandas as pd
import matplotlib.pyplot as plt
import io
import base64
from datetime import datetime, timedelta

# MongoDB Connection
uri = "mongodb+srv://webserver:LplGjU7BzROtM3bd@cluster0.sy8fc.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0"
client = MongoClient(uri, server_api=ServerApi('1'))
db = client.get_database('sensor_data_db')  # Database name
collection = db.get_collection('sensor_data')  # Collection name

# Flask App
app = Flask(__name__)

# Root route
@app.route('/')
def index():
    return """
    <h1>Welcome to the Sensor Data Dashboard</h1>
    <p>Available routes:</p>
    <ul>
        <li><a href="/history?zone=lab">Hourly History</a></li>
        <li><a href="/interactions">Object Interactions</a></li>
        <li><a href="/visualization">Comparative Visualization</a></li>
    </ul>
    """

# Favicon route
@app.route('/favicon.ico')
def favicon():
    return "", 204  # No content for favicon requests

@app.route('/sensor_data', methods=['GET'])
def sensor_data():
    # Extract the data sent from Arduino
    motion = request.args.get('motion', default=None)
    light_red = request.args.get('light_red', default=None)
    light_green = request.args.get('light_green', default=None)
    temperature = request.args.get('temperature', default=None)
    
    # Print the received data to the console (for debugging)
    print(f"Received data: motion={motion}, light_red={light_red}, light_green={light_green}, temperature={temperature}")

    # Process and store the data (if needed)
    # Return a response
    return 'Data received successfully', 200


# Route for hourly history
@app.route('/history', methods=['GET'])
def hourly_history():
    try:
        zone = request.args.get('zone', 'lab')  # Zone specified in query params
        end_time = datetime.now()
        start_time = end_time - timedelta(days=1)

        # Query for the last day's data
        cursor = collection.find({
            "zone": zone,
            "timestamp": {"$gte": start_time, "$lte": end_time}
        })
        data = pd.DataFrame(list(cursor))

        # Check if data exists
        if data.empty:
            return "No data available for the specified zone and time range."

        # Group by hour for summaries
        data['hour'] = data['timestamp'].dt.floor('H')
        summary = data.groupby('hour').mean()[['temperature', 'noise', 'light']]

        # Return JSON response
        return jsonify(summary.reset_index().to_dict(orient='records'))
    except Exception as e:
        return f"An error occurred: {e}"

# Route for object interactions
@app.route('/interactions', methods=['GET'])
def object_interactions():
    try:
        end_time = datetime.now()
        start_time = end_time - timedelta(days=1)

        # Query for object interactions
        cursor = collection.find({
            "event_type": "interaction",
            "timestamp": {"$gte": start_time, "$lte": end_time}
        })
        data = pd.DataFrame(list(cursor))

        # Check if data exists
        if data.empty:
            return "No interactions found in the specified time range."

        # Return JSON response
        return jsonify(data.to_dict(orient='records'))
    except Exception as e:
        return f"An error occurred: {e}"

# Route for comparative visualization
@app.route('/visualization', methods=['GET'])
def comparative_visualization():
    try:
        end_time = datetime.now()
        start_time = end_time - timedelta(days=1)

        # Query data
        cursor = collection.find({
            "timestamp": {"$gte": start_time, "$lte": end_time}
        })
        data = pd.DataFrame(list(cursor))

        if data.empty:
            return "No data available for visualization."

        # Create a comparative plot
        plt.figure(figsize=(10, 6))
        plt.plot(data['timestamp'], data['temperature'], label='Temperature')
        plt.plot(data['timestamp'], data['noise'], label='Noise')
        plt.plot(data['timestamp'], data['light'], label='Light')
        plt.legend()
        plt.title('Comparative Sensor Data')
        plt.xlabel('Time')
        plt.ylabel('Values')
        plt.grid()

        # Convert plot to image
        img = io.BytesIO()
        plt.savefig(img, format='png')
        img.seek(0)
        img_base64 = base64.b64encode(img.getvalue()).decode()
        plt.close()

        return f'<img src="data:image/png;base64,{img_base64}"/>'
    except Exception as e:
        return f"An error occurred: {e}"

# Run the app
if __name__ == '__main__':
    app.run(host="0.0.0.0",debug=True)
