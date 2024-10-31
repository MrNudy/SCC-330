CREATE DATABASE IF NOT EXISTS environmentdata_db;
USE environmentdata_db;

CREATE TABLE IF NOT EXISTS EnvironmentalData (
    id INT AUTO_INCREMENT PRIMARY KEY,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    temperature FLOAT,
    ambient_light FLOAT,
    sound_level FLOAT,
    object_usage BOOLEAN,
    water_level FLOAT
);
