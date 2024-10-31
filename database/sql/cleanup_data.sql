USE environmentdata_db;

DROP EVENT IF EXISTS cleanup_old_data;

CREATE EVENT cleanup_old_data
ON SCHEDULE EVERY 1 HOUR
DO
BEGIN
    DELETE FROM EnvironmentalData WHERE timestamp <= NOW() - INTERVAL 24 HOUR;
END;
