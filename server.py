from typing import Tuple

# Database connection settings
DB_SETTINGS = {
    'host': "localhost",
    'database': "postgres",
    'user': "newuser",
    'password': "password"
}

from flask import Flask, request
import psycopg2
from psycopg2.extras import execute_values

app = Flask(__name__)

@app.route("/clear_db", methods=["POST"])
def clear_database() -> Tuple[str, int]:
    """
    Clear all entries from the point_cloud table and reset the scan_id sequence

    :return: Tuple containing a message and status code
    """
    try:
        connection = psycopg2.connect(**DB_SETTINGS)
        cursor = connection.cursor()
        cursor.execute("TRUNCATE TABLE point_cloud;")

        cursor.execute("ALTER SEQUENCE scan_id_seq RESTART WITH 1;")

        connection.commit()
        return "All entries deleted from the database", 200
    except Exception as e:
        return f"Error clearing database: {e}", 500
    finally:
        if 'connection' in locals():
            cursor.close()
            connection.close()

@app.route("/", methods=["POST"])
def upload_point_cloud() -> Tuple[str, int]:
    """
    Upload a point cloud to the database

    :return: Tuple containing a message and status code
    """
    try:
        # Parse JSON payload
        data = request.get_json()

        # Validate the data
        if not isinstance(data, list) or not all(len(point) == 3 for point in data):
            return "Invalid point cloud data", 400

        # Prepare the database connection
        connection = psycopg2.connect(**DB_SETTINGS)
        cursor = connection.cursor()

        # Fetch the latest scan_id, the scan id is how we know
        cursor.execute("SELECT nextval('scan_id_seq')")
        new_scan_id = cursor.fetchone()
        print(new_scan_id)


        # Format the data for bulk insertion
        tuples = [(new_scan_id, point['x'], point['y'], point['z']) for point in data]

        # Insert into the database
        query = "INSERT INTO point_cloud (scan_id, x, y, z) VALUES %s"
        execute_values(cursor, query, tuples)

        connection.commit()
        print(f"Inserted {len(tuples)} points with scan_id {new_scan_id}")

        return f"Inserted {len(tuples)} points with scan_id {new_scan_id}", 200
    except Exception as e:
        return f"Error inserting data into database: {e}", 500
    finally:
        if 'connection' in locals():
            cursor.close()
            connection.close()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)

