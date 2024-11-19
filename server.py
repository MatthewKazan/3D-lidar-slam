# from http.server import HTTPServer, SimpleHTTPRequestHandler
# import psycopg2
# import pandas as pd
# from psycopg2.extras import execute_values
# from io import BytesIO
#
# Database connection settings
DB_SETTINGS = {
    'host': "localhost",
    'database': "postgres",
    'user': "newuser",
    'password': "password"
}
#
# class UploadHandler(SimpleHTTPRequestHandler):
#     def clear_database(self):
#         """Function to quickly delete all entries in the point_cloud table."""
#         try:
#             connection = psycopg2.connect(**DB_SETTINGS)
#             cursor = connection.cursor()
#             cursor.execute("TRUNCATE TABLE point_cloud;")  # Faster than DELETE for clearing all records
#             connection.commit()
#             cursor.close()
#             connection.close()
#             print("All entries deleted from the database.")
#
#             # Respond to the client
#             self.send_response(200)
#             self.end_headers()
#             self.wfile.write(b"All entries deleted from the database.")
#         except Exception as e:
#             print("Error clearing database:", e)
#             self.send_response(500)
#             self.end_headers()
#             self.wfile.write(b"Error clearing database.")
#
#     def do_POST(self):
#         if self.path == "/clear_db":
#             self.clear_database()
#             return
#
#         content_length = int(self.headers['Content-Length'])
#         boundary = self.headers['Content-Type'].split("boundary=")[-1]
#         boundary = boundary.encode()  # Ensure boundary is in bytes
#
#         # Read the raw data from the request
#         raw_data = self.rfile.read(content_length)
#
#         # Split the data using the boundary
#         parts = raw_data.split(boundary)
#
#         # Find the part containing the file data
#         for part in parts:
#             if b"Content-Disposition" in part and b"filename=" in part:
#                 # The file content starts after two CRLFs following headers
#                 file_data = part.split(b"\r\n\r\n", 1)[1].rsplit(b"\r\n", 1)[0]
#
#                 # Convert file data to DataFrame
#                 file_io = BytesIO(file_data)
#                 df = pd.read_csv(file_io, skiprows=1, names=['x', 'y', 'z'])
#
#                 # Insert data into the PostgreSQL database with incremented scan_id
#                 try:
#                     connection = psycopg2.connect(**DB_SETTINGS)
#                     cursor = connection.cursor()
#
#                     # Fetch the latest scan_id from the table and increment
#                     cursor.execute("SELECT COALESCE(MAX(scan_id), 0) FROM point_cloud;")
#                     last_scan_id = cursor.fetchone()[0]
#                     new_scan_id = last_scan_id + 1
#
#                     # Prepare data for bulk insert with the new scan_id
#                     tuples = [(new_scan_id, row['x'], row['y'], row['z']) for _, row in df.iterrows()]
#
#                     # Use execute_values for a bulk insert
#                     query = "INSERT INTO point_cloud (scan_id, x, y, z) VALUES %s"
#                     execute_values(cursor, query, tuples)
#
#                     connection.commit()
#                     cursor.close()
#                     connection.close()
#                     print(f"File uploaded and data inserted into database with scan_id {new_scan_id}.")
#
#                     # Respond to the client
#                     self.send_response(200)
#                     self.end_headers()
#                     self.wfile.write(b"File uploaded and data inserted into database successfully.")
#                 except Exception as e:
#                     print("Error inserting data into database:", e)
#                     self.send_response(500)
#                     self.end_headers()
#                     self.wfile.write(b"Error inserting data into database.")
#                 return
#
#         # If no file was found in the request, send a bad request response
#         self.send_response(400)
#         self.end_headers()
#         self.wfile.write(b"Bad request")
#
#     def upload_point_cloud():
#         try:
#             # Parse JSON payload
#             data = request.get_json()
#
#             # Validate the data
#             if not isinstance(data, list) or not all(len(point) == 3 for point in data):
#                 return "Invalid point cloud data", 400
#
#             # Prepare the database connection
#             connection = psycopg2.connect(**DB_SETTINGS)
#             cursor = connection.cursor()
#
#             # Fetch the latest scan_id
#             cursor.execute("SELECT COALESCE(MAX(scan_id), 0) FROM point_cloud;")
#             last_scan_id = cursor.fetchone()[0]
#             new_scan_id = last_scan_id + 1
#
#             # Format the data for bulk insertion
#             tuples = [(new_scan_id, point[0], point[1], point[2]) for point in data]
#
#             # Insert into the database
#             query = "INSERT INTO point_cloud (scan_id, x, y, z) VALUES %s"
#             execute_values(cursor, query, tuples)
#
#             connection.commit()
#             return f"Inserted {len(tuples)} points with scan_id {new_scan_id}", 200
#         except Exception as e:
#             return f"Error inserting data into database: {e}", 500
#         finally:
#             if 'connection' in locals():
#                 cursor.close()
#                 connection.close()
#
#
#
# server_address = ('', 8080)
# httpd = HTTPServer(server_address, UploadHandler)
# print("Serving on port 8080")
# httpd.serve_forever()


from flask import Flask, request, jsonify
import psycopg2
from psycopg2.extras import execute_values

app = Flask(__name__)

@app.route("/clear_db", methods=["POST"])
def clear_database():
    try:
        connection = psycopg2.connect(**DB_SETTINGS)
        cursor = connection.cursor()
        cursor.execute("TRUNCATE TABLE point_cloud;")
        connection.commit()
        return "All entries deleted from the database", 200
    except Exception as e:
        return f"Error clearing database: {e}", 500
    finally:
        if 'connection' in locals():
            cursor.close()
            connection.close()

@app.route("/", methods=["POST"])
def upload_point_cloud():
    try:
        # Parse JSON payload
        data = request.get_json()

        # Validate the data
        if not isinstance(data, list) or not all(len(point) == 3 for point in data):
            return "Invalid point cloud data", 400

        # Prepare the database connection
        connection = psycopg2.connect(**DB_SETTINGS)
        cursor = connection.cursor()

        # Fetch the latest scan_id
        cursor.execute("SELECT COALESCE(MAX(scan_id), 0) FROM point_cloud;")
        last_scan_id = cursor.fetchone()[0]
        new_scan_id = last_scan_id + 1

        # Format the data for bulk insertion
        tuples = [(new_scan_id, point['x'], point['y'], point['z']) for point in data]

        # Insert into the database
        query = "INSERT INTO point_cloud (scan_id, x, y, z) VALUES %s"
        execute_values(cursor, query, tuples)

        connection.commit()
        return f"Inserted {len(tuples)} points with scan_id {new_scan_id}", 200
    except Exception as e:
        print(e.with_traceback())
        return f"Error inserting data into database: {e}", 500
    finally:
        if 'connection' in locals():
            cursor.close()
            connection.close()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)

