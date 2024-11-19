import os
import pandas as pd
import requests

def send_data():
    # Define the URL for the HTTP server
    url = "http://localhost:8080"  # Make sure this matches your server address and port

    # Load the CSV file
    current_dir = os.getcwd()
    file_path = os.path.join(current_dir, "uploaded_pointcloud.csv")

    # Open the file in binary mode for uploading
    with open(file_path, "rb") as file:
        # Send a POST request with the file
        files = {'file': file}
        response = requests.post(url, files=files)

    # Print the server's response
    print("Response from server:", response.text)

def clear_database():
    # Define the URL for the HTTP server
    url = "http://localhost:8080/clear_db"  # Make sure this matches your server address and port

    # Send a POST request to clear the database
    response = requests.post(url)

    # Print the server's response
    print("Response from server:", response.text)
# Run the client
# send_data()
clear_database()
