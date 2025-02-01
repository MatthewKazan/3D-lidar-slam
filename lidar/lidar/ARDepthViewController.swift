//
//  ARDepthViewController.swift
//  lidar
//
//  Created by matt kazan on 10/29/24.
//

import UIKit
import ARKit
import Starscream

struct Point: Codable {
    let x: Float
    let y: Float
    let z: Float
}

class ARDepthViewController: UIViewController, ARSessionDelegate {
    var arView: ARSCNView!
    var capturedPointCloud: [SIMD3<Float>] = []
    var isScanning = false // Track the scanning state
    var scanningTimer: Timer?
    var socket: WebSocket!


    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Initialize the ARSCNView
        arView = ARSCNView(frame: self.view.bounds)
        self.view.addSubview(arView)
        arView.session.delegate = self

        // Enable LiDAR depth data collection
        let configuration = ARWorldTrackingConfiguration()
        configuration.frameSemantics = .sceneDepth
        arView.session.run(configuration)
        
        var request = URLRequest(url: URL(string: "http://10.0.0.199:9090")!) //https://localhost:8080
        request.timeoutInterval = 5
        socket = WebSocket(request: request)
        socket.connect() // Make sure this is the correct WebSocket URL
        
    }

    func startScanning() {
        isScanning = true
        scanningTimer = Timer.scheduledTimer(withTimeInterval: 0.30, repeats: true) { _ in
            self.capturePointCloud()
        }
    }

    func stopScanning() {
        isScanning = false
        scanningTimer?.invalidate()
        scanningTimer = nil
    }

    func toggleScanning() {
        if isScanning {
            print("stop scanning")
            stopScanning()
        } else {
            print("start scanning")
            startScanning()
        }
    }

    func capturePointCloud() {
        if isScanning == false {
            return
        }
        guard let frame = arView.session.currentFrame,
              let depthData = frame.sceneDepth?.depthMap else {
            print("Depth data is unavailable.")
            return
        }
//        let pointCloud = savePointCloud(from: depthData, cameraIntrinsics: frame.camera.intrinsics, resolution: frame.camera.imageResolution)
////        print(frame.camera.intrinsics)
////        printDepthData(depthData)
//        uploadCSV()
        uploadPointCloud(from: depthData)
    }
    
    func uploadPointCloud(from depthData: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(depthData, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthData, .readOnly) }

        let width = Int(CVPixelBufferGetWidth(depthData))
        let height = Int(CVPixelBufferGetHeight(depthData))
        let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthData), to: UnsafeMutablePointer<Float32>.self)

        var pointData = Data()
        var validPointsCount = 0

        // Pack each valid point (x, y, depth) as 32-bit floats in little-endian order
        for y in 0..<height {
            for x in 0..<width {
                let depth = depthPointer[y * width + x]
                if depth > 0 {
                    var xVal = Float(x)
                    var yVal = Float(y)
                    var zVal = depth
                    // Append x, y, and z as binary data
                    withUnsafeBytes(of: &xVal) { pointData.append(contentsOf: $0) }
                    withUnsafeBytes(of: &yVal) { pointData.append(contentsOf: $0) }
                    withUnsafeBytes(of: &zVal) { pointData.append(contentsOf: $0) }
                    validPointsCount += 1
                }
            }
        }

        // Base64-encode the binary data for the JSON message
        let base64EncodedData = pointData.base64EncodedString()

        // Prepare a header with a timestamp and a frame id (adjust frame_id as needed)
        let currentTime = Date().addingTimeInterval(0.1)
        let timeInterval = currentTime.timeIntervalSince1970
        let secs = Int32(timeInterval)
        let nsecs = Int32((timeInterval - Double(secs)) * 1_000_000_000)
        let header: [String: Any] = [
            "stamp": [
                "secs": secs,
                "nsecs": nsecs
            ],
            "frame_id": "camera_link"  // adjust as appropriate
        ]

        // Build the sensor_msgs/PointCloud2 message payload
        let pointCloudMessage: [String: Any] = [
            "header": header,
            "height": 1,                   // unorganized point cloud
            "width": validPointsCount,     // number of valid points
            "fields": [
                ["name": "x", "offset": 0, "datatype": 7, "count": 1],
                ["name": "y", "offset": 4, "datatype": 7, "count": 1],
                ["name": "z", "offset": 8, "datatype": 7, "count": 1]
            ],
            "is_bigendian": false,
            "point_step": 12,              // 3 floats * 4 bytes each
            "row_step": 12 * validPointsCount,
            "data": base64EncodedData,
            "is_dense": true
        ]

        // Wrap in a rosbridge-style JSON message
        let jsonMessage: [String: Any] = [
            "op": "publish",
            "topic": "/input_pointcloud",
            "msg": pointCloudMessage
        ]

        // Serialize to JSON and send over the websocket
        do {
            let jsonData = try JSONSerialization.data(withJSONObject: jsonMessage, options: [])
            if let jsonString = String(data: jsonData, encoding: .utf8) {
                socket?.write(string: jsonString)
            }
            print(timeInterval)
        } catch {
            print("Failed to encode point cloud JSON: \(error)")
        }
    }
    
//    func uploadPointCloud(from depthData: CVPixelBuffer) {
//        CVPixelBufferLockBaseAddress(depthData, .readOnly)
//        defer { CVPixelBufferUnlockBaseAddress(depthData, .readOnly) }
//
//        let width = Int(CVPixelBufferGetWidth(depthData))
//        let height = Int(CVPixelBufferGetHeight(depthData))
//        let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthData), to: UnsafeMutablePointer<Float32>.self)
//
//        // Create the point cloud as raw tuples
//        var pointCloud: [Point] = []
//        for y in 0..<height {
//            for x in 0..<width {
//                let depth = depthPointer[y * width + x]
//                if depth > 0 {
//                    pointCloud.append(Point(x: Float(x), y: Float(y), z: depth))
//                }
//            }
//        }
//
//        // Prepare the JSON payload
//        do {
//            let jsonData = try JSONEncoder().encode(pointCloud)
//            
//            // Upload the JSON directly
//            var request = URLRequest(url: URL(string: "http://10.0.0.199:9090")!)
//            request.httpMethod = "POST"
//            request.setValue("application/json", forHTTPHeaderField: "Content-Type")
//            request.httpBody = jsonData
//
//            URLSession.shared.dataTask(with: request) { data, response, error in
//                if let error = error {
//                    print("Error uploading point cloud: \(error)")
//                    return
//                }
//                print("Point cloud uploaded successfully")
//            }.resume()
//        } catch {
//            print("Failed to encode point cloud: \(error)")
//        }
//    }
//    
}
