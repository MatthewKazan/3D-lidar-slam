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

import UIKit
import ARKit
import Starscream  // ✅ Ensure Starscream is imported for WebSocket

class ARDepthViewController: UIViewController, ARSessionDelegate, WebSocketDelegate {
    var arView: ARSCNView!
    var capturedPointCloud: [SIMD3<Float>] = []
    var isScanning = false
    var scanningTimer: Timer?
    var socket: WebSocket?
    var isConnected = false  // ✅ Track WebSocket connection status
    var selectedIP = UserDefaults.standard.string(forKey: "SavedIP") ?? ""
    var num_scans = 0

    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Initialize AR View
        arView = ARSCNView(frame: self.view.bounds)
        self.view.addSubview(arView)
        arView.session.delegate = self


        
        if selectedIP.isEmpty { return }
        self.setIPAddress(ip: selectedIP)
    }

    // ✅ Start scanning and connect WebSocket only if not already connected
    func startScanning() {
        if isScanning { return }
        // Enable LiDAR depth data collection
        let configuration = ARWorldTrackingConfiguration()
        configuration.frameSemantics = .sceneDepth
        arView.session.run(configuration)
        num_scans = 0

        isScanning = true
        scanningTimer = Timer.scheduledTimer(withTimeInterval: 0.40, repeats: true) { _ in
            self.capturePointCloud()
        }
        
        if socket == nil || !isConnected {
            print("Connecting WebSocket for scanning...")
            socket?.connect()
        }
    }

    // ✅ Stop scanning and disconnect WebSocket safely
    func stopScanning() {
        if !isScanning { return }
        isScanning = false
        scanningTimer?.invalidate()
        scanningTimer = nil
//        arView.session.pause()

        if isConnected {
            print("Disconnecting WebSocket after scanning...")
            socket?.disconnect()
        }
    }

    // ✅ Toggle scanning state
    func toggleScanning() {
        isScanning ? stopScanning() : startScanning()
    }

    func capturePointCloud() {
        guard isScanning, let frame = arView.session.currentFrame, let depthData = frame.sceneDepth?.depthMap else {
            print("Depth data is unavailable.")
            return
        }
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

        for y in 0..<height {
            for x in 0..<width {
                let depth = depthPointer[y * width + x]
                if depth > 0 && Bool.random() {
                    var xVal = Float(x)
                    var yVal = Float(y)
                    var zVal = depth
                    withUnsafeBytes(of: &xVal) { pointData.append(contentsOf: $0) }
                    withUnsafeBytes(of: &yVal) { pointData.append(contentsOf: $0) }
                    withUnsafeBytes(of: &zVal) { pointData.append(contentsOf: $0) }
                    validPointsCount += 1
                }
            }
        }

        let base64EncodedData = pointData.base64EncodedString()
        let currentTime = Date().addingTimeInterval(0.1)
        let timeInterval = currentTime.timeIntervalSince1970
        let secs = Int32(timeInterval)
        let nsecs = Int32((timeInterval - Double(secs)) * 1_000_000_000)

        let header: [String: Any] = [
            "stamp": ["secs": secs, "nsecs": nsecs],
            "frame_id": "camera_link"
        ]

        let pointCloudMessage: [String: Any] = [
            "header": header,
            "height": 1,
            "width": validPointsCount,
            "fields": [
                ["name": "x", "offset": 0, "datatype": 7, "count": 1],
                ["name": "y", "offset": 4, "datatype": 7, "count": 1],
                ["name": "z", "offset": 8, "datatype": 7, "count": 1]
            ],
            "is_bigendian": false,
            "point_step": 12,
            "row_step": 12 * validPointsCount,
            "data": base64EncodedData,
            "is_dense": true
        ]
        num_scans+=1
        print(num_scans)
        self.publishToTopic(msg: pointCloudMessage, topic: "/input_pointcloud")
    }
    
    // ✅ Handle WebSocket Reconnection and Disconnects
    func setIPAddress(ip: String) {
        print("Setting new IP: \(ip)")
        selectedIP = ip
        socket?.disconnect()  // ✅ Ensure clean disconnect before reconnecting
        
        var request = URLRequest(url: URL(string: "ws://\(selectedIP):9090")!)
        request.timeoutInterval = 5
        socket = WebSocket(request: request)
        socket?.delegate = self  // ✅ Ensure WebSocket delegate is set
        self.isConnected = false
    }

    // ✅ Reset the WebSocket and send a reset message
    func sendResetRequest() {

        // Attempt to connect, then send reset once connected
        socket?.connect()
        
        // Use DispatchQueue to delay sending reset until connection is established
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            if self.isConnected {
                print("✅ WebSocket connected, now sending reset request...")
                self.publishToTopic(msg: ["data": ""], topic: "/reset")
            } else {
                print("❌ WebSocket failed to connect. Reset request not sent.")
            }
        }
    
        socket?.disconnect()
    }
    
    func sendSaveRequest() {

        // Attempt to connect, then send reset once connected
        socket?.connect()
        
        // Use DispatchQueue to delay sending reset until connection is established
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            if self.isConnected {
                print("✅ WebSocket connected, now sending save request...")
                self.sendServiceRequest(service: "/save_global_map")
            } else {
                print("❌ WebSocket failed to connect. Save request not sent.")
            }
        }
    
        socket?.disconnect()
    }

    // ✅ WebSocket Delegate Methods
    func didReceive(event: Starscream.WebSocketEvent, client: any Starscream.WebSocketClient) {
        switch event {
        case .connected(_):
            isConnected = true
            print("✅ WebSocket Connected to \(selectedIP)")

        case .disconnected(let reason, let code):
            isConnected = false
            print("❌ WebSocket Disconnected: \(reason) (Code: \(code))")

        case .error(let error):
            isConnected = false
            print("⚠️ WebSocket Error: \(error?.localizedDescription ?? "Unknown error")")

        default:
            break
        }
    }
    
    func sendServiceRequest(service: String) {
        let message: [String: Any] = [
            "op": "call_service",
            "service": service,
            "args": [:]
        ]

        do {
            let jsonData = try JSONSerialization.data(withJSONObject: message, options: [])
            if let jsonString = String(data: jsonData, encoding: .utf8) {
                socket?.write(string: jsonString)
                print("Sent save request via WebSocket")
            }
        } catch {
            print("Failed to encode JSON: \(error)")
        }
    }

    
    func publishToTopic(msg: Any, topic: String) {
        
        // Wrap in a rosbridge-style JSON message
        let jsonMessage: [String: Any] = [
            "op": "publish",
            "topic": topic,
            "msg": msg
        ]

        // Serialize to JSON and send over the websocket
        do {
            let jsonData = try JSONSerialization.data(withJSONObject: jsonMessage, options: [])
            if let jsonString = String(data: jsonData, encoding: .utf8) {
                socket?.write(string: jsonString)
            }
        } catch {
            print("Failed to publish to topic \(error)")
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
