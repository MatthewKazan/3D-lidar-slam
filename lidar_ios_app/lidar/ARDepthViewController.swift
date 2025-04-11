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
    var x: Float
    var y: Float
    var z: Float
}

struct CameraIntrinsics: Codable {
    let lidarWidth: Float
    let lidarHeight: Float
    var refWidth: Float = 1920
    var refHeight: Float = 1440
    
    let scaleX: Float
    let scaleY: Float
    
    var fx: Float
    var fy: Float
    var cx: Float
    var cy: Float
    
    init(lidarWidth: Float, lidarHeight: Float, intrinsics: simd_float3x3) {
        self.lidarWidth = lidarWidth
        self.lidarHeight = lidarHeight
        self.scaleX = self.lidarWidth / self.refWidth
        self.scaleY = self.lidarHeight / self.refHeight
        
        let fx = intrinsics.columns.0.x
        let fy = intrinsics.columns.1.y
        let cx = intrinsics.columns.2.x
        let cy = intrinsics.columns.2.y
        self.fx = fx * self.scaleX
        self.fy = fy * self.scaleY
        self.cx = cx * self.scaleX
        self.cy = cy * self.scaleY
    }
}


/// **ARDepthViewController**
/// This class manages an **ARKit-based LiDAR depth capture** session, processes the depth data,
/// and transmits the **3D point cloud** via a **WebSocket connection** to a ROS2 system.
///
/// - Captures LiDAR depth data using ARKit.
/// - Filters and encodes the data as a **PointCloud2** message.
/// - Sends data via a **WebSocket connection**.
/// - Provides start/stop functionality for scanning.
/// - Handles WebSocket reconnections automatically.
class ARDepthViewController: UIViewController, ARSessionDelegate, WebSocketDelegate, ObservableObject {
    var arView: ARSCNView!
    var capturedPointCloud: [SIMD3<Float>] = []
    var isScanning = false
    var scanningTimer: DispatchSourceTimer?//Timer?
    var socket: WebSocket?
    var isConnected = false  // ✅ Track WebSocket connection status
    var selectedIP = UserDefaults.standard.string(forKey: "SavedIP") ?? "172.20.10.7"
    @Published var num_scans = 0
    @Published var availableAlgorithms: [String] = []
    var isLoading: Bool = false
    var cameraIntrinsics: CameraIntrinsics!

    override func viewDidLoad() {
        super.viewDidLoad()

        // Initialize AR View
        arView = ARSCNView(frame: self.view.bounds)
        self.view.addSubview(arView)
        arView.session.delegate = self

        self.setIPAddress(ip: self.selectedIP)
        self.sendGetAlgorithmsRequest()
    }

    // MARK: - **Scanning Control Methods**
    
    /// Starts LiDAR scanning and begins sending point cloud data.
    func startScanning() {
        if isScanning { return }
        self.setIPAddress(ip: self.selectedIP)
        // Enable LiDAR depth data collection
        let configuration = ARWorldTrackingConfiguration()
        configuration.frameSemantics = .sceneDepth
        // Delay to ensure ARKit fully resets
        self.arView.session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        
        num_scans = 0
        let sessionStartTime = CACurrentMediaTime()
        isScanning = true
//        scanningTimer = Timer.scheduledTimer(withTimeInterval: 0.25, repeats: true) { _ in
//            self.capturePointCloud(sessionStartTime: sessionStartTime)
//        }
        scanningTimer = DispatchSource.makeTimerSource(queue: DispatchQueue.global(qos: .userInitiated))
        scanningTimer?.schedule(deadline: .now(), repeating: 0.25)
        scanningTimer?.setEventHandler { [weak self] in
            guard let self = self else { return }
            self.capturePointCloud(sessionStartTime: sessionStartTime)
        }
        scanningTimer?.resume()
    }

    /// Stops LiDAR scanning and terminates point cloud transmission.
    func stopScanning() {
        if !isScanning { return }
        isScanning = false
        self.arView.session.pause()
        scanningTimer?.cancel()
        scanningTimer = nil
//        arView.session.pause()

    }

    /// Toggles scanning between **start** and **stop**.
    func toggleScanning() {
        isScanning ? stopScanning() : startScanning()
    }
    
    // MARK: - **LiDAR Point Cloud Capture**
      
    /// Captures the current **LiDAR depth map** from ARKit and processes it into a point cloud.
    func capturePointCloud(sessionStartTime: CFTimeInterval) {
        guard isScanning, let frame = self.arView.session.currentFrame, frame.timestamp >= sessionStartTime, let depthData = frame.sceneDepth?.depthMap else {
            print("Depth data is unavailable.")
            return
        }

        DispatchQueue.global(qos: .userInitiated).async {
            self.uploadPointCloud(from: depthData)
        }
    }
    
    /// Converts a **CVPixelBuffer depth map** into a **PointCloud2 format** and sends it via WebSocket.
    func uploadPointCloud(from depthData: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(depthData, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthData, .readOnly) }
        let currentTime = Date()
        let timeInterval = currentTime.timeIntervalSince1970
        
        let width = Int(CVPixelBufferGetWidth(depthData))
        let height = Int(CVPixelBufferGetHeight(depthData))
        let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthData), to: UnsafeMutablePointer<Float32>.self)
        self.cameraIntrinsics = CameraIntrinsics(lidarWidth: Float(width), lidarHeight: Float(height), intrinsics: self.arView.session.currentFrame!.camera.intrinsics)
        var pointData = Data()
        var validPointsCount = 0

        for y in 0..<height {
            for x in 0..<width {
                let depth = depthPointer[y * width + x]
                if depth > 0{// && Bool.random() {
                    let point = projecPixelTo3D(x: Float(x), y: Float(y), z: depth)
                    var xVal = Float(point.x)//Float(point.x)
                    var yVal = Float(point.y)//Float(point.y)
                    var zVal = Float(point.z)
//                    print(point)
                    withUnsafeBytes(of: &xVal) { pointData.append(contentsOf: $0) }
                    withUnsafeBytes(of: &yVal) { pointData.append(contentsOf: $0) }
                    withUnsafeBytes(of: &zVal) { pointData.append(contentsOf: $0) }
                    validPointsCount += 1
                }
            }
        }

        let base64EncodedData = pointData.base64EncodedString()

        let secs = Int32(timeInterval)
        let nsecs = Int32((timeInterval - Double(secs)) * 1_000_000_000)

        // Construct **ROS2 PointCloud2 message**
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
        DispatchQueue.main.async {
            self.num_scans += 1
        }
        print(num_scans)
        self.publishToTopic(msg: pointCloudMessage, topic: "/input_pointcloud")
        let newTime = Date()
        print(newTime.timeIntervalSince1970 - timeInterval)
    }
    
    // MARK: - **WebSocket Connection Handling**
        
    /// Establishes a WebSocket connection to the **ROS2 bridge server**.
    func setIPAddress(ip: String) {
        print("Setting new IP: \(ip)")
        self.selectedIP = ip
        self.socket?.disconnect()  // ✅ Ensure clean disconnect before reconnecting
        
        var request = URLRequest(url: URL(string: "ws://\(self.selectedIP):9090")!)
        request.timeoutInterval = 1
        // A lot of the following probably isn't necessary but the websocket has been finnicky so im not touching it
        // ✅ Force WebSocket to send packets immediately (disable Nagle’s Algorithm)
        request.setValue("Upgrade", forHTTPHeaderField: "Connection")
        request.setValue("Keep-Alive", forHTTPHeaderField: "Proxy-Connection")
        request.setValue("no-cache", forHTTPHeaderField: "Cache-Control")

        // ✅ Enable WebSocket compression (reduces data size)
        request.setValue("permessage-deflate", forHTTPHeaderField: "Sec-WebSocket-Extensions")

        // ✅ Prevent WiFi from putting the connection to sleep
        request.setValue("true", forHTTPHeaderField: "WebSocket-Stay-Awake")

        self.socket = WebSocket(request: request)
        self.socket?.delegate = self  // ✅ Ensure WebSocket delegate is set
        self.isConnected = false
        self.socket?.connect()
        self.socket?.request.setValue("8.8.8.8", forHTTPHeaderField: "DNS-Resolver")

    }

    // MARK: - **WebSocket Delegate Methods**
        
    /// Sends a request to **reset the ROS2 system** via WebSocket.
    func sendResetRequest() {
        self.setIPAddress(ip: self.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            self.publishToTopic(msg: [:], topic: "/reset")
        }
    }
    
    /// Sends a request to **save the current global map** in ROS2.
    func sendSaveRequest() {
        self.setIPAddress(ip: self.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            self.sendServiceRequest(service: "/save_global_map")
        }
    }
    func sendToggleSaveInputRequest() {
        self.setIPAddress(ip: self.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            self.sendServiceRequest(service: "/toggle_save_inputs")
        }
    }
    
    func sendGetAlgorithmsRequest() {
        self.isLoading = true
        self.setIPAddress(ip: self.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            self.sendServiceRequest(service: "/get_algorithms_list", type: "custom_interfaces/srv/GetAlgorithmsList")
        }
    }
    
    func changeAlgorithms(alg_str: String) {
        self.setIPAddress(ip: self.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            self.sendServiceRequest(service: "/set_algorithm", args: ["algorithm": alg_str], type: "custom_interfaces/srv/SetAlgorithm")
        }
    }

    /// Handles WebSocket connection events.
    func didReceive(event: Starscream.WebSocketEvent, client: any Starscream.WebSocketClient) {
        print(event, client)

        switch event {
        case .connected(_):
            isConnected = true
            print("✅ WebSocket Connected to \(self.selectedIP)")

        case .disconnected(let reason, let code):
            isConnected = false
            print("❌ WebSocket Disconnected: \(reason) (Code: \(code))")
        case .text(let string):
            print("string", string)
            parseReceivedString(msg: string)
        case .binary(let data):
            print("binary", data)
        case .error(let error):
            isConnected = false
            print("⚠️ WebSocket Error: \(error?.localizedDescription ?? "Unknown error")")

        default:
            print("unkown")
            break
        }
    }
    
    /// Sends a service request to a **ROS2 service** via WebSocket.
    func sendServiceRequest(service: String, args: [AnyHashable:Any] = [:], type: String = "std_srvs/srv/Trigger") {
        let message: [String: Any] = [
            "op": "call_service",
            "service": service,
//            "type": type,
            "id": service,
            "args": args
        ]

        do {
            let jsonData = try JSONSerialization.data(withJSONObject: message, options: [])
            if let jsonString = String(data: jsonData, encoding: .utf8) {
                socket?.write(string: jsonString)
                print("Sent \(service) request via WebSocket")
            }
        } catch {
            print("Failed to encode JSON: \(error)")
        }
    }
       
   /// Publishes a **ROS2 topic message** over the WebSocket.
    func publishToTopic(msg: Any, topic: String) {
        
        // Wrap in a rosbridge-style JSON message
        let jsonMessage: [String: Any] = [
            "op": "publish",
            "topic": topic,
            "compression": "cbor",
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
    
    func parseReceivedString(msg: String) {
        print(msg)
        if let data = msg.data(using: .utf8) {
            do {
                if let json = try JSONSerialization.jsonObject(with: data, options: []) as? [String: Any],
                   let id = json["id"] as? String,
                   let values = json["values"] as? [String: Any]
                {
                    switch id {
                    case "/get_algorithms_list":
                        handleGetAlgorithmList(values: values)
                    default:
                        break
                    }
                }
            } catch {
                print("Failed to parse JSON: \(error)")
            }
        }
    }
    func handleGetAlgorithmList(values: [String: Any]) {
        self.availableAlgorithms = values["algorithms"] as? [String] ?? []
        self.isLoading = false
        print(self.availableAlgorithms)
        
    }
    
    func projecPixelTo3D(x: Float, y: Float, z: Float) -> Point {
        let xn = (x - self.cameraIntrinsics.cx) * z / self.cameraIntrinsics.fx
        let yn = (y - self.cameraIntrinsics.cy) * z / self.cameraIntrinsics.fy
        return Point(x: xn, y: yn, z: z)
    }
    
   
    
}
