//
//  ARDepthViewController.swift
//  lidar
//
//  Created by matt kazan on 10/29/24.
//

import UIKit
import ARKit
import Starscream
import SwiftUICore

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
class ARDepthViewController: UIViewController, ARSessionDelegate, ObservableObject {
    var arView: ARSCNView!
    var capturedPointCloud: [SIMD3<Float>] = []
    var isScanning = false
    var scanningTimer: DispatchSourceTimer?//Timer?
    var cameraIntrinsics: CameraIntrinsics!
    var connectionManager: ROS2ConnectionManager?
    @ObservedObject var state = ROS2AppState()

    override func viewDidLoad() {
        super.viewDidLoad()

        // Initialize AR View
        arView = ARSCNView(frame: self.view.bounds)
        self.view.addSubview(arView)
        arView.session.delegate = self

        self.setIPAddress(ip: self.state.selectedIP)
        self.connectionManager = ROS2ConnectionManager(ip: self.state.selectedIP, state: self.state)
        self.state.connectionManager = self.connectionManager
        self.connectionManager?.connect()
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            self.state.triggerRefresh()
        }
    }

    // MARK: - **Scanning Control Methods**
    
    /// Starts LiDAR scanning and begins sending point cloud data.
    func startScanning() {
        if isScanning { return }
//        self.setIPAddress(ip: self.state.selectedIP)
        // Enable LiDAR depth data collection
        let configuration = ARWorldTrackingConfiguration()
        configuration.frameSemantics = .sceneDepth
        // Delay to ensure ARKit fully resets
        self.arView.session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        
        self.state.numScans = 0
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
            self.state.numScans += 1
        }
        self.connectionManager?.publishToTopic(msg: pointCloudMessage, topic: "/input_pointcloud")
        let newTime = Date()
        print(newTime.timeIntervalSince1970 - timeInterval)
    }
    
    // MARK: - **WebSocket Connection Handling**
        
    /// Establishes a WebSocket connection to the **ROS2 bridge server**.
    func setIPAddress(ip: String) {
        print("Setting new IP: \(ip)")
        DispatchQueue.main.async {
            self.state.selectedIP = ip
        }
        self.connectionManager?.setIPAddress(ip: ip)
        self.state.connectionManager = self.connectionManager

    }

    // MARK: - **WebSocket Delegate Methods**
        
    /// Sends a request to **reset the ROS2 system** via WebSocket.
    func sendResetRequest() {
//        self.setIPAddress(ip: self.state.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            self.connectionManager?.publishToTopic(msg: [:], topic: "/reset")
        }
    }
    
    /// Sends a request to **save the current global map** in ROS2.
    func sendSaveRequest() {
//        self.setIPAddress(ip: self.state.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            self.connectionManager?.sendServiceRequest(service: "/save_global_map")
        }
    }
    func sendToggleSaveInputRequest() {
//        self.setIPAddress(ip: self.state.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            self.state.toggleIsSavingInputs()
        }
    }
    
    func sendGetAlgorithmsRequest() {
//        self.setIPAddress(ip: self.state.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            self.state.getAlgorithmsList()

        }
    }
    
    func changeAlgorithms(alg_str: String) {
//        self.setIPAddress(ip: self.state.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            self.state.updateAlgorithm(alg_str: alg_str)
        }
    }
    func changeDescriptor(desc_str: String) {
//        self.setIPAddress(ip: self.state.selectedIP)
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            self.state.updateDescriptor(desc_str: desc_str)
        }
    }
    
    func sendSetParameter(param: Parameter) {
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            self.state.updateParameter(param: param)
        }
    }
    
   
    func projecPixelTo3D(x: Float, y: Float, z: Float) -> Point {
        let xn = (x - self.cameraIntrinsics.cx) * z / self.cameraIntrinsics.fx
        let yn = (y - self.cameraIntrinsics.cy) * z / self.cameraIntrinsics.fy
        return Point(x: xn, y: yn, z: z)
    }
    

}
