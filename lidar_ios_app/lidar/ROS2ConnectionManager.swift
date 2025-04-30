//
//  ROS2ConnectionManager.swift
//  lidar
//
//  Created by matt kazan on 4/29/25.
//

import Starscream
import Foundation

class ROS2ConnectionManager: WebSocketDelegate, ROS2ConnectionManaging {
    
    private var socket: WebSocket!
    private var heartbeatTimer: Timer?
    private(set) var isConnected = false
    weak var state: ROS2AppState!

    init(ip: String, state: ROS2AppState) {
        var request = URLRequest(url: URL(string: "ws://\(ip):9090")!)
        request.timeoutInterval = 5 // Short timeout for initial connect
        socket = WebSocket(request: request)
        socket.delegate = self
        self.state = state
    }

    func connect() {
        socket.connect()
    }

    func disconnect() {
        stopHeartbeat()
        socket.disconnect()
    }

    /// WebSocket Delegate
    func didReceive(event: WebSocketEvent, client: WebSocketClient) {
        switch event {
        case .connected(_):
            isConnected = true
            print("✅ Connected")
            startHeartbeat()
            self.state.triggerRefresh()
            
        case .text(let string):
//            print("string", string)
            parseReceivedString(msg: string)
            
        case .disconnected(let reason, let code):
            isConnected = false
            print("❌ Disconnected: \(reason) (Code: \(code))")
//            stopHeartbeat()

        case .error(let error):
            isConnected = false
            print("⚠️ Error: \(error?.localizedDescription ?? "Unknown error")")
//            stopHeartbeat()

        default:
//            print(event)
            break
        }
    }

    /// Start sending heartbeats (pings)
    private func startHeartbeat() {
        heartbeatTimer?.invalidate()
        heartbeatTimer = Timer.scheduledTimer(withTimeInterval: 10, repeats: true) { [weak self] _ in
            self?.heartbeatAction()
        }
    }

    /// Stop heartbeats
    private func stopHeartbeat() {
        heartbeatTimer?.invalidate()
        heartbeatTimer = nil
    }

    private func heartbeatAction() {
            if isConnected {
                socket.write(ping: Data())
//                print("📡 Sent heartbeat ping")
            } else {
                print("🔄 Attempting reconnect...")
                recreateSocketAndConnect()
            }
        }
    private func recreateSocketAndConnect() {
        var request = URLRequest(url: socket.request.url!) // Reuse the same URL
        request.timeoutInterval = 5
        let newSocket = WebSocket(request: request)
        newSocket.delegate = self
        socket = newSocket
        socket.connect()
    }
    
    /// Establishes a WebSocket connection to the **ROS2 bridge server**.
    func setIPAddress(ip: String) {
        self.disconnect()  // ✅ Ensure clean disconnect before reconnecting
        
        var request = URLRequest(url: URL(string: "ws://\(ip):9090")!)
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
        self.socket?.request.setValue("8.8.8.8", forHTTPHeaderField: "DNS-Resolver")
        self.connect()
    }
    
    // helper to serialize + send
    func send(json: [String:Any]) {
        do {
            let data = try JSONSerialization.data(withJSONObject: json, options: [])
            if let s = String(data: data, encoding: .utf8) {
                socket?.write(string: s)
//                print(s)
            }
        } catch {
            print("JSON error:", error)
        }
    }
    
    func parseReceivedString(msg: String) {
//        print(msg)
        if let data = msg.data(using: .utf8) {
            do {
                if let json = try JSONSerialization.jsonObject(with: data, options: []) as? [String: Any],
                   let id = json["service"] as? String,
                   let values = json["values"] as? [String: Any]
                {
                    self.state.updateFromServer(id: id, values: values)
                }
            } catch {
                print("Failed to parse JSON: \(error)")
            }
        }
    }
    
    /// Sends a service request to a **ROS2 service** via WebSocket.
    func sendServiceRequest(service: String, args: [AnyHashable:Any] = [:], type: String = "std_srvs/srv/Trigger") {
        let message: [String: Any] = [
            "op": "call_service",
            "service": service,
            "id": service,
            "args": args
        ]

        self.send(json: message)
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
        self.send(json: jsonMessage)
    }
    
    func updateParameterValue(node: String, param: Parameter) {
        let message: [String: Any] = [
            "op":   "call_service",
            "service": node + "/" + "set_parameters",
            "args": [
                "parameters": [
                    [
                        "name": param.name,
                        "value": [
                            "type": param.type,
                            param.value_key: param.value
                        ]
                    ]
                ]
            ],
        ]
        self.send(json: message)
    }
    
    func getParametersList(node: String) {
        let service = node + "/list_parameters"
        self.sendServiceRequest(service: service)
    }
    
    func getParamTypesValues(node: String, params: [String]) {
        let service = node + "/get_parameters"

        let args = [
            "names": params
        ]
        self.sendServiceRequest(service: service, args: args)

        
    }
    
    func requestRefresh() {
        self.state.isLoading = true
    }
    
}
