//
//  ROS2AppState.swift
//  lidar
//
//  Created by matt kazan on 4/29/25.
//

import Foundation

class Parameter {
    let name: String
    let type: Int
    let value_key: String
    var value: Any

    static let typeMap: [Int: (String, Any.Type)] = [
        1: ("bool_value", Bool.self),
        2: ("integer_value", Int.self),
        3: ("double_value", Double.self),
        4: ("string_value", String.self),
        5: ("byte_array_value", [UInt8].self),
        6: ("bool_array_value", [Bool].self),
        7: ("integer_array_value", [Int].self),
        8: ("double_array_value", [Double].self),
        9: ("string_array_value", [String].self),
    ]

    // MARK: Init from full value (user input)
    init?(name: String, value: Any) {
        self.name = name

        // Infer type and key from value type
        if let (typeCode, key) = Parameter.inferTypeAndKey(from: value) {
            self.type = typeCode
            self.value_key = key
            self.value = value
        } else {
            print("Unknown value type: \(value)")
            return nil
        }
        if self.name == "algorithm_type" || self.name == "descriptor_type" {
            self.value = (self.value as! String).uppercased()
        }
    }

    // MARK: Init from ROS2-style dictionary
    init?(name: String, type: Int, possible_values: [String: Any]) {
        self.name = name
        self.type = type

        guard let (key, _) = Parameter.typeMap[type],
              let val = possible_values[key] else {
            print("Unknown type or missing value for type \(type)")
            return nil
        }

        self.value_key = key
        self.value = val
        if self.name == "algorithm_type" || self.name == "descriptor_type" {
            self.value = (self.value as! String).uppercased()
        }
    }

    // MARK: Helper to infer type from raw Swift value
    private static func inferTypeAndKey(from value: Any) -> (Int, String)? {
        switch value {
        case is Bool:        return (1, "bool_value")
        case is Int:         return (2, "integer_value")
        case is Double:      return (3, "double_value")
        case is String:      return (4, "string_value")
        case is [UInt8]:     return (5, "byte_array_value")
        case is [Bool]:      return (6, "bool_array_value")
        case is [Int]:       return (7, "integer_array_value")
        case is [Double]:    return (8, "double_array_value")
        case is [String]:    return (9, "string_array_value")
        default:             return nil
        }
    }

    // Optional: serialize for sending over rosbridge
    func toROSMessageDict() -> [String: Any] {
        return [
            "name": name,
            "value": [
                "type": type,
                value_key: value
            ]
        ]
    }
}
class ROS2AppState : ObservableObject {
    weak var connectionManager: ROS2ConnectionManaging?
    @Published var availableAlgorithms: [String] = []
    @Published var availableDescriptors: [String] = []
    @Published var isLoading: Bool = false
    @Published var numScans = 0
    @Published var isSavingInputs = false
    @Published var selectedIP = UserDefaults.standard.string(forKey: "SavedIP") ?? "172.20.10.7"
    @Published var cur_algorithm: String = ""
    @Published var cur_descriptor: String = ""


    @Published var parameters: [Parameter] = []
    var params: [String] = []

    
    func triggerRefresh() {
        self.isLoading = true
        self.getParametersList()
        self.getAlgorithmsList()
    }
    
    func getAlgorithmsList() {
        self.isLoading = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) {
            self.connectionManager?.sendServiceRequest(service: "/get_algorithms_list", args: [:], type: "custom_interfaces/srv/GetAlgorithmsList")
        }
    }
    
    func getParametersList() {
        self.isLoading = true
        self.connectionManager?.getParametersList(node: "/slam_processor")
    }
    
    func getAllParamTypesValues() {
        self.isLoading = true
        self.connectionManager?.getParamTypesValues(node: "/slam_processor", params: self.params)

    }
    
    
    func updateFromServer(id: String, values: [String: Any]) {
        // I know this is bad design but the ios app isn't hugely important and i can't spend more time on this
        switch id {
        case "/get_algorithms_list":
            self.handleGetAlgorithmList(values: values)
        case "/slam_processor/list_parameters":
            let result = values["result"] as! [String: [String]]
            self.params = result["names"]!
            self.getAllParamTypesValues()
        case "/slam_processor/get_parameters":
            self.handleGetParameters(json: values)
        case "/slam_processor/set_parameters":
            self.triggerRefresh()
        case "/pointclouds_subscriber/set_parameters":
            self.triggerRefresh()
        default:
            break
        }
        self.isLoading = false
    }
    
    func handleGetAlgorithmList(values: [String: Any]) {
        self.availableAlgorithms = values["algorithms"] as? [String] ?? []
        self.availableDescriptors = values["descriptors"] as? [String] ?? []

        self.isLoading = false

    }
    
    func handleGetParameters(json: [String: Any]) {
        guard let valueList = json["values"] as? [[String: Any]] else { return }

        var updatedParameters: [Parameter] = []

        for (index, paramValue) in valueList.enumerated() {
            guard index < self.params.count else { continue }

            let name = self.params[index]
            guard let type = paramValue["type"] as? Int,
                  let param = Parameter(name: name, type: type, possible_values: paramValue)
            else {
                continue
            }
            
            updatedParameters.append(param)

            DispatchQueue.main.async {
                self.saveValue(name: name, param: param)
            }
        }

        // Remove duplicates by name (last occurrence wins)
        let unique = Dictionary(grouping: updatedParameters, by: \.name)
            .compactMapValues { $0.last }  // keep last version per name
            .values

        self.parameters = Array(unique)

    }
    
    func saveValue(name: String, param: Parameter) {
        switch name {
        case "is_saving_inputs":
            self.isSavingInputs = param.value as! Bool
        case "algorithm_type":
            self.cur_algorithm = (param.value as! String).uppercased()
        case "descriptor_type":
            self.cur_descriptor = (param.value as! String).uppercased()
        default:
            break
        }
    }
    
    func toggleIsSavingInputs() {
        let param = Parameter(name: "is_saving_inputs", value: !self.isSavingInputs)!
        self.updateParameter(param: param)

    }
    
    func updateAlgorithm(alg_str: String) {
        let param = Parameter(name: "algorithm_type", value: alg_str)!
        self.updateParameter(param: param)

    }
    
    func updateDescriptor(desc_str: String) {
        let param = Parameter(name: "descriptor_type", value: desc_str)!
        self.updateParameter(param: param)

    }
    
    func updateParameter(param: Parameter) {
        // Find the current value from the server's parameter list
        if let existing = self.parameters.first(where: { $0.name == param.name }) {
            // Check for type and value equality
            if let serverValue = existing.value as? NSObject,
               let newValue = param.value as? NSObject,
               serverValue == newValue {
                // Same value → no need to update
                return
            }
        }

        // Either not found, or different → send update
        self.connectionManager?.updateParameterValue(node: "/slam_processor", param: param)
    }

    
}
