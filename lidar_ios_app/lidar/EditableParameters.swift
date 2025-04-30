//
//  EditableParameters.swift
//  lidar
//
//  Created by matt kazan on 4/29/25.
//

import Foundation

class ParameterEditViewModel: ObservableObject {
    @Published var editableParameters: [EditableParameter]

    private var originalValues: [String: String]

    init(parameters: [Parameter]) {
        let temp = parameters.map { EditableParameter(from: $0) }
        self.originalValues = Dictionary(uniqueKeysWithValues:
            temp.map { ($0.name, $0.rawValue) }
        )
        editableParameters = temp
    }

    func getChangedParams() -> [Parameter] {
        editableParameters.compactMap {
            guard let original = originalValues[$0.name], original != $0.rawValue else { return nil }
            return $0.toParameter()
        }
    }
}

struct EditableParameter: Identifiable {
    let id = UUID()
    let name: String
    let type: Int
    let value_key: String
    var rawValue: String

    init(from param: Parameter) {
        self.name = param.name
        self.type = param.type
        self.value_key = param.value_key
        self.rawValue = String(describing: param.value)
    }

    func toParameter() -> Parameter? {
        switch type {
        case 1: return Parameter(name: name, value: rawValue.lowercased() == "true")
        case 2: return Parameter(name: name, value: Int(rawValue) ?? 0)
        case 3: return Parameter(name: name, value: Double(rawValue) ?? 0.0)
        case 4: return Parameter(name: name, value: rawValue)
        default:
            return nil
        }
    }
}
