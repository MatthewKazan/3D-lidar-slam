import SwiftUI

@main
struct MyARApp: App {
    @StateObject private var arViewController = ARDepthViewController()
    @StateObject private var state: ROS2AppState

    init() {
        // Assign the shared state from the controller here
        let controller = ARDepthViewController()
        _arViewController = StateObject(wrappedValue: controller)
        _state = StateObject(wrappedValue: controller.state)
    }

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(arViewController)
                .environmentObject(state)
        }
    }
}

struct ContentView: View {
    @State private var isSidebarOpen = false
    @State private var selectedAlg = ""
    @State private var selectedDesc = ""
    @State private var isScanning = false
    @State private var isShowingIPMenu = false
    @EnvironmentObject var viewController: ARDepthViewController
    @EnvironmentObject var state: ROS2AppState


    @State private var selectedIP = UserDefaults.standard.string(forKey: "SavedIP") ?? ""
    
    var body: some View {
        ZStack(alignment: .leading) {
            ARViewContainer()
                .edgesIgnoringSafeArea(.all)
                .onTapGesture {
                    if isSidebarOpen {
                        withAnimation {
                            isSidebarOpen.toggle()
                        }
                    }
               }

            VStack {
                HStack {
                    Button(action: {
                        withAnimation {
                            isSidebarOpen.toggle()
                        }
                    }) {
                        Image(systemName: "line.horizontal.3")
                            .imageScale(.large)
                            .padding()
                    }
                    Spacer()
                }
                .padding(.top, 40)
                
                Spacer()
                
                Button(action: {
                    isScanning.toggle()
                    viewController.toggleScanning()
//                    state.triggerRefresh()
                    
                }) {
                    Text(isScanning ? "Stop Scanning" : "Start Scanning")
                        .padding()
                        .background(Color.blue)
                        .foregroundColor(.white)
                        .cornerRadius(10)
                }
                .padding(.bottom, 10)
                Text("Num Scans: \(state.numScans)")
                    .padding()
//                    .background(Color.blue)
                    .foregroundColor(.white)
                    .cornerRadius(10)
            }
            .navigationViewStyle(StackNavigationViewStyle())
            if isShowingIPMenu {
                IPMenu(isShowing: $isShowingIPMenu, selectedIP: $selectedIP)

            }
            if isSidebarOpen {
                SidebarView(isSidebarOpen: $isSidebarOpen,
                            selectedAlg: $selectedAlg,
                            selectedDesc: $selectedDesc,
                            isShowingIPMenu: $isShowingIPMenu,
                            )
                    .frame(width: 250)
                    .transition(.move(edge: .leading))
                    .zIndex(1)
                    .environmentObject(viewController)
                    .environmentObject(state)
            }
            if isShowingIPMenu {
                IPMenu(isShowing: $isShowingIPMenu, selectedIP: $selectedIP)

            }
        }
        .edgesIgnoringSafeArea(.all)
        
        
    }
}

struct SidebarView: View {
    @Binding var isSidebarOpen: Bool
    @Binding var selectedAlg: String
    @Binding var selectedDesc: String
    @Binding var isShowingIPMenu: Bool
    @State private var isShowingParamEditor = false

    @EnvironmentObject var viewController: ARDepthViewController
    @EnvironmentObject var state: ROS2AppState


        
    var body: some View {
        VStack(alignment: .leading) {
            Button(action: {
                withAnimation {
                    isSidebarOpen.toggle()
                }
            }) {
                HStack {
                    Image(systemName: "xmark")
                    Text("Close")
                }
                .padding()
            }
            .buttonStyle(PlainButtonStyle())
            .padding(.top, 40) // Moves the close button down
            
            Divider()
            
            Button("Set IP") {
                withAnimation {
                isSidebarOpen.toggle()
                    isShowingIPMenu.toggle()

                }
            }
                .padding()
            Button("Reset") {
                viewController.sendResetRequest()

            }
            .padding()
            
            Button("Save Global Map") {
                viewController.sendSaveRequest()
            }
            .padding()
            
            Button(state.isSavingInputs ? "Stop Saving Inputs" : "Start Saving Inputs") {
                DispatchQueue.main.async {
                    viewController.sendToggleSaveInputRequest()
                }
            }
                .padding()
            
            Divider()
            
            Text("Select an Algorithm:")
            .font(.headline)
            .padding()

            Picker("Options", selection: $state.cur_algorithm) {
                if viewController.state.availableAlgorithms.isEmpty {
                    Text("Loading...").tag("")
                } else {
                    Text("").tag("")
                    ForEach(state.availableAlgorithms, id: \.self) { option in
                        Text(option).tag(option)
                    }
                }
            }
            .pickerStyle(.automatic)
            .padding()
            .onAppear {
                DispatchQueue.main.async {
                    viewController.sendGetAlgorithmsRequest()
                }
            }
            .onChange(of: state.cur_algorithm) {
                DispatchQueue.main.async {
                    viewController.changeAlgorithms(alg_str: state.cur_algorithm)
                }
            }
            
            Divider()
            
            Text("Select a Descriptor Fn:")
            .font(.headline)
            .padding()

            Picker("Options", selection: $state.cur_descriptor) {
                if viewController.state.availableDescriptors.isEmpty {
                    Text("Loading...").tag("")
                } else {
                    Text("").tag("")
                    ForEach(state.availableDescriptors, id: \.self) { option in
                        Text(option).tag(option)
                    }
                }
            }
            .pickerStyle(.automatic)
            .padding()
            .onAppear {
                DispatchQueue.main.async {
                    
                    viewController.sendGetAlgorithmsRequest()
                }
            }
            .onChange(of: viewController.state.cur_descriptor) {
                DispatchQueue.main.async {
                    viewController.changeDescriptor(desc_str: state.cur_descriptor)
                }
            }
            
            Divider()
            
            Button("Edit Other Parameters") {
                withAnimation {
//                    isSidebarOpen.toggle()
                    isShowingParamEditor.toggle()
                }
            }
            .padding()


            
            Spacer()
        }
        .frame(alignment: .trailing)
        .background(Color(.systemGray6))
        .edgesIgnoringSafeArea(.vertical)
        .offset(x: 0, y: 0)
        .fullScreenCover(isPresented: $isShowingParamEditor) {
            let vm = ParameterEditViewModel(parameters: state.parameters)

            ParameterEditorView(viewModel: vm) { updatedParams in
                let changedParams = vm.getChangedParams()
                for param in changedParams {
                    viewController.sendSetParameter(param: param)
                }
                isShowingParamEditor = false
            }
        }
    }
}

struct ParameterEditorView: View {
    @ObservedObject var viewModel: ParameterEditViewModel
    @Environment(\.dismiss) private var dismiss

    var onSubmit: ([Parameter]) -> Void

    var body: some View {
        VStack {
            Button("Cancel") {
                dismiss()
            }
            .padding()
            .foregroundColor(.red)
            ScrollView {
                ForEach($viewModel.editableParameters) { $param in
                    VStack(alignment: .leading) {
                        Text("\(param.name) (Type \(param.type))")
                            .font(.headline)
                        TextField("Value", text: $param.rawValue)
                            .textFieldStyle(RoundedBorderTextFieldStyle())
                    }
                    .padding()
                }
            }

            Button("Submit All") {
                let params = viewModel.getChangedParams()
                onSubmit(params)
            }
            .padding()
            .background(Color.blue)
            .foregroundColor(.white)
            .cornerRadius(8)
            .padding()
        }
    }
}


struct IPMenu: View {
    @Binding var isShowing: Bool
    @Binding var selectedIP: String
    @EnvironmentObject var viewController: ARDepthViewController

    
    var body: some View {
        ZStack {
            Color.black.opacity(0.5)
                .edgesIgnoringSafeArea(.all)
                .onTapGesture {
                    isShowing = false
                }
            
            VStack {
                Text("Enter IP Address")
                    .font(.headline)
                    .padding()
                
                TextField("IP Address", text: $selectedIP)
                    .textFieldStyle(RoundedBorderTextFieldStyle())
                    .padding()
                    .keyboardType(.decimalPad)
                    .frame(width: 300)
                
                Button("Save") {
                    UserDefaults.standard.set(selectedIP, forKey: "SavedIP")
                    isShowing = false
                    viewController.setIPAddress(ip: selectedIP)

                }
                .padding()
                .background(Color.blue)
                .foregroundColor(.white)
                .cornerRadius(10)
                
                Button("Close") {
                    isShowing = false
                }
                .padding()
            }
            .frame(width: 350, height: 300)
            .background(Color.white)
            .cornerRadius(10)
            .shadow(radius: 10)
        }
    }
}

struct ARViewContainer: UIViewControllerRepresentable {
    @EnvironmentObject var viewController: ARDepthViewController
    
    func makeUIViewController(context: Context) -> ARDepthViewController {
        return viewController
    }
    
    func updateUIViewController(_ uiViewController: ARDepthViewController, context: Context) {}
}
//    func toggleScanning() {
//        viewController.toggleScanning()
//    }
//    
//    func updateIPAddress(_ ip: String) {
//        viewController.setIPAddress(ip: ip)
//    }
//    
//    func sendResetRequest() {
//        viewController.sendResetRequest()
//    }
//    func sendSaveRequest() {
//        viewController.sendSaveRequest()
//    }
//    func sendToggleSaveInputRequest() {
//        viewController.sendToggleSaveInputRequest()
//    }
//    func sendGetAlgorithmsRequest() {
//        viewController.sendGetAlgorithmsRequest()
//    }
//    func changeAlgorithms(_ alg_str: String) {
//        viewController.changeAlgorithms(alg_str: alg_str)
//    }
//}
