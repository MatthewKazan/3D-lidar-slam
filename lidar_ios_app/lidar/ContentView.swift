import SwiftUI

let arViewContainer = ARViewContainer()

struct ContentView: View {
    @State private var isSidebarOpen = false
    @State private var selectedOption = ""
    @State private var isScanning = false
    @State private var isShowingIPMenu = false
    @State private var isSavingInputs = false

    @State private var selectedIP = UserDefaults.standard.string(forKey: "SavedIP") ?? ""
    
    var body: some View {
        ZStack(alignment: .leading) {
            arViewContainer
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
                    arViewContainer.toggleScanning()
                    
                }) {
                    Text(isScanning ? "Stop Scanning" : "Start Scanning")
                        .padding()
                        .background(Color.blue)
                        .foregroundColor(.white)
                        .cornerRadius(10)
                }
                .padding(.bottom, 40)
            }
            .navigationViewStyle(StackNavigationViewStyle())
            if isShowingIPMenu {
                IPMenu(isShowing: $isShowingIPMenu, selectedIP: $selectedIP)

            }
            if isSidebarOpen {
                SidebarView(isSidebarOpen: $isSidebarOpen,
                            selectedOption: $selectedOption,
                            isShowingIPMenu: $isShowingIPMenu,
                            isSavingInputs: $isSavingInputs
                            )
                    .frame(width: 250)
                    .transition(.move(edge: .leading))
                    .zIndex(1)
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
    @Binding var selectedOption: String
    @Binding var isShowingIPMenu: Bool
    @Binding var isSavingInputs: Bool
        
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
                self.isSavingInputs.toggle()
                arViewContainer.sendResetRequest()

            }
            .padding()
            
            Button("Save Global Map") {
                arViewContainer.sendSaveRequest()
            }
            .padding()
            
            Button(isSavingInputs ? "Stop Saving Inputs" : "Start Saving Inputs") {
                arViewContainer.sendToggleSaveInputRequest()
                isSavingInputs.toggle()
            }
                .padding()
            
            Divider()
            
            Text("Select an Algorithm:")
            .font(.headline)
            .padding()

            Picker("Options", selection: $selectedOption) {
                if arViewContainer.viewController.availableAlgorithms.isEmpty {
                    Text("Loading...").tag("")
                } else {
                    Text("").tag("")
                    ForEach(arViewContainer.viewController.availableAlgorithms, id: \.self) { option in
                        Text(option).tag(option)
                    }
                }
            }
            .pickerStyle(.automatic)
            .padding()
            .onAppear {
                arViewContainer.viewController.sendGetAlgorithmsRequest() // Request data when view appears
            }
            .onChange(of: selectedOption) {
                self.isSavingInputs.toggle()
                arViewContainer.changeAlgorithms(selectedOption)
            }

            
            Spacer()
        }
        .frame(alignment: .trailing)
        .background(Color(.systemGray6))
        .edgesIgnoringSafeArea(.vertical)
        .offset(x: 0, y: 0)
    }
}

struct IPMenu: View {
    @Binding var isShowing: Bool
    @Binding var selectedIP: String
    
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
                    arViewContainer.updateIPAddress(selectedIP)

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
    @State var viewController = ARDepthViewController()
    
    func makeUIViewController(context: Context) -> ARDepthViewController {
        return viewController
    }

    func updateUIViewController(_ uiViewController: ARDepthViewController, context: Context) {}
    
    func toggleScanning() {
        viewController.toggleScanning()
    }
    
    func updateIPAddress(_ ip: String) {
        viewController.setIPAddress(ip: ip)
    }
    
    func sendResetRequest() {
        viewController.sendResetRequest()
    }
    func sendSaveRequest() {
        viewController.sendSaveRequest()
    }
    func sendToggleSaveInputRequest() {
        viewController.sendToggleSaveInputRequest()
    }
    func sendGetAlgorithmsRequest() {
        viewController.sendGetAlgorithmsRequest()
    }
    func changeAlgorithms(_ alg_str: String) {
        viewController.changeAlgorithms(alg_str: alg_str)
    }
}
