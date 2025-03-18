import SwiftUI

let arViewContainer = ARViewContainer()

struct ContentView: View {
    @State private var isScanning = false
    @State private var isShowingMenu = false
    @State private var selectedIP = UserDefaults.standard.string(forKey: "SavedIP") ?? ""
    
    var body: some View {
        ZStack {
            arViewContainer
                .edgesIgnoringSafeArea(.all)
            
            VStack {
                HStack {
                    Button(action: {
                        isShowingMenu.toggle()
                    }) {
                        Text("Set IP")
                            .padding()
                            .background(Color.gray.opacity(0.8))
                            .foregroundColor(.white)
                            .cornerRadius(10)
                    }
                    .padding()
                    Spacer()
                }
                Spacer()
                
                if !isShowingMenu {
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
                    .padding()
                    HStack {
                        
                        Button(action: {
                            arViewContainer.sendResetRequest()
                        }) {
                            Text("Clear DB")
                                .padding()
                                .background(Color.green)
                                .foregroundColor(.white)
                                .cornerRadius(10)
                        }
                        .padding()
                        
                        Button(action: {
                            arViewContainer.sendSaveRequest()
                        }) {
                            Text("Save Map")
                                .padding()
                                .background(Color.orange)
                                .foregroundColor(.white)
                                .cornerRadius(10)
                        }
                        .padding()
                        
                        Button(action: {
                            arViewContainer.sendToggleSaveInputRequest()
                        }) {
                            Text("Save Inputs")
                                .padding()
                                .background(Color.indigo)
                                .foregroundColor(.white)
                                .cornerRadius(10)
                        }
                        .padding()
                    }
                }
            }
            
            if isShowingMenu {
                SideMenu(isShowing: $isShowingMenu, selectedIP: $selectedIP)
            }
        }
    }
}

struct SideMenu: View {
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
    let viewController = ARDepthViewController()
    
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
}

#Preview {
    ContentView()
}
