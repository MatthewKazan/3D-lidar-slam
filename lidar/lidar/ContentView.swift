import SwiftUI

struct ContentView: View {
    @State private var isScanning = false
    let arViewContainer = ARViewContainer()

    var body: some View {
        ZStack {
            arViewContainer
                .edgesIgnoringSafeArea(.all)
            
            VStack {
                Spacer()
                
                // Button to toggle scanning
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
                
                // Button to send POST request
                Button(action: {
                    sendPostRequest()
                }) {
                    Text("Clear DB")
                        .padding()
                        .background(Color.green)
                        .foregroundColor(.white)
                        .cornerRadius(10)
                }
                .padding()
            }
        }
    }
    
    // Function to send POST request
    private func sendPostRequest() {
        guard let url = URL(string: "http://10.0.0.169:8080/clear_db") else {
            print("Invalid URL")
            return
        }

        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        // Example JSON body
        let body: [String: Any] = ["message": "Hello from SwiftUI"]
        request.httpBody = try? JSONSerialization.data(withJSONObject: body)

        // Perform network request
        URLSession.shared.dataTask(with: request) { data, response, error in
            if let error = error {
                print("Error: \(error.localizedDescription)")
                return
            }

            if let httpResponse = response as? HTTPURLResponse, httpResponse.statusCode == 200 {
                print("POST request succeeded")
            } else {
                print("POST request failed")
            }
        }.resume()
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
}

#Preview {
    ContentView()
}
