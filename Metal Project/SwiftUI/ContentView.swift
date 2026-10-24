//
//  ContentView.swift
//  Metal Project
//
//  Created by Oliver Homer on 12/10/2026.
//

import SwiftUI

struct ContentView: View {
    var body: some View {
        HStack{
            MetalView()
            VStack {
                Image(systemName: "globe")
                    .imageScale(.large)
                    .foregroundStyle(.tint)
                Text("Hello, world!")
                Button("Run"){
                    engine.RunApp()
                }
            }
            
        }
        .padding()
    }
}

#Preview {
    ContentView()
}
