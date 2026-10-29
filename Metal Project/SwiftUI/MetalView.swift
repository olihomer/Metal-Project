//
//  MetalView.swift
//  Metal Project
//
//  Created by Oliver Homer on 13/10/2026.
//

import Foundation
import MetalKit
import Metal
import SwiftUI

let device = MTLCreateSystemDefaultDevice()
var engine = Engine(OpaquePointer(Unmanaged.passUnretained(device as AnyObject).toOpaque()))
let clock = ContinuousClock()
var lastTime = clock.now

struct MetalView: NSViewRepresentable
{
    func makeNSView(context: Context) -> MTKView {
        let device = device
        let view = MTKView(frame: .zero, device: device)
        
        view.colorPixelFormat = MTLPixelFormat.bgra8Unorm
        view.clearColor = MTLClearColor(red: 0.1, green: 0.2, blue: 0.3, alpha: 1.0)
        view.preferredFramesPerSecond = 60
        view.isPaused = false
        view.enableSetNeedsDisplay = false
        view.delegate = context.coordinator
        
        return view
    }
    
    func updateNSView(_ view: MTKView, context: Context) {
    
    }
    
    func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    class Coordinator: NSObject, MTKViewDelegate{
        func draw(in view: MTKView){
            guard
                let pass = view.currentRenderPassDescriptor,
                let drawable = view.currentDrawable
            else { return }
            
            var thisTime = clock.now
            engine.Update((thisTime-lastTime).components.attoseconds)
            lastTime = thisTime
            engine.RenderFrame(
                Unmanaged.passUnretained(pass).toOpaque(),
                Unmanaged.passUnretained(drawable as AnyObject).toOpaque()
            )
        }
        func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
            
        }
        
    }
}
