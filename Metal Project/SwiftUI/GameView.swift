//
//  GameView.swift
//  Metal Project
//
//  Created by Oliver Homer on 06/11/2026.
//

import Foundation
import MetalKit

class GameView: MTKView {
    
    override var acceptsFirstResponder: Bool {
        true
    }
    
    override func keyDown(with event: NSEvent) {
        engine.KeyDown(Int32(event.keyCode))
    }
    
    override func keyUp(with event: NSEvent) {
        engine.KeyUp(Int32(event.keyCode))
    }
    
    override func mouseMoved(with event: NSEvent) {
        engine.MouseMoved(Float(event.deltaX), Float(event.deltaY))
    }
    
    override func viewDidMoveToWindow() {
          super.viewDidMoveToWindow()

          window?.acceptsMouseMovedEvents = true
          window?.makeFirstResponder(self)
    }
    
}
