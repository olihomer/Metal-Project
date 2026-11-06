//
//  InputSystem.hpp
//  Metal Project
//
//  Created by Oliver Homer on 06/11/2026.
//

#ifndef InputSystem_hpp
#define InputSystem_hpp

#include <array>

class InputSystem
{
public:
    void keyDown(int key)
    {
        keys[key] = true;
    }
    void keyUp(int key)
    {
        keys[key] = false;
    }
    bool isKeyDown(int key) const
    {
        return keys[key];
    }
    
private:
    std::array<bool,256> keys{};
    
};

#endif /* InputSystem_hpp */
