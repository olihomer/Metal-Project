//
//  Pool.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef Pool_hpp
#define Pool_hpp


#include <vector>

enum class ElementType {Entity,RigidBody,Mesh,Light,Transform};

struct Handle
{
    std::uint32_t slotIndex; //points to a slot
    std::uint32_t generation;
};

struct Slot
{
    bool isActive;
    std::uint32_t generation;
    std::uint32_t physicalIndex; //points to an element
};

template<typename T>
class Pool
{
public:
    void addElement();
    void deleteElement();
    
private:
    std::vector<T> elements_; //vector of the actual components
    std::vector<Slot> slots_; //vector of slots, each of which points to a physicalIndex of the element vector
    std::vector<Handle> handles_; //vector of handles so that we can reverse lookup a slot/handle from an element/physical index
};




#endif // !Pool_hpp
