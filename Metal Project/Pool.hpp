//
//  Pool.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef Pool_hpp
#define Pool_hpp


#include <vector>
#include <queue>
#include <iostream>

namespace ECS {

class Entity;

template<typename T>
struct Handle
{
    std::uint32_t slotIndex; //points to a slot
    std::uint32_t generation;
    void print(){std::cout<<"Handle: "<<slotIndex<<" / "<<generation<<std::endl;};
};

struct Slot
{
    bool isActive;
    std::uint32_t generation;
    std::uint32_t physicalIndex; //points to an element
    Handle<Entity> owner;
};

template<typename T>
class Pool
{
    static_assert(std::is_nothrow_move_constructible_v<T>,"Pool elements must be nothrow move constructible");
    static_assert(std::is_nothrow_move_assignable_v<T>,"Pool elements must be nothrow move assignable");
    
public:
    template<typename... Args>
    Handle<T> addElement(Args&&... args); //adds element with no owning entity
    
    void deleteElement(Handle<T> handle);
    void setOwner(Handle<T> handle, Handle<Entity> owner);
    T& get(Handle<T> handle);
    void print();
    
private:
    std::vector<T> elements_; //vector of the actual components
    std::vector<Slot> slots_; //vector of slots, each of which points to a physicalIndex of the element vector
    std::vector<Handle<T>> handles_; //vector of handles so that we can reverse lookup a slot/handle from an element/physical index
    std::queue<std::uint32_t> freeList_; //queue of available slot indexes
    bool validateHandle(Handle<T> handle);
    friend class ComponentManager;
};



template<typename T>
template<typename... Args>
Handle<T> Pool<T>::addElement(Args&&... args)
{
    std::cout << "Adding Element" << std::endl;
    Handle<T> newHandle;
        
    if(freeList_.empty()) //no spare slots, so just add one at the end
    {
        Slot newSlot;
        newSlot.isActive = true;
        newSlot.generation = 0;
        newSlot.physicalIndex = std::uint32_t(elements_.size());
        
        newHandle.generation = 0;
        newHandle.slotIndex = std::uint32_t(slots_.size());
        
        slots_.push_back(newSlot);
        try{
            handles_.push_back(newHandle);
        }
        catch(...){
            slots_.pop_back(); //undo the slot push_back
            throw;
        }
        
        try{
            elements_.emplace_back(std::forward<Args>(args)...);
        }
        catch(...){
            slots_.pop_back(); //undo the slot push_back
            handles_.pop_back(); //undo the handle push_back
            throw;
        }
    }
    else //get first spare slot index from freeList
    {
        auto slotIndex = freeList_.front();

        Slot freeSlot = slots_[slotIndex];
        
        freeSlot.isActive = true;
        freeSlot.generation++;
        freeSlot.physicalIndex = std::uint32_t(elements_.size());
        
        newHandle.generation = freeSlot.generation;
        newHandle.slotIndex = slotIndex;
        
        handles_.push_back(newHandle); //CAN THROW BUT DOESN'T MATTER
        try{
            elements_.emplace_back(std::forward<Args>(args)...);
        }
        catch(...){
            handles_.pop_back(); //undo the handle push_back
            throw;
        }
        
        //do these things last
        
        freeList_.pop(); // CANNOT THROW
        slots_[slotIndex] = std::move(freeSlot);
    }

    return newHandle;
}


template<typename T>
void Pool<T>::deleteElement(Handle<T> handle)
{
    std::cout << "Deleting element" << std::endl;
    handle.print();
    
    if(!validateHandle(handle))
        throw std::runtime_error("Pool::deleteElement() with stale handle");
    
    Slot& deadSlot = slots_[handle.slotIndex];
    deadSlot.isActive = false; //mark slot inactive
    freeList_.push(handle.slotIndex); //add slot to free list
    
    //pop and swap elements

    Handle<T>& finalHandle = handles_[elements_.size()-1]; //get the handle of the final element that is about to be swapped
    Slot& finalSlot = slots_[finalHandle.slotIndex]; //get the slot that points to that element
    finalSlot.physicalIndex = deadSlot.physicalIndex;
    
    elements_[deadSlot.physicalIndex] = std::move(elements_.back()); //move end element to take place of dead element
    elements_.pop_back(); //remove final element
    handles_[deadSlot.physicalIndex] = std::move(handles_.back()); //make sure the handles list matches
    handles_.pop_back(); //remove final element

}


template<typename T>
void Pool<T>::print()
{
    std::cout << "Elements size: " << elements_.size() << std::endl;
    
    if constexpr (std::is_same_v<T,Entity>)
    {
        for(auto& element: elements_)
            element.print();
    }
    
    std::cout << "Slots: " << slots_.size() << std::endl;
    for(std::size_t i = 0; i < slots_.size(); i++)
    {
        std::cout << "Slot " << i << ":" << std::endl;
        std::cout << "isActive: " << (slots_[i].isActive ? "true" : "false") << std::endl;
        std::cout << "generation: " << slots_[i].generation << std::endl;
        std::cout << "physicalIndex: " << slots_[i].physicalIndex << std::endl;
    }
    
    std::cout << "Handles size: " << handles_.size() << std::endl;
    for(std::size_t i = 0; i < handles_.size(); i++)
    {
        std::cout << "Handle " << i << ":" << std::endl;
        std::cout << "slotIndex: " << handles_[i].slotIndex << std::endl;
        std::cout << "generation: " << handles_[i].generation << std::endl;
    }
    
    std::cout << "Freelist size: " << freeList_.size() << std::endl;
    
    auto temp = freeList_;
    for(std::size_t i = 0; i < freeList_.size(); i++)
    {
        auto x = temp.front();
        temp.pop();
        std::cout << "Free Slot: " << x << std::endl;
    }
    
}

template<typename T>
bool Pool<T>::validateHandle(Handle<T> handle)
{
    if(handle.slotIndex >= slots_.size()) return false;
    Slot& s = slots_[handle.slotIndex];
    if(handle.generation == s.generation && s.isActive == true)
        return true;
    else
        return false;
}


template<typename T>
T& Pool<T>::get(Handle<T> handle)
{
    if(!validateHandle(handle))
        throw std::runtime_error("Pool::get() with stale handle");
    
    Slot& s = slots_[handle.slotIndex];
    return elements_[s.physicalIndex];
}

template<typename T>
void Pool<T>::setOwner(Handle<T> handle, Handle<Entity> owner)
{
    if(validateHandle(handle))
        slots_[handle.slotIndex].owner = owner;
    else
        throw(std::runtime_error("invalid handle passed to setowner"));
}


}

#endif // !Pool_hpp
