//
//  Entity.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef Entity_hpp
#define Entity_hpp

#include <vector>
#include <string>
#include "Pool.hpp"
#include "ComponentReference.hpp"

namespace ECS {

class Entity
{
public:
    Entity(Handle<Entity> handle, std::string name = "")
    {
        handle_ = handle;
        entityName_ = name;
    }
    void print(){std::cout << "Entity: " << entityName_ << " has " << components_.size() << " components."<< std::endl;};
private:
    Handle<Entity> handle_;
    std::string entityName_;
    std::vector<ECS::ComponentReference> components_;
    void registerComponent(ComponentReference component){components_.push_back(component);}
    bool hasComponent(ComponentType type);

    template<typename T>
    bool hasComponent();

    friend class EntityManager;
};

template<typename T>
bool Entity::hasComponent()
{
    for(auto component: components_)
        if(component.type == componentType<T>) return true;
    return false;
}


}
#endif // !Entity_hpp
