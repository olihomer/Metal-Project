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
    void print(){std::cout << components_.size() << std::endl;};
    
private:
    Handle<Entity> handle_;
    std::string entityName_;
    std::vector<ECS::ComponentReference> components_;
    void registerComponent(ComponentReference component){components_.push_back(component);}
    friend class EntityManager;
};

}

#endif // !Entity_hpp
