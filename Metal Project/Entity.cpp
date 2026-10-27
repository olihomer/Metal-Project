//
//  Entity.cpp
//  Metal Project
//
//  Created by Oliver Homer on 23/10/2026.
//

#include "Entity.hpp"

namespace ECS
{

bool Entity::hasComponent(ComponentType type)
{
    for(auto component: components_)
        if(component.type == type) return true;
    return false;
}

}
