#import <XCTest/XCTest.h>

#include <cstdint>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

#define private public
#include "../Metal Project/World.hpp"
#undef private

namespace {

struct ThrowingElement
{
    explicit ThrowingElement(int newValue, bool shouldThrow = false) : value(newValue)
    {
        if(shouldThrow) throw std::runtime_error("construction failed");
    }

    ThrowingElement(ThrowingElement&&) noexcept = default;
    ThrowingElement& operator=(ThrowingElement&&) noexcept = default;

    int value;
};

template<typename Operation>
bool throwsRuntimeError(Operation operation)
{
    try {
        operation();
    } catch(const std::runtime_error&) {
        return true;
    }
    return false;
}

}

@interface WorldECSTests : XCTestCase
@end

@implementation WorldECSTests

- (void)testNewEntityStartsWithoutComponents
{
    World world;
    auto entity = world.newEntity("Player");

    XCTAssertFalse(world.hasComponent<Light>(entity));
    XCTAssertFalse(world.hasComponent(entity, ECS::ComponentType::Light));
}

- (void)testAddingComponentRegistersItWithEntity
{
    World world;
    auto entity = world.newEntity();

    world.addComponent<Light>(entity);

    XCTAssertTrue(world.hasComponent<Light>(entity));
    XCTAssertTrue(world.hasComponent(entity, ECS::ComponentType::Light));
}

- (void)testDifferentComponentTypesCanBeAdded
{
    World world;
    auto entity = world.newEntity();

    world.addComponent<Mesh>(entity);
    world.addComponent<RigidBody>(entity);
    world.addComponent<Transform>(entity);

    XCTAssertTrue(world.hasComponent<Mesh>(entity));
    XCTAssertTrue(world.hasComponent<RigidBody>(entity));
    XCTAssertTrue(world.hasComponent<Transform>(entity));
    XCTAssertFalse(world.hasComponent<Light>(entity));
}

- (void)testComponentsBelongToTheirOwnEntity
{
    World world;
    auto firstEntity = world.newEntity();
    auto secondEntity = world.newEntity();

    world.addComponent<Light>(firstEntity);

    XCTAssertTrue(world.hasComponent<Light>(firstEntity));
    XCTAssertFalse(world.hasComponent<Light>(secondEntity));
}

- (void)testAddingDuplicateComponentThrows
{
    World world;
    auto entity = world.newEntity();
    world.addComponent<Light>(entity);

    bool didThrow = false;
    try {
        world.addComponent<Light>(entity);
    } catch (const std::runtime_error&) {
        didThrow = true;
    }

    XCTAssertTrue(didThrow);
}

- (void)testDeletingComponentDeregistersIt
{
    World world;
    auto entity = world.newEntity();
    auto light = world.addComponent<Light>(entity);

    world.deleteComponent<Light>(entity, light);

    XCTAssertFalse(world.hasComponent<Light>(entity));
}

- (void)testDeletingComponentFromWrongEntityThrows
{
    World world;
    auto owner = world.newEntity();
    auto otherEntity = world.newEntity();
    auto light = world.addComponent<Light>(owner);

    bool didThrow = false;
    try {
        world.deleteComponent<Light>(otherEntity, light);
    } catch (const std::runtime_error&) {
        didThrow = true;
    }

    XCTAssertTrue(didThrow);
    XCTAssertTrue(world.hasComponent<Light>(owner));
}

- (void)testDeletingEntityInvalidatesItsHandle
{
    World world;
    auto entity = world.newEntity();
    world.addComponent<Light>(entity);

    world.deleteEntity(entity);

    bool didThrow = false;
    try {
        (void)world.hasComponent<Light>(entity);
    } catch (const std::runtime_error&) {
        didThrow = true;
    }

    XCTAssertTrue(didThrow);
}

- (void)testReusedEntitySlotGetsNewGeneration
{
    World world;
    auto deletedEntity = world.newEntity();
    world.deleteEntity(deletedEntity);

    auto replacementEntity = world.newEntity();

    XCTAssertEqual(deletedEntity.slotIndex, replacementEntity.slotIndex);
    XCTAssertNotEqual(deletedEntity.generation, replacementEntity.generation);
}

- (void)testThrowingConstructorForNewSlotLeavesPoolUnchanged
{
    ECS::Pool<ThrowingElement> pool;
    auto existing = pool.addElement(1);
    const auto elementCount = pool.elements_.size();
    const auto slotCount = pool.slots_.size();
    const auto handleCount = pool.handles_.size();

    XCTAssertTrue(throwsRuntimeError([&] { (void)pool.addElement(2, true); }));
    XCTAssertEqual(pool.elements_.size(), elementCount);
    XCTAssertEqual(pool.slots_.size(), slotCount);
    XCTAssertEqual(pool.handles_.size(), handleCount);
    XCTAssertEqual(pool.get(existing).value, 1);
}

- (void)testThrowingConstructorForReusedSlotLeavesPoolUnchanged
{
    ECS::Pool<ThrowingElement> pool;
    auto removed = pool.addElement(1);
    auto survivor = pool.addElement(2);
    pool.deleteElement(removed);
    const auto elementCount = pool.elements_.size();
    const auto slotCount = pool.slots_.size();
    const auto handleCount = pool.handles_.size();
    const auto freeCount = pool.freeList_.size();
    const auto generation = pool.slots_[removed.slotIndex].generation;

    XCTAssertTrue(throwsRuntimeError([&] { (void)pool.addElement(3, true); }));
    XCTAssertEqual(pool.elements_.size(), elementCount);
    XCTAssertEqual(pool.slots_.size(), slotCount);
    XCTAssertEqual(pool.handles_.size(), handleCount);
    XCTAssertEqual(pool.freeList_.size(), freeCount);
    XCTAssertEqual(pool.slots_[removed.slotIndex].generation, generation);
    XCTAssertFalse(pool.slots_[removed.slotIndex].isActive);
    XCTAssertEqual(pool.get(survivor).value, 2);
}

- (void)testReallocationDuringInsertionPreservesExistingHandles
{
    ECS::Pool<ThrowingElement> pool;
    std::vector<ECS::Handle<ThrowingElement>> handles;
    for(int value = 0; value < 256; ++value) {
        handles.push_back(pool.addElement(value));
    }

    for(int value = 0; value < 256; ++value) {
        XCTAssertEqual(pool.get(handles[value]).value, value);
    }
}

- (void)testDeletingFirstMiddleAndLastElementsPreservesRemainingHandles
{
    for(const std::size_t deletedIndex : {std::size_t(0), std::size_t(2), std::size_t(4)}) {
        ECS::Pool<ThrowingElement> pool;
        std::vector<ECS::Handle<ThrowingElement>> handles;
        for(int value = 0; value < 5; ++value) handles.push_back(pool.addElement(value));

        pool.deleteElement(handles[deletedIndex]);

        XCTAssertTrue(throwsRuntimeError([&] { (void)pool.get(handles[deletedIndex]); }));
        for(std::size_t index = 0; index < handles.size(); ++index) {
            if(index != deletedIndex) XCTAssertEqual(pool.get(handles[index]).value, int(index));
        }
    }
}

- (void)testReusedSlotRejectsOldHandleAndIncrementsGeneration
{
    ECS::Pool<ThrowingElement> pool;
    auto oldHandle = pool.addElement(1);
    pool.deleteElement(oldHandle);

    auto newHandle = pool.addElement(2);

    XCTAssertEqual(newHandle.slotIndex, oldHandle.slotIndex);
    XCTAssertEqual(newHandle.generation, oldHandle.generation + 1);
    XCTAssertTrue(throwsRuntimeError([&] { (void)pool.get(oldHandle); }));
    XCTAssertEqual(pool.get(newHandle).value, 2);
}

- (void)testThrowDuringComponentRegistrationRollsBackNewComponent
{
    World world;
    auto entity = world.newEntity();
    auto& lightPool = world.componentManager_.lights_;
    const auto elementCount = lightPool.elements_.size();
    const auto handleCount = lightPool.handles_.size();
    world.entityManager_.beforeComponentRegistration_ = [] {
        throw std::runtime_error("injected registration failure");
    };

    XCTAssertTrue(throwsRuntimeError([&] { (void)world.addComponent<Light>(entity); }));
    XCTAssertEqual(lightPool.elements_.size(), elementCount);
    XCTAssertEqual(lightPool.handles_.size(), handleCount);
    XCTAssertFalse(world.hasComponent<Light>(entity));
}

- (void)testSetOwnerWithInvalidHandleThrowsWithoutChangingState
{
    ECS::Pool<ThrowingElement> pool;
    auto validHandle = pool.addElement(1);
    ECS::Handle<ECS::Entity> originalOwner{3, 4};
    ECS::Handle<ECS::Entity> replacementOwner{7, 8};
    pool.setOwner(validHandle, originalOwner);
    auto invalidHandle = validHandle;
    invalidHandle.generation++;

    XCTAssertTrue(throwsRuntimeError([&] { pool.setOwner(invalidHandle, replacementOwner); }));
    const auto owner = pool.slots_[validHandle.slotIndex].owner;
    XCTAssertEqual(owner.slotIndex, originalOwner.slotIndex);
    XCTAssertEqual(owner.generation, originalOwner.generation);
    XCTAssertEqual(pool.elements_.size(), std::size_t(1));
    XCTAssertEqual(pool.handles_.size(), std::size_t(1));
}

@end
