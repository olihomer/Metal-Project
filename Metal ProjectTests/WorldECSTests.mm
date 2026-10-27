#import <XCTest/XCTest.h>

#include "../Metal Project/World.hpp"

@interface WorldECSTests : XCTestCase
@end

@implementation WorldECSTests

- (void)testNewEntityStartsWithoutComponents
{
    World world;
    auto entity = world.newEntity("Player");

    XCTAssertFalse(world.entityHasComponent<Light>(entity));
    XCTAssertFalse(world.entityHasComponent(entity, ECS::ComponentType::Light));
}

- (void)testAddingComponentRegistersItWithEntity
{
    World world;
    auto entity = world.newEntity();

    world.addComponent<Light>(entity);

    XCTAssertTrue(world.entityHasComponent<Light>(entity));
    XCTAssertTrue(world.entityHasComponent(entity, ECS::ComponentType::Light));
}

- (void)testDifferentComponentTypesCanBeAdded
{
    World world;
    auto entity = world.newEntity();

    world.addComponent<Mesh>(entity);
    world.addComponent<RigidBody>(entity);
    world.addComponent<Transform>(entity);

    XCTAssertTrue(world.entityHasComponent<Mesh>(entity));
    XCTAssertTrue(world.entityHasComponent<RigidBody>(entity));
    XCTAssertTrue(world.entityHasComponent<Transform>(entity));
    XCTAssertFalse(world.entityHasComponent<Light>(entity));
}

- (void)testComponentsBelongToTheirOwnEntity
{
    World world;
    auto firstEntity = world.newEntity();
    auto secondEntity = world.newEntity();

    world.addComponent<Light>(firstEntity);

    XCTAssertTrue(world.entityHasComponent<Light>(firstEntity));
    XCTAssertFalse(world.entityHasComponent<Light>(secondEntity));
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

    XCTAssertFalse(world.entityHasComponent<Light>(entity));
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
    XCTAssertTrue(world.entityHasComponent<Light>(owner));
}

- (void)testDeletingEntityInvalidatesItsHandle
{
    World world;
    auto entity = world.newEntity();
    world.addComponent<Light>(entity);

    world.deleteEntity(entity);

    bool didThrow = false;
    try {
        (void)world.entityHasComponent<Light>(entity);
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

@end
