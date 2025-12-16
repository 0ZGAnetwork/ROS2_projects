import py_trees
import py_trees.common
import py_trees.display
import time
import matplotlib.pyplot as plt


class WorldState:
    def __init__(self):
        self.position = (0, 0)
        self.start_position = (0, 0)
        self.carrying_object = None
        self.objects = {
            "obj1": {"location": (2, 3), "picked": False},
            "obj2": {"location": (1, 1), "picked": False}
        }
        self.place_pos = (5, 5)


class MoveTo(py_trees.behaviour.Behaviour):
    def __init__(self, name, world, target):
        super().__init__(name)
        self.world = world
        self.target = target

    def update(self):
        self.world.position = self.target
        print(f"[MoveTo] Moving to {self.target}")
        return py_trees.common.Status.SUCCESS


class PickUpObj(py_trees.behaviour.Behaviour):
    def __init__(self, name, world, object_name):
        super().__init__(name)
        self.world = world
        self.object_name = object_name

    def update(self):
        obj = self.world.objects[self.object_name]

        if self.world.position == obj["location"] and self.world.carrying_object is None:
            self.world.carrying_object = self.object_name
            obj["picked"] = True
            print(f"[PickUp] Picked {self.object_name}")
            return py_trees.common.Status.SUCCESS

        print(f"[PickUp] Failed to pick {self.object_name}")
        return py_trees.common.Status.FAILURE


class PlaceObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, world):
        super().__init__(name)
        self.world = world

    def update(self):
        if self.world.carrying_object is not None:
            obj_name = self.world.carrying_object
            self.world.objects[obj_name]["location"] = self.world.position
            self.world.carrying_object = None
            print(f"[Place] Placed {obj_name}")
            return py_trees.common.Status.SUCCESS

        print("[Place] Nothing to place")
        return py_trees.common.Status.FAILURE


def draw_scene(world, ax):
    ax.clear()
    ax.set_xlim(-1, 7)
    ax.set_ylim(-1, 7)
    ax.set_title("Pick-and-Place Simulation")

    ax.plot(*world.position, "bo", label="Robot")

    for name, obj in world.objects.items():
        ax.plot(*obj["location"], "rs")
        ax.text(obj["location"][0] + 0.1, obj["location"][1] + 0.1, name)

    ax.legend()
    ax.grid(True)
    plt.draw()
    plt.pause(0.5)

def create_behavior_tree(world):
    root = py_trees.composites.Sequence("Root", memory=True)

    for obj_name in ["obj1", "obj2"]:
        obj_position = world.objects[obj_name]["location"]

        root.add_children([
            MoveTo(f"MoveTo_{obj_name}", world, obj_position),
            PickUpObj(f"PickUp_{obj_name}", world, obj_name),
            MoveTo(f"MoveToPlace_{obj_name}", world, world.place_pos),
            PlaceObject(f"Place_{obj_name}", world)
        ])

    root.add_child(
        MoveTo("ReturnToStart", world, world.start_position)
    )

    return py_trees.trees.BehaviourTree(root)


if __name__ == "__main__":
    world = WorldState()
    tree = create_behavior_tree(world)

    print(py_trees.display.ascii_tree(tree.root))
    tree.setup()

    plt.ion()
    fig, ax = plt.subplots()

    while True:
        print("\n--- Robot State ---")
        print(f"Position: {world.position}")
        print(f"Carrying: {world.carrying_object}")
        print(f"Tree status: {tree.root.status}")

        
        tree.tick()
        draw_scene(world, ax)

        if tree.root.status == py_trees.common.Status.SUCCESS:
            print("\nTask completed successfully.")
            break

    plt.ioff()
    plt.show()
