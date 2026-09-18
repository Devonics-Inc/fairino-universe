#!/usr/bin/env python3
"""Fake infeed conveyor for a MoveIt 2 palletizing demo (no physics simulator).

- Adds the conveyor body as a static collision object.
- Spawns boxes at the belt start and slides them along +x in the planning scene.
- Boxes queue up behind each other and the lead box stops at the pick point.
- Publishes conveyor/box_present (Bool) and conveyor/box_at_pick (String id).
- The palletizer attaches the box to the gripper, then calls
  conveyor/box_picked (std_srvs/Trigger) so the conveyor forgets that box.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

EPS = 1e-6


class FakeConveyor(Node):
    def __init__(self):
        super().__init__('fake_conveyor')
        p = self.declare_parameter
        self.frame = p('frame_id', 'world').value
        self.belt_z = p('belt_height', 0.40).value        # top surface height (m)
        self.belt_y = p('belt_y', -0.60).value            # belt centerline y (m)
        self.belt_width = p('belt_width', 0.40).value
        self.start_x = p('start_x', -0.80).value          # where boxes appear
        self.pick_x = p('pick_x', 0.0).value              # where the lead box stops
        self.box = list(p('box_size', [0.30, 0.20, 0.15]).value)  # x, y, z
        self.speed = p('speed', 0.15).value               # m/s
        self.gap = p('gap', 0.05).value                   # spacing in the queue
        self.spawn_period = p('spawn_period', 4.0).value  # s
        self.max_boxes = p('max_boxes', 50).value
        rate = p('rate', 20.0).value

        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.present_pub = self.create_publisher(Bool, 'conveyor/box_present', 10)
        self.id_pub = self.create_publisher(String, 'conveyor/box_at_pick', 10)
        self.create_service(Trigger, 'conveyor/box_picked', self.on_picked)

        self.boxes = []          # [[id, x], ...], index 0 is the lead box
        self.count = 0
        self.last_spawn = None
        self.ready = False
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.step)

    # ---------- main loop ----------
    def step(self):
        if not self.ready:
            # Wait for move_group to subscribe, otherwise the first diff is lost.
            if self.scene_pub.get_subscription_count() == 0:
                return
            self.publish([self.make_belt()])
            self.ready = True
            self.get_logger().info('Conveyor added to planning scene')

        now = self.get_clock().now().nanoseconds * 1e-9
        objs = []

        # Advance boxes: lead stops at pick_x, each other box stops behind the one ahead.
        limit = self.pick_x
        for b in self.boxes:
            new_x = min(b[1] + self.speed * self.dt, limit)
            if new_x > b[1] + EPS:
                b[1] = new_x
                objs.append(self.make_box(b[0], new_x, CollisionObject.MOVE))
            limit = b[1] - self.box[0] - self.gap

        # Spawn a new box if it's time and there's room at the belt start.
        due = self.last_spawn is None or now - self.last_spawn >= self.spawn_period
        room = not self.boxes or self.boxes[-1][1] - self.start_x >= self.box[0] + self.gap
        if due and room and self.count < self.max_boxes:
            self.count += 1
            box_id = f'box_{self.count}'
            self.boxes.append([box_id, self.start_x])
            objs.append(self.make_box(box_id, self.start_x, CollisionObject.ADD))
            self.last_spawn = now

        if objs:
            self.publish(objs)

        at_pick = self.lead_at_pick()
        self.present_pub.publish(Bool(data=at_pick))
        self.id_pub.publish(String(data=self.boxes[0][0] if at_pick else ''))

    def on_picked(self, request, response):
        if self.lead_at_pick():
            response.success = True
            response.message = self.boxes.pop(0)[0]
        else:
            response.success = False
            response.message = 'no box at pick point'
        return response

    # ---------- helpers ----------
    def lead_at_pick(self):
        return bool(self.boxes) and abs(self.boxes[0][1] - self.pick_x) < EPS

    def make_belt(self):
        length = (self.pick_x - self.start_x) + self.box[0] + 0.10
        pose = Pose()
        pose.position.x = (self.start_x + self.pick_x) / 2.0
        pose.position.y = self.belt_y
        pose.position.z = self.belt_z / 2.0
        pose.orientation.w = 1.0
        return self.make_object('conveyor', pose,
                                [length, self.belt_width, self.belt_z],
                                CollisionObject.ADD)

    def make_box(self, box_id, x, op):
        pose = Pose()
        pose.position.x = x
        pose.position.y = self.belt_y
        pose.position.z = self.belt_z + self.box[2] / 2.0 + 0.002  # tiny lift off belt
        pose.orientation.w = 1.0
        return self.make_object(box_id, pose, self.box, op)

    def make_object(self, obj_id, pose, dims, op):
        obj = CollisionObject()
        obj.header.frame_id = self.frame
        obj.id = obj_id
        obj.operation = op
        obj.pose = pose
        if op == CollisionObject.ADD:
            obj.primitives.append(
                SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(dims)))
            identity = Pose()
            identity.orientation.w = 1.0
            obj.primitive_poses.append(identity)
        return obj

    def publish(self, objs):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = objs
        self.scene_pub.publish(scene)


def main():
    rclpy.init()
    node = FakeConveyor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()