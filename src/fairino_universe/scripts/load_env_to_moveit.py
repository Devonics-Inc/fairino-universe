#!/usr/bin/env python3
"""
Reads env_config.yaml and pushes the described primitives/meshes into the
MoveIt planning scene as collision objects, via the /apply_planning_scene
service. Runs once at startup and exits.
"""
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh
from geometry_msgs.msg import Pose
import os

def load_mesh_from_file(filepath, scale=(1.0, 1.0, 1.0)):
    import trimesh
    from shape_msgs.msg import Mesh, MeshTriangle
    from geometry_msgs.msg import Point

    tm = trimesh.load(filepath, force="mesh")
    tm.apply_scale(scale)

    mesh = Mesh()
    for face in tm.faces:
        tri = MeshTriangle()
        tri.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
        mesh.triangles.append(tri)
    for v in tm.vertices:
        mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))

    return mesh



def pose_from_list(p):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = p[0], p[1], p[2]
    # roll/pitch/yaw -> quaternion
    import math
    roll, pitch, yaw = p[3], p[4], p[5]
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    pose.orientation.w = cr * cp * cy + sr * sp * sy
    pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.orientation.z = cr * cp * sy - sr * sp * cy
    return pose


def build_collision_objects(env_config, frame_id):
    objects = []

    for name, props in env_config.get("primitives", {}).get("box", {}).items():
        co = CollisionObject()
        co.header.frame_id = frame_id
        co.id = name
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(props["size"])
        co.primitives.append(prim)
        co.primitive_poses.append(pose_from_list(props["pose"]))
        co.operation = CollisionObject.ADD
        objects.append(co)

    for name, props in env_config.get("primitives", {}).get("cylinder", {}).items():
        co = CollisionObject()
        co.header.frame_id = frame_id
        co.id = name
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.CYLINDER
        # SolidPrimitive cylinder order is [height, radius]
        prim.dimensions = [props["length"], props["radius"]]
        co.primitives.append(prim)
        co.primitive_poses.append(pose_from_list(props["pose"]))
        co.operation = CollisionObject.ADD
        objects.append(co)
        
    for name, props in env_config.get("stl", {}).items():
            co = CollisionObject()
            co.header.frame_id = frame_id
            co.id = name
            try:
                mesh_path = props["mesh"].replace("package://", "")
                pkg, rel = mesh_path.split("/", 1)
                full_path = os.path.join(get_package_share_directory(pkg), rel)
                mesh = load_mesh_from_file(full_path, props.get("scale", [1.0, 1.0, 1.0]))
                co.meshes.append(mesh)
                co.mesh_poses.append(pose_from_list(props["pose"]))
                co.operation = CollisionObject.ADD
                objects.append(co)
            except Exception as e:
                print(f"[WARN] Skipping mesh '{name}': {e}")

    return objects


def main():
    rclpy.init()
    node = rclpy.create_node("load_env_to_moveit")
    
    
    node.declare_parameter("env_config_file", "")
    env_config_filename = node.get_parameter("env_config_file").get_parameter_value().string_value

    if not env_config_filename:
        node.get_logger().info("No env_config filename provided, skipping environment load")
        rclpy.shutdown()
        return

    config_path = os.path.join(
        get_package_share_directory("fairino_universe"),
        "config",
        env_config_filename,
    )
    try:
        with open(config_path) as f:
            env_config = yaml.safe_load(f) or {}
    except OSError:
        node.get_logger().info(f"No {env_config_filename} found at {config_path}, nothing to load")
        rclpy.shutdown()
        return

    frame_id = "world"  # change if your planning frame differs
    objects = build_collision_objects(env_config, frame_id)
    if not objects:
        node.get_logger().info(f"{env_config_filename} had no objects to load")
        rclpy.shutdown()
        return

    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("/apply_planning_scene service not available")
        rclpy.shutdown()
        sys.exit(1)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = objects

    req = ApplyPlanningScene.Request()
    req.scene = scene
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None and future.result().success:
        node.get_logger().info(f"Loaded {len(objects)} object(s) into planning scene")
    else:
        node.get_logger().error("Failed to apply planning scene")

    rclpy.shutdown()


if __name__ == "__main__":
    main()