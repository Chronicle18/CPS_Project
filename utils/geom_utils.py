import pybullet as p

def world_to_local(body, world_xyz):
    """Convert WORLD → LOCAL coordinates for an arbitrary point."""
    pos, orn = p.getBasePositionAndOrientation(body)
    inv_pos, inv_orn = p.invertTransform(pos, orn)
    local_xyz, _ = p.multiplyTransforms(inv_pos, inv_orn, world_xyz, [0,0,0,1])
    return local_xyz


def local_to_world(body, local_xyz):
    """Convert LOCAL → WORLD coordinates."""
    pos, orn = p.getBasePositionAndOrientation(body)
    world_xyz, _ = p.multiplyTransforms(pos, orn, local_xyz, [0,0,0,1])
    return world_xyz