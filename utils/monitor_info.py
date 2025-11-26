import pybullet as p

def hasLanded(car, plane, ramp, task_states):
    # car_pos, car_orn = p.getBasePositionAndOrientation(car)
    # _, pitch, _ = p.getEulerFromQuaternion(car_orn)

    if len(p.getContactPoints(car, ramp)) > 0:
        # print("On Ramp at step:", time_step)
        task_states["launched"] = False
        task_states["ascend"] = True
    
    if task_states["ascend"] and not task_states["launched"] and len(p.getContactPoints(car, plane)) == 0:
        # print("Airborne at step:", time_step)
        task_states["launched"] = True
    
    if task_states["ascend"] and task_states["launched"] and len(p.getContactPoints(car, plane)) > 0 and not task_states["landed"]:
        # print("Landed at step:", time_step)
        task_states["landed"] = True
        # time_step = MAX_STEPS - 200     
        return True
    
    return False
