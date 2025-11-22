import pybullet as p


class Actor:
    def __init__(self, actor_cfg):
        self.cfg = actor_cfg
    
    def get_mass(self):
        dyn = p.getDynamicsInfo(self.body_id, -1)
        return dyn[0]
    

class Car(Actor):
    pass

class Cube(Actor):
    pass