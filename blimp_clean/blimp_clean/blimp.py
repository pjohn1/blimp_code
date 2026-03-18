
class Blimp(object):
    def __init__(self, id_, port, goal=None):
        self.id = id_
        self.port = port
        self.goal = goal

    def write_motor_commands(self, voltages):
        msg = MotorMsg()
        msg.id = self.id
        msg.voltages = voltages
        self.publisher.publish(msg)
    
    def fly_at_velocity(self, velocity):
        return