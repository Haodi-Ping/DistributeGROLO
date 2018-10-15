

class message(object):
    def __init__(self):
        self.robot_id = -1

        self.robot_coord = [] # shape is (1,2)
        self.parent1_coord = []
        self.parent2_coord = []
        # triangle extension message
        self.state = -1
        self.root1 = -1
        self.root2 = -1
        self.parent1 = -1
        self.parent2 = -1
        self.parent1_coord = []
        self.parent2_coord = []


    def getMessage(self):
        return

    def sendMessage(self):
        return
