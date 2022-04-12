"""
    This module is used for communication between almostimplemented game
    engine and RoboCheckers node
"""

import rospy
from std_msgs.msg import String

class RosComm(object):
    """
        Class initializing ROS Node with one publisher and one subscriber
        for communication with game_intergace node
    """
    def __init__(self):
        # Node initialization
        rospy.init_node('game_logic')
        self.ai_move = rospy.Publisher('/robocheckers/ai_move', String, queue_size=10)
        rospy.Subscriber('/robocheckers/human_move', String, self.get_human_move_raw)

        # Prepare human_move tuple attribute
        self.human_move_raw = None
        self.human_move = None
        # Prepare choice attribute, 0 -> Human go first, 1 -> AI go first
        self.choice = None


    def send_ai_move(self, move, options):
        """
            Method for sending data to game_interface node
            tuple move:
            list of tuples options:
        """
        test = [move, options]
        msg = '|'.join(str(e) for e in test)

        self.ai_move.publish(msg)


    def get_human_move_raw(self, data):
        """
            Method for receiving data from game_interface node
        """
        self.human_move_raw = eval(data.data)


    def get_human_move(self):
        """
            Method for recalculating move data from 2-coords to 1-coord
        """
        while not self.human_move_raw:
            continue
        rospy.sleep(0.001)

        # Calculate the square number from coordinations
        self.human_move = []
        for pos in self.human_move_raw:
            tmp = (pos[1]-1)*4 + (pos[0]+1-(pos[1]%2))/2
            self.human_move.append(tmp)

        # If choice is 1, rotate the chessboard
        if self.choice:
            self.human_move[0] = 33 - self.human_move[0]
            self.human_move[1] = 33 - self.human_move[1]

        self.human_move_raw = None
        self.human_move = tuple(self.human_move)
        return self.human_move


    def get_starting_player(self):
        """
            Method to determine which player go first
        """
        if self.choice != None:
            return None

        while not self.human_move_raw:
            continue
        rospy.sleep(0.001)

        if (-1, -1) in self.human_move_raw:
            self.choice = 1
            self.human_move_raw = None
        else:
            self.choice = 0

        return self.choice

if __name__ == '__main__':
    node = RosComm()

    while True:
        while not node.human_move: continue
        rospy.sleep(0.001)
        print node.human_move
        
        node.human_move = ()

        node.send_ai_move((13, 2), [(23, 14), (25, 16), (7, 18), (29, 30)])

        if raw_input("Send `q` to quit: ") == "q":
            break
