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
        rospy.Subscriber('/robocheckers/human_turn', String, self.receive_human_turn)

        # Prepare human_turn tuple attribute
        self.human_turn_tuples = None
        self.human_move_list = None
        # Prepare player_order attribute, 0 -> Human go first, 1 -> AI go first
        self.player_order = None


    def send_ai_move(self, move, options):
        """
            Method for sending data to game_interface node
            tuple move:
            list of tuples options:
        """
        move = self.tuple_move_to_list(move)
        options_tuples = []
        for option in options:
            options_tuples.append(self.tuple_move_to_list(option))

        msg = '|'.join(str(e) for e in [move, options_tuples])

        self.ai_move.publish(msg)


    def receive_human_turn(self, data):
        """
            Method for receiving data from game_interface node
        """
        self.human_turn_tuples = eval(data.data)


    def get_human_move_list(self):
        """
            Method for recalculating move data from 2-coords to 1-coord
        """
        while not self.human_turn_tuples:
            continue
        rospy.sleep(0.001)

        # Calculate the square number from coordinations
        human_turn = []
        for pos in self.human_turn_tuples:
            print "Run:", pos
            tmp = (pos[1]-1)*4 + (pos[0]+1-(pos[1]%2))/2
            human_turn.append(tmp)

        # If player_order is 1, rotate the chessboard
        if self.player_order:
            human_turn[0] = 33 - human_turn[0]
            human_turn[1] = 33 - human_turn[1]

        # Change individual positions through the move to list of moves
        self.human_move_list = []
        for i in range(len(human_turn)-1):
            self.human_move_list.append((human_turn[i], human_turn[i+1]))

        self.human_turn_tuples = None
        return self.human_move_list


    def tuple_move_to_list(self, ai_move):
        """
            Method for recalculating move data from tuple of ints to 2 list of tuples
        """
        ai_move_list = []
        for pos in ai_move:
            y = int((pos-1)/4+1)
            x = ((pos-1)%4+1)*2-1+y%2

            # If player_order is 1, rotate the chessboard
            if self.player_order:
                x = 9-x
                y = 9-y

            ai_move_list.append((x, y))

        return ai_move_list


    def get_starting_player(self):
        """
            Method to determine which player go first
        """
        if self.player_order != None:
            return None

        while True:
            while not self.human_turn_tuples:
                continue
            rospy.sleep(0.001)

            if (0, 1) in self.human_turn_tuples:
                self.player_order = 1
                break
            elif (0, 0) in self.human_turn_tuples:
                self.player_order = 0
                break
            else:
                print "ERROR: Invalid starting message! Expecting '[(0, 0)]' or '[(0, 1)]'!"
                self.human_turn_tuples = None

        self.human_turn_tuples = None

        return self.player_order
