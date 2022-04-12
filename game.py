"""
    This module implements the game playing harness.
"""
# Andrew Edwards -- almostimplemented.com
# =======================================
# Harness for running a checkers match.
#
# Last updated: July 21, 2014

import sys
import rospy

import checkers
import agent
import arthur
import robocheckers_link as robolink

BLACK, WHITE = 0, 1

def main():
    # Communication and CPU definition
    node = robolink.RosComm()
    cpu = agent.CheckersAgent(arthur.move_function)

    # Determin who starts, 0 -> Human go first, 1 -> AI go first
    choice = node.get_starting_player()

    turn = 0
    B = checkers.CheckerBoard()
    current_player = B.active
    
    while not B.is_over():
        print B
        if turn % 2 == choice:
            legal_moves = B.get_moves()

            if B.jump:
                print "Make jump."
                print ""
            else:
                print "Turn %i" % (turn + 1)
                print ""
            for (i, move) in enumerate(get_move_strings(B)):
                print "Move " + str(i) + ": " + str(move)
            while True:
                #move_idx = raw_input("Enter your move number: ")
                print node.get_human_move()
                if node.human_move in get_move_strings(B):
                    print "YES"
                else:
                    print "NO"


                #try:
                #    move_idx = int(move_idx)
                #except ValueError:
                #    print "Please input a valid move number."
                #    continue
                #if move_idx in range(len(legal_moves)):
                #    break
                #else:
                #    print "Please input a valid move number."
                #    continue
            #B.make_move(legal_moves[move_idx])
            # If jumps remain, then the board will not update current player
            if B.active == current_player:
                print "Jumps must be taken."
                continue
            else:
                current_player = B.active
                turn += 1
        else:
            B.make_move(cpu.make_move(B))
            if B.active == current_player:
                print "Jumps must be taken."
                continue
            else:
                current_player = B.active
                turn += 1
    print B
    if B.active == WHITE:
        print "Congrats Black, you win!"
    else:
        print "Congrats White, you win!"
    return 0


def game_over(board):
    return len(board.get_moves()) == 0


def get_mandatory_moves(board):
    jumps_bin = []
    jumps = []
    # Pop unused bits
    for i, move in enumerate(board.mandatory_jumps):
        move = abs(move)
        jumps_bin.append(move & (0xFF))
        jumps_bin[i] = jumps_bin[i] | ((move & (0xFF << 9)) >> 1)
        jumps_bin[i] = jumps_bin[i] | ((move & (0xFF << 18)) >> 2)
        jumps_bin[i] = jumps_bin[i] | ((move & (0xFF << 27)) >> 3)

        pom = 0
        # Find both ones in bin representation of piece and write them as tuple
        for j in range(32):
            if jumps_bin[i] & (1 << j):
                if pom:
                    jumps.append((pom, j + 1))
                    jumps.append((j + 1, pom))
                    break
                pom = j + 1
        
    return jumps


def remove_moves_by_mandatory(mandatory_jumps, jumps):
    if not jumps:
        return None

    for jump in jumps:
        if jump not in mandatory_jumps:
            jumps.remove(jump)
    return jumps


def get_move_strings(board):
    rfj = board.right_forward_jumps()
    lfj = board.left_forward_jumps()
    rbj = board.right_backward_jumps()
    lbj = board.left_backward_jumps()

    if (rfj | lfj | rbj | lbj) != 0:
        rfj = [(1 + i - i//9, 1 + (i + 8) - (i + 8)//9)
                    for (i, bit) in enumerate(bin(rfj)[::-1]) if bit == '1']
        lfj = [(1 + i - i//9, 1 + (i + 10) - (i + 8)//9)
                    for (i, bit) in enumerate(bin(lfj)[::-1]) if bit == '1']
        rbj = [(1 + i - i//9, 1 + (i - 8) - (i - 8)//9)
                    for (i, bit) in enumerate(bin(rbj)[::-1]) if bit ==  '1']
        lbj = [(1 + i - i//9, 1 + (i - 10) - (i - 10)//9)
                    for (i, bit) in enumerate(bin(lbj)[::-1]) if bit == '1']

        if board.mandatory_jumps:
            mandatory_jumps = get_mandatory_moves(board)
            remove_moves_by_mandatory(mandatory_jumps, rfj)
            remove_moves_by_mandatory(mandatory_jumps, lfj)
            remove_moves_by_mandatory(mandatory_jumps, rbj)
            remove_moves_by_mandatory(mandatory_jumps, lbj)


        if board.active == BLACK:
            return rfj + lfj + rbj + lbj
        else:
            return rfj + lfj + rbj + lbj


    rf = board.right_forward()
    lf = board.left_forward()
    rb = board.right_backward()
    lb = board.left_backward()

    rf = [(1 + i - i//9, 1 + (i + 4) - (i + 4)//9)
                for (i, bit) in enumerate(bin(rf)[::-1]) if bit == '1']
    lf = [(1 + i - i//9, 1 + (i + 5) - (i + 5)//9)
                for (i, bit) in enumerate(bin(lf)[::-1]) if bit == '1']
    rb = [(1 + i - i//9, 1 + (i - 4) - (i - 4)//9)
                for (i, bit) in enumerate(bin(rb)[::-1]) if bit ==  '1']
    lb = [(1 + i - i//9, 1 + (i - 5) - (i - 5)//9)
                for (i, bit) in enumerate(bin(lb)[::-1]) if bit == '1']

    if board.active == BLACK:
        return rf + lf + rb + lb
    else:
        return rf + lf + rb + lb

if __name__ == '__main__':
    try:
        status = main()
        sys.exit(status)
    except KeyboardInterrupt:
        print ""
        print "Game terminated."
        sys.exit(1)
