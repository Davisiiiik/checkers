#!/usr/bin/env python
"""
    This module implements the game playing harness.
"""
# Andrew Edwards -- almostimplemented.com
# =======================================
# Harness for running a checkers match.
#
# Last updated: July 21, 2014
#
# =======================================
# David Lichosyt -- vut.cz
#
# Last modified: April 28, 2022

import sys
from numpy import log2

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
    player_order = node.get_starting_player()

    turn = 0
    B = checkers.CheckerBoard()
    current_player = B.active

    while not B.is_over():
        if turn % 2 == player_order:
            if player_order == 0 and turn == 0:
                print B
                legal_move_tuples = get_move_tuples(B)

                for (i, move) in enumerate(legal_move_tuples):
                    print "Move " + str(i) + ": " + str(move)

                node.send_ai_move([], legal_move_tuples, robolink.OPTS)

            while True:
                human_move_list = node.get_human_move_list()

                print "human_move_list:", human_move_list

                if check_human_turn(B, human_move_list):
                    print "Turn is legal"
                    break
                else:
                    print "ERROR: Turn is not legal!"
                    node.send_ai_move((-1, -1), get_move_tuples(B), robolink.OPTS)
                    continue

            # Move for every move in move_list
            for human_move in human_move_list:
                legal_moves = B.get_moves()
                move_idx = get_move_tuples(B).index(human_move)
                B.make_move(legal_moves[move_idx])
                print B

        else:
            # AI Move
            ai_move_int = cpu.make_move(B, 5)
            ai_move = int_move_to_tuple(B, ai_move_int)
            B.make_move(ai_move_int)

            legal_moves = B.get_moves()
            legal_move_tuples = get_move_tuples(B)

            print B
            for (i, move) in enumerate(legal_move_tuples):
                print "Move " + str(i) + ": " + str(move)

            node.send_ai_move(ai_move, legal_move_tuples, robolink.MOVE + robolink.OPTS)

        # If jumps remain, then the board will not update current player
        if B.active == current_player:
            print "Jumps must be taken."
            if turn % 2 == player_order:
                node.send_ai_move([], get_move_tuples(B),robolink.OPTS)
            continue
        else:
            current_player = B.active
            turn += 1

    print B
    if B.active == WHITE:
        print "Congrats Black, you win!"
    else:
        print "Congrats White, you win!"

    # Send message about win
    node.send_ai_move([(B.active, player_order)], [], robolink.NONE)
    return 0


def game_over(board):
    return len(board.get_moves()) == 0


def pop_unused_bits(move):
    tmp = move & (0xFF)
    tmp = tmp | ((move & (0xFF << 9)) >> 1)
    tmp = tmp | ((move & (0xFF << 18)) >> 2)
    tmp = tmp | ((move & (0xFF << 27)) >> 3)
    return tmp


def int_move_to_tuple(board, move):
    # FUNCTION MUST BE USED BEFORE MOVE!!!
    move = abs(move)

    # Determine which is piece and which is destination position
    piece = move & (board.pieces[0] | board.pieces[1])
    dest = move & ~piece

    # Pop unused bits and get bit positions
    piece = int(log2(pop_unused_bits(piece)) + 1)
    dest = int(log2(pop_unused_bits(dest)) + 1)

    return (piece, dest)


def get_mandatory_moves(board):
    jumps = []
    for move in board.mandatory_jumps:
        jumps.append(int_move_to_tuple(board, move))

    return jumps


def check_human_turn(board, human_move_list):
    for human_move in human_move_list:
        print "Checking human_move:", human_move
        # Get legal moves
        legal_moves = board.get_moves()

        # Check if move is valid, if not, return False
        try:
            move_idx = get_move_tuples(board).index(human_move)
        except ValueError:
            print "ERROR: Move is not legal!"
            return False
        else:
            print "Move is legal"

        # Update testing board
        board = board.peek_move(legal_moves[move_idx])

    return True


def get_move_tuples(board):
    if board.mandatory_jumps:
        return get_mandatory_moves(board)

    rfj = board.right_forward_jumps()
    lfj = board.left_forward_jumps()
    rbj = board.right_backward_jumps()
    lbj = board.left_backward_jumps()

    if (rfj | lfj | rbj | lbj) != 0:
        rfj = [(1 + i - i//9, 1 + (i + 8) - (i + 8)//9)
                    for (i, bit) in enumerate(bin(rfj)[::-1]) if bit == '1']
        lfj = [(1 + i - i//9, 1 + (i + 10) - (i + 10)//9)
                    for (i, bit) in enumerate(bin(lfj)[::-1]) if bit == '1']
        rbj = [(1 + i - i//9, 1 + (i - 8) - (i - 8)//9)
                    for (i, bit) in enumerate(bin(rbj)[::-1]) if bit ==  '1']
        lbj = [(1 + i - i//9, 1 + (i - 10) - (i - 10)//9)
                    for (i, bit) in enumerate(bin(lbj)[::-1]) if bit == '1']

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
