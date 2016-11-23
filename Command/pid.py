# Under MIT License, see LICENSE.txt
"""
    Gestion de l'asservissement en position des robots.
    Implémente un régulateur proportionnel, intégrale et dérivatif.
    La commande à la sortie est filtré en moyennant les 5 dernières commandes.
"""

import math

from ..Util.Position import Position
from ..Util.Pose import Pose


DEFAULT_STATIC_GAIN = 0.115 / 1000
DEFAULT_INTEGRAL_GAIN = 0.159 / 1000
DEFAULT_DERIVATIVE_GAIN = 0.021 / 1000
DEFAULT_THETA_GAIN = 0
MAX_INTEGRAL_PART = 0.55


class PID(object):
    """
        Asservissement PID en position

        u = up + ui + ud
    """

    def __init__(self, static_gain=DEFAULT_STATIC_GAIN, integral_gain=DEFAULT_INTEGRAL_GAIN, differential_gain=DEFAULT_DERIVATIVE_GAIN, theta_gain=DEFAULT_THETA_GAIN):
        self.accumulator_x = 0
        self.accumulator_y = 0
        self.static_gain = static_gain
        self.integral_gain = integral_gain
        self.differential_gain = differential_gain
        self.ktheta = theta_gain
        self.last_command_x = []
        self.last_command_y = []
        self.last_error_x = 0
        self.last_error_y = 0

    def update_pid_and_return_speed_command(self, position_command):
        """ Met à jour les composants du pid et retourne une commande en vitesse. """
        if position_command.stop_cmd:
            return Pose(Position(0, 0))

        r_x, r_y = position_command.pose.position.x, position_command.pose.position.y
        t_x, t_y = position_command.player.pose.position.x, position_command.player.pose.position.y
        e_x = r_x - t_x
        e_y = r_y - t_y

        # composante proportionnel
        up_x = self.static_gain * e_x
        up_y = self.static_gain * e_y

        # composante integrale
        ui_x = self.integral_gain * e_x
        ui_y = self.integral_gain * e_y
        self.accumulator_x = self.accumulator_x + ui_x
        self.accumulator_y = self.accumulator_y + ui_y

        # composante diff
        ud_x = self.differential_gain * (e_x - self.last_error_x)
        ud_y = self.differential_gain * (e_y - self.last_error_y)

        self.accumulator_x = self.accumulator_x if self.accumulator_x < MAX_INTEGRAL_PART else MAX_INTEGRAL_PART
        self.accumulator_y = self.accumulator_y if self.accumulator_y < MAX_INTEGRAL_PART else MAX_INTEGRAL_PART
        self.accumulator_x = self.accumulator_x if self.accumulator_x > -MAX_INTEGRAL_PART else -MAX_INTEGRAL_PART
        self.accumulator_y = self.accumulator_y if self.accumulator_y > -MAX_INTEGRAL_PART else -MAX_INTEGRAL_PART
        if position_command.player.id == 0:
            print("X INT: {} -- Y INT: {}".format(self.accumulator_x, self.accumulator_y))

        u_x = up_x + self.accumulator_x + ud_x
        u_y = up_y + self.accumulator_y + ud_y

        # correction frame referentiel et saturation
        orientation = position_command.player.pose.orientation
        pos_x, pos_y = _correct_for_referential_frame(u_x, u_y, orientation)

        # correction de theta
        e_theta = 0 - orientation
        theta = self.ktheta * e_theta
        # theta = theta if theta < MAX_THETA_CMD else MAX_THETA_CMD
        # theta = theta if theta > MIN_THETA_CMD else MIN_THETA_CMD

        self.last_error_x = e_x
        self.last_error_y = e_y

        cmd = Pose(Position(pos_x, pos_y), theta)

        return cmd


def _correct_for_referential_frame(pos_x, pos_y, orientation):
    cos = math.cos(-orientation)
    sin = math.sin(-orientation)

    corrected_x = (pos_x * cos - pos_y * sin)
    corrected_y = (pos_y * cos + pos_x * sin)
    return corrected_x, corrected_y
