# Under MIT License, see LICENSE.txt
"""
    Point de départ du moteur pour l'intelligence artificielle. Construit les
    objets nécessaires pour maintenir l'état du jeu, acquiert les frames de la
    vision et construit la stratégie. Ensuite, la stratégie est exécutée et un
    thread est lancé qui contient une boucle qui se charge de l'acquisition des
    frames de la vision. Cette boucle est la boucle principale et appel le
    prochain état du **Coach**.
"""
from collections import namedtuple
import signal
import threading
import time

from .Game.Game import Game
from .Game.Referee import Referee
from .Communication.vision import Vision
from .Communication.referee import RefereeServer
from .Communication.udp_server import GrSimCommandSender, DebugCommandSender,\
                                      DebugCommandReceiver
from .Communication.serial_command_sender import SerialCommandSender
from .Command.command import Stop
from .Command.pid import PID
from .Util.exception import StopPlayerError
from .Util.constant import TeamColor

LOCAL_UDP_MULTICAST_ADDRESS = "224.5.23.2"
UI_DEBUG_MULTICAST_ADDRESS = "127.0.0.1"
AI_DELTA_TIME = 0.180

GameState = namedtuple('GameState', ['field', 'referee', 'friends',
                                     'enemies', 'timestamp', 'debug'])

class Framework(object):
    """
        La classe contient la logique nécessaire pour démarrer une logique et
        mettre en place l'état du jeu.
    """

    def __init__(self, is_team_yellow=False):
        """ Constructeur de la classe, établis les propriétés de bases. """
        # TODO: refactor pour avoir des objets qui contiennent les petits
        # détails d'implémentations (12/7 fields)
        self.command_sender = None
        self.debug_sender = None
        self.debug_receiver = None
        self.game = None
        self.ai_coach = None
        self.referee = None
        self.running_thread = False
        self.last_frame_number = 0
        self.thread_terminate = threading.Event()
        self.timestamp = 0
        self.vision = None
        self.last_ai_iteration_timestamp = 0
        self.robots_pi = [PID(), PID(), PID(), PID(), PID(), PID()]

        self.ia_coach_mainloop = None
        # callable pour mettre la couleur de l'équipe dans l'IA lors de la création de la partie (create_game)
        self.ia_coach_initializer = None
        self.is_team_yellow = self._sanitize_team_color(is_team_yellow)

    def create_game(self):
        """
            Créé l'arbitre et la Game. De plus initialize(team color) l'IA(à être refractorer out - MGL 2016/11/08).

            :return: Game, le **GameState**
        """

        self.referee = Referee()
        self.ia_coach_initializer(self.is_team_yellow)

        self.game = Game(self.referee, self.is_team_yellow == TeamColor.YELLOW_TEAM)

        return self.game


    def update_game_state(self):
        """ Met à jour le **GameState** selon la vision et l'arbitre. """
        # TODO: implémenter correctement la méthode
        pass

    def update_players_and_ball(self, vision_frame):
        """ Met à jour le GameState selon la frame de vision obtenue. """
        time_delta = self._compute_vision_time_delta(vision_frame)
        self.game.update(vision_frame, time_delta)

    def _is_frame_number_different(self, vision_frame):
        if vision_frame is not None:
            return vision_frame.detection.frame_number != self.last_frame_number
        else:
            return False

    def _compute_vision_time_delta(self, vision_frame):
        self.last_frame_number = vision_frame.detection.frame_number
        this_time = vision_frame.detection.t_capture
        time_delta = this_time - self.timestamp
        self.timestamp = this_time
        # FIXME: hack
        # print("frame: %i, time: %d, delta: %f, FPS: %d" % \
        #        (vision_frame.detection.frame_number, this_time, time_delta, 1/time_delta))
        return time_delta

    def update_strategies(self):
        """ Change l'état du **Coach** """

        game_state = self.get_game_state()

        # FIXME: il y a probablement un refactor à faire avec ça
        # state = self.referee.command.name
        state = "NORMAL_START"
        if state == "NORMAL_START":
            return self.ia_coach_mainloop(game_state)

        elif state == "STOP":
            # TODO implement
            pass
            # self.ai_coach.stop(game_state)

    def get_game_state(self):
        """ Retourne le **GameState** actuel. *** """

        game = self.game
        return GameState(
            field=game.field,
            referee=game.referee,
            friends=game.friends,
            enemies=game.enemies,
            timestamp=self.timestamp,
            debug=self.debug_receiver.receive_command()
        )

    def start_game(self, p_ia_coach_mainloop, p_ia_coach_initializer, async=False, serial=False):
        """ Démarrage du moteur de l'IA initial. """

        # on peut eventuellement demarrer une autre instance du moteur
        # TODO: method extract -> _init_communication_serveurs()
        if not self.running_thread:
            if serial:
                self.command_sender = SerialCommandSender()
            else:
                self.command_sender = GrSimCommandSender("127.0.0.1", 20011)

            self.debug_sender = DebugCommandSender(UI_DEBUG_MULTICAST_ADDRESS, 20021)
            self.debug_receiver = DebugCommandReceiver(UI_DEBUG_MULTICAST_ADDRESS, 10021)
            self.referee = RefereeServer(LOCAL_UDP_MULTICAST_ADDRESS)
            self.vision = Vision(LOCAL_UDP_MULTICAST_ADDRESS)
        else:
            self.stop_game()

        self.ia_coach_mainloop = p_ia_coach_mainloop
        self.ia_coach_initializer = p_ia_coach_initializer
        self.create_game()

        signal.signal(signal.SIGINT, self._sigint_handler)
        self.running_thread = threading.Thread(target=self.game_thread_main_loop)
        self.running_thread.start()

        if not async:
            self.running_thread.join()

    def game_thread_main_loop(self):
        """ Fonction exécuté et agissant comme boucle principale. """

        self._wait_for_first_frame()

        # TODO: Faire arrêter quand l'arbitre signal la fin de la partie
        while not self.thread_terminate.is_set():
            # TODO: method extract
            # Mise à jour
            current_vision_frame = self._acquire_vision_frame()
            if self._is_frame_number_different(current_vision_frame):
                self.update_game_state()
                self.update_players_and_ball(current_vision_frame)

                if self.timestamp - self.last_ai_iteration_timestamp > AI_DELTA_TIME:

                    robot_commands, debug_commands = self.update_strategies()
                    self._send_robot_commands(robot_commands)
                    self._send_debug_commands(debug_commands)

    def _acquire_vision_frame(self):
        return self.vision.get_latest_frame()

    def stop_game(self):
        """
            Nettoie les ressources acquises pour pouvoir terminer l'exécution.
        """
        self.thread_terminate.set()
        self.running_thread.join()
        self.thread_terminate.clear()
        try:
            if self.is_team_yellow == TeamColor.YELLOW_TEAM:
                team = self.game.yellow_team
            else:
                team = self.game.blue_team

            for player in team.players:
                command = Stop(player)
                self.command_sender.send_command(command)
        except:
            print("Could not stop players")
            raise StopPlayerError("Au nettoyage il a été impossible d'arrêter\
                                    les joueurs.")

    def _wait_for_first_frame(self):
        while not self.vision.get_latest_frame():
            time.sleep(0.01)
            print("En attente d'une image de la vision.")

    def _send_robot_commands(self, commands):
        """ Envoi les commades des robots au serveur. """
        for idx, command in enumerate(commands):
            pi_cmd = self.robots_pi[idx].update_pid_and_return_speed_command(command)
            command.pose = pi_cmd
            self.command_sender.send_command(command)

    def _send_debug_commands(self, debug_commands):
        """ Envoie les commandes de debug au serveur. """
        if debug_commands:
            self.debug_sender.send_command(debug_commands)

    def _sigint_handler(self, signum, frame):
        self.stop_game()

    @staticmethod
    def _sanitize_team_color(p_is_team_yellow):
        if p_is_team_yellow:
            return TeamColor.YELLOW_TEAM
        else:
            return TeamColor.BLUE_TEAM
