#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ariac_msgs.msg import CompetitionState as CompetitionStateMsg
import traceback


class CompetitionInterface(Node):
    """
    Simplified class for monitoring competition state and handling start/end services.
    """

    _competition_states = {
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }

    def __init__(self):
        super().__init__('CompetitionInterface')
        self._logger = self.get_logger()

        try:
            # Create a subscriber for competition state
            self._subscriber = self.create_subscription(
                CompetitionStateMsg,
                "/ariac/competition_state",
                self.competition_state_cb,
                10
            )

            # Create clients for starting and ending competition
            self._start_competition_client = self.create_client(Trigger, "/ariac/start_competition")
            self._end_competition_client = self.create_client(Trigger, "/ariac/end_competition")

            # Store competition state
            self._competition_state = None

            self._logger.info("Competition Interface initialized!")

        except Exception as e:
            self._logger.error(f"Error initializing CompetitionInterface: {e}")
            self._logger.error(traceback.format_exc())

    def competition_state_cb(self, msg: CompetitionStateMsg):
        """
        Callback for receiving the current competition state.
        """
        try:
            if self._competition_state != msg.competition_state:
                self._logger.info(f"Competition state: {self._competition_states[msg.competition_state]}")

            self._competition_state = msg.competition_state

        except Exception as e:
            self._logger.error(f"Error handling competition state update: {e}")
            self._logger.error(traceback.format_exc())


    def start_competition(self):
        """
        Calls the service to start the competition.
        """
        if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
            self._logger.error("Start competition service not available.")
            return

        self._logger.info("Starting competition...")
        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self._logger.info("Competition started successfully.")
        else:
            self._logger.error("Failed to start competition.")

    def end_competition(self):
        """
        Calls the service to end the competition.
        """
        if not self._end_competition_client.wait_for_service(timeout_sec=3.0):
            self._logger.error("End competition service not available.")
            return

        self._logger.info("Ending competition...")
        request = Trigger.Request()
        future = self._end_competition_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self._logger.info("Competition ended successfully.")
        else:
            self._logger.error("Failed to end competition.")






    def get_competition_state(self):
        """Returns the current competition state."""
        return self._competition_state