"""
@file mir_restapi_server.py
@brief ROS2 node that provides services to interact with the MiR REST API
@details This node creates a bridge between ROS2 services and the MiR REST API,
         allowing other ROS2 nodes to control and monitor the MiR robot.
"""

import time
import sys

import rclpy
from rclpy.node import Node

import mir_restapi.mir_restapi_lib
from std_srvs.srv import Trigger
from mir_msgs.srv import ExecMission
from rcl_interfaces.msg import SetParametersResult


class MirRestAPIServer(Node):
    """
    @class MirRestAPIServer
    @brief ROS2 node that provides services to interact with the MiR REST API
    @details Creates services that allow other ROS2 nodes to control and monitor
             the MiR robot through its REST API.
    """
    def __init__(self):
        """
        @brief Constructor for the MirRestAPIServer class
        @details Initializes the node, declares parameters, and sets up the API handle
        """
        super().__init__('mir_restapi_server')
        self.get_logger().info("started")

        # parameters: hostname, api_token
        self.declare_parameter('mir_hostname', "")
        self.hostname = self.get_parameter('mir_hostname').get_parameter_value().string_value
        self.declare_parameter('mir_restapi_auth', "")
        self.auth = self.get_parameter('mir_restapi_auth').get_parameter_value().string_value
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.api_handle = None

        self.setup_api_handle()

        if self.api_handle is None:
            self.get_logger().warn(
                """
            Hostname and API token are not set! Run as follows:

            ros2 run mir_restapi mir_restapi_server
            --ros-args -p mir_hostname:='MIR_IP_ADDR' -p mir_restapi_auth:='YOUR_API_KEY'
            """
            )

    def setup_api_handle(self):
        """
        @brief Sets up the MiR REST API handle
        @details Creates a MirRestAPI object if hostname and auth token are set
                 and creates the ROS2 services
        """
        if self.hostname != "" and self.auth != "":
            self.api_handle = mir_restapi.mir_restapi_lib.MirRestAPI(self.get_logger(), self.hostname, self.auth)
            self.get_logger().info("created MirRestAPI handle")
            self.create_services()
            self.get_logger().info("created services")

    def parameters_callback(self, params):
        """
        @brief Callback for parameter changes
        @param params List of parameters that have changed
        @return SetParametersResult indicating success
        """
        for param in params:
            if param.name == "mir_restapi_auth":
                self.get_logger().info("Received auth token")
                self.auth = param.value
            if param.name == "mir_hostname":
                self.get_logger().info("Set mir hostname")
                self.hostname = param.value
        self.setup_api_handle()
        return SetParametersResult(successful=True)

    def create_services(self):
        """
        @brief Creates all the ROS2 services for interacting with the MiR REST API
        @details Sets up services for time synchronization, status retrieval, sound control,
                 emergency halt checking, mission execution, and system information retrieval
        """
        self.create_service(Trigger, 'mir_sync_time', self.sync_time_callback)
        self.get_logger().info("Listening on 'mir_sync_time'")

        self.create_service(Trigger, 'mir_get_status', self.get_status_callback)
        self.get_logger().info("Listening on 'mir_get_status'")

        self.create_service(Trigger, 'mir_get_sounds', self.get_sounds_callback)
        self.get_logger().info("Listening on 'mir_get_sounds'")

        self.create_service(Trigger, 'mir_is_emergency_halt', self.is_emergency_halt_callback)
        self.get_logger().info("Listening on 'mir_is_emergency_halt'")

        self.create_service(Trigger, 'mir_get_missions', self.get_missions_callback)
        self.get_logger().info("Listening on 'mir_get_missions'")

        self.create_service(ExecMission, 'mir_execute_mission', self.exec_mission_callback)
        self.get_logger().info("Listening on 'mir_execute_mission'")

        self.create_service(Trigger, 'mir_honk', self.honk_callback)
        self.get_logger().info("Listening on 'mir_honk'")

        self.create_service(Trigger, 'mir_get_system_info', self.get_system_info_callback)
        self.get_logger().info("Listening on 'mir_get_system_info'")

        self.create_service(Trigger, 'mir_get_settings', self.get_settings_callback)
        self.get_logger().info("Listening on 'mir_get_settings'")

        self.create_service(Trigger, 'mir_set_manual_control', self.set_manual_control_callback)
        self.get_logger().info("Listening on 'mir_set_manual_control'")

    def test_api_connection(self):
        """
        @brief Tests the connection to the MiR REST API
        @return -1 if API handle is None, 0 if connection failed, 1 if connection successful
        """
        if self.api_handle is None:
            return -1

        self.get_logger().info('REST API: Waiting for connection')
        i = 1
        while not self.api_handle.is_connected():
            if not rclpy.ok():
                sys.exit(0)
            if i > 5:
                self.get_logger().error('REST API: Could not connect, giving up')
                return 0
            i += 1
            time.sleep(1)
        return 1

    def reponse_api_handle_not_exists(self, response):
        """
        @brief Sets the response for when the API handle does not exist
        @param response The response object to modify
        @return The modified response object
        """
        response.success = False
        response.message = 'API token and/or hostname not set yet'
        self.get_logger().error(response.message)
        return response

    def call_restapi_function(self, service_fct, request, response, args=None):
        """
        @brief Calls a MiR REST API function and sets the response accordingly
        @param service_fct The MiR REST API function to call
        @param request The request object
        @param response The response object to modify
        @param args Optional arguments for the service function
        @return The modified response object
        """
        if self.test_api_connection() == -1:
            response = self.reponse_api_handle_not_exists(response)
            return response
        if self.api_handle.is_connected(print=False):
            if args is None:
                response.message = str(service_fct())
            else:
                response.message = str(service_fct(args))
            if "Error" in response.message:
                response.success = False
            else:
                response.success = True
            return response
        else:
            response.success = False
            response.message = "ERROR: Couldn't connect to REST API"
        self.get_logger().error(response.message)
        return response
    
    def set_manual_control_callback(self, request, response):
        self.get_logger().info('Setting manual control mode...')
        response = self.call_restapi_function(self.api_handle.set_manual_control, request, response)
        return response
        
        
    def sync_time_callback(self, request, response):
        """
        @brief Callback for the mir_sync_time service
        @param request The request object
        @param response The response object to modify
        @return The modified response object
        """
        self.get_logger().info('Syncing host time with REST API...')
        response = self.call_restapi_function(self.api_handle.sync_time, request, response)
        return response

    def get_status_callback(self, request, response):
        """
        @brief Callback for the mir_get_status service
        @param request The request object
        @param response The response object to modify
        @return The modified response object
        """
        self.get_logger().info('Getting status from REST API...')
        response = self.call_restapi_function(self.api_handle.get_status, request, response)
        return response

    def get_sounds_callback(self, request, response):
        """
        @brief Callback for the mir_get_sounds service
        @param request The request object
        @param response The response object to modify
        @return The modified response object
        """
        self.get_logger().info('Getting sounds from REST API...')
        response = self.call_restapi_function(self.api_handle.get_sounds, request, response)
        return response

    def is_emergency_halt_callback(self, request, response):
        """
        @brief Callback for the mir_is_emergency_halt service
        @param request The request object
        @param response The response object to modify
        @return The modified response object with message set to "True" if in emergency halt, "False" otherwise
        """
        self.get_logger().info('Checking REST API for emergency halt...')
        response = self.call_restapi_function(self.api_handle.get_state_id, request, response)

        if response.success:
            state_id = int(response.message)
            #  self.get_logger().info("Returned state_id as %i" % state_id)
            STATE_ID_EMERGENCY = 10
            if state_id == STATE_ID_EMERGENCY:
                response.message = str(True)
                self.get_logger().info("Emergency Halt")
            else:
                response.message = str(False)
                # self.get_logger().info("no emergency halt")
        return response

    def get_missions_callback(self, request, response):
        """
        @brief Callback for the mir_get_missions service
        @param request The request object
        @param response The response object to modify
        @return The modified response object
        """
        self.get_logger().info('Getting missions from REST API...')
        response = self.call_restapi_function(self.api_handle.get_missions, request, response)
        return response

    def honk_callback(self,request,response):
        """
        @brief Callback for the mir_honk service
        @param request The request object
        @param response The response object to modify
        @return The modified response object
        @details Executes the "honk" mission
        """        
        req = ExecMission.Request()
        req.mission_name = "honk"
        resp = ExecMission.Response()

        self.exec_mission_callback(req, resp)
        response.success = resp.success
        return response

    def exec_mission_callback(self, request, response):
        """
        @brief Callback for the mir_execute_mission service
        @param request The request object containing the mission name
        @param response The response object to modify
        @return The modified response object
        @details Adds the mission to the queue, checks for emergency halt,
                 executes the mission, and waits for completion
        """
        mission_name = request.mission_name

        queue_success, mission_queue_id = self.api_handle.add_mission_to_queue(mission_name)
        if not queue_success:
            response.message = "Execution Mission '{}' failed due to mission queue error".format(mission_name)
            self.get_logger().error(response.message)
            response.success = False
            return response
        self.get_logger().info("Put mission {} into queue with mission_queue_id={}".format(mission_name,
            mission_queue_id))

        emerg_response = self.is_emergency_halt_callback(request, response)
        if emerg_response.message == str(True):
            response.message = "Can't execute mission, emergency halt"
            self.get_logger().error(response.message)
            response.success = False
        else:
            response.message = "Executing mission '{}'".format(mission_name)
            self.get_logger().info(response.message)
            STATE_ID_RUN_MISSION = 3
            STATE_ID_PAUSE = 4

            self.api_handle.set_state_id(STATE_ID_RUN_MISSION)

            while not self.api_handle.is_mission_done(mission_queue_id):
                time.sleep(1)

            self.api_handle.set_state_id(STATE_ID_PAUSE)
            self.api_handle.http.__del__()
            response.success = True
        return response

    def get_system_info_callback(self, request, response):
        """
        @brief Callback for the mir_get_system_info service
        @param request The request object
        @param response The response object to modify
        @return The modified response object with system information
        @details Retrieves system information from the MiR robot through the REST API
        """
        self.get_logger().info('Getting system info from REST API...')
        response = self.call_restapi_function(self.api_handle.get_system_info, request, response)
        return response

    def get_settings_callback(self, request, response):
        """
        @brief Callback for the mir_get_settings service
        @param request The request object
        @param response The response object to modify
        @return The modified response object with robot settings
        @details Retrieves all settings from the MiR robot through the REST API
        """
        self.get_logger().info('Getting settings from REST API...')
        response = self.call_restapi_function(self.api_handle.get_all_settings, request, response)
        return response


def main(args=None):
    """
    @brief Main function for the MiR REST API server node
    @param args Command line arguments
    @details Initializes the ROS2 node, creates a MirRestAPIServer instance, and spins the node
    """
    rclpy.init(args=args)

    mir_restapi_server = MirRestAPIServer()

    rclpy.spin(mir_restapi_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
