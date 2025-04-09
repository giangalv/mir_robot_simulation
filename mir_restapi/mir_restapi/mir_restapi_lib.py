"""
@file mir_restapi_lib.py
@brief Python library for interacting with the MiR robot REST API
@details This module provides classes for HTTP communication with the MiR robot's REST API
         and high-level functions to control and monitor the robot.
"""

import json
import time
import http.client
from datetime import datetime


class HttpConnection:
    """
    @class HttpConnection
    @brief Handles low-level HTTP communication with the MiR REST API
    @details Creates and manages HTTP connections to the MiR robot's REST API,
             providing methods for GET, POST, and PUT requests.
    """
    def __init__(self, logger, address, auth, api_prefix):
        """
        @brief Initialize HTTP connection to the MiR robot
        @param logger Logger object for output messages
        @param address IP address and port of the MiR robot (e.g., "192.168.12.20:80")
        @param auth Authentication token for the REST API
        @param api_prefix API version prefix (e.g., "/api/v2.0.0")
        """
        self.logger = logger
        self.api_prefix = api_prefix
        self.http_headers = {"Accept-Language": "en-EN", "Authorization": auth, "Content-Type": "application/json"}
        try:
            self.logger.info(address)
            self.connection = http.client.HTTPConnection(host=address, timeout=5)
        except Exception as e:
            self.logger.warn('Creation of http connection failed')
            self.logger.warn(str(e))

    def __del__(self):
        """
        @brief Destructor to close the HTTP connection
        """
        if self.is_valid():
            self.connection.close()

    def is_valid(self):
        """
        @brief Check if the HTTP connection is valid
        @return bool True if connection exists, False otherwise
        """
        return self.connection is not None

    def get(self, path):
        """
        @brief Perform HTTP GET request
        @param path API endpoint path
        @return HTTP response object
        """
        if not self.is_valid():
            self.connection.connect()
        self.connection.request("GET", self.api_prefix + path, headers=self.http_headers)
        resp = self.connection.getresponse()
        if resp.status < 200 or resp.status >= 300:
            self.logger.warn("GET failed with status {} and reason: {}".format(resp.status, resp.reason))
        return resp

    def post(self, path, body):
        """
        @brief Perform HTTP POST request
        @param path API endpoint path
        @param body JSON body for the request
        @return Parsed JSON response
        """
        self.connection.request("POST", self.api_prefix + path, body=body, headers=self.http_headers)
        resp = self.connection.getresponse()
        if resp.status < 200 or resp.status >= 300:
            self.logger.warn("POST failed with status {} and reason: {}".format(resp.status, resp.reason))
        return json.loads(resp.read())

    def put(self, path, body):
        """
        @brief Perform HTTP PUT request
        @param path API endpoint path
        @param body JSON body for the request
        @return Parsed JSON response
        """
        self.connection.request("PUT", self.api_prefix + path, body=body, headers=self.http_headers)
        resp = self.connection.getresponse()
        # self.logger.info(resp.read())
        if resp.status < 200 or resp.status >= 300:
            self.logger.warn("POST failed with status {} and reason: {}".format(resp.status, resp.reason))
        return json.loads(resp.read())

    def put_no_response(self, path, body):
        """
        @brief Perform HTTP PUT request without expecting a response
        @param path API endpoint path
        @param body JSON body for the request
        """
        self.connection.request("PUT", self.api_prefix + path, body=body, headers=self.http_headers)


class MirRestAPI:
    """
    @class MirRestAPI
    @brief High-level interface for the MiR robot REST API
    @details Provides methods to control and monitor the MiR robot through its REST API,
             including status checks, mission control, and settings management.
    """
    def __init__(self, logger, hostname, auth=""):
        """
        @brief Initialize the MiR REST API client
        @param logger Logger object for output messages
        @param hostname Hostname or IP address of the MiR robot
        @param auth Authentication token for the REST API
        """
        self.logger = logger
        if hostname is not None:
            address = hostname + ":80"
            print("REST API: Connecting to " + address)
        else:
            address="130.251.13.90:80"
            print("REST API: Connecting to default address " + address)
        self.http = HttpConnection(logger, address, auth, "/api/v2.0.0")

    def close(self):
        """
        @brief Close the HTTP connection to the MiR robot
        """
        self.http.__del__()
        self.logger.info("REST API: Connection closed")

    def is_connected(self, print=True):
        """
        @brief Check if the connection to the MiR robot is active
        @param print Whether to print connection status messages
        @return bool True if connected, False otherwise
        """
        if not self.http.is_valid():
            self.logger.warn('REST API: Http-Connection is not valid')
            return False
        try:
            self.http.connection.connect()
            self.http.connection.close()
            if print:
                self.logger.info("REST API: Connected!")
        except Exception as e:
            if print:
                self.logger.warn('REST API: Attempt to connect failed: ' + str(e))
            return False
        return True

    def is_available(self):
        """
        @brief Check if the MiR robot's REST API is available
        @return bool True if available, False if service unavailable
        """
        status = json.dumps(self.get_status())
        if "service_unavailable" in status:
            return False
        else:
            return True

    def wait_for_available(self):
        """
        @brief Wait until the MiR robot's REST API becomes available
        @details Polls the API until it responds successfully
        """
        while True:
            if self.is_connected(print=False):
                if self.is_available():
                    self.logger.info('REST API: available')
                    break
                else:
                    self.logger.info('REST API: unavailable... waiting')
                    time.sleep(1)

    def get_status(self):
        """
        @brief Get the current status of the MiR robot
        @return dict Status information from the robot
        """
        response = self.http.get("/status")
        return json.loads(response.read())

    def get_state_id(self):
        """
        @brief Get the current state ID of the MiR robot
        @return int State ID (3=Ready, 4=Pause, 11=Manual Control)
        """
        status = self.get_status()
        state_id = status["state_id"]
        return state_id

    
    def set_manual_control(self):
        """
        @brief Set the status of the MiR robot
        @param newStatus New status to set
        @return dict Response from the API
        """
        manual_status ={
            "state_id": 11,
            "web_session_id": "manualControl"
        }
        return self.http.put("/status", json.dumps(manual_status))
    
    def set_pause_control(self):
        """
        @brief Set the status of the MiR robot
        @param newStatus New status to set
        @return dict Response from the API
        """
        pause_status ={
            "state_id": 4,
            "web_session_id": "pause"
        }
        return self.http.put("/status", json.dumps(pause_status))
    
    def set_ready_control(self):
        """
        @brief Set the status of the MiR robot
        @param newStatus New status to set
        @return dict Response from the API
        """
        ready_status ={
            "state_id": 3,
            "web_session_id": "ready"
        }
        return self.http.put("/status", json.dumps(ready_status))
    
# To be fixed and tested
    def set_mapping_mode(self):
        map_mode = {
            "mode_id": 3,
            "web_session_id": "mapping"
        }
        return self.http.put("/status", json.dumps(map_mode))
# To be fixed and tested
    def set_mission_mode(self):
        mission_mode = {
            "mode_id": 7,
            "web_session_id": "mission"
        }
        return self.http.put("/status", json.dumps(mission_mode))
    
    def is_ready(self):
        """
        @brief Check if the MiR robot is in the "Ready" state
        @return bool True if ready, False otherwise
        """
        status = self.get_status()
        if status["state_id"] != 3:  # 3=Ready, 4=Pause, 11=Manualcontrol
            self.logger.warn("MIR currently occupied. System state: {}".format(status["state_text"]))
            return False
        else:
            return True

    def get_all_settings(self, advanced=False, listGroups=False):
        """
        @brief Get all settings from the MiR robot
        @param advanced Whether to get advanced settings
        @param listGroups Whether to get setting groups instead of settings
        @return dict Settings information
        """
        if advanced:
            response = self.http.get("/settings/advanced")
        elif listGroups:
            response = self.http.get("/setting_groups")
        else:
            response = self.http.get("/settings")
        return json.loads(response.read())

    def get_group_settings(self, groupID):
        """
        @brief Get settings for a specific group
        @param groupID ID of the settings group
        @return dict Group settings information
        """
        response = self.http.get("/setting_groups/" + groupID + "/settings")
        return json.loads(response.read())

    def set_setting(self, settingID, settingData):
        """
        @brief Update a specific setting on the MiR robot
        @param settingID ID of the setting to update
        @param settingData New value for the setting
        @return dict Response from the API
        """
        return self.http.put("/setting", json.dumps({settingID: settingData}))

    def sync_time(self):
        """
        @brief Synchronize the MiR robot's time with the host computer
        @details Sets the robot's datetime to the current time of the host
        @return str Response message
        """
        timeobj = datetime.now()
        dT = timeobj.strftime("%Y-%m-%dT%X")
        response = 'REST API: '
        try:
            response += str(self.http.put("/status", json.dumps({'datetime': dT})))
        except Exception as e:
            if str(e) == "timed out":
                # setting datetime over REST API seems not to be intended
                # that's why there is no response accompanying the PUT request,
                # therefore a time out occurs, however time has been set correctly
                response += "Set datetime to " + dT
                self.logger.warn(
                    "REST API: Setting time Mir triggers emergency stop, \
                                  please unlock."
                )
                self.logger.info(response)

                # this is needed, because a timeset restarts the restAPI
                self.wait_for_available()

                return response
        response += " Error setting datetime"
        return response

    def get_distance_statistics(self):
        """
        @brief Get distance statistics from the MiR robot
        @return dict Distance statistics information
        """
        response = self.http.get("/statistics/distance")
        return json.loads(response.read())

    def get_positions(self):
        """
        @brief Get all defined positions from the MiR robot
        @return list List of position definitions
        """
        response = self.http.get("/positions")
        return json.loads(response.read())

    def get_pose_guid(self, pos_name):
        """
        @brief Get the GUID of a position by name
        @param pos_name Name of the position
        @return str GUID of the position or None if not found
        """
        positions = self.get_positions()
        return next((pos["guid"] for pos in positions if pos["name"] == pos_name), None)

    def get_missions(self):
        """
        @brief Get all defined missions from the MiR robot
        @return list List of mission definitions
        """
        response = self.http.get("/missions")
        return json.loads(response.read())

    def get_mission_guid(self, mission_name):
        """
        @brief Get the GUID of a mission by name
        @param mission_name Name of the mission
        @return str GUID of the mission or None if not found
        """
        missions = self.get_missions()
        return next((mis["guid"] for mis in missions if mis["name"] == mission_name), None)

    def get_sounds(self):
        """
        @brief Get all available sounds from the MiR robot
        @return list List of sound definitions
        """
        response = self.http.get("/sounds")
        return json.loads(response.read())

    def move_to(self, position, mission="move_to"):
        """
        @brief Move the MiR robot to a specified position
        @param position Name of the target position
        @param mission Name of the mission to use for movement (default: "move_to")
        @details Executes a mission that moves the robot to the specified position
                 and waits for completion
        """
        mis_guid = self.get_mission_guid(mission)
        pos_guid = self.get_pose_guid(position)

        for (var, txt, name) in zip((mis_guid, pos_guid), ("Mission", "Position"), (mission, position)):
            if var is None:
                self.logger.warn("No {} named {} available on MIR - Aborting move_to".format(txt, name))
                return

        body = json.dumps(
            {
                "mission_id": mis_guid,
                "message": "Externally scheduled mission from the MIR Python Client",
                "parameters": [{"value": pos_guid, "input_name": "target"}],
            }
        )

        data = self.http.post("/mission_queue", body)
        self.logger.info("Mission scheduled for execution under id {}".format(data["id"]))

        while data["state"] != "Done":
            resp = self.http.get("/mission_queue/{}".format(data["id"]))
            data = json.loads(resp.read())
            if data["state"] == "Error":
                self.logger.warn("Mission failed as robot is in error")
                return
            self.logger.info(data["state"])
            time.sleep(2)

        self.logger.info("Mission executed successfully")

    def add_mission_to_queue(self, mission_name):
        """
        @brief Add a mission to the MiR robot's mission queue
        @param mission_name Name of the mission to add
        @return tuple (bool, int) Success status and mission queue ID (-1 if failed)
        @details Adds the specified mission to the queue without waiting for completion
        """
        mis_guid = self.get_mission_guid(mission_name)
        if mis_guid is None:
            self.logger.warn("No Mission named '{}' available on MIR - Aborting move_to".format(mission_name))
            return False, -1

        # put in mission queue
        body = json.dumps(
            {"mission_id": str(mis_guid), "message": "Mission scheduled by ROS node mir_restapi_server", "priority": 0}
        )

        data = self.http.post("/mission_queue", body)
        try:
            self.logger.info("Mission scheduled for execution under id {}".format(data["id"]))
            return True, int(data["id"])
        except KeyError:
            self.logger.warn("Couldn't schedule mission")
            self.logger.warn(str(data))
        return False, -1

    def is_mission_done(self, mission_queue_id):
        """
        @brief Check if a mission in the queue is completed
        @param mission_queue_id ID of the mission in the queue
        @return bool True if mission is done, False otherwise
        @details Checks the status of a specific mission in the queue by its ID
        """
        try:
            # mis_guid = self.get_mission_guid(mission_name)
            response = self.http.get("/mission_queue")

        except http.client.ResponseNotReady or http.client.CannotSendRequest:
            self.logger.info("Http error: Mission with queue_id {} is still in queue".format(mission_queue_id))
            self.http.__del__()
            return False

        # self.logger.info("Mission with queue_id {} is in queue".format(mission_queue_id))
        # self.logger.info("Response status {}".format(response.status))
        data = json.loads(response.read())

        for d in data:
            if d["id"] == mission_queue_id:
                if d["state"] == 'Done':
                    self.logger.info("Mission {} is done".format(mission_queue_id))
                    return True

        self.logger.info("Mission with queue_id {} is still in queue".format(mission_queue_id))
        return False

    def get_system_info(self):
        """
        @brief Get system information from the MiR robot
        @return dict System information including software version, serial number, etc.
        @details Retrieves detailed system information from the robot
        """
        response = self.http.get("/system/info")
        return json.loads(response.read())
