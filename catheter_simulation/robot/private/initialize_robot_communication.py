'''
initialize_robot.py runs all the communication commands that were originially in IDM.py
Performs the initial contact, homing and tensioning.
'''
import sys, time
from robot.devices.xbox_controller.xbox_wrapper import xbox


# The maximum velocity at which the IDMApp will allow us to go (mm / sec).
maxVelMMS = 150.0
# Length, in milliseconds, of a single cycle of the IDM App. 
# remove some time for system delay
cycleTimeS = 4.0 / 1000.0

# Possible states of the IDM
IDMStates = {   "Init" : 0,
                "Initialized" : 1,
                "Powering" : 2,
                "Powered": 3,
                "Homing" : 4,
                "Ready" : 5,
                "Follow" : 6,
                "Tensioning" : 7,
                "Fault" : 8}
# Possible Commands to send to the IDMApp.
IDMCmd = {      "POWERON" : 0,
                "POWEROFF" : 1,
                "START_HOME" : 2,
                "STOP_HOME" : 3,
                "START_FOLLOW" : 6,
                "STOP_FOLLOW" : 7,
                "TENSION_SHEATH" : 8,
                "TENSION_LEADER" : 9,
                "TENSION_BOTH" : 10,
                "TENSION_STOP" : 11}


# key function:
def initialize_system(domainId = 5, # not sure what 5 is doing
                     sheath = "BAS08", 
                     leader = "BAL08", 
                     tension = False):

    # handle to auris communication
    from robot.private.AurisLowLevel import PyAurisInterface
    g_PyAuris = PyAurisInterface() 
    # Setup Network
    err = g_PyAuris.Initialize(domainId, None)
    if (err != 0):
        print("Failed to connect RTI (", err, ")")
        input("Press Enter to exit ...")
        sys.exit(-1)
    g_PyAuris.EstablishIDMConnection()
    if (err != 0):
        print("Failed to connect to IDM (", err, ")")
        input("Press Enter to exit ...")
        sys.exit(-1)

    # home the system and place it in the ready state
    err = GoToReadyState(g_PyAuris)
    if (err != 0):
        print("Failed to enter Ready State.")
        input("Press Enter to exit ...")
        sys.exit(-1)

    tension_sheath = False
    tension_leader = False
    if tension:
        input("Load Catheter Now. Press enter to load and tension ...\n")
        tension_input = input("Load Catheter Now. Do you want to tension the sheath? (y/n)...\n")
        tension_sheath = True if tension_input.lower() == 'y' else False
        leader_input = input("Do you want to tension the leader too? (y/n)...\n")
        tension_leader = True if leader_input.lower() == 'y' else False

    err = LoadCathetersAndFollow(g_PyAuris, sheath = sheath, leader = leader, tension_sheath = tension_sheath, tension_leader = tension_leader)
    if (err != 0):
        print("Failed to Load Catheters.")
        input("Press Enter to exit ...")
        sys.exit(-1)

    xbox_handle = InitializeGamePad(g_PyAuris)

    return g_PyAuris, xbox_handle
    

#### IDM States functions #####
def StateFromInt(idmState):
  """ Returns a string IDM state given the integer number of the state.
      Returns an empty string if no such state is found. """
  keys = [i for key, value in IDMStates.items() if value == idmState]
  if len(keys) != 1:
      return ""
  else:
      return keys[0]

def IntFromState(idmState):
  """ Returns an integer IDM state given the string name of the state.
      Throws KeyError. """
  return IDMStates[idmState]

def CmdFromInt(idmCmd):
  """ Returns a string IDM state given the integer number of the state.
      Returns an empty string if no such state is found. """
  keys = [value for key, value in IDMCmd.items() if value == idmCmd]
  if len(keys) != 1:
      return ""
  else:
      return keys[0]

def IntFromCmd(idmCmd):
  """ Returns an integer IDM state given the string name of the state.
      Throws KeyError. """
  return IDMCmd[idmCmd]


################# Basic IDM Commands to Initialize ######################################

def LoadLeader(g_PyAuris, instrument):
    """ Loads a leader instrument. """
    return g_PyAuris.LoadLeader(instrument)

def LoadSheath(g_PyAuris, instrument):
    """ Loads a sheath instrument. """
    return g_PyAuris.LoadSheath(instrument)

def EnablePower(g_PyAuris):
    """ Sends enable power command to the IDM. """
    return g_PyAuris.SendIDMCommand(IntFromCmd("POWERON"))

def DisablePower(g_PyAuris):
    """ Sends disable power command to the IDM. """
    return g_PyAuris.SendIDMCommand(IntFromCmd("POWEROFF"))

def StartHoming(g_PyAuris):
    """ Sends start homing the IDM. """
    return g_PyAuris.SendIDMCommand(IntFromCmd("START_HOME"))

def StartFollow(g_PyAuris, timeout=2000):
    """ Figures out how to get IDM to start following (ie, be able to accept
        commands).
        Args:
        timeout: milliseconds of timeout. """
    end_time = time.time() + timeout / 1000.0
    while time.time() < end_time:
        status = GetStatus()
        print(repr(status))
        if status == IDMStates["Follow"]:
            return 0
        elif status == IDMStates["Fault"]:
            print("IDM in Fault state.")
            return -1
        elif status == IDMStates["Ready"]:
            g_PyAuris.SendIDMCommand(IDMCmd["START_FOLLOW"])
            # Wait a moment to let the follow command take effect.
            time.sleep(0.6)
            return 0
        elif status == IDMStates["Init"] or status == IDMStates["Initialized"]:
            # Sometimes there are issues when we just started a connection and
            #   still need some time to get the data.
            time.sleep(0.6)
            if status == IDMStates["Init"] or status == IDMStates["Initialized"]:
                g_PyAuris.SendIDMCommand(IDMCmd["POWERON"])
        elif status == IDMStates["Powered"]:
            g_PyAuris.SendIDMCommand(IDMCmd["START_HOME"])
        time.sleep(0.1)

def InitializeGamePad(g_PyAuris):
    """ Turn the game pad on. Must init system first. """
    xbox_handle = xbox(g_PyAuris)
    return xbox_handle

def InitializeSystemMinimally(g_PyAuris, domainId):
    # Setup Network
    err = g_PyAuris.Initialize(domainId, None)
    if (err != 0):
        print("Failed to connect RTI (", err, ")")
        input("Press Enter to exit ...")
        sys.exit(-1)
    g_PyAuris.EstablishIDMConnection()
    if (err != 0):
        print("Failed to connect to IDM (", err, ")")
        input("Press Enter to exit ...")
        sys.exit(-1)



def GetStatus(g_PyAuris):
    """ Returns the current IDM status (int). """
    return g_PyAuris.GetIDMStatus()

def GoToReadyState(g_PyAuris):
    """ Moves IDM into Ready State so we can next load a catheter. """
    print("Putting IDM in Ready State")
    state = "START"
    done = False
    printOnce = True
    while (not done):
        sys.stdout.flush()
        time.sleep(0.01)
        status = GetStatus(g_PyAuris)
        if (state == "START"):
            # figure out where we are at start
            if (status == IDMStates["Ready"]):
                state = "READY"
                continue
            elif (status == IDMStates["Init"]):
                if (printOnce):
                    print("Waiting for system to Initialize.")
                    printOnce = False
                continue
            elif (status == IDMStates["Initialized"]):
                print("System Initialized. Enabling Power.")
                g_PyAuris.SendIDMCommand(IntFromCmd("POWERON"))
                state = "DO_POWER"
                continue
            elif (status == IDMStates["Powering"] \
                    or status == IDMStates["Powered"]):
                print("Waiting for System to Power.")
                state = "DO_POWER"
                continue
            elif (status == IDMStates["Homing"]):
                print("Waiting for System to Home.")
                state = "DO_HOME"
                continue
            elif (status == IDMStates["Follow"]):
                print("Stopping Follow Master.")
                g_PyAuris.SendIDMCommand(IntFromCmd("STOP_FOLLOW"))
                state = "DO_STOP"
                continue
            elif (status == IDMStates["Fault"]):
                print("System is in Fault. Please restart the IDMApp",
                        "before running this script")
                return -1
            else:
                print("Not sure what to do in state (", StateFromInt(status), ")")
                return -1
        elif (state == "DO_POWER"):
            if (status == IDMStates["Powered"]):
                print("System Powered. Doing Homing.\n")
                input("Press enter to home. Make sure the catheter module is disconnected.\n")
                # print(g_PyAuris.GetMotorCurrents())
                g_PyAuris.SendIDMCommand(IntFromCmd("START_HOME"))
                state = "DO_HOME"
            continue
        elif (state == "DO_HOME"):
            if (status == IDMStates["Ready"]):
                # print("System Homed.\n")
                state = "READY"
            continue
        elif (state == "DO_STOP"):
            if (status == IDMStates["Ready"]):
                print("System Stopped.\n")
                state = "READY"
            continue
        elif (state == "READY"):
            if (status == IDMStates["Ready"]):
                print("System Ready.\n")
                return 0
        else:
            print("Unknown State (", state,
                    "): status ("+ StateFromInt(status), ")")

def LoadCathetersAndFollow(g_PyAuris, sheath = 'BS38', leader = 'BL36', tension_sheath = False, tension_leader = False):
    """ Load the catheter tension and start follow. """
    print("Loading Catheters.")
    state = "START"
    done = False
    printOnce = True
    while (not done):
        sys.stdout.flush()
        time.sleep(0.01)
        status = GetStatus(g_PyAuris)
        if (state == "START"):
            if (status != IDMStates["Ready"]):
                print("System must be in Ready State.")
                return -1
            else:
                state = "LOAD_LEADER"
            continue
        elif (state == "LOAD_LEADER"):
            if leader:
                print("Loading Leader.")
                err = LoadLeader(g_PyAuris, leader)
                if (err != 0):
                    print("Failed to Load Leader (%s)." % (str(err)));
                    return -1
            state = "LOAD_SHEATH"
            continue
        elif (state == "LOAD_SHEATH"):
            if sheath:
                print("Loading Sheath.")
                err = LoadSheath(g_PyAuris, sheath)
                if (err != 0):
                    print("Failed to Load Sheath (%s)." % (str(err)));
                    return -1
            state = "DO_TENSION"
            continue
        elif (state == "DO_TENSION"):
            if tension_leader or tension_sheath:
              print("Catheters Selected. Tensioning.")
            else:
                state = "WAIT_FOLLOW"
                continue
            # inserted the tension_leader flag to skip over this step if only using the sheath
            if tension_leader:
                print("Tensioning Leader ...")
                sys.stdout.flush()
                # LEADER TENSIONING
                g_PyAuris.SendIDMCommand(IntFromCmd("TENSION_LEADER"))
                while (True):
                    if (g_PyAuris.GetLeaderTensionStatus() == 2):
                        state = "WAIT_FOLLOW"
                        time.sleep(0.05)
                        break
            if tension_sheath:
                print("Tensioning Sheath ...")
                # print("D'application de tension a le gaine ...")
                sys.stdout.flush()
                g_PyAuris.SendIDMCommand(IntFromCmd("TENSION_SHEATH"))
                while (True):
                    # print(g_PyAuris.GetSheathTensionStatus())
                    if (g_PyAuris.GetSheathTensionStatus() == 2):
                        # print("a:    ", g_PyAuris.GetSheathTensionStatus())
                        state = "WAIT_FOLLOW"
                        time.sleep(0.05)
                        break
            continue
        elif (state == "WAIT_FOLLOW"):
            if (status == IDMStates["Ready"]):
                g_PyAuris.SendIDMCommand(IntFromCmd("START_FOLLOW"))
                state = "DO_FOLLOW"
            continue
        elif (state == "DO_FOLLOW"):
            if (status == IDMStates["Ready"]):
                time.sleep(0.5)
                g_PyAuris.SendIDMCommand(IntFromCmd("START_FOLLOW"))
            if (status == IDMStates["Follow"]):
                print("System Ready to drive.")
                return 0
        else:
            print("Unknown state ("+ state +"): status ("+ StateFromInt(status) +")")