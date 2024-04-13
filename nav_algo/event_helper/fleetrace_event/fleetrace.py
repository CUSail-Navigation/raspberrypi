# TODO: figure out what else to import

def fleetRace(NavigationController):
    # While the configuration is in fleet race mode, read servo angles
    # over the radio
    NavigationController.configuration.write_output(
        "Starting Fleet Race\nSend angles of the form 'sail_angle rudder_angle'\n"
    )
    while NavigationController.configuration.radio.fleetRace:
        try:
            NavigationController.configuration.radio.receiveString()  # timeout is 1 sec
        except:
            pass
        NavigationController.configuration.boat.updateSensors()
        NavigationController.configuration.write_data()
