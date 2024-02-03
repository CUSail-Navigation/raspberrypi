def fleetRace(self):
    # While the configuration is in fleet race mode, read servo angles
    # over the radio
    self.configuration.write_output(
        "Starting Fleet Race\nSend angles of the form 'sail_angle rudder_angle'\n"
    )
    while self.configuration.radio.fleetRace:
        try:
            self.configuration.radio.receiveString()  # timeout is 1 sec
        except:
            pass
        self.configuration.boat.updateSensors()
        self.configuration.write_data()