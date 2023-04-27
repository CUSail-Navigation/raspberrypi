The configuration of an instance of the navigation algorithm must be set using
a configuration file. This file sets whether the sensors and servos are real
or fake, what version of the algorithm is being used (reinforcement learning 
model or the old navigation algorithm), what event is being run, and whether
output is printed to the terminal only or to the radio as well.

The configuration file follows the JSON format. It should look like the 
example shown below (follow this format EXACTLY). The options for each field
are shown in brackets. Look at example.json to see a functional example.

NOTE: the "model_path" field is only required if "rl" (reinforcement learning
algorithm) is used. It gives the path to the pickle file that holds that actor 
neural net.

{
    "peripherals":{
        "gps": ["fake" or "real"],
        "imu": ["fake" or "real"],
        "anemometer": ["fake" or "real"],
        "servos": ["fake" or "real"]
    },
    "algo":{
        "event": ["endurance", "station keeping", "precision navigation", 
                  "collision avoidance", "search", "fleet race", or "none"],
        "type": ["rl" or "basic"],
        "model_path": "../example/path/to/best_actor.pickle"
    },
    "output": ["radio" or "terminal"]
}
