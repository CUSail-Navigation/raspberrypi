import torch
from torch import nn
import numpy as np

import nav_algo.boat as b

class Actor(nn.Module):
    """ The actor from the Deep Deterministic Policy Gradient (DDPG)
    reinforcement learning algorithm. Given a state, it chooses the best action
    to take (the sail and rudder angles). 
    
    NOTE: The output of the actor is in the range -1 to 1 and must be scaled by 
    the maximum angle ranges for the sail and rudder (90 and 30).
    """

    def __init__(self,
                 state_dim=9,
                 action_dim=2,
                 h1=400,
                 h2=300,
                 init_w=0.003):
        super(Actor, self).__init__()

        self.linear1 = nn.Linear(state_dim, h1)
        self.ln1 = nn.LayerNorm(h1)

        self.linear2 = nn.Linear(h1, h2)
        self.ln2 = nn.LayerNorm(h2)

        self.linear3 = nn.Linear(h2, action_dim)
        self.linear3.weight.data.uniform_(-init_w, init_w)

        self.relu = nn.ReLU()
        self.tanh = nn.Tanh()

    def forward(self, state):
        
        x = self.linear1(state)
        x = self.ln1(x)
        x = self.relu(x)

        x = self.linear2(x)
        x = self.ln2(x)
        x = self.relu(x)

        x = self.linear3(x)
        x = self.tanh(x)
        return x

    def get_action(self, state):
        state.type(torch.FloatTensor)

        action = self.forward(state)
        return action.detach().cpu().numpy()
    
class RL:
    def __init__(self, actor_filename):
        self.actor = Actor()
        self.actor.load_state_dict(torch.load(actor_filename))
        self.actor = self.actor.to(torch.double)
        self.actor.eval()
    
    def step(self, boat : b.BoatController, waypoint):
        # 1. x velocity 
        vel_x = boat.sensors.velocity.x
        # 2. y velocity
        vel_y = boat.sensors.velocity.y
        # TODO 3. angular velocity 
        vel_angular = boat.sensors.angular_velocity
        # 4. sail angle
        sail_angle = boat.servos.currentSail
        # 5. rudder angle 
        rudder_angle = boat.servos.currentTail
        # 6. relative wind x
        rel_wind_x = np.cos(boat.sensors.rawWind*np.pi/180)
        # 7. relative wind y 
        rel_wind_y = np.sin(boat.sensors.rawWind*np.pi/180)
        # 8. distance from goal x component
        boat_position = boat.getPosition()
        dist_goal_x = np.absolute(boat_position.x - waypoint.x)
        # 9. distance from goal y component
        dist_goal_y = np.absolute(boat_position.y - waypoint.y)

        state_vector = np.array([vel_x, vel_y, vel_angular, sail_angle, 
                                 rudder_angle, rel_wind_x, rel_wind_y, 
                                 dist_goal_x, dist_goal_y])
        
        tensor = torch.from_numpy(state_vector)
        output = self.actor.get_action(tensor)

        return output[0]*90, output[1]*30