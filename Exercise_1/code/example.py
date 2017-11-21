from jointCtlComp import *
from taskCtlComp import *

# Controller in the joint space. The robot has to reach a fixed position.
jointCtlComp(['P'], True)
jointCtlComp(['PD'], True)
jointCtlComp(['PID'], True)
jointCtlComp(['PD_Grav'], True)
jointCtlComp(['ModelBased'], True)

# Same controller, but this time the robot has to follow a fixed trajectory.

# jointCtlComp(['P'], False)
# jointCtlComp(['PD'], False)
jointCtlComp(['PID'], False)
# jointCtlComp(['PD_Grav'], False)
# jointCtlComp(['ModelBased'], False)

# Controller in the task space.
# taskCtlComp(['JacNullSpace'], resting_pos=np.mat([0, pi]).T)

raw_input('Finish?')
