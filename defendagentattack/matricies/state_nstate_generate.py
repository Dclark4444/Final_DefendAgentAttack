import numpy as np
from ament_index_python.packages import get_package_share_directory

import os

def main():

    # Fetch Actions and states (saved files)
    share = get_package_share_directory('defendagentattack')

    action_pth = os.path.join(share, 'matrices', 'action.txt')
    actions = np.loadtxt(action_pth)

    state_pth = os.path.join(share, 'matrices', 'states.txt')
    states = np.loadtxt(state_pth)

    None 
    # do something here to generate the data




if __name__ == "__main__":
    main()