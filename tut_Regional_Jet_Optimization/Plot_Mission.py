# Plot_Mission.py
# 
# Created:  May 2015, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units
from SUAVE.Plots.Performance.Mission_Plots import *
import pylab as plt

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(nexus,line_style='bo-'):

    results   = nexus.results.base

    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)
    
    # Drag Components
    plot_drag_components(results, line_style)

    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)

    return

if __name__ == '__main__': 
    main()    
    plt.show()