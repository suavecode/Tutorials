# Plot_Mission.py
# 
# Created:  Feb 2016, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Core import Units
import pylab as plt

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results,line_style='bo-'):
    
    # Plot Solar Conditions 
    plot_solar_flux(results)
    
    # Plot Aircraft Electronics
    plot_electronic_conditions(results)

    return

if __name__ == '__main__': 
    main()    
    plt.show()