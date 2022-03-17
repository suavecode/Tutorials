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
    
    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)

    return

if __name__ == '__main__': 
    plt.show()