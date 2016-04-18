# Plot_Mission.py
# 
# Created:  Feb 2016, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

from SUAVE.Core import Units
import pylab as plt

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results,line_style='bo-'):
    
    axis_font = {'fontname':'Arial', 'size':'14'}  
    
    # ------------------------------------------------------------------    
    #   Battery Energy
    # ------------------------------------------------------------------
    plt.figure("Battery Energy")
    axes = plt.gca()    
    for i in range(len(results.segments)):     
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        energy = results.segments[i].conditions.propulsion.battery_energy[:,0] 
        axes.plot(time, energy, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Battery Energy (J)')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False) 
    axes.grid(True)   
    plt.savefig("opt_battery_energy.png")
    
    # ------------------------------------------------------------------    
    #   Solar Flux
    # ------------------------------------------------------------------
    plt.figure("Solar Flux")
    axes = plt.gca()    
    for i in range(len(results.segments)):     
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        energy = results.segments[i].conditions.propulsion.solar_flux[:,0] 
        axes.plot(time, energy, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Solar Flux ($W/m^{2}$)')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)     
    axes.grid(True)    
    plt.savefig("opt_solar_flux.png")
    
    # ------------------------------------------------------------------    
    #   Battery Draw
    # ------------------------------------------------------------------
    plt.figure("Battery Charging")
    axes = plt.gca()    
    for i in range(len(results.segments)):     
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        energy = results.segments[i].conditions.propulsion.battery_draw[:,0] 
        axes.plot(time, energy, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Battery Charging (Watts)')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)     
    axes.grid(True)    
    plt.savefig("opt_battery_draw.png")
    
    plt.show()     

    return

if __name__ == '__main__': 
    main()    
    plt.show()