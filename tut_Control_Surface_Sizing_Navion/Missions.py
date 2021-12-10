# Missions.py 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units 
import numpy as np

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def stick_fixed_stability_setup(analyses,vehicle): 
    missions                     = SUAVE.Analyses.Mission.Mission.Container()  
    max_speed_multiplier         = 1.0 # this multiplier is used to compute V_max from V_nominal
    missions.stick_fixed_cruise  = base_mission_setup(vehicle,max_speed_multiplier) 
 
    return missions   

def elevator_sizing_setup(analyses,vehicle): 
    missions = SUAVE.Analyses.Mission.Mission.Container() 
    max_speed_multiplier      = 1.4 # this multiplier is used to compute V_max from V_nominal
    missions.elevator_sizing  = base_mission_setup(vehicle,max_speed_multiplier)   
 
    return missions   

def aileron_rudder_sizing_setup(analyses,vehicle): 
    missions = SUAVE.Analyses.Mission.Mission.Container() 
    max_speed_multiplier      = 1.0     
    missions.aileron_sizing   = base_mission_setup(vehicle,max_speed_multiplier)  
    max_speed_multiplier      = 1.4   # this multiplier is used to compute V_max from V_nominal   
    missions.turn_criteria    = base_mission_setup(vehicle,max_speed_multiplier) 
 
    return missions   
    
def flap_sizing_setup(analyses,vehicle): 
    missions = SUAVE.Analyses.Mission.Mission.Container() 
    max_speed_multiplier     = 1.0      
    missions.flap_sizing     = base_mission_setup(vehicle,max_speed_multiplier)   
    return missions        
    

# ------------------------------------------------------------------
#   Initialize the Mission
# ------------------------------------------------------------------    
    
def base_mission_setup(vehicle,max_speed_multiplier):   
    '''
    This sets up the nominal cruise of the aircraft
    '''
     
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0. * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments 
    
    # base segment
    base_segment = Segments.Segment() 
    base_segment.process.initialize.initialize_battery       = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery 
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.state.numerics.number_control_points        = 4  
 
    #   Cruise Segment: constant Speed, constant altitude 
    segment                           = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                       = "cruise"  
    segment.battery_energy            = vehicle.networks.battery_propeller.battery.max_energy * 0.89
    segment.altitude                  = 8012   * Units.feet
    segment.air_speed                 = 120.91 * Units['mph'] * max_speed_multiplier
    segment.distance                  =  20.   * Units.nautical_mile   
    segment                           = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)    
    mission.append_segment(segment)     
    
    return mission
