# Missions.py
# 
# Created:  Jan 2017, M. Vegh
# Modified: 


# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units

import numpy as np

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
    
def setup(analyses):
    
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------
    base_mission = base(analyses)
    missions.base = base_mission


    return missions  

    
def base(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    sol_tol = 1E-12
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'embraer_e190ar test mission'

    # atmospheric model
    atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    planet = SUAVE.Attributes.Planets.Earth()

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    mission.airport = airport
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments    


    # ------------------------------------------------------------------
    #   First Climb Segment: constant Mach, constant segment angle 
    # ------------------------------------------------------------------
    
    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_1"
    
    segment.analyses.extend( analyses.takeoff )
    
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 1.0   * Units.km
    segment.air_speed      = 80. * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']
    segment.state.numerics.tolerance_solution = sol_tol
    # add to misison
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: constant Speed, constant segment angle 
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_2"
    
    segment.analyses.extend( analyses.cruise )
    
    segment.altitude_end   = 2.0   * Units.km
    segment.air_speed      = 100.* Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']
    segment.state.numerics.tolerance_solution = sol_tol
    # add to mission
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------
    #   Third Climb Segment: constant Mach, constant segment angle 
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_3"
    
    segment.analyses.extend( analyses.cruise )
    
    segment.altitude_end = 10. * Units.km
    segment.air_speed    =140* Units['m/s']
    segment.climb_rate   = 3.0    * Units['m/s']
    segment.state.numerics.tolerance_solution = sol_tol
    # add to mission
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------    
    
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude()
    segment.tag = "cruise"
    
    segment.analyses.extend( analyses.cruise )
    
    segment.air_speed  = 230 * Units['m/s']
    segment.distance   = 2000 * Units.nautical_miles
    segment.state.numerics.tolerance_solution = sol_tol
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------    
    #   First Descent Segment: consant speed, constant segment rate
    # ------------------------------------------------------------------    
    
    segment = Segments.Descent.Constant_Speed_Constant_Rate()
    segment.tag = "descent_1"
    
    segment.analyses.extend( analyses.cruise )
    
    segment.altitude_end = 2.   * Units.km
    segment.air_speed    = 120.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']
    segment.state.numerics.tolerance_solution = sol_tol
    
    # add to mission
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------    
    #   Second Descent Segment: consant speed, constant segment rate
    # ------------------------------------------------------------------    

    segment = Segments.Descent.Constant_Speed_Constant_Rate()
    segment.tag = "descent_2"

    segment.analyses.extend( analyses.cruise )
    
    segment.altitude_end = 0.0   * Units.km
    segment.air_speed    = 100.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']
    segment.state.numerics.tolerance_solution = sol_tol
    
    # append to mission
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------
    

    return mission

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    import vehicles
    import analyses
    
    vehicles = vehicles.setup()
    analyses = analyses.setup(vehicles)
    missions = setup(analyses)
    
    vehicles.finalize()
    analyses.finalize()
    missions.finalize()
    
    missions.base.evaluate()