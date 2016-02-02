# Procedure.py
# 
# Created:  May 2015, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
import copy
from SUAVE.Analyses.Process import Process
from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
from SUAVE.Methods.Geometry.Three_Dimensional.compute_span_location_from_chord_length import compute_span_location_from_chord_length
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity import compute_component_centers_of_gravity
from SUAVE.Methods.Center_of_Gravity.compute_aircraft_center_of_gravity import compute_aircraft_center_of_gravity
from SUAVE.Methods.Aerodynamics.Fidelity_Zero.Lift.compute_max_lift_coeff import compute_max_lift_coeff


# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    procedure = Data()
    procedure.simple_sizing = simple_sizing
    
    # find the weights
    procedure.weights = weight
    # finalizes the data dependencies
    procedure.finalize = finalize
    
    # performance studies
    procedure.missions                   = Process()
    procedure.missions.design_mission    = design_mission
    #procedure.missions.short_field       = short_field_mission
    #procedure.missions.max_range_mission = max_range_mission
    
    # calculate field lengths
    procedure.takeoff_field_length       = takeoff_field_length
    procedure.landing_field_length       = landing_field_length
    #procedure.short_takeoff_field_length = short_takeoff_field_length
     
    # evaluate noise
    procedure.noise                      = noise
    
    # post process the results
    procedure.post_process = post_process
        
    # done!
    return procedure


# ----------------------------------------------------------------------        
#   Target Range Function
# ----------------------------------------------------------------------    

def find_target_range(nexus,mission):
    
    segments=mission.segments
    cruise_altitude=mission.segments['climb_5'].altitude_end
    climb_1=segments['climb_1']
    climb_2=segments['climb_2']
    climb_3=segments['climb_3']
    climb_4=segments['climb_4']
    climb_5=segments['climb_5']
  
    descent_1=segments['descent_1']
    descent_2=segments['descent_2']
    descent_3=segments['descent_3']

    x_climb_1=climb_1.altitude_end/np.tan(np.arcsin(climb_1.climb_rate/climb_1.air_speed))
    x_climb_2=(climb_2.altitude_end-climb_1.altitude_end)/np.tan(np.arcsin(climb_2.climb_rate/climb_2.air_speed))
    x_climb_3=(climb_3.altitude_end-climb_2.altitude_end)/np.tan(np.arcsin(climb_3.climb_rate/climb_3.air_speed))
    x_climb_4=(climb_4.altitude_end-climb_3.altitude_end)/np.tan(np.arcsin(climb_4.climb_rate/climb_4.air_speed))
    x_climb_5=(climb_5.altitude_end-climb_4.altitude_end)/np.tan(np.arcsin(climb_5.climb_rate/climb_5.air_speed))
    x_descent_1=(climb_5.altitude_end-descent_1.altitude_end)/np.tan(np.arcsin(descent_1.descent_rate/descent_1.air_speed))
    x_descent_2=(descent_1.altitude_end-descent_2.altitude_end)/np.tan(np.arcsin(descent_2.descent_rate/descent_2.air_speed))
    x_descent_3=(descent_2.altitude_end-descent_3.altitude_end)/np.tan(np.arcsin(descent_3.descent_rate/descent_3.air_speed))
    
    cruise_range=mission.design_range-(x_climb_1+x_climb_2+x_climb_3+x_climb_4+x_climb_5+x_descent_1+x_descent_2+x_descent_3)
  
    segments['cruise'].distance=cruise_range
    
    return nexus

# ----------------------------------------------------------------------        
#   Design Mission
# ----------------------------------------------------------------------    
def design_mission(nexus):
    
    mission = nexus.missions.base
    mission.design_range = 1500.*Units.nautical_miles
    find_target_range(nexus,mission)
    results = nexus.results
    results.base = mission.evaluate()
    
    return nexus



# ----------------------------------------------------------------------        
#   Sizing
# ----------------------------------------------------------------------    

def simple_sizing(nexus):
    configs=nexus.vehicle_configurations
    base=configs.base
    Sref=m_guess/base.wing_loading
    base.reference_area=Sref
    
    for config in configs:
        
        config.mass_properties.max_zero_fuel = nexus.MZFW_ratio*config.mass_properties.max_takeoff
        configs=config.wings['main_wing'].areas.reference=base.reference_area
        config.wings.horizontal_stabilizer.areas.reference = (26.0/92.0)*config.wings.main_wing.areas.reference
            
        for wing in config.wings:
            
            wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)
            
            wing.areas.exposed  = 0.8 * wing.areas.wetted
            wing.areas.affected = 0.6 * wing.areas.reference
            
        # fuselage seats
        #config.fuselages['fuselage'].number_coach_seats = config.passengers


        air_speed   = nexus.missions.base.segments['cruise'].air_speed 
        altitude    = nexus.missions.base.segments['climb_5'].altitude_end
        atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
        
        p, T, rho, a, mu = atmosphere.compute_values(altitude)
        
        mach_number = air_speed/a

        turbofan_sizing(config.propulsors['turbo_fan'], mach_number, altitude)
        
        #vehicle_configurations.short_field_takeoff.wings.main_wing.flaps.angle
        
        # diff the new data
        config.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = nexus.vehicle_configurations.landing
    landing_conditions = Data()
    landing_conditions.freestream = Data()

    # landing weight
    landing.mass_properties.landing = 0.85 * config.mass_properties.takeoff
    
    # Landing CL_max
    altitude = nexus.missions.base.segments[-1].altitude_end
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    p, T, rho, a, mu = atmosphere.compute_values(altitude)
    landing_conditions.freestream.velocity           = nexus.missions.base.segments['descent_3'].air_speed
    landing_conditions.freestream.density            = rho
    landing_conditions.freestream.dynamic_viscosity  = mu/rho
    CL_max_landing,CDi = compute_max_lift_coeff(landing,landing_conditions)
    landing.maximum_lift_coefficient = CL_max_landing
    # diff the new data
    landing.store_diff()
    
    
    #Takeoff CL_max
    takeoff = nexus.vehicle_configurations.takeoff
    takeoff_conditions = Data()
    takeoff_conditions.freestream = Data()    
    altitude = nexus.missions.takeoff.airport.altitude
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    p, T, rho, a, mu = atmosphere.compute_values(altitude)
    takeoff_conditions.freestream.velocity           = nexus.missions.takeoff.segments.climb_1.air_speed
    takeoff_conditions.freestream.density            = rho
    takeoff_conditions.freestream.dynamic_viscosity  = mu/rho 
    max_CL_takeoff,CDi = compute_max_lift_coeff(takeoff,takeoff_conditions) 
    takeoff.maximum_lift_coefficient = max_CL_takeoff
    
    takeoff.store_diff()
    
   

    #Base config CL_max
    base = nexus.vehicle_configurations.base
    base_conditions = Data()
    base_conditions.freestream = Data()    
    altitude = nexus.missions.base.airport.altitude
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    p, T, rho, a, mu = atmosphere.compute_values(altitude)
    base_conditions.freestream.velocity           = nexus.missions.base.segments.climb_1.air_speed
    base_conditions.freestream.density            = rho
    base_conditions.freestream.dynamic_viscosity  = mu/rho 
    max_CL_base,CDi = compute_max_lift_coeff(base,base_conditions) 
    base.maximum_lift_coefficient = max_CL_base    
    base.store_diff()
    
    # done!
    
    return nexus

# ----------------------------------------------------------------------        
#   Weights
# ----------------------------------------------------------------------    

def weight(nexus):
    vehicle=nexus.vehicle_configurations.base
    
    
    # weight analysis
    weights = nexus.analyses.base.weights.evaluate()
    
    
    compute_component_centers_of_gravity(vehicle)
    nose_load_fraction=.06
    compute_aircraft_center_of_gravity(vehicle,nose_load_fraction)
   
    
    weights = nexus.analyses.cruise.weights.evaluate()
    weights = nexus.analyses.landing.weights.evaluate()
    weights = nexus.analyses.takeoff.weights.evaluate()
    weights = nexus.analyses.short_field_takeoff.weights.evaluate()
    for config in nexus.vehicle_configurations:
        config.mass_properties.zero_fuel_center_of_gravity=vehicle.mass_properties.zero_fuel_center_of_gravity
        config.fuel                                       =vehicle.fuel
    return nexus


# ----------------------------------------------------------------------
#   Finalizing Function (make part of optimization nexus)[needs to come after simple sizing doh]
# ----------------------------------------------------------------------    

def finalize(nexus):
    
    nexus.analyses.finalize()   
    
    return nexus         

       # ----------------------------------------------------------------------
#   Takeoff Field Length Evaluation
# ----------------------------------------------------------------------    
    
def takeoff_field_length(nexus):
    
    # import tofl analysis module
    estimate_tofl = SUAVE.Methods.Performance.estimate_take_off_field_length
    
    # unpack data
    results  = nexus.results
    summary = nexus.summary
    analyses = nexus.analyses
    missions = nexus.missions
    config   = nexus.vehicle_configurations.takeoff
    
    # defining required data for tofl evaluation
    takeoff_airport = missions.takeoff.airport    
    ref_weight      = config.mass_properties.takeoff  

    takeoff_field_length,second_segment_climb_gradient_takeoff = estimate_tofl(config,analyses, takeoff_airport,1)
    
    # pack results
    summary.takeoff_field_length = takeoff_field_length
    summary.second_segment_climb_gradient_takeoff = second_segment_climb_gradient_takeoff
    

    return nexus

# ----------------------------------------------------------------------
#   landing Field Length Evaluation
# ----------------------------------------------------------------------    
    
def landing_field_length(nexus):
    
    # import tofl analysis module
    estimate_landing = SUAVE.Methods.Performance.estimate_landing_field_length
    
    # unpack data
    results  = nexus.results
    summary = nexus.summary
    analyses = nexus.analyses
    missions = nexus.missions
    config   = nexus.vehicle_configurations.landing
    
    # defining required data for tofl evaluation
    takeoff_airport = missions.takeoff.airport    
    ref_weight      = config.mass_properties.landing  
    
    landing_field_length = estimate_landing(config,analyses, takeoff_airport)
    
    # pack results
    summary.landing_field_length = landing_field_length

    return nexus



# ----------------------------------------------------------------------
#   Noise Evaluation
# ----------------------------------------------------------------------    

def noise(nexus):
    
    # unpack noise analysis
    evaluate_noise = SUAVE.Methods.Noise.Correlations.shevell
    
    # unpack data
    vehicle = nexus.vehicle_configurations.base
    results = nexus.results
    summary = nexus.summary
    mission_profile = results.base
    
    weight_landing    = mission_profile.segments[-1].conditions.weights.total_mass[-1,0]
    number_of_engines = vehicle.propulsors['turbo_fan'].number_of_engines
    thrust_sea_level  = vehicle.propulsors['turbo_fan'].design_thrust
    thrust_landing    = mission_profile.segments[-1].conditions.frames.body.thrust_force_vector[-1,0]
    
    # evaluate
    summary.noise = evaluate_noise( weight_landing    , 
                              number_of_engines , 
                              thrust_sea_level  , 
                              thrust_landing     )
    
    return nexus  

    
# ----------------------------------------------------------------------
#   Post Process Results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    # Unpack data
    vehicle               = nexus.vehicle_configurations.base  
    short_vehicle         = nexus.vehicle_configurations.short_field_takeoff
    results               = nexus.results
    summary = nexus.summary
    missions              = nexus.missions  
    
    # Static stability calculations
    CMA = -10.
    for segment in results.base.segments.values():
        max_CMA=np.max(segment.conditions.stability.static.cm_alpha[:,0])
        if max_CMA>CMA:
            CMA=max_CMA
            
    for segment in results.max_range.segments.values():
        max_CMA=np.max(segment.conditions.stability.static.cm_alpha[:,0])
        if max_CMA>CMA:
            CMA=max_CMA  
            
    for segment in results.short_field.segments.values():
        max_CMA=np.max(segment.conditions.stability.static.cm_alpha[:,0])
        if max_CMA>CMA:
            CMA=max_CMA    
            
    summary.static_stability = CMA
    
    #throttle in design mission
    max_throttle=0
    for segment in results.base.segments.values():
        max_segment_throttle = np.max(segment.conditions.propulsion.throttle[:,0])
        if max_segment_throttle > max_throttle:
            max_throttle = max_segment_throttle
    for segment in results.max_range.segments.values():
        max_segment_throttle = np.max(segment.conditions.propulsion.throttle[:,0])
        if max_segment_throttle > max_throttle:
            max_throttle = max_segment_throttle            
    for segment in results.short_field.segments.values():
        max_segment_throttle = np.max(segment.conditions.propulsion.throttle[:,0])
        if max_segment_throttle > max_throttle:
            max_throttle = max_segment_throttle 
            
    summary.max_throttle = max_throttle
    
    # Fuel margin and base fuel calculations

    operating_empty          = vehicle.mass_properties.operating_empty
    payload                  = vehicle.passenger_weights.mass_properties.mass 
    short_landing_weight     = results.short_field.segments[-1].conditions.weights.total_mass[-1]
    max_range_landing_weight = results.max_range.segments[-1].conditions.weights.total_mass[-1]
    design_landing_weight    = results.base.segments[-1].conditions.weights.total_mass[-1]
    design_takeoff_weight    = vehicle.mass_properties.takeoff
    max_takeoff_weight       = nexus.vehicle_configurations.takeoff.mass_properties.max_takeoff
    max_zero_fuel_weight     = nexus.vehicle_configurations.base.mass_properties.max_zero_fuel
        
    #summary.base_fuel = design_takeoff_weight - design_landing_weight     
    summary.max_zero_fuel_margin    = (design_landing_weight - max_zero_fuel_weight)/max_zero_fuel_weight
    summary.short_field_fuel_margin = (short_landing_weight - max_zero_fuel_weight)/max_zero_fuel_weight
    summary.max_range_fuel_margin   = (max_range_landing_weight - max_zero_fuel_weight)/max_zero_fuel_weight
    summary.MZFW_consistency        = (max_zero_fuel_weight - (operating_empty + payload))/ max_zero_fuel_weight

    #addition of mission and reserves fuelburn
    #base
    #summary.base_mission_fuelburn   = results.base.segments[0].conditions.weights.total_mass[0] - results.base.segments['descent_3'].conditions.weights.total_mass[-1]
    summary.base_mission_fuelburn   = design_takeoff_weight - results.base.segments['descent_3'].conditions.weights.total_mass[-1]
    summary.base_reserve_fuelburn   = results.base.segments['descent_3'].conditions.weights.total_mass[-1] - results.base.segments[-1].conditions.weights.total_mass[-1]

    #max range
    summary.maxrange_mission_fuelburn   = max_takeoff_weight - results.max_range.segments['descent_3'].conditions.weights.total_mass[-1]
    summary.maxrange_reserve_fuelburn   = results.max_range.segments['descent_3'].conditions.weights.total_mass[-1] - results.max_range.segments[-1].conditions.weights.total_mass[-1]



    hf = vehicle.fuselages.fuselage.heights.at_wing_root_quarter_chord
    wf = vehicle.fuselages.fuselage.width
    Lf = vehicle.fuselages.fuselage.lengths.total
    Sw = vehicle.wings.main_wing.areas.reference
    cw = vehicle.wings.main_wing.chords.mean_aerodynamic
    b  = vehicle.wings.main_wing.spans.projected
    Sh = vehicle.wings.horizontal_stabilizer.areas.reference
    Sv = vehicle.wings.vertical_stabilizer.areas.reference
    lh = vehicle.wings.horizontal_stabilizer.origin[0] + vehicle.wings.horizontal_stabilizer.aerodynamic_center[0] - vehicle.mass_properties.center_of_gravity[0]
    lv = vehicle.wings.vertical_stabilizer.origin[0] + vehicle.wings.vertical_stabilizer.aerodynamic_center[0] - vehicle.mass_properties.center_of_gravity[0]
    
    #h_tail_x = wf**2*Lf/Sw/cw
    #h_tail_y = .4036 + .4481*h_tail_x
    h_tail_V = lh*Sh/cw/Sw
    h_tail_190 = 1.2499986467855719
    summary.horizontal_tail_volume_coefficient = h_tail_V - h_tail_190 
    
    #v_tail_x = hf**2*Lf/Sw/b
    #v_tail_y = .038 + .31*v_tail_x
    v_tail_V = lv*Sv/b/Sw
    v_tail_190 = 0.088337744129052806
    summary.vertical_tail_volume_coefficient = v_tail_V - v_tail_190
    vec = np.array([])
    
    for mission in results.keys():
        for segment in results[mission].segments.keys():
            max_cl = missions[mission].segments[segment].analyses.sizing.features.vehicle.maximum_lift_coefficient
            segment_cls = results[mission].segments[segment].conditions.aerodynamics.lift_breakdown.compressible_wings
            vec = np.append(vec,max_cl-segment_cls)
    vec[vec > 0] = vec[vec > 0]*0.
    p = 8
    summary.maximum_cl_norm = np.sum(vec**p)**(1./p)
    
    print 'cg=', vehicle.mass_properties.center_of_gravity
    print 'ac=', vehicle.wings.main_wing.aerodynamic_center[0]+vehicle.wings.main_wing.origin[0]
    
    return nexus    
