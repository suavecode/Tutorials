# Procedure.py 
# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------     
import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
from SUAVE.Analyses.Process import Process  
from SUAVE.Methods.Weights.Correlations.UAV        import empty 
from SUAVE.Methods.Weights.Buildups.eVTOL.empty    import empty    

from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics import Aerodynamics
# Routines  
import Missions 

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------    

def stick_fixed_stability_and_drag_procedure(): 
    procedure                 = Process()
    procedure.modify_vehicle  = modify_stick_fixed_vehicle   
    procedure.post_process    = longitudinal_static_stability_and_drag_post_process   
        
    return procedure 

def elevator_sizing_setup(): 
    procedure = Process()  
    procedure.post_process  = elevator_sizing_post_process   
    return procedure   

def aileron_rudder_sizing_setup(): 
    procedure = Process()  
    procedure.post_process  = aileron_rudder_sizing_post_process   
    return procedure   

def flap_sizing_setup(): 
    procedure = Process()    
    procedure.post_process  = flap_sizing_post_process 
    return procedure  

# ----------------------------------------------------------------------      
#   Modify Vehicle 
# ----------------------------------------------------------------------  

def modify_stick_fixed_vehicle(nexus): 
    '''
    This function takes the updated design variables and modifies the aircraft 
    '''
    # Pull out the vehicles
    vehicle = nexus.vehicle_configurations.stick_fixed_cruise
     
    # Update Wing    
    for wing in vehicle.wings: 
        update_chords(wing)  
        
    # ----------------------------------------------------------------------
    # Update Vehicle Mass
    # ---------------------------------------------------------------------- 
    #weight_breakdown = empty(vehicle) 
    #vehicle.mass_properties.max_takeoff   = weight_breakdown.total
    #vehicle.mass_properties.takeoff       = weight_breakdown.total 
     
    # Update Mission  
    nexus.missions = Missions.stick_fixed_stability_setup(nexus.analyses,vehicle)    
    
    # diff the new data
    vehicle.store_diff() 
    
    return nexus   
 
def update_chords(wing):
    '''
    Updates the wing planform each iteration
    '''
    Sref  = wing.areas.reference      # fixed 
    span  = wing.spans.projected      # optimization input
    taper = wing.taper                # optimization input
    croot = 2*Sref/((taper+1)*span)   # set by Sref and current design point
    ctip  = taper * croot             # set by Sref and current design point 
    wing.chords.root = croot
    wing.chords.tip  = ctip 
    
    # Wing Segments
    if 'Segments' in wing:
        for seg in wing.Segments:
            seg.twist = (wing.twists.tip-wing.twists.root)*seg.percent_span_location  + wing.twists.root
    
    return wing          

def longitudinal_static_stability_and_drag_post_process(nexus): 
    '''
    This function analyses and post processes the aircraft at cruise conditions. 
    The objective of is to minimize the drag  of a trimmed aircraft 
    '''
    summary                                                 = nexus.summary 
    vehicle                                                 = nexus.vehicle_configurations.stick_fixed_cruise 
    g                                                       = 9.81   
    L                                                       = g*vehicle.mass_properties.max_takeoff
    S                                                       = vehicle.reference_area
    atmosphere                                              = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                               = atmosphere.compute_values(altitude = \
                                                                nexus.missions['stick_fixed_cruise'].segments['cruise'].altitude )       
                                     
                                     
    run_conditions                                          = Aerodynamics()
    run_conditions.freestream.density                       = atmo_data.density[0,0] 
    run_conditions.freestream.gravity                       = g           
    run_conditions.freestream.speed_of_sound                = atmo_data.speed_of_sound[0,0]  
    run_conditions.freestream.velocity                      = nexus.missions['stick_fixed_cruise'].segments['cruise'].air_speed
    run_conditions.freestream.mach_number                   = run_conditions.freestream.velocity/run_conditions.freestream.speed_of_sound
    run_conditions.aerodynamics.side_slip_angle             = 0.0 
    run_conditions.aerodynamics.angle_of_attack             = np.array([0.0])
    run_conditions.aerodynamics.lift_coefficient            = L/(S*(0.5*run_conditions.freestream.density*(run_conditions.freestream.velocity**2)))
    run_conditions.aerodynamics.roll_rate_coefficient       = 0.0
    run_conditions.aerodynamics.pitch_rate_coefficient      = 0.0
    
    stability_stick_fixed = SUAVE.Analyses.Stability.AVL() 
    stability_stick_fixed.settings.filenames.avl_bin_name   = '/Users/matthewclarke/Documents/AVL/avl3.35'   # change to path of AVL  
    stability_stick_fixed.geometry                          = nexus.vehicle_configurations.stick_fixed_cruise
    results_stick_fixed                                     = stability_stick_fixed.evaluate_conditions(run_conditions, trim_aircraft = True ) 

     
    summary.CD              = results_stick_fixed.aerodynamics.drag_breakdown.induced.total[0,0] 
    summary.CM_residual     = abs(results_stick_fixed.aerodynamics.pitch_moment_coefficient[0,0])
    summary.spiral_criteria = results_stick_fixed.stability.static.spiral_criteria[0,0]
    NP                      = results_stick_fixed.stability.static.neutral_point[0,0]
    cg                      = vehicle.mass_properties.center_of_gravity[0][0]
    MAC                     = vehicle.wings.main_wing.chords.mean_aerodynamic
    summary.static_margin   = (NP - cg)/MAC
    summary.CM_alpha        = results_stick_fixed.stability.static.Cm_alpha[0,0]  
 
    if np.count_nonzero(vehicle.mass_properties.moments_of_inertia.tensor) > 0:  
        summary.phugoid_damping_ratio   = results_stick_fixed.dynamic_stability.LongModes.phugoidDamp[0,0]
        summary.short_period_frequency  = results_stick_fixed.dynamic_stability.LongModes.shortPeriodFreqHz[0,0] 
        summary.dutch_roll_frequency    = results_stick_fixed.dynamic_stability.LatModes.dutchRollFreqHz[0,0]
        summary.spiral_doubling_time    = results_stick_fixed.dynamic_stability.LatModes.spiralTimeDoubleHalf[0,0] 
        print("Drag Coefficient      : " + str(summary.CD))
        print("Moment Coefficient    : " + str(summary.CM_residual))
        print("Static Margin         : " + str(summary.static_margin))
        print("CM alpla              : " + str(summary.CM_alpha))   
        print("Phugoid Damping Ratio : " + str(summary.phugoid_damping_ratio))
        print("Short Period Frequency: " + str(summary.short_period_frequency))
        print("Dutch Roll Frequency  : " + str(summary.dutch_roll_frequency))
        print("Spiral Doubling Time  : " + str(summary.spiral_doubling_time)) 
        print("Spiral Criteria       : " + str(summary.spiral_criteria))
        print("\n\n") 

    else: 
        summary.phugoid_damping_ratio   = 0
        summary.short_period_frequency  = 0 
        summary.dutch_roll_frequency    = 0
        summary.spiral_doubling_time    = 0
        summary.spiral_criteria         = 0 
        print("Drag Coefficient      : " + str(summary.CD))
        print("Moment Coefficient    : " + str(summary.CM_residual))
        print("Static Margin         : " + str(summary.static_margin))
        print("CM alpla              : " + str(summary.CM_alpha))    
        print("Spiral Criteria       : " + str(summary.spiral_criteria))
        print("\n\n")    
        
        
    vehicle.trim_cl        = run_conditions.aerodynamics.lift_coefficient 
    vehicle.trim_airspeed  =  run_conditions.freestream.velocity 
    
    return nexus  
    
def elevator_sizing_post_process(nexus): 
    '''
    This function analyses and post processes the aircraft at the flight conditions required to size
    the elevator. These conditions are:
    1) Stick pull maneuver with a load factor of 3.0
    2) Stick push maneuver with a load factor of -1
    ''' 
    summary                                            = nexus.summary 
    trim_aircraft                                      = True
    g                                                  = 9.81 
    vehicle                                            = nexus.vehicle_configurations.elevator_sizing 
    m                                                  = vehicle.mass_properties.max_takeoff
    S                                                  = vehicle.reference_area 
    V_trim                                             = vehicle.trim_airspeed  
    max_defl                                           = vehicle.maximum_elevator_deflection
    V_max                                              = nexus.missions['elevator_sizing'].segments['cruise'].air_speed
                                
    atmosphere                                         = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                          = atmosphere.compute_values(altitude = nexus.missions['elevator_sizing'].segments['cruise'].altitude )       
    run_conditions                                     = Aerodynamics()
    run_conditions.freestream.density                  = atmo_data.density[0,0] 
    run_conditions.freestream.gravity                  = g           
    run_conditions.freestream.speed_of_sound           = atmo_data.speed_of_sound[0,0]  
    run_conditions.aerodynamics.side_slip_angle        = 0.0
    run_conditions.aerodynamics.angle_of_attack        = np.array([0.0])
    run_conditions.aerodynamics.roll_rate_coefficient  = 0.0
    run_conditions.aerodynamics.pitch_rate_coefficient = 0.0 
    
    q            = 0.5*(V_trim**2)*atmo_data.density[0,0] 
    CL_pull_man  = vehicle.maxiumum_load_factor*m*g/(S*q)  
    CL_push_man  = vehicle.minimum_load_factor*m*g/(S*q) 
                                      
    stability_pull_maneuver                                   = SUAVE.Analyses.Stability.AVL() 
    stability_pull_maneuver.settings.filenames.avl_bin_name   = '/Users/matthewclarke/Documents/AVL/avl3.35'   # change to path of AVL  
    run_conditions.aerodynamics.lift_coefficient              =  CL_pull_man
    run_conditions.freestream.velocity                        = V_max 
    run_conditions.freestream.mach_number                     = run_conditions.freestream.velocity/run_conditions.freestream.speed_of_sound
    stability_pull_maneuver.settings.number_spanwise_vortices = 40
    stability_pull_maneuver.geometry                          = vehicle
    results_pull_maneuver                                     = stability_pull_maneuver.evaluate_conditions(run_conditions, trim_aircraft )
    AoA_pull                                                  = results_pull_maneuver.aerodynamics.AoA[0,0]
    elevator_pull_deflection                                  = results_pull_maneuver.stability.static.control_surfaces_cases['case_0001_0001'].control_surfaces.elevator.deflection

    stability_push_maneuver = SUAVE.Analyses.Stability.AVL() 
    stability_push_maneuver.settings.filenames.avl_bin_name   = '/Users/matthewclarke/Documents/AVL/avl3.35'  # change to path of AVL  
    run_conditions.aerodynamics.lift_coefficient              = CL_push_man 
    run_conditions.freestream.velocity                        = V_trim
    run_conditions.freestream.mach_number                     = run_conditions.freestream.velocity/run_conditions.freestream.speed_of_sound
    stability_pull_maneuver.settings.number_spanwise_vortices = 40
    stability_push_maneuver.geometry                          = vehicle
    results_push_maneuver                                     = stability_push_maneuver.evaluate_conditions(run_conditions, trim_aircraft ) 
    AoA_push                                                  = results_push_maneuver.aerodynamics.AoA[0,0]
    elevator_push_deflection                                  = results_push_maneuver.stability.static.control_surfaces_cases['case_0001_0001'].control_surfaces.elevator.deflection
     
    summary.elevator_pull_deflection_residual = (max_defl/Units.degrees  - abs(elevator_pull_deflection))*Units.degrees
    summary.elevator_push_deflection_residual = (max_defl/Units.degrees  - abs(elevator_push_deflection))*Units.degrees
    
    # compute control surface area 
    control_surfaces = ['elevator'] 
    total_control_surface_area = compute_control_surface_areas(control_surfaces,vehicle)  
    summary.elevator_surface_area =  total_control_surface_area
    

    print("Elevator Area      : " + str(summary.elevator_surface_area))
    print("Aircraft CL Pull   : " + str(CL_pull_man))
    print("Aircraft AoA Pull  : " + str(AoA_pull))
    print("Elevator Pull Defl.: " + str(elevator_pull_deflection)) 
    print("Aircraft CL Push   : " + str(CL_push_man))
    print("Aircraft AoA Push  : " + str(AoA_push))
    print("Elevator Push Defl.: " + str(elevator_push_deflection)) 
    print("\n\n")     
         
    return nexus    

 

def aileron_rudder_sizing_post_process(nexus):  
    '''
    This function analyses and post processes the aircraft at the flight conditions required to size
    the aileron and rudder. These conditions are:
    1) A controlled roll at a  rate of 0.07
    2) Trimmed flight in a 20 knot crosswind
    ''' 
    summary                                                   = nexus.summary 
    trim_aircraft                                             = True
    g                                                         = 9.81 
    vehicle                                                   = nexus.vehicle_configurations.aileron_rudder_sizing 
    CL_trim                                                   = vehicle.trim_cl
    max_defl                                                  = vehicle.maximum_aileron_rudder_deflection
    V_crosswind                                               = vehicle.crosswind_velocity
                                       
    atmosphere                                                = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                                 = atmosphere.compute_values(altitude = nexus.missions['aileron_sizing'].segments['cruise'].altitude )       
    run_conditions                                            = Aerodynamics()
    run_conditions.freestream.density                         = atmo_data.density[0,0] 
    run_conditions.freestream.gravity                         = g           
    run_conditions.freestream.speed_of_sound                  = atmo_data.speed_of_sound[0,0]  
    run_conditions.aerodynamics.side_slip_angle               = 0.0
    run_conditions.aerodynamics.angle_of_attack               = np.array([0.0]) 
    
    
    stability_roll_maneuver = SUAVE.Analyses.Stability.AVL() 
    stability_roll_maneuver.settings.filenames.avl_bin_name   = '/Users/matthewclarke/Documents/AVL/avl3.35' # change to path of AVL    
    stability_roll_maneuver.settings.number_spanwise_vortices = 40
    run_conditions.aerodynamics.lift_coefficient              = CL_trim 
    stability_roll_maneuver.geometry                          = vehicle
    run_conditions.freestream.velocity                        = nexus.missions['aileron_sizing'].segments['cruise'].air_speed
    run_conditions.freestream.mach_number                     = run_conditions.freestream.velocity/run_conditions.freestream.speed_of_sound
    run_conditions.aerodynamics.roll_rate_coefficient         = 0.07
    run_conditions.aerodynamics.pitch_rate_coefficient        = 0.0
    run_conditions.aerodynamics.side_slip_angle               = 0.0
    results_roll_maneuver                                     = stability_roll_maneuver.evaluate_conditions(run_conditions, trim_aircraft )
    aileron_roll_deflection                                   = results_roll_maneuver.stability.static.control_surfaces_cases['case_0001_0001'].control_surfaces.aileron.deflection 
    
    summary.aileron_roll_deflection_residual = (max_defl/Units.degrees  - abs(aileron_roll_deflection))*Units.degrees
    if vehicle.rudder_flag: 
        rudder_roll_deflection  = results_roll_maneuver.stability.static.control_surfaces_cases['case_0001_0001'].control_surfaces.rudder.deflection
        summary.rudder_roll_deflection_residual = (max_defl/Units.degrees  - abs(rudder_roll_deflection))*Units.degrees  
    else:
        rudder_roll_deflection = 0
        summary.rudder_roll_deflection_residual = 0       
        
    stability_cross_wind_maneuver = SUAVE.Analyses.Stability.AVL() 
    stability_cross_wind_maneuver.settings.filenames.avl_bin_name = '/Users/matthewclarke/Documents/AVL/avl3.35'  # change to path of AVL   
    run_conditions.aerodynamics.lift_coefficient                  = CL_trim 
    stability_cross_wind_maneuver.geometry                        = vehicle
    run_conditions.freestream.velocity                            = nexus.missions['aileron_sizing'].segments['cruise'].air_speed
    run_conditions.freestream.mach_number                         = run_conditions.freestream.velocity/run_conditions.freestream.speed_of_sound
    run_conditions.aerodynamics.roll_rate_coefficient             = 0.0
    run_conditions.aerodynamics.pitch_rate_coefficient            = 0.0
    run_conditions.aerodynamics.side_slip_angle                   = np.tan(V_crosswind/nexus.missions['aileron_sizing'].segments['cruise'].air_speed) # beta
    results_cross_wind_maneuver                                   = stability_cross_wind_maneuver.evaluate_conditions(run_conditions, trim_aircraft )
    aileron_cross_wind_deflection                                 = results_cross_wind_maneuver.stability.static.control_surfaces_cases['case_0001_0001'].control_surfaces.aileron.deflection 
    
    # criteria 
    summary.aileron_crosswind_deflection_residual = (max_defl/Units.degrees  - abs(aileron_cross_wind_deflection))*Units.degrees

    if vehicle.rudder_flag: 
        rudder_cross_wind_deflection  = results_cross_wind_maneuver.stability.static.control_surfaces_cases['case_0001_0001'].control_surfaces.rudder.deflection
        summary.rudder_crosswind_deflection_residual = (max_defl/Units.degrees  - abs(rudder_cross_wind_deflection))*Units.degrees  
    else:
        rudder_cross_wind_deflection = 0
        summary.rudder_crosswind_deflection_residual = 0  
        
    # compute control surface area 
    control_surfaces = ['aileron','rudder'] 
    total_control_surface_area = compute_control_surface_areas(control_surfaces,vehicle)   
    summary.aileron_rudder_surface_area =  total_control_surface_area 

    print("Total Rudder Aileron Surface Area : " + str(summary.aileron_rudder_surface_area)) 
    print("Aileron Roll Defl                 : " + str(aileron_roll_deflection)) 
    print("Rudder Roll Defl                  : " + str(rudder_roll_deflection))  
    print("Aileron Crosswind Defl            : " + str(aileron_cross_wind_deflection)) 
    print("Rudder  Crosswind Defl            : " + str(rudder_cross_wind_deflection )) 
    print("\n\n")     
  
    return nexus     


def flap_sizing_post_process(nexus): 
    '''
    This function analyses and post processes the aircraft at the flight conditions required to size
    the flap. These conditions are:
    1) A comparison of clean and deployed flap at 12 deg. angle of attack
    ''' 
    summary                                                  = nexus.summary 
    trim_aircraft                                            = False
    g                                                        = 9.81 
    vehicle                                                  = nexus.vehicle_configurations.flap_sizing 
    max_defl                                                 = vehicle.maximum_flap_deflection
    V_max                                                    = nexus.missions['flap_sizing'].segments['cruise'].air_speed
          
    atmosphere                                               = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                                = atmosphere.compute_values(altitude = nexus.missions['flap_sizing'].segments['cruise'].altitude )       
    run_conditions                                           = Aerodynamics()
    run_conditions.freestream.density                        = atmo_data.density[0,0] 
    run_conditions.freestream.gravity                        = g           
    run_conditions.freestream.speed_of_sound                 = atmo_data.speed_of_sound[0,0]  
    run_conditions.freestream.velocity                       = V_max 
    run_conditions.freestream.mach_number                    = run_conditions.freestream.velocity/run_conditions.freestream.speed_of_sound
    run_conditions.aerodynamics.side_slip_angle              = 0.0
    run_conditions.aerodynamics.lift_coefficient             = None
    run_conditions.aerodynamics.angle_of_attack              = np.array([12.0])*Units.degrees
    run_conditions.aerodynamics.roll_rate_coefficient        = 0.0
    run_conditions.aerodynamics.pitch_rate_coefficient       = 0.0
    
    stability_no_flap = SUAVE.Analyses.Stability.AVL() 
    stability_no_flap.settings.filenames.avl_bin_name        = '/Users/matthewclarke/Documents/AVL/avl3.35' # change to path of AVL    
    stability_no_flap.settings.number_spanwise_vortices      = 40
    vehicle.wings.main_wing.control_surfaces.flap.deflection = 0.0
    stability_no_flap.geometry                               = vehicle
    results_no_flap                                          = stability_no_flap.evaluate_conditions(run_conditions, trim_aircraft) 
    CL_12_deg_no_flap                                        = results_no_flap.aerodynamics.lift_coefficient[0,0]  
      
    
    stability_flap = SUAVE.Analyses.Stability.AVL() 
    stability_flap.settings.filenames.avl_bin_name           = '/Users/matthewclarke/Documents/AVL/avl3.35' # change to path of AVL   
    stability_flap.settings.number_spanwise_vortices         = 40
    vehicle.wings.main_wing.control_surfaces.flap.deflection = max_defl 
    stability_flap.geometry                                  = vehicle
    results_flap                                             = stability_flap.evaluate_conditions(run_conditions, trim_aircraft) 
    CL_12_deg_flap                                           = results_flap.aerodynamics.lift_coefficient[0,0]     
    
    # critera     
    flap_criteria  = (CL_12_deg_flap-CL_12_deg_no_flap) - 0.95*(CL_12_deg_flap-CL_12_deg_no_flap) 
    # compute control surface area 
    control_surfaces = ['flap'] 
    total_control_surface_area = compute_control_surface_areas(control_surfaces,vehicle)  
    summary.flap_surface_area  =  total_control_surface_area
    summary.flap_criteria      =  flap_criteria

    print("Flap Area     : " + str(summary.flap_surface_area))
    print("Flap Criteria : " + str(flap_criteria))  # https://aviation.stackexchange.com/questions/48715/how-is-the-area-of-flaps-determined
    print("\n\n")     
         
    return nexus    


def  compute_control_surface_areas(control_surfaces,vehicle): 
    '''
    This function computes the control suface area used in the objectives of the 
    control surface sizing scripts 
    '''
    total_control_surface_area = 0
    for cs_idx in range(len(control_surfaces)):
        for wing in vehicle.wings:
            if getattr(wing,'control_surfaces',False):  
                for CS in wing.control_surfaces:
                    if CS.tag == control_surfaces[cs_idx]:
                        if wing.Segments: 
                            num_segs = len(wing.Segments)
                            for seg_id in range(num_segs-1): 
                                if (CS.span_fraction_start >= wing.Segments[seg_id].percent_span_location) \
                                   and (CS.span_fraction_end <= wing.Segments[seg_id+1].percent_span_location): 
                                    root_chord             = wing.Segments[seg_id].root_chord_percent*wing.chords.root
                                    tip_chord              = wing.Segments[seg_id+1].root_chord_percent*wing.chords.root
                                    span                   = (wing.Segments[seg_id+1].percent_span_location-wing.Segments[seg_id].percent_span_location)*wing.spans.projected
                                    rel_start_percent_span = CS.span_fraction_start - wing.Segments[seg_id].percent_span_location
                                    rel_end_percent_span   = CS.span_fraction_end - wing.Segments[seg_id].percent_span_location
                                    chord_fraction         = CS.chord_fraction 
                                    area = conpute_control_surface_area(root_chord,tip_chord,span,rel_start_percent_span,rel_end_percent_span,chord_fraction)
                                    total_control_surface_area += area
                        else: 
                            root_chord             = wing.chords.root
                            tip_chord              = wing.chords.tip
                            span                   = wing.spans.projected
                            rel_start_percent_span = CS.span_fraction_start  
                            rel_end_percent_span   = CS.span_fraction_end 
                            chord_fraction         = CS.chord_fraction 
                            area = conpute_control_surface_area(root_chord,tip_chord,span,rel_start_percent_span,rel_end_percent_span,chord_fraction)                            
                            total_control_surface_area += area 
         
    return total_control_surface_area

def conpute_control_surface_area(root_chord,tip_chord,span,rel_start_percent_span,rel_end_percent_span,chord_fraction):  
    '''
    This is a simple function that computes the area of a single control surface
    '''
    cs_start_chord =  (root_chord +   ((tip_chord-root_chord)/span)*(rel_start_percent_span*span))*chord_fraction
    cs_end_chord   =  (root_chord +   ((tip_chord-root_chord)/span)*(rel_end_percent_span*span))*chord_fraction
    cs_span        = (rel_end_percent_span-rel_start_percent_span)*span
    cs_area        = 0.5*(cs_start_chord+cs_end_chord)*cs_span
    return cs_area
    
