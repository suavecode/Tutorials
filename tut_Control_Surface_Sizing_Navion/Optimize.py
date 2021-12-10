# Optimize.py 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------  
import SUAVE
from SUAVE.Core import Units, Data 
import numpy as np
import Vehicles
import Missions
import Procedure
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
from SUAVE.Optimization       import Nexus     
import time 
# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main(): 
    '''
    STICK FIXED (STATIC STABILITY AND DRAG) OTIMIZATION
    '''
    ti = time.time()  
    solver_name       = 'SLSQP' 
    planform_optimization_problem = stick_fixed_stability_and_drag_optimization_setup()
    output = scipy_setup.SciPy_Solve(planform_optimization_problem,solver=solver_name, sense_step = 1E-3, tolerance = 1E-3)  
    print (output)    
    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Stick Fixed Stability and Drag Otimization Simulation Time: ' + str(elapsed_time))    
    
    '''
    ELEVATOR SIZING
    '''      
    # define vehicle for elevator sizing 
    optimized_vehicle_v1                             = planform_optimization_problem.vehicle_configurations.stick_fixed_cruise 
    optimized_vehicle_v1.maximum_elevator_deflection = 30*Units.degrees 
    optimized_vehicle_v1.maxiumum_load_factor = 3.0
    optimized_vehicle_v1.minimum_load_factor = -1
    
    ti = time.time()   
    solver_name       = 'SLSQP'  
    elevator_sizing_optimization_problem = elevator_sizing_optimization_setup(optimized_vehicle_v1)
    output = scipy_setup.SciPy_Solve(elevator_sizing_optimization_problem,solver=solver_name, sense_step = 1E-3, tolerance = 1E-3) 
    print (output)     
    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Elevator Sizing Simulation Time: ' + str(elapsed_time))   
     
    '''
    AILERON AND RUDDER SIZING
    '''      
    # define vehicle for aileron and rudder sizing
    optimized_vehicle_v2    = elevator_sizing_optimization_problem.vehicle_configurations.elevator_sizing 
    optimized_vehicle_v2.rudder_flag                       = True 
    optimized_vehicle_v2.maximum_aileron_rudder_deflection = 30*Units.degrees 
    optimized_vehicle_v2.crosswind_velocity                = 20 * Units.knots

    ti = time.time()   
    solver_name       = 'SLSQP'  
    aileron_rudder_sizing_optimization_problem = aileron_rudder_sizing_optimization_setup(optimized_vehicle_v2)
    output = scipy_setup.SciPy_Solve(aileron_rudder_sizing_optimization_problem,solver=solver_name, sense_step = 1E-3, tolerance = 1E-3) 
    print (output)     
    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Aileron and Rudder Sizing Simulation Time: ' + str(elapsed_time))   

    '''
    FLAP SIZING
    '''      
    # define vehicle for flap sizing     
    optimized_vehicle_v3 = aileron_rudder_sizing_optimization_problem.vehicle_configurations.aileron_rudder_sizing
    optimized_vehicle_v3.maximum_flap_deflection = 40*Units.degrees
    
    ti = time.time()   
    solver_name       = 'SLSQP'  
    flap_sizing_optimization_problem = flap_sizing_optimization_setup(optimized_vehicle_v3)
    output = scipy_setup.SciPy_Solve(flap_sizing_optimization_problem,solver=solver_name, sense_step = 1E-3, tolerance = 1E-3) 
    print (output)     
    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Flap Sizing Simulation Time: ' + str(elapsed_time))   

    '''
    PRINT VEHICLE CONTROL SURFACES
    '''          
    optimized_vehicle_v4  = flap_sizing_optimization_problem.vehicle_configurations.flap_sizing 
    print_vehicle_control_surface_geoemtry(optimized_vehicle_v4)
    
    return
  
def stick_fixed_stability_and_drag_optimization_setup(): 
    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    #                 [ tag                       , initial,  (lb , ub) , scaling , units ]  
    problem.inputs = np.array([          
                  #[ 'mw_taper'                   , 0.5  , 0.4  , 1.0  , 1.0  ,  1*Units.less],    
                  #[ 'mw_area'                    , 15.39, 14.0 , 16.0 , 100. ,  1*Units.meter**2],    
                  #[ 'mw_AR'                      , 11.0 , 9.0  , 14.0 , 100  ,  1*Units.less],         
                  [ 'mw_root_twist'               , 3.0  , -5.0 , 5.0 , 10. ,  1*Units.degree], 
                  [ 'mw_tip_twist'                , 0.0  , -5.0 , 5.0  , 10.  ,  1*Units.degree],
                  #[ 'mw_dihedral'                , 0.0  , 0.0  , 5.0  , 10.  ,  1*Units.degree], 
                  #[ 'hs_AR'                      , 4.287, 4.0  , 4.2  , 10.  ,  1*Units.less], 
                  #[ 'hs_area'                    , 2.54 , 2.30 , 3.0  , 10.  ,  1*Units.meter**2],   
                  #[ 'hs_root_twist'              , 0.0  , -5.0 , 5.0  , 10.  , 1*Units.degree],
                  #[ 'hs_tip_twist'               , 0.0  , -5.0 , 5.0  , 10.  , 1*Units.degree], 
                  [ 'c_g_x'                       , 3.1  , 2.0  , 4.0  , 10   ,  1*Units.less],
                  
    ],dtype=object)   

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # [ tag, scaling, units ]
    problem.objective = np.array([ 
                                 [  'CD'  ,  1.0  ,    1*Units.less] 
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        [ 'CM_residual'           ,   '<' ,   1E-2 ,   1E-2  , 1*Units.less], # close to zero 2 works 
        [ 'static_margin'         ,   '>' ,   0.1  ,   0.1   , 1*Units.less],
        [ 'CM_alpha'              ,   '<' ,   0.0  ,   1.0   , 1*Units.less],  
        #[ 'phugoid_damping_ratio' ,   '>' ,   0.04 ,   1.0   , 1*Units.less],  
        #[ 'short_period_frequency',   '>' ,   1.34 ,   1.0   , 1*Units.less],  
        #[ 'dutch_roll_frequency'  ,   '>' ,   1.0  ,   1.0   , 1*Units.less],  
        #[ 'spiral_doubling_time'  ,   '>' ,   4.0  ,   1.0   , 1*Units.less],  
        #[ 'spiral_criteria'       ,   '>' ,   1.0  ,   1.0   , 1*Units.less],  
    ],dtype=object)
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ] 
    problem.aliases = [ 
        [ 'CD'                                , 'summary.CD' ],
        [ 'CM_residual'                       , 'summary.CM_residual' ],  
        [ 'CM_alpha'                          , 'summary.CM_alpha' ],    
        [ 'static_margin'                     , 'summary.static_margin' ], 
        #[ 'phugoid_damping_ratio'             , 'summary.phugoid_damping_ratio' ],  
        #[ 'short_period_frequency'            , 'summary.short_period_frequency' ],  
        #[ 'dutch_roll_frequency'              , 'summary.dutch_roll_frequency' ],  
        #[ 'spiral_doubling_time'              , 'summary.spiral_doubling_time' ],   
        #[ 'spiral_criteria'                   , 'summary.spiral_criteria' ],      
        #[ 'mw_area'                           , 'vehicle_configurations.*.wings.main_wing.aspect_ratio'],
        #[ 'mw_taper'                          , 'vehicle_configurations.*.wings.main_wing.taper'],
        #[ 'mw_AR'                             , 'vehicle_configurations.*.wings.main_wing.aspect_ratio'],         
        [ 'mw_root_twist'                     , 'vehicle_configurations.*.wings.main_wing.twists.root' ], 
        [ 'mw_tip_twist'                      , 'vehicle_configurations.*.wings.main_wing.twists.tip'  ],     
        #[ 'mw_dihedral'                       , 'vehicle_configurations.*.wings.main_wing.dihedral'  ],        
        #[ 'hs_AR'                             , 'vehicle_configurations.*.wings.horizontal_stabilizer.aspect_ratio'],     
        #[ 'hs_area'                           , 'vehicle_configurations.*.wings.horizontal_stabilizer.aspect_ratio'],
        #[ 'hs_taper'                          , 'vehicle_configurations.*.wings.horizontal_stabilizer.taper'],  
        #[ 'hs_root_twist'                     , 'vehicle_configurations.*.wings.horizontal_stabilizer.twists.root' ], 
        #[ 'hs_tip_twist'                      , 'vehicle_configurations.*.wings.horizontal_stabilizer.twists.tip'  ], 
        #[ 'hs_dihedral'                       , 'vehicle_configurations.*.wings.horizontal_stabilizer.dihedral'  ],   
        [ 'c_g_x'                             , 'vehicle_configurations.*.mass_properties.center_of_gravity[0][0]'  ], 
    ]      
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.stick_fixed_stability_setup()
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = None 
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.stick_fixed_stability_setup(nexus.analyses,nexus.vehicle_configurations.stick_fixed_cruise)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.stick_fixed_stability_and_drag_procedure()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()     
    return nexus 

def elevator_sizing_optimization_setup(vehicle):

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    #   [ tag                   , initial,         (lb , ub)        , scaling , units ]  
    problem.inputs = np.array([            
                  [ 'hs_elevator_chord_fraction' , 0.2    , 0.1  , 0.3  ,  1.0 ,  1*Units.less],
                  [ 'hs_elevator_span_frac_start', 0.25   , 0.05 , 0.45 ,  1.0 ,  1*Units.less], 
                  [ 'hs_elevator_span_frac_end'  , 0.75   , 0.55 , 0.9  ,  1.0 ,  1*Units.less],     
                  
    ],dtype=object)   

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # [ tag, scaling, units ]
    problem.objective = np.array([ 
                                  [ 'elevator_surface_area', 1. , 1*Units.kg],
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([ 
        [ 'elevator_push_deflection_residual'           ,   '>' ,  0  , 1.0   , 1*Units.less], 
        [ 'elevator_pull_deflection_residual'           ,   '>' ,  0  , 1.0   , 1*Units.less], 
    ],dtype=object)
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ] 
    problem.aliases = [ 
        [ 'elevator_surface_area'             , 'summary.elevator_surface_area' ], 
        [ 'elevator_push_deflection_residual' , 'summary.elevator_push_deflection_residual' ],   
        [ 'elevator_pull_deflection_residual' , 'summary.elevator_pull_deflection_residual' ],     
        [ 'hs_elevator_chord_fraction'        , 'vehicle_configurations.*.wings.horizontal_stabilizer.control_surfaces.elevator.chord_fraction'],    
        [ 'hs_elevator_span_frac_start'       , 'vehicle_configurations.*.wings.horizontal_stabilizer.control_surfaces.elevator.span_fraction_start'],    
        [ 'hs_elevator_span_frac_end'         , 'vehicle_configurations.*.wings.horizontal_stabilizer.control_surfaces.elevator.span_fraction_end'],  
    ]      
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.elevator_sizing_setup(vehicle)
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = None 
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.elevator_sizing_setup(nexus.analyses,nexus.vehicle_configurations.elevator_sizing)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.elevator_sizing_setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()     
    return nexus  


 
 
def aileron_rudder_sizing_optimization_setup(vehicle):

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    #   [ tag                   , initial,         (lb , ub)        , scaling , units ]  
    if vehicle.rudder_flag:
        problem.inputs = np.array([             
                      [ 'mw_aileron_chord_fraction'  , 0.2    , 0.15 , 0.3  ,  1.0 ,  1*Units.less],
                      [ 'mw_aileron_span_frac_start' , 0.75   , 0.55 , 0.8  ,  1.0 ,  1*Units.less],
                      [ 'mw_aileron_span_frac_end'   , 0.9    , 0.85 , 0.95 ,  1.0 ,  1*Units.less],  
                      [ 'vs_rudder_chord_fraction'   , 0.2    , 0.15 , 0.3  ,  1.0 ,  1*Units.less],
                      [ 'vs_rudder_span_frac_start'  , 0.25   , 0.05 , 0.35 ,  1.0 ,  1*Units.less],
                      [ 'vs_rudder_span_frac_end'    , 0.75   , 0.5  , 0.95 ,  1.0 ,  1*Units.less] ],dtype=object)   
    else:
        problem.inputs = np.array([             
                      [ 'mw_aileron_chord_fraction'  , 0.2    , 0.15 , 0.3  ,  1.0 ,  1*Units.less],
                      [ 'mw_aileron_span_frac_start' , 0.75   , 0.55 , 0.8  ,  1.0 ,  1*Units.less],
                      [ 'mw_aileron_span_frac_end'   , 0.9    , 0.85 , 0.95 ,  1.0 ,  1*Units.less],   
                      
        ],dtype=object)      

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # [ tag, scaling, units ]
    problem.objective = np.array([ 
                                  [ 'aileron_rudder_surface_area', 1. , 1*Units.kg],
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    if vehicle.rudder_flag:
        problem.constraints = np.array([
            [ 'aileron_roll_deflection_residual'       ,   '>' ,  0  ,  1.0    , 1*Units.less], 
            [ 'rudder_roll_deflection_residual'        ,   '>' ,  0  ,  1.0    , 1*Units.less],  
            [ 'aileron_crosswind_deflection_residual'  ,   '>' ,  0  ,  1.0    , 1*Units.less], 
            [ 'rudder_crosswind_deflection_residual'   ,   '>' ,  0  ,  1.0    , 1*Units.less],  
        ],dtype=object)
    else:
        problem.constraints = np.array([
            [ 'aileron_roll_deflection_residual'       ,   '>' ,  0  ,  1.0    , 1*Units.less],  
            [ 'aileron_crosswind_deflection_residual'  ,   '>' ,  0  ,  1.0    , 1*Units.less],  
        ],dtype=object)
        
        
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ] 
    if vehicle.rudder_flag:
        problem.aliases = [ 
            [ 'aileron_rudder_surface_area'            , 'summary.aileron_rudder_surface_area' ],  
            [ 'aileron_roll_deflection_residual'       , 'summary.aileron_roll_deflection_residual' ],  
            [ 'rudder_roll_deflection_residual'        , 'summary.rudder_roll_deflection_residual' ], 
            [ 'aileron_crosswind_deflection_residual'  , 'summary.aileron_crosswind_deflection_residual' ],      
            [ 'rudder_crosswind_deflection_residual'   , 'summary.rudder_crosswind_deflection_residual' ],         
            [ 'mw_aileron_chord_fraction'              , 'vehicle_configurations.*.wings.main_wing.control_surfaces.aileron.chord_fraction'],  
            [ 'mw_aileron_span_frac_start'             , 'vehicle_configurations.*.wings.main_wing.control_surfaces.aileron.span_fraction_start'],    
            [ 'mw_aileron_span_frac_end'               , 'vehicle_configurations.*.wings.main_wing.control_surfaces.aileron.span_fraction_end'],     
            [ 'vs_rudder_chord_fraction'               , 'vehicle_configurations.*.wings.vertical_stabilizer.control_surfaces.rudder.chord_fraction'],    
            [ 'vs_rudder_span_frac_start'              , 'vehicle_configurations.*.wings.vertical_stabilizer.control_surfaces.rudder.span_fraction_start'],    
            [ 'vs_rudder_span_frac_end'                , 'vehicle_configurations.*.wings.vertical_stabilizer.control_surfaces.rudder.span_fraction_end']]      
    else:
        problem.aliases = [ 
            [ 'aileron_rudder_surface_area'            , 'summary.aileron_rudder_surface_area' ],  
            [ 'aileron_roll_deflection_residual'       , 'summary.aileron_roll_deflection_residual' ],          
            [ 'aileron_crosswind_deflection_residual'  , 'summary.aileron_crosswind_deflection_residual' ],   
            [ 'mw_aileron_chord_fraction'              , 'vehicle_configurations.*.wings.main_wing.control_surfaces.aileron.chord_fraction'],  
            [ 'mw_aileron_span_frac_start'             , 'vehicle_configurations.*.wings.main_wing.control_surfaces.aileron.span_fraction_start'],    
            [ 'mw_aileron_span_frac_end'               , 'vehicle_configurations.*.wings.main_wing.control_surfaces.aileron.span_fraction_end']] 
        
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.aileron_rudder_sizing_setup(vehicle)
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = None
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.aileron_rudder_sizing_setup(nexus.analyses,nexus.vehicle_configurations.aileron_rudder_sizing)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.aileron_rudder_sizing_setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()     
    return nexus  


def flap_sizing_optimization_setup(optimized_vehicle):

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    #   [ tag                   , initial,         (lb , ub)        , scaling , units ]  
    problem.inputs = np.array([           
                  [ 'mw_flap_chord_fraction'     , 0.2    , 0.15 , 0.4  ,  1.0 ,  1*Units.less],
                  [ 'mw_flap_span_frac_start'    , 0.2    , 0.05 , 0.25 ,  1.0 ,  1*Units.less],
                  [ 'mw_flap_span_frac_end'      , 0.4    , 0.3  , 0.5  ,  1.0 ,  1*Units.less],   
                  
    ],dtype=object)   

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # [ tag, scaling, units ]
    problem.objective = np.array([ 
                                  [ 'flap_surface_area', 1. , 1*Units.kg],
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        [ 'flap_criteria'    ,   '>' ,  0.   ,  1.0   , 1*Units.less],  
    ],dtype=object)
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ] 
    problem.aliases = [ 
        [ 'flap_surface_area'                 , 'summary.flap_surface_area' ], 
        [ 'flap_criteria'                     , 'summary.flap_criteria' ],   
        [ 'mw_flap_chord_fraction'            , 'vehicle_configurations.*.wings.main_wing.control_surfaces.flap.chord_fraction'],    
        [ 'mw_flap_span_frac_start'           , 'vehicle_configurations.*.wings.main_wing.control_surfaces.flap.span_fraction_start'],    
        [ 'mw_flap_span_frac_end'             , 'vehicle_configurations.*.wings.main_wing.control_surfaces.flap.span_fraction_end'],    
    ]      
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.flap_sizing_setup(optimized_vehicle)
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = None
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.flap_sizing_setup(nexus.analyses,nexus.vehicle_configurations.flap_sizing)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.flap_sizing_setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()     
    return nexus  
    
def print_vehicle_control_surface_geoemtry(vehicle): 
    for wing in vehicle.wings:
        if 'control_surfaces' in wing:  
            for CS in wing.control_surfaces:  
                print('Wing                : ' + wing.tag)
                print('Control Surface     : ' + CS.tag)
                print('Span Fraction Start : ' + str(CS.span_fraction_start))
                print('Span Fraction End   : ' + str(CS.span_fraction_end)) 
                print('Chord Fraction      : ' + str(CS.chord_fraction)) 
                print("\n\n")     

    return 
if __name__ == '__main__':
    main()
