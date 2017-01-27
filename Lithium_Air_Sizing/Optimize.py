# Optimize.py
# 
# Created:  Jan 2017, M. Vegh
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
import Vehicle
import Analyses
import Mission
import Sizing as Procedure
import Plot_Mission
from SUAVE.Optimization.Nexus import Nexus
import SUAVE.Optimization.Package_Setups.pyopt_setup as pyopt_setup
from SUAVE.Sizing.Sizing_Loop import Sizing_Loop
# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    
    problem = setup()

    output = pyopt_setup.Pyopt_Solve(problem, sense_step = 1E-2, solver='SNOPT')
    print output
    
    Plot_Mission.plot_mission(problem)
    

    return

# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  

def setup():

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem
    nexus.sizing_loop = Sizing_Loop()

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------   
    
    # [ tag , initial, [lb,ub], scaling, units ]
    problem.inputs = np.array([
        [ 'thrust_loading'            ,   .2          , (.05       ,      .3    ) ,  .1,  'continuous',               Units.less],   
        [  'fan_pressure_ratio'       ,  1.5          , (1.05      ,   2.6    ) ,  1.,  'continuous',               Units.less], 
        #[  'aspect_ratio'             ,  14.         , (5.        ,    14.     ) ,  10., 'continuous',               Units.less], 
        [  'wing_loading'             ,  400.         , (200.      ,    800.    ) ,  100.,'continuous', Units.kg/Units.meter**2.],
        #[  'taper'                    ,  0.1000      , (.1        ,    .3      ) ,  .1,  'continuous',              Units.less ],
        [  'wing_thickness'           ,  .105         , (0.07      ,    0.20    ) ,  .1,  'continuous',              Units.less ],
        [  'v_climb_1'                ,  80.          , (50.       ,    230.    ) ,  1E2, 'continuous', Units.meter/Units.second],
        [  'v_climb_2'                ,  100.         , (50.       ,    230    ) ,  1E2, 'continuous', Units.meter/Units.second],
        [  'v_climb_3'                ,  140          , (50.       ,  230     ) ,  1E2, 'continuous', Units.meter/Units.second],
        [  'cruise_altitude'          ,  33000.       , (20000.    ,    35000   ) ,  1E4, 'continuous',                 Units.ft],
        [  'climb_alt_fraction_1'     ,  0.1          , (.1        ,    1.      ) ,  1.,  'continuous',               Units.less],
        [  'climb_alt_fraction_2'     , .2            , (.2        ,    1.      ) ,  1.,  'continuous',               Units.less],
        [  'descent_alt_fraction_1'   ,  .2           , (.1        ,    1.      ) ,  1.,  'continuous',               Units.less],
    ])
    
    
    
    

    
    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # throw an error if the user isn't specific about wildcards
    # [ tag, scaling, units ]
    problem.objective = np.array([
        [ 'landing_weight', 1E4, Units.kg ]
    ])
    
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    
    problem.constraints = np.array([
        #[ 'power_margin'               , '>',  0     ,  1E6   , Units.meters],
        #['lithium_ion_energy_margin'   , '>',  0      ,   1E0  , Units.less],
        [ 'climb_constraint_1'          , '>',  0     ,  1E2   , Units.meters],
        [ 'climb_constraint_2'         , '>',  0     ,  1E3   , Units.meters], #handled by bounds
        [ 'descent_constraint_1'        , '>',  0     ,  1E2   , Units.meters],
        [ 'min_alpha_constraint'        , '>',  0     , 1E1    , Units.degrees],
        [ 'max_alpha_constraint'        , '>',  0     , 1E1    , Units.degrees],
        [ 'takeoff_field_constraint'    , '>',  0     , 1E3    , Units.ft],
        [ 'landing_field_constraint'    , '>',  0     , 1E3    , Units.ft],
        [ 'max_throttle_constraint'     , '>',  0     ,   1    , Units.less],
        ['climb_velocity_constraint_1'  , '>',  0     , 1E2   , Units.meters/Units.second],
        ['climb_velocity_constraint_2'  , '>',  0     , 1E2   , Units.meters/Units.second],
        [ 'iteration_constraint'        , '>',  0     ,   1    , Units.less],
        
    ])
    
    '''
    problem.constraints = np.array([
        [ 'power_margin'           , '>',  0   ,  1E6   , Units.meters],
        #[ 'range_margin'           , '>',  0   ,  1E6   , Units.meters],
        [ 'washout'                , '>',  0   ,  1E0   , Units.radians],
        [ 'climb_constraint_1'     , '>',  0   ,  1E3   , Units.km],
        [ 'climb_constraint_2'     , '>',  0   ,  1E3   , Units.km],
        [ 'climb_constraint_3'     , '>',  0   ,  1E3   , Units.km],
        [ 'climb_constraint_4'     , '>',  0   ,  1E3   , Units.km],
        [ 'descent_constraint_1'   , '>',  0   ,  1E3   , Units.km],
        [ 'descent_constraint_2'   , '>',  0   ,  1E3   , Units.km],
        [ 'min_alpha_constraint'   , '>',  0   , 1E1    , Units.degrees],
        [ 'max_alpha_constraint'   , '>',  0   , 1E1    , Units.degrees],
        #[ 'takeoff_field_constraint'   , '>',  0   , 1E3    , Units.ft],
        #[ 'landing_field_constraint'   , '>',  0   , 1E3    , Units.ft],
   
    ])
    '''
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        #Inputs

   
        [ 'thrust_loading'           ,  'vehicle_configurations.*.thrust_loading'          ],
        [ 'fan_pressure_ratio'       , 'vehicle_configurations.*.energy_network.propulsor.fan.pressure_ratio'],
                                       
        [ 'aspect_ratio'             ,  'vehicle_configurations.*.wings.main_wing.aspect_ratio'      ],
        
        [ 'wing_loading'             , 'vehicle_configurations.*.wing_loading'],
        [ 'wing_thickness'           ,  'vehicle_configurations.*.wings.main_wing.thickness_to_chord'],
        [ 'taper'                    ,  'vehicle_configurations.*.wings.main_wing.taper'],
                                     
        [ 'climb_alt_fraction_1'     , 'missions.base.climb_alt_fraction_1'],
        [ 'climb_alt_fraction_2'     , 'missions.base.climb_alt_fraction_2'],
        [ 'descent_alt_fraction_1'   , 'missions.base.descent_alt_fraction_1' ],
        [ 'cruise_altitude'          , 'missions.base.segments.climb_3.altitude_end' ],
        [ 'v_climb_1'                , 'missions.base.segments.climb_1.air_speed'],
        [ 'v_climb_2'                , 'missions.base.segments.climb_2.air_speed'],
        [ 'v_climb_3'                , 'missions.base.segments.climb_3.air_speed'],

        
        #outputs
        [ 'landing_weight'           ,  'results.segments[-1].conditions.weights.total_mass[-1,0]'   ],
        [ 'total_range'              ,  'results.total_range'  ], 
        [ 'max_power'                ,  'results.Pmax'   ],

        #constraints
        [ 'max_throttle_constraint'  ,    'results.max_throttle_constraint'        ],   
        [ 'climb_constraint_1'   ,  'results.climb_constraint_1'    ], 
        [ 'climb_constraint_2'   ,  'results.climb_constraint_2'    ], 
        [ 'descent_constraint_1' ,  'results.descent_constraint_1'  ], 
        [ 'max_alpha_constraint'     ,  'results.max_alpha_constraint'  ],
        [ 'min_alpha_constraint'     ,  'results.min_alpha_constraint'  ], 
        [ 'takeoff_field_constraint' ,  'results.takeoff_field_constraint'  ],  
        [ 'landing_field_constraint' ,  'results.landing_field_constraint'  ], 
        [ 'cost_constraint'          ,  'results.cost_constraint'  ],
        [ 'climb_velocity_constraint_1' ,  'results.climb_velocity_constraint_1'  ],
        [ 'climb_velocity_constraint_2' ,  'results.climb_velocity_constraint_2'  ],
        [ 'iteration_constraint'     ,  'iteration_constraint'],  

    ]    
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicle.setup()
    
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations)
    
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Mission.setup(nexus.analyses)
    
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.setup()
    nexus.total_number_of_iterations = 0
    
    return nexus


if __name__ == '__main__':
    main()
    
    
