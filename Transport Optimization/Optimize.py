# Optimize.py
# Optimize.py
# Created:  Feb 2016, M. Vegh
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
import Vehicles
import Analyses
import Missions
import Procedure
import Plot_Mission
from SUAVE.Optimization.Nexus import Nexus
import SUAVE.Optimization.Package_Setups.pyopt_setup as pyopt_setup

# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    

    iter = 0
    input_vector = np.array([])
    

    
    
    problem = setup() 
    output = pyopt_setup.Pyopt_Solve(problem)

    #output = scipy_setup.SciPy_Solve(problem)
    
    #output = problem.objective([ 1.,  1., 1., -1.,  1.,  1.,		
                                                  #1.,  1., 1., 1.,  1.,  1.,		
                                                   #1.,  1.,  1.,  1.,  1.,  1.,		
                                                    #1.])
                                                    
    #output = problem.objective([1.37703722,  1.19442195 , 0.22999966 , 1.27314355 , 1.31985865 ,0.11138383,
  #0.88106571 , 0.00384615 , 0.65065341 , 0.00625    , 1.16682949 , 0.95629193,
  #1.02090053 , 0.99130877 , 1.00000069 , 0.75160447 , 1.00000019 , 1.23636133,
  #0.95423206])    

    print output
    #print 'Extra translate'

    problem.translate()

    #problem.translate([ 0.90895071,  1.63642911, -1.00473092,  1.52173913,  1.31549092,
        #0.81789846,  1.00024095,  0.74359393,  0.83377009,  1.15592173,
        #1.15283596,  1.08165246,  0.99747007,  1.32816022,  1.        ,
        #1.09608415,  1.04495539])
    
    problem.translate(output[1])

    #Plot_Mission.plot_mission(problem.results.max_range)
    
    return


def variable_sweep(nvars,level):    
   
    var_array = np.zeros((nvars*(level-1)+1,nvars))
    zero_vec = np.zeros(nvars)
    delta = 1. / (level - 1)
    
    line = 1
    for variables in range(0,nvars):
        level_id = 1
        for levels in range(1,level):
            vec = 1. * zero_vec
            vec[variables] = level_id * delta
            var_array[line,:] = 1. * vec
            level_id += 1
            line += 1
    
    return var_array
    
# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  

def setup():

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    # [ tag , initial, [lb,ub], scaling, units ]
    problem.inputs = np.array([
        [ 'wing_loading'                 ,    610.   , (    286.    , 700   ) ,   500  , Units.kg/Units.meter**2],
        #[ 'wing_area'                    ,    92.   , (    70.    ,   200.   ) ,    92.  , Units.meter**2],
        ['cruise_altitude'               , 30,000.  , (25,000     ,  40,000  ) ,  10,000  , Units.ft      ],
        [ 'MTOW'                         , 56200.   , ( 20000.    ,100000.   ) , 51800.  ,       Units.kg],
        [ 'MZFW_ratio'                   , .765, (     0.7   ,     0.99 ) ,    0.807,     Units.less], 
        [ 'design_TOW'                   , 50000.   , ( 20000.    ,100000.   ) , 49000.  ,       Units.kg],
    ])
    
    
    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # throw an error if the user isn't specific about wildcards
    # [ tag, scaling, units ]
    problem.objective = np.array([
        [ 'fuel_burn', 10000, Units.kg ]
    ])
    
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        #[ 'wing_max_cl_norm' , '=' , 0. , 1. , Units.less],
        #[ 'MZFW consistency' , '>' , 0. , 1000. , Units.kg],
        [ 'MZFW consistency' , '>' , 0. , .1 , Units.less],
        [ 'design_range_fuel_margin' , '>', 0., .1, Units.less],
        [ 'short_field_fuel_margin' , '>' , 0. , .1, Units.less],
        [ 'max_range_fuel_margin' , '>' , 0. , .1, Units.less],
        [ 'wing_span' , '<', 118., 92., Units.ft],
        #[ 'engine_fan_diameter' , '<', 2.05, 2., Units.meters],
        [ 'noise_takeoff' , '<', 92.5, 92., Units.less],
        [ 'noise_sideline' , '<', 95., 95., Units.less],
        [ 'noise_landing' , '<', 99., 99., Units.less],
        #[ 'static_stability' , '<', 0., 1., Units.less],
        [ 'takeoff_field_length' , '<', 2056., 2000., Units.meters],
        [ 'landing_field_length' , '<', 1700., 1700., Units.meters],

        
        [ '2nd_segment_climb_max_range' , '>', .024, .024, Units.less],
        [ '2nd_segment_climb_short_field' , '>', .024, .024, Units.less],
        [ 'max_throttle' , '<', 1., 1, Units.less],

        #[ 'vertical_tail_volume_coefficient' , '>' , 0. , 1. , Units.less],
        #[ 'horizontal_tail_volume_coefficient' , '>' , 0. , 1. , Units.less],
        #[ 'max_range' , '>' , 2300. , 2300. , Units.nmi],
    ])
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        [ 'wing_loading'             , 'vehicle_configurations.*.wing_loading'],
        [ 'MTOW'                             ,   ['vehicle_configurations.*.mass_properties.takeoff'   ,
                                                  'vehicle_configurations.*.mass_properties.max_takeoff'               ]],
        [ 'design_TOW'                       ,    'vehicle_configurations.base.mass_properties.takeoff'                 ],
        [ 'cruise_altitude'                  , 'missions.base.segments.climb_5.altitude_end' ]
        [ 'fuel_burn'                        ,    'summary.base_mission_fuelburn'                                                   ],
        [ 'wing_span'                        ,    'vehicle_configurations.base.wings.main_wing.spans.projected'         ],
        #[ 'engine_fan_diameter'              ,    'vehicle_configurations.base.turbofan.nacelle_diameter'               ],
        [ 'noise_takeoff'                    ,    'summary.noise.takeoff'                                               ],
        [ 'noise_sideline'                   ,    'summary.noise.side_line'                                             ],
        [ 'noise_landing'                    ,    'summary.noise.landing'                                               ],
        [ 'static_stability'                 ,    'summary.static_stability'                                            ],
        [ 'vertical_tail_volume_coefficient' ,    'summary.vertical_tail_volume_coefficient'                            ],
        [ 'horizontal_tail_volume_coefficient',   'summary.horizontal_tail_volume_coefficient'                          ],
        [ 'wing_max_cl_norm'                 ,    'summary.maximum_cl_norm'                                             ],
        [ 'design_range_fuel_margin'         ,    'summary.max_zero_fuel_margin'                                        ],
        [ 'takeoff_field_length'             ,    'summary.takeoff_field_length'                                        ],
        [ 'landing_field_length'             ,    'summary.landing_field_length'                                        ],
        [ 'short_takeoff_field_length'       ,    'summary.short_takeoff_field_length'                                  ],
        [ '2nd_segment_climb_max_range'      ,    'summary.second_segment_climb_gradient_takeoff'                       ],
        [ '2nd_segment_climb_short_field'    ,    'summary.second_segment_climb_gradient_short_field'                   ],
        [ 'max_throttle'                     ,    'summary.max_throttle'                                                ],        
        [ 'short_field_fuel_margin'          ,    'summary.short_field_fuel_margin'                                     ],
        [ 'max_range_fuel_margin'            ,    'summary.max_range_fuel_margin'                                       ],
        [ 'max_range'                        ,    'missions.max_range_distance'                                         ],
        [ 'MZFW consistency'                 ,    'summary.MZFW_consistency'                                            ],
        [ 'MZFW_ratio'                       ,    'MZFW_ratio'                                                          ],
    ]    
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.setup()
    
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations)
    
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.setup(nexus.analyses)
    
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()    
    
    return nexus


if __name__ == '__main__':
    main()
    
    
