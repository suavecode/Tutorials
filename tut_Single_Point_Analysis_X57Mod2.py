# tut_Single_Point_Analsis_X57_Mod2.py
#
# Created: Oct 2021, M. Clarke 

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# SUAVE Imports
import SUAVE 
from SUAVE.Core import Data, Units
from SUAVE.Plots.Performance.Mission_Plots                   import *
from SUAVE.Plots.Geometry                                    import * 
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics import Aerodynamics 
from SUAVE.Components.Energy.Networks.Battery_Propeller      import Battery_Propeller
from SUAVE.Methods.Propulsion                                import propeller_design
from SUAVE.Methods.Power.Battery.Sizing                      import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing          import size_optimal_motor
from SUAVE.Methods.Geometry.Two_Dimensional.Planform         import segment_properties
 

# Python Imports
import numpy as np 
from copy import deepcopy
 
 
def main():  
    
    vehicle                                                   = vehicle_setup()    
    
    # Get properties of atmosphere at specified altitude 
    atmosphere                                                = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                                 = atmosphere.compute_values(altitude = 8012   * Units.feet) 
    
    # Define run conditions 
    run_conditions                                            = Aerodynamics()
    run_conditions.freestream.density                         = atmo_data.density[0,0] 
    run_conditions.freestream.gravity                         = 9.81           
    run_conditions.freestream.speed_of_sound                  = atmo_data.speed_of_sound[0,0]  
    run_conditions.aerodynamics.side_slip_angle               = 0.0
    run_conditions.aerodynamics.angle_of_attack               = np.array([0.0])  
    run_conditions.aerodynamics.lift_coefficient              = 0.547 
    run_conditions.freestream.velocity                        = 120.91 * Units['mph'] 
    run_conditions.freestream.mach_number                     = run_conditions.freestream.velocity/run_conditions.freestream.speed_of_sound
    run_conditions.aerodynamics.roll_rate_coefficient         = 0.07
    run_conditions.aerodynamics.pitch_rate_coefficient        = 0.0
    run_conditions.aerodynamics.side_slip_angle               = 0.0
    
    # Call AVL Stability Analysis
    stability_roll_maneuver                                   = SUAVE.Analyses.Stability.AVL() 
    stability_roll_maneuver.settings.filenames.avl_bin_name   = '/Users/matthewclarke/Documents/AVL/avl3.35' # change to path of AVL    
    stability_roll_maneuver.settings.number_spanwise_vortices = 40 
    stability_roll_maneuver.geometry                          = vehicle
    stability_roll_maneuver.geometry._base                    = Data()
    stability_roll_maneuver.geometry._base.tag                = vehicle.tag   
    results_roll_maneuver                                     = stability_roll_maneuver.evaluate_conditions(run_conditions, trim_aircraft = True) 
    
    # Extract data 
    CL                      = results_roll_maneuver.aerodynamics.lift_coefficient[0,0] 
    AoA                     = results_roll_maneuver.aerodynamics.angle_of_attack[0,0] 
    CD                      = results_roll_maneuver.aerodynamics.drag_breakdown.induced.total[0,0] 
    CM                      = results_roll_maneuver.aerodynamics.pitch_moment_coefficient[0,0]
    spiral_criteria         = results_roll_maneuver.stability.static.spiral_criteria[0,0]
    NP                      = results_roll_maneuver.stability.static.neutral_point[0,0]
    cg                      = vehicle.mass_properties.center_of_gravity[0][0]
    MAC                     = vehicle.wings.main_wing.chords.mean_aerodynamic
    static_margin           = (NP - cg)/MAC
    CM_alpha                = results_roll_maneuver.stability.static.Cm_alpha[0,0]  
    phugoid_damping_ratio   = results_roll_maneuver.dynamic_stability.LongModes.phugoidDamp[0,0]
    short_period_frequency  = results_roll_maneuver.dynamic_stability.LongModes.shortPeriodFreqHz[0,0] 
    dutch_roll_frequency    = results_roll_maneuver.dynamic_stability.LatModes.dutchRollFreqHz[0,0]
    spiral_doubling_time    = results_roll_maneuver.dynamic_stability.LatModes.spiralTimeDoubleHalf[0,0] 
    aileron_roll_deflection = results_roll_maneuver.stability.static.control_surfaces_cases['case_0001_0001'].control_surfaces.aileron.deflection 
    rudder_roll_deflection  = results_roll_maneuver.stability.static.control_surfaces_cases['case_0001_0001'].control_surfaces.rudder.deflection  

    print("\n\n")     
    print("************** RESULTS ************** ")        
    print("Angle of Attack        : " + str(AoA))   
    print("Lift Coefficient       : " + str(CL))
    print("Drag Coefficient       : " + str(CD))
    print("Moment Coefficient     : " + str(CM))
    print("Static Margin          : " + str(static_margin))
    print("CM alpla               : " + str(CM_alpha))   
    print("Phugoid Damping Ratio  : " + str(phugoid_damping_ratio))
    print("Short Period Frequency : " + str(short_period_frequency))
    print("Dutch Roll Frequency   : " + str(dutch_roll_frequency))
    print("Spiral Doubling Time   : " + str(spiral_doubling_time)) 
    print("Spiral Criteria        : " + str(spiral_criteria)) 
    print("Aileron Roll Defl      : " + str(aileron_roll_deflection)) 
    print("Rudder Roll Defl       : " + str(rudder_roll_deflection))  
    
    return   



def vehicle_setup():
    # ----------------------------------------------------------------------
    #   Define Vehicle
    # ---------------------------------------------------------------------  
     
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'X57_Mod2' 

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff   = 2550. * Units.pounds
    vehicle.mass_properties.takeoff       = 2550. * Units.pounds
    vehicle.mass_properties.max_zero_fuel = 2550. * Units.pounds 
    vehicle.mass_properties.moments_of_inertia.tensor = np.array([[164627.7,0.0,0.0],[0.0,471262.4,0.0],[0.0,0.0,554518.7]]) # Navion
    vehicle.envelope.ultimate_load        = 5.7
    vehicle.envelope.limit_load           = 3.8 
    vehicle.reference_area                = 14.76
    vehicle.passengers                    = 4
    vehicle.systems.control               = "fully powered"
    vehicle.systems.accessories           = "commuter"    
    
    cruise_speed                          = 135.*Units['mph']    
    altitude                              = 2500. * Units.ft
    atmo                                  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    freestream                            = atmo.compute_values (0.)
    freestream0                           = atmo.compute_values (altitude)
    mach_number                           = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.design_dynamic_pressure       = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.design_mach_number            =  mach_number
    
    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------    
    wing                                  = SUAVE.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 14.76
    wing.spans.projected                  = 11.4 
    wing.chords.root                      = 1.46
    wing.chords.tip                       = 0.92
    wing.chords.mean_aerodynamic          = 1.19
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 3.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[2.93, 0., 1.01]]
    wing.aerodynamic_center               = [3., 0., 1.01] 
    wing.vertical                         = False
    wing.symmetric                        = True
    wing.high_lift                        = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0  
    airfoil                               = SUAVE.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               = 'Airfoils/NACA_63_412.txt'
    
    cg_x = wing.origin[0][0] + 0.25*wing.chords.mean_aerodynamic
    cg_z = wing.origin[0][2] - 0.2*wing.chords.mean_aerodynamic
    vehicle.mass_properties.center_of_gravity = [[cg_x,   0.  ,  cg_z ]]  # SOURCE: Design and aerodynamic analysis of a twin-engine commuter aircraft

    # Wing Segments
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 3. * Units.degrees   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)

    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.5438
    segment.twist                         = 2.* Units.degrees 
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0. 
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    # Wing Segments
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'winglet'
    segment.percent_span_location         = 0.98
    segment.twist                         = 1.  * Units.degrees 
    segment.root_chord_percent            = 0.630
    segment.dihedral_outboard             = 75. * Units.degrees 
    segment.sweeps.quarter_chord          = 15. * Units.degrees 
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment) 

    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0. * Units.degrees 
    segment.root_chord_percent            = 0.12
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)    
    

    aileron                       = SUAVE.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.9 
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.2
    wing.append_control_surface(aileron)     

    flap                          = SUAVE.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.2
    flap.span_fraction_end        = 0.5
    flap.deflection               = 0.0 * Units.degrees 
    flap.chord_fraction           = 0.20
    wing.append_control_surface(flap)       
    
    # Fill out more segment properties automatically
    wing = segment_properties(wing)     
    
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------       
    wing                                  = SUAVE.Components.Wings.Wing()
    wing.tag                              = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 2.540 
    wing.spans.projected                  = 3.3  * Units.meter 
    wing.sweeps.quarter_chord             = 0 * Units.deg 
    wing.chords.root                      = 0.769 * Units.meter 
    wing.chords.tip                       = 0.769 * Units.meter 
    wing.chords.mean_aerodynamic          = 0.769 * Units.meter  
    wing.taper                            = 1. 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[7.7, 0., 0.25]]
    wing.aerodynamic_center               = [7.8, 0., 0.25] 
    wing.vertical                         = False
    wing.winglet_fraction                 = 0.0  
    wing.symmetric                        = True
    wing.high_lift                        = False 
    wing.dynamic_pressure_ratio           = 0.9  

    elevator                       = SUAVE.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.1
    elevator.span_fraction_end     = 0.9
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.3
    wing.append_control_surface(elevator)      
        
    # add to vehicle
    vehicle.append_component(wing)
    

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    wing                                  = SUAVE.Components.Wings.Wing()
    wing.tag                              = 'vertical_stabilizer'     
    wing.sweeps.quarter_chord             = 25. * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 2.258 * Units['meters**2']  
    wing.spans.projected                  = 1.854   * Units.meter  
    wing.chords.root                      = 1.6764 * Units.meter 
    wing.chords.tip                       = 0.6858 * Units.meter 
    wing.chords.mean_aerodynamic          = 1.21   * Units.meter 
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[6.75 ,0, 0.0]]
    wing.aerodynamic_center               = [0.508 ,0,0]  
    wing.vertical                         = True 
    wing.symmetric                        = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0 

    rudder                       = SUAVE.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.2
    rudder.span_fraction_end     = 0.8
    rudder.deflection            = 0.0  * Units.deg
    rudder.chord_fraction        = 0.2
    wing.append_control_surface(rudder)     
    
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'
    fuselage.seats_abreast                      = 2.
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.lengths.tail                       = 161. * Units.inches
    fuselage.lengths.cabin                      = 105. * Units.inches
    fuselage.lengths.total                      = 332.2* Units.inches
    fuselage.lengths.fore_space                 = 0.
    fuselage.lengths.aft_space                  = 0.
    fuselage.width                              = 42. * Units.inches
    fuselage.heights.maximum                    = 62. * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches
    fuselage.areas.side_projected               = 8000.  * Units.inches**2.
    fuselage.areas.wetted                       = 30000. * Units.inches**2.
    fuselage.areas.front_projected              = 42.* 62. * Units.inches**2.
    fuselage.effective_diameter                 = 50. * Units.inches 

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_0'
    segment.percent_x_location                  = 0
    segment.percent_z_location                  = 0
    segment.height                              = 0.01
    segment.width                               = 0.01
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  = 0.007279116466
    segment.percent_z_location                  = 0.002502014453
    segment.height                              = 0.1669064748
    segment.width                               = 0.2780205877
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.01941097724
    segment.percent_z_location                  = 0.001216095397
    segment.height                              = 0.3129496403
    segment.width                               = 0.4365777215
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.06308567604
    segment.percent_z_location                  = 0.007395489231
    segment.height                              = 0.5841726619
    segment.width                               = 0.6735119903
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.1653761217
    segment.percent_z_location                  = 0.02891281352
    segment.height                              = 1.064028777
    segment.width                               = 1.067200529
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.2426372155
    segment.percent_z_location                  = 0.04214148761
    segment.height                              = 1.293766653
    segment.width                               = 1.183058255
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.2960174029
    segment.percent_z_location                  = 0.04705241831
    segment.height                              = 1.377026712
    segment.width                               = 1.181540054
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  = 0.3809404284
    segment.percent_z_location                  = 0.05313580461
    segment.height                              = 1.439568345
    segment.width                               = 1.178218989
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_8'
    segment.percent_x_location                  = 0.5046854083
    segment.percent_z_location                  = 0.04655492473
    segment.height                              = 1.29352518
    segment.width                               = 1.054390707
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_9'
    segment.percent_x_location                  = 0.6454149933
    segment.percent_z_location                  = 0.03741966266
    segment.height                              = 0.8971223022
    segment.width                               = 0.8501926505
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_10'
    segment.percent_x_location                  = 0.985107095
    segment.percent_z_location                  = 0.04540283436
    segment.height                              = 0.2920863309
    segment.width                               = 0.2012565415
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_11'
    segment.percent_x_location                  = 1
    segment.percent_z_location                  = 0.04787575562
    segment.height                              = 0.1251798561
    segment.width                               = 0.1206021048
    fuselage.Segments.append(segment)

    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'nacelle_1'
    nacelle.length         = 2
    nacelle.diameter       = 42 * Units.inches
    nacelle.areas.wetted   = 0.01*(2*np.pi*0.01/2)
    nacelle.origin         = [[2.5,2.5,1.0]]
    nacelle.flow_through   = False  
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.1  
    nac_segment.height             = 0.5
    nac_segment.width              = 0.65
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.3  
    nac_segment.height             = 0.52
    nac_segment.width              = 0.7
    nacelle.append_segment(nac_segment)  
     
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.5  
    nac_segment.height             = 0.5
    nac_segment.width              = 0.65
    nacelle.append_segment(nac_segment)  
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.7 
    nac_segment.height             = 0.4
    nac_segment.width              = 0.6
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_6'
    nac_segment.percent_x_location = 0.9 
    nac_segment.height             = 0.3
    nac_segment.width              = 0.5
    nacelle.append_segment(nac_segment)  
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_7'
    nac_segment.percent_x_location = 1.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)    
    
    vehicle.append_component(nacelle)  

    nacelle_2          = deepcopy(nacelle)
    nacelle_2.tag      = 'nacelle_2'
    nacelle_2.origin   = [[2.5,-2.5,1.0]]
    vehicle.append_component(nacelle_2)    
    
    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    # build network
    net = Battery_Propeller()
    net.number_of_propeller_engines  = 2. 
    net.identical_propellers         = True 

    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc

    # Component 2 the Propeller 
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.tag = 'propeller_1'
    prop.number_of_blades       = 2.0
    prop.freestream_velocity    = 135.*Units['mph']
    prop.angular_velocity       = 1300.  * Units.rpm
    prop.tip_radius             = 76./2. * Units.inches
    prop.hub_radius             = 8.     * Units.inches
    prop.design_Cl              = 0.8
    prop.design_altitude        = 12000. * Units.feet
    prop.design_altitude        = 12000. * Units.feet
    prop.design_thrust          = 1200.
    prop.origin                 = [[2.,2.5,0.784]]
    prop.rotation               = -1
    prop.symmetry               = True
    prop.variable_pitch         = True 
    prop.airfoil_geometry       =  ['Airfoils/NACA_4412.txt']
    prop.airfoil_polars         = [['Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                    'Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                    'Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                    'Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                    'Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

    prop.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    prop                        = propeller_design(prop)

    prop_left = deepcopy(prop)
    prop_left.tag = 'propeller_2' 
    prop_left.origin   = [[2.,-2.5,0.784]]
    prop_left.rotation = 1
    
    net.propellers.append(prop)
    net.propellers.append(prop_left)


    # Component 3 the Battery 
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiFePO4_18650()  
    
    bat.mass_properties.mass = 500. * Units.kg  
    bat.max_voltage          = 500.             
    initialize_from_mass(bat)
    
    # Assume a battery pack module shape. This step is optional but
    # required for thermal analysis of the pack
    number_of_modules                = 10
    bat.module_config.total          = int(np.ceil(bat.pack_config.total/number_of_modules))
    bat.module_config.normal_count   = int(np.ceil(bat.module_config.total/bat.pack_config.series))
    bat.module_config.parallel_count = int(np.ceil(bat.module_config.total/bat.pack_config.parallel))
    net.battery                      = bat      
    
    net.battery              = bat
    net.voltage              = bat.max_voltage   

    # Component 4 Miscellaneous Systems
    sys = SUAVE.Components.Systems.System()
    sys.mass_properties.mass = 5 # kg
 
    # Component 5 the Motor  
    motor                         = SUAVE.Components.Energy.Converters.Motor()
    motor.efficiency              = 0.95
    motor.gearbox_efficiency      = 1.
    motor.origin                  = [[2.,  2.5, 0.784]]
    motor.nominal_voltage         = bat.max_voltage *3/4
    motor.propeller_radius        = prop.tip_radius
    motor.no_load_current         = 4.0
    motor                         = size_optimal_motor(motor,prop)
    motor.mass_properties.mass    = 10. * Units.kg 
    
    # append right motor
    net.propeller_motors.append(motor)
    
    # append left motor 
    motor_left = deepcopy(motor)
    motor_left.origin = [[2., -2.5, 0.784]] 
    net.propeller_motors.append(motor_left) 

    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 10. # Watts
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload                  = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 20. # Watts
    net.avionics        = avionics

    # add the solar network to the vehicle
    vehicle.append_component(net)

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    
    return vehicle
# ---------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)  

    # done!
    return configs 


if __name__ == '__main__': 
    main()     