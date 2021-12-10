# tut_Aerodynamic_Polars_X57.py
#
# Created: Dec 2021, M. Clarke

""" setup file for the Whisper Drone Vehicle Polar Analysis 
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# SUAVE Imports
import SUAVE
#assert SUAVE.__version__=='2.5.0', 'These tutorials only work with the SUAVE 2.5.0 release'

from SUAVE.Core import Units, Data
from SUAVE.Plots.Performance.Mission_Plots                                import *
from SUAVE.Plots.Geometry                                                 import *
from SUAVE.Methods.Aerodynamics.Common.Fidelity_Zero.Lift                 import VLM  
from SUAVE.Components.Energy.Networks.Lift_Cruise                         import Lift_Cruise 
from SUAVE.Methods.Propulsion                                             import propeller_design
from SUAVE.Methods.Power.Battery.Sizing                                   import initialize_from_mass 
from SUAVE.Methods.Propulsion.electric_motor_sizing                       import size_optimal_motor
from SUAVE.Methods.Geometry.Two_Dimensional.Planform                      import segment_properties  
from copy import deepcopy
import matplotlib.cm as cm    

# ----------------------------------------------------------------------
#   Generic Function Call
# ---------------------------------------------------------------------- 
def main(): 
    # call X57 - Mod 3 vehicle setup function 
    vehicle = vehicle_setup()  
    
    # define wing and control surface
    wing_tag                           = 'main_wing' 
    control_surface_tag                = 'flap' # can leave empty ie.  control_surface_tag  = ''
    control_surface_deflection_angles  = np.linspace(-10,20,7)*Units.degrees # can be zero i.e. control_surface_deflection_angles = np.array([0])
    airspeed                           = 150.* Units['mph']  
    altitude                           = 2500.0*Units.feet
    alpha_range                        = np.linspace(-2,10,13)*Units.degrees 
    
    # call polar analysis function 
    setup_vehicle_polar_analyses(vehicle,wing_tag,control_surface_tag,control_surface_deflection_angles,airspeed,altitude ,alpha_range)
        
    return 
 
# -----------------------------------------
# Setup for Aircraft Polars  
# -----------------------------------------
def setup_vehicle_polar_analyses(vehicle,wing_tag,control_surface_tag,deflection_angles,V,Alt,alpha_range):
    
    MAC      = vehicle.wings['main_wing'].chords.mean_aerodynamic
    S_ref    = vehicle.reference_area   
    num_def = len(deflection_angles)
    
    #------------------------------------------------------------------------
    # setup figures
    #------------------------------------------------------------------------ 

    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 24,
                  'legend.fontsize': 20,
                  'xtick.labelsize': 24,
                  'ytick.labelsize': 24,
                  'axes.titlesize': 28}
    plt.rcParams.update(parameters)
    
    header_text = ' : $S_{ref}$ = ' + str(round(S_ref,2)) + ' MAC =' + str(round(MAC,2))
    
    fig1 = plt.figure('C_L_vs_AoA')
    fig1.set_size_inches(8,6)
    axes1 = fig1.add_subplot(1,1,1)
    axes1.set_title('$C_L$ vs AoA' + header_text )
    axes1.set_ylabel('Coefficient of Lift')
    axes1.set_xlabel('Angle of Attack (degrees)') 

    fig2 = plt.figure('C_Di_vs_AoA')
    fig2.set_size_inches(8,6)
    axes2 = fig2.add_subplot(1,1,1)
    axes2.set_title('$C_{Di}$ vs. AoA'+ header_text)
    axes2.set_ylabel('Coefficient of Drag')
    axes2.set_xlabel('Angle of Attack (degrees)') 

    fig3 = plt.figure('C_Di_vs_C_L2')
    fig3.set_size_inches(8,6)
    axes3 = fig3.add_subplot(1,1,1)
    axes3.set_title('$C_{Di}$ vs. $C_L^{2}$'+ header_text)
    axes3.set_ylabel('Coefficient of Drag')
    axes3.set_xlabel('Lineraized Coefficient of Lift ($CL^2$)') 

    fig4 = plt.figure('C_M_vs_AoA')
    fig4.set_size_inches(8,6)
    axes4 = fig4.add_subplot(1,1,1)
    axes4.set_title('$C_M$ vs. AoA'+ header_text)
    axes4.set_ylabel('Coefficient of Moment')
    axes4.set_xlabel('Angle of Attack (degrees)') 
    
    linecolor_1 = cm.jet(np.linspace(0, 1,num_def))   
    linestyle_1 = ['-']*num_def 
    marker_1    = ['o']*num_def  

    #------------------------------------------------------------------------
    # setup flight conditions
    #------------------------------------------------------------------------ 
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(altitude=Alt) 
    a              = atmo_data.speed_of_sound[0] 
    Mach           = V/a  
    AoA_range      = np.atleast_2d(alpha_range).T  
    
    # check if control surface is defined 
    CS_flag = False
    for wing in vehicle.wings:
        if 'control_surfaces' in wing:
            if control_surface_tag in wing.control_surfaces:
                CS_flag = True 
    
    for i in range (num_def): 
        # change control surface deflection  
        if CS_flag:
            vehicle.wings[wing_tag].control_surfaces[control_surface_tag].deflection = deflection_angles[i] 
        
        #  compute polar
        results    = compute_polars(vehicle,AoA_range,Mach,Alt) 
        if CS_flag:
            line_label =  wing_tag + ',' + control_surface_tag + ' ' +\
                str(round( deflection_angles[i]/Units.degrees,3)) + '$\degree$ defl.'
        else: 
            line_label =  ''
        
        # plot  
        plot_polars(axes1,axes2,axes3,axes4,AoA_range,Mach,results,linestyle_1[i], linecolor_1[i],marker_1[i],line_label)
        
    # append legend   
    axes1.legend(loc='upper left', prop={'size': 14})   
    axes2.legend(loc='upper left', prop={'size': 14})   
    axes3.legend(loc='upper left', prop={'size': 14})   
    axes4.legend(loc='upper right', prop={'size': 14})    
    
    # format figure 
    fig1.tight_layout()  
    fig2.tight_layout() 
    fig3.tight_layout() 
    fig4.tight_layout()    
    return 

 
# -----------------------------------------
# Compute Aircraft Polars  
# -----------------------------------------
def compute_polars(vehicle,AoA_range,Mach,Alt):  

    MAC            = vehicle.wings['main_wing'].chords.mean_aerodynamic 
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(altitude=Alt)
    P              = atmo_data.pressure[0]
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]  
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0] 
    V              = a*Mach
    re             = (V*rho*MAC)/mu  
    
    n_aoa          = len(AoA_range) 
    vortices       = 4 
    
    state = SUAVE.Analyses.Mission.Segments.Conditions.State()
    state.conditions = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics() 
    state.conditions.freestream.mach_number       = Mach  * np.ones_like(AoA_range)
    state.conditions.freestream.density           = rho * np.ones_like(AoA_range)
    state.conditions.freestream.dynamic_viscosity = mu  * np.ones_like(AoA_range)
    state.conditions.freestream.temperature       = T   * np.ones_like(AoA_range)
    state.conditions.freestream.pressure          = P   * np.ones_like(AoA_range)
    state.conditions.freestream.reynolds_number   = re  * np.ones_like(AoA_range)
    state.conditions.freestream.velocity          = V   * np.ones_like(AoA_range)
    state.conditions.aerodynamics.angle_of_attack = AoA_range  
 
    # -----------------------------------------------------------------
    # VLM No Surrogate (Inviscid)
    # -----------------------------------------------------------------
    settings = Data()
    settings.use_surrogate = False
    settings.number_spanwise_vortices         = vortices **2
    settings.number_chordwise_vortices        = vortices
    settings.propeller_wake_model             = False
    settings.initial_timestep_offset          = 0
    settings.wake_development_time            = 0.05
    settings.use_bemt_wake_model              = False
    settings.number_of_wake_timesteps         = 30
    settings.leading_edge_suction_multiplier  = 1.0
    settings.spanwise_cosine_spacing          = True
    settings.model_fuselage                   = False
    settings.model_nacelle                    = False
    settings.wing_spanwise_vortices           = None
    settings.wing_chordwise_vortices          = None
    settings.fuselage_spanwise_vortices       = None
    settings.discretize_control_surfaces      = True
    settings.fuselage_chordwise_vortices      = None
    settings.floating_point_precision         = np.float32
    settings.use_VORLAX_matrix_calculation    = False 
    settings.use_surrogate                    = True 
    results =  VLM(state.conditions,settings,vehicle)
     
    
    # pack results 
    Aero_Results = Data()
    Aero_Results.CL_Inv  = results.CL 
    Aero_Results.CDi_Inv = results.CDi
    Aero_Results.CM_Inv  = results.CM 

    
    # plot aircraft 
    plot_vehicle_vlm_panelization(vehicle, elevation_angle = 30,azimuthal_angle = 135, axis_limits = 6  ,plot_control_points = False,save_filename = 'X47_M3')    
    
    return Aero_Results 

 
# -----------------------------------------
# Plot Polars Aircraft Polars  
# -----------------------------------------
def plot_polars(axes1,axes2,axes3,axes4,AoA_range,Mach,results,linestyle_1, 
                linecolor_1,marker_1,line_label): 

    CL_Inv  = results.CL_Inv
    CDi_Inv = results.CDi_Inv
    CM_Inv  = results.CM_Inv   
    
    axes1.plot(AoA_range/Units.degrees,CL_Inv,linestyle = linestyle_1,linewidth = 2, markersize = 10, color = linecolor_1, marker = marker_1,label = line_label)  
    
    axes2.plot(AoA_range/Units.degrees,CDi_Inv,linestyle = linestyle_1,linewidth = 2, markersize = 10, color = linecolor_1, marker = marker_1,label = line_label)  
     
    axes3.plot(CL_Inv**2,CDi_Inv,linestyle = linestyle_1,linewidth = 2, markersize = 10, color = linecolor_1, marker = marker_1,label = line_label)  
     
    axes4.plot(AoA_range/Units.degrees,CM_Inv,linestyle = linestyle_1,linewidth = 2, markersize = 10, color = linecolor_1, marker = marker_1,label = line_label) 
    
    return
  
def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'X57_Modification_3'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff       = 2550. * Units.pounds
    vehicle.mass_properties.takeoff           = 2550. * Units.pounds
    vehicle.mass_properties.max_zero_fuel     = 2550. * Units.pounds
    vehicle.mass_properties.cargo             = 0.
    vehicle.mass_properties.center_of_gravity = [[ 3.35,   0. , 0.34   ]]

    # envelope properties
    vehicle.envelope.ultimate_load = 5.7
    vehicle.envelope.limit_load    = 3.8

    # basic parameters
    vehicle.reference_area = 66.66 *Units.feet**2
    vehicle.passengers     = 4

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing                         = SUAVE.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'

    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.sweeps.leading_edge     = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.areas.reference         = 66.66 *Units.feet**2
    wing.spans.projected         = 31.633 * Units.feet
    wing.chords.root             = 0.7 * Units.meter
    wing.chords.tip              = 0.6 * Units.meter
    wing.chords.mean_aerodynamic = 0.649224 # 2.13 * Units.feet
    wing.taper                   = wing.chords.tip / wing.chords.root 
    wing.aspect_ratio            = wing.spans.projected ** 2. / wing.areas.reference 
    wing.twists.root             = 0.0 * Units.degrees  
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [[3.05, 0., 0.784]]
    wing.aerodynamic_center      = [0.558, 0., 0.784] 
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    airfoil                      = SUAVE.Components.Airfoils.Airfoil()
    #airfoil.coordinate_file      = 'NACA_63_412.txt' 
    wing.append_airfoil(airfoil)
    wing.dynamic_pressure_ratio  = 1.0

    # Wing Segments
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 3. * Units.degrees   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    #segment.append_airfoil(airfoil)
    wing.append_segment(segment) 
   
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 3. * Units.degrees 
    segment.root_chord_percent            = wing.taper
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    #segment.append_airfoil(airfoil)
    wing.append_segment(segment)    


    flap                          = SUAVE.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.15
    flap.span_fraction_end        = 0.8
    flap.deflection               = 20.0 * Units.degrees 
    flap.chord_fraction           = 0.20
    wing.append_control_surface(flap)
    
    segment_properties(wing)
    
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
    wing.twists.root                      = 1.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[7.7, 0., 0.25]]
    wing.aerodynamic_center               = [7.8, 0., 0.25] 
    wing.vertical                         = False
    wing.winglet_fraction                 = 0.0  
    wing.symmetric                        = True
    wing.high_lift                        = False 
    wing.dynamic_pressure_ratio           = 0.9

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
    wing.origin                           = [[6.75 ,0, 0.]]
    wing.aerodynamic_center               = [0.508 ,0,0]  
    wing.vertical                         = True 
    wing.symmetric                        = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'
    fuselage.seats_abreast = 2.
    fuselage.fineness.nose = 1.6
    fuselage.fineness.tail = 2.
    fuselage.lengths.nose = 60. * Units.inches
    fuselage.lengths.tail = 161. * Units.inches
    fuselage.lengths.cabin = 105. * Units.inches
    fuselage.lengths.total = 332.2 * Units.inches
    fuselage.lengths.fore_space = 0.
    fuselage.lengths.aft_space = 0.
    fuselage.width = 42. * Units.inches
    fuselage.heights.maximum = 62. * Units.inches
    fuselage.heights.at_quarter_length = 62. * Units.inches
    fuselage.heights.at_three_quarters_length = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches
    fuselage.areas.side_projected = 8000. * Units.inches ** 2.
    fuselage.areas.wetted = 30000. * Units.inches ** 2.
    fuselage.areas.front_projected = 42. * 62. * Units.inches ** 2.
    fuselage.effective_diameter = 50. * Units.inches

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_0'
    segment.percent_x_location = 0
    segment.percent_z_location = 0
    segment.height = 0.01
    segment.width = 0.01
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_1'
    segment.percent_x_location = 0.007279116466
    segment.percent_z_location = 0.002502014453
    segment.height = 0.1669064748
    segment.width = 0.2780205877
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_2'
    segment.percent_x_location = 0.01941097724
    segment.percent_z_location = 0.001216095397
    segment.height = 0.3129496403
    segment.width = 0.4365777215
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_3'
    segment.percent_x_location = 0.06308567604
    segment.percent_z_location = 0.007395489231
    segment.height = 0.5841726619
    segment.width = 0.6735119903
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_4'
    segment.percent_x_location = 0.1653761217
    segment.percent_z_location = 0.02891281352
    segment.height = 1.064028777
    segment.width = 1.067200529
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_5'
    segment.percent_x_location = 0.2426372155
    segment.percent_z_location = 0.04214148761
    segment.height = 1.293766653
    segment.width = 1.183058255
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_6'
    segment.percent_x_location = 0.2960174029
    segment.percent_z_location = 0.04705241831
    segment.height = 1.377026712
    segment.width = 1.181540054
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_7'
    segment.percent_x_location = 0.3809404284
    segment.percent_z_location = 0.05313580461
    segment.height = 1.439568345
    segment.width = 1.178218989
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_8'
    segment.percent_x_location = 0.5046854083
    segment.percent_z_location = 0.04655492473
    segment.height = 1.29352518
    segment.width = 1.054390707
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_9'
    segment.percent_x_location = 0.6454149933
    segment.percent_z_location = 0.03741966266
    segment.height = 0.8971223022
    segment.width = 0.8501926505
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_10'
    segment.percent_x_location = 0.985107095
    segment.percent_z_location = 0.04540283436
    segment.height = 0.2920863309
    segment.width = 0.2012565415
    fuselage.Segments.append(segment)

    # Segment
    segment = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag = 'segment_11'
    segment.percent_x_location = 1
    segment.percent_z_location = 0.04787575562
    segment.height = 0.1251798561
    segment.width = 0.1206021048
    fuselage.Segments.append(segment)

    # add to vehicle
    vehicle.append_component(fuselage)

    #---------------------------------------------------------------------------------------------
    # DEFINE NETWORK
    #--------------------------------------------------------------------------------------------- 
    # Component 1  
    net = Lift_Cruise() 
    net.number_of_propeller_engines = 2
    net.propeller_thrust_angle      = 0.   * Units.degrees
    net.propeller_nacelle_diameter  = 1.166 * Units.feet  
    net.propeller_engine_length     = 3  * Units.feet
    
    net.number_of_rotor_engines     = 12
    net.rotor_thrust_angle          = 0.   * Units.degrees
    net.rotor_nacelle_diameter      = 0.5  * Units.feet  
    net.rotor_engine_length         = 1 * Units.feet    
     
    net.voltage                     = 400.
    
    # Component 2 Electronic Speed Controller -------------------------------------------------------- 
    rotor_esc              = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    rotor_esc.efficiency   = 0.95
    net.rotor_esc          = rotor_esc 

    propeller_esc            = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    propeller_esc.efficiency = 0.95
    net.propeller_esc        = propeller_esc
    
    # Component 3 the Propeller  ------------------------------------------------------------- 
    # Cruise Propeller 
    prop_cr = SUAVE.Components.Energy.Converters.Propeller()  
    prop_cr.tag                 = 'cruise_propeller'   
    prop_cr.number_of_blades    = 3 
    prop_cr.freestream_velocity = 173.984 * Units['mph']       
    prop_cr.tip_radius          = 1.523/2
    prop_cr.hub_radius          = 0.1     
    prop_cr.design_Cl           = 0.75
    prop_cr.design_tip_mach     = 0.6   
    prop_cr.angular_velocity    = 2250 * Units.rpm #  prop_cr.design_tip_mach*speed_of_sound/prop_cr.tip_radius  
    prop_cr.design_altitude     = 8000. * Units.feet 
    prop_cr.design_power        = 48100 # 115.  
    prop_cr_origins              = [[2.5, 4.97584, 1.01],[2.5, -4.97584, 1.01]]     
    prop_cr_rotations            = [-1,1] 
    prop_cr                     = propeller_design(prop_cr) 
    prop_cr.symmetry            = True
    

    # Appending rotors with different origins    
    for ii in range(net.number_of_propeller_engines):
        cruise_prop                        = deepcopy(prop_cr)
        cruise_prop.tag                    = 'cruise_prop_' + str(ii+1)
        cruise_prop.rotation               = prop_cr_rotations[ii]
        cruise_prop.origin                 = [prop_cr_origins[ii]] 
        net.propellers.append(cruise_prop)    
        
    # Design Highlift Propeller 
    prop_hl = SUAVE.Components.Energy.Converters.Rotor()  
    prop_hl.tag                 = 'high_lift_propeller'   
    prop_hl.number_of_blades    = 5   
    prop_hl.freestream_velocity = 63.379  * Units['mph']        
    prop_hl.tip_radius          = 0.58/2 
    prop_hl.hub_radius          = 0.1     
    prop_hl.design_Cl           = 0.75
    prop_hl.design_tip_mach     = 0.6   
    prop_hl.angular_velocity    = 2250 * Units.rpm  
    prop_hl.design_altitude     = 100. * Units.feet 
    prop_hl.design_power        = 1400   
    prop_pitch                  = 0.6 
    prop_hl_origins              = [[2.5, (1.05 + prop_pitch*0), 1.01], [2.5, (1.05 + prop_pitch*1), 1.01],[2.5, (1.05 + prop_pitch*2), 1.01],
                                   [2.5, (1.05 + prop_pitch*3), 1.01],[2.5,  (1.05 + prop_pitch*4), 1.01],[2.5, (1.05 + prop_pitch*5), 1.01],
                                   [2.5,-(1.05 + prop_pitch*0) ,1.01], [2.5,-(1.05 + prop_pitch*1), 1.01],[2.5,-(1.05 + prop_pitch*2), 1.01],
                                   [2.5,-(1.05 + prop_pitch*3), 1.01],[2.5 ,-(1.05 + prop_pitch*4), 1.01],[2.5,-(1.05 + prop_pitch*5), 1.01]]     
    prop_hl_rotations            = [-1,-1,-1,-1,-1,-1,1,1,1,1,1,1] 
    prop_hl                     = propeller_design(prop_hl) 
    prop_hl.symmetry            = True
        
    for ii in range(net.number_of_rotor_engines):
        prop_high_lift                        = deepcopy(prop_hl)
        prop_high_lift.tag                    = 'high_lift_propeller_' + str(ii+1)
        prop_high_lift.rotation               = prop_hl_rotations[ii]
        prop_high_lift.origin                 = [prop_hl_origins[ii]] 
        net.lift_rotors.append(prop_high_lift)   
        
    # Component 4 the Battery --------------------------------------------------------------------
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 300. * Units.kg  
    bat.specific_energy      = 200. * Units.Wh/Units.kg
    bat.resistance           = 0.006
    bat.max_voltage          = 400.
    
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat 
    net.voltage              = bat.max_voltage
    
    # Component 5 the Motor --------------------------------------------------------------------
    # Cruise Propeller  motor 
    motor_cr                      = SUAVE.Components.Energy.Converters.Motor() 
    motor_cr.efficiency           = 0.95
    motor_cr.gearbox_efficiency   = 1.  
    motor_cr.nominal_voltage      = bat.max_voltage*0.75
    motor_cr.propeller_radius     = prop_cr.tip_radius    
    motor_cr.no_load_current      = 0.1
    motor_cr.origin               = prop_cr.origin 
    motor_cr                      = size_optimal_motor(motor_cr,prop_cr) 
    net.propeller_motor           = motor_cr  
    
    # High Lift Propeller  motor 
    motor_hl                      = SUAVE.Components.Energy.Converters.Motor() 
    motor_hl.efficiency           = 0.9 
    motor_hl.gearbox_efficiency   = 1.  
    motor_hl.nominal_voltage      = bat.max_voltage 
    motor_hl.propeller_radius     = prop_hl.tip_radius    
    motor_hl.no_load_current      = 0.1
    motor_hl.origin               = prop_hl.origin 
    motor_hl                      = size_optimal_motor(motor_hl,prop_hl) 
    net.rotor_motor               = motor_hl 
    
    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 
    nacelle                           = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag                       = 'rotor_nacelle'
    nacelle.length                    = 0.5
    nacelle.diameter                  = 0.2
    nacelle.orientation_euler_angles  = [0,0,0.]    
    nacelle.flow_through              = False  
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)    
    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.1
    nac_segment.height             = 0.25
    nac_segment.width              = 0.25
    nacelle.append_segment(nac_segment)    
    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.35  
    nac_segment.height             = 0.3
    nac_segment.width              = 0.3
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.5 
    nac_segment.height             = 0.4
    nac_segment.width              = 0.4
    nacelle.append_segment(nac_segment)    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.85
    nac_segment.height             = 0.3
    nac_segment.width              = 0.3
    nacelle.append_segment(nac_segment)        

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_6'
    nac_segment.percent_x_location = 0.9
    nac_segment.height             = 0.25
    nac_segment.width              = 0.25
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_6'
    nac_segment.percent_x_location = 1.0
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)      
 
    lift_rotor_nacelle_origins   = prop_hl_origins
 
    for ii in range(net.number_of_rotor_engines):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.tag      = 'rotor_nacelle_' + str(ii+1) 
        rotor_nacelle.origin   = [lift_rotor_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)   
    
    
    # Update for cruise propeller 
    nacelle.tag            = 'cruise_prop_nacelle'
    nacelle.length         = 1.0
    nacelle.diameter       = 0.6 

    propeller_nacelle_origins   = [[2.5, 4.97584, 1.01],[2.5, -4.97584, 1.01]]  

    for ii in range(net.number_of_propeller_engines):
        propeller_nacelle          = deepcopy(nacelle)
        propeller_nacelle.tag      = 'propeller_nacelle_' + str(ii+1) 
        propeller_nacelle.origin   = [propeller_nacelle_origins[ii]]
        vehicle.append_component(propeller_nacelle)   
    
    # Component 6 the Payload ----------------------------------------------------------------- 
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 10. #Watts 
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload                  = payload
    
    # Component 7 the Avionics----------------------------------------------------------------- 
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 20. #Watts  
    net.avionics        = avionics          
    
    # Component 9 Miscellaneous Systems 
    sys = SUAVE.Components.Systems.System()
    sys.mass_properties.mass = 5 # kg 
 
    
    # add the solar network to the vehicle
    vehicle.append_component(net)    

    return vehicle 

if __name__ == '__main__': 
    main()    
    plt.show() 
    
    
    
    
    
    
    