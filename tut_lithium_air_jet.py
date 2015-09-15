# tut_lithium_air_jet.py
#
# Created:  Jun 2015, SUAVE Team
# Modified: 

""" setup file for a mission with an all electric airliner
"""


# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units

import numpy as np
import pylab as plt

import copy, time

from SUAVE.Core import (
Data, Container, Data_Exception, Data_Warning,
)

from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing

from SUAVE.Methods.Performance  import payload_range


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    # define the problem
    configs, analyses = full_setup()
    
    configs.finalize()
    analyses.finalize()    
    
    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      
    
    # mission analysis
    mission = analyses.missions
    results = evaluate_mission(configs,mission)  
    
    # plot the results
    plot_mission(results,configs)
    
    
    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)
    
    # vehicle analyses
    configs_analyses = analyses_setup(configs)
    
    # mission analyses
    mission = mission_setup(configs_analyses)
    
    #initialize battery in mission
    mission.segments[0].battery_energy=configs.base.energy_network.battery.max_energy    

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = mission
    
    return configs, analyses


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    
    analyses = SUAVE.Analyses.Analysis.Container()
    
    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis
        
    # adjust analyses for configs
    
    # takeoff_analysis
    analyses.takeoff.aerodynamics.drag_coefficient_increment = 0.0000
    
    # landing analysis
    aerodynamics = analyses.landing.aerodynamics
    
    
    # -----------------------------------
    #   Battery Setup
    # -----------------------------------
    
    # required mission energy, chosen via guess and check
    Ereq = 251.58 * 10.**9.
    Preq = 18.67 * 10.**6.

    # initialize the battery
    battery = configs.base.energy_network['battery']
    battery.specific_energy=2000*Units.Wh/Units.kg
    battery.specific_power =.67*Units.kW/Units.kg
    SUAVE.Methods.Power.Battery.Sizing.initialize_from_energy_and_power(battery,Ereq,Preq)
    battery.current_energy=[battery.max_energy] 
    configs.base.store_diff()

    # Update all configs with new base data    
    for config in configs:
        config.pull_base()
        
    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()
    
    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)
    
    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights()
    weights.settings.empty_weight_method= \
           SUAVE.Methods.Weights.Correlations.Tube_Wing.empty_custom_eng # SEE THIS
    weights.vehicle = vehicle
    analyses.append(weights)
    
    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)
    
    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)
    
    # ------------------------------------------------------------------
    #  Propulsion Analysis
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.energy_network                              # SEE THIS
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)
    
    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   
    
    # done!
    return analyses    


# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Embraer_E190'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff               = 92110. #use landing mass as 
    vehicle.mass_properties.operating_empty           = 34551. 
    vehicle.mass_properties.takeoff                   = 80721. 
    vehicle.mass_properties.max_zero_fuel             = 92110. #equivalent landing mass
    vehicle.mass_properties.cargo                     = 0.0 
    vehicle.mass_properties.max_payload               = 0.0 
    vehicle.mass_properties.max_fuel                  = 0.0

    # envelope properties
    vehicle.envelope.ultimate_load = 3.5
    vehicle.envelope.limit_load    = 1.5
    
    # basic parameters
    vehicle.reference_area         = 100.0
    vehicle.passengers             = 114
    vehicle.systems.control        = "partially powered"
    vehicle.systems.accessories    = "medium range"
    
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 8.3
    wing.sweep                   = 0.002 * Units.deg
    wing.thickness_to_chord      = 0.105
    wing.taper                   = 0.28
    wing.span_efficiency         = 1.0

    wing.spans.projected         = 28.81

    wing.chords.root             = 5.4
    wing.chords.tip              = 1.5185
    wing.chords.mean_aerodynamic = 3.8371

    wing.areas.reference         = 100.0
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = 150.0
    wing.areas.affected          = 60.0

    wing.twists.root             = -0.311 * Units.degrees
    wing.twists.tip              = -0.315 * Units.degrees

    wing.origin                  = [00,0,0]
    wing.aerodynamic_center      = [0.914,0,0]

    wing.vertical                = False
    wing.symmetric               = True

    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 5.5
    wing.sweep                   = 0.002 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.11
    wing.span_efficiency         = 0.9

    wing.spans.projected         = 11.213

    wing.chords.root             = 3.6733
    wing.chords.tip              = 0.4040
    wing.chords.mean_aerodynamic = 2.4755

    wing.areas.reference         = 22.85
    wing.areas.wetted            = 45.71
    wing.areas.exposed           = 34.28
    wing.areas.affected          = 13.71

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [50,0,0]
    wing.aerodynamic_center      = [16.9144, 0.0, 0.0]

    wing.vertical                = False
    wing.symmetric               = True

    wing.dynamic_pressure_ratio  = 0.9

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'

    wing.aspect_ratio            = 1.7      #
    wing.sweep                   = 0.001 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.10
    wing.span_efficiency         = 0.9

    wing.spans.projected         = 4.6945     #

    wing.chords.root             = 5.0209
    wing.chords.tip              = 0.5020
    wing.chords.mean_aerodynamic = 3.3777

    wing.areas.reference         = 12.96
    wing.areas.wetted            = 25.92
    wing.areas.exposed           = 19.44
    wing.areas.affected          = 7.778

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [0,0,0]
    wing.aerodynamic_center      = [0,0,0]

    wing.vertical                = True
    wing.symmetric               = False

    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'

    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 4
    fuselage.seat_pitch            = 0.7455

    fuselage.fineness.nose         = 1.5
    fuselage.fineness.tail         = 1.8

    fuselage.lengths.nose          = 4.5
    fuselage.lengths.tail          = 5.4
    fuselage.lengths.cabin         = 21.24675
    fuselage.lengths.total         = 31.14675
    fuselage.lengths.fore_space    = 0.
    fuselage.lengths.aft_space     = 0.

    fuselage.width                 = 3.0

    fuselage.heights.maximum       = 3.4    #
    fuselage.heights.at_quarter_length          = 3.4
    fuselage.heights.at_three_quarters_length   = 3.4
    fuselage.heights.at_wing_root_quarter_chord = 3.4

    fuselage.areas.side_projected  = 79.462
    fuselage.areas.wetted          = 288.521422366
    fuselage.areas.front_projected = 8.011

    fuselage.effective_diameter    = 3.2

    fuselage.differential_pressure = 10**5 * Units.pascal 

    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #  Propulsion
    # ------------------------------------------------------------------
        
    atm = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    p1, T1, rho1, a1, mew1 = atm.compute_values(0.)
    p2, T2, rho2, a2, mew2 = atm.compute_values(6.255*Units.km)
  
    
    sizing_segment = SUAVE.Core.Data()
    sizing_segment.M   = 0.729      
    sizing_segment.alt = 6.255
    sizing_segment.T   = T2           
    sizing_segment.p   = p2     
    
    #create battery
    battery = SUAVE.Components.Energy.Storages.Batteries.Variable_Mass.Lithium_Air()
    battery.tag = 'battery'
   
    # attributes
    ducted_fan= SUAVE.Components.Propulsors.Ducted_Fan()
    ducted_fan.tag                       ='ducted_fan'
    ducted_fan.diffuser_pressure_ratio   = 0.98
    ducted_fan.fan_pressure_ratio        = 1.65
    ducted_fan.fan_nozzle_pressure_ratio = 0.99
    ducted_fan.design_thrust             = 121979.18 # Preq*1.5/V_cruise 
    ducted_fan.number_of_engines         = 2.0    
    ducted_fan.engine_sizing_ducted_fan(sizing_segment)   #calling the engine sizing method 
    
    # ------------------------------------------------------------------
    #  Energy Network
    # ------------------------------------------------------------------ 
    
    #define the energy network
    net     = SUAVE.Components.Energy.Networks.Battery_Ducted_Fan()
    net.tag = 'network'
    
    net.propulsor = ducted_fan
    net.append(ducted_fan)
    net.battery = battery
    
    net.nacelle_diameter  = ducted_fan.nacelle_diameter
    net.engine_length     = ducted_fan.engine_length    
    net.number_of_engines = ducted_fan.number_of_engines
    net.motor_efficiency  =.95
    vehicle.propulsors.append(ducted_fan)
    vehicle.energy_network = net
    
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    

    return vehicle


# ----------------------------------------------------------------------
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
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------
    
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    
    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg
    
    config.V2_VS_ratio = 1.21
    config.maximum_lift_coefficient = 2.
    
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'
    
    config.wings['main_wing'].flaps_angle = 30. * Units.deg
    config.wings['main_wing'].slats_angle = 25. * Units.deg

    config.Vref_VS_ratio = 1.23
    config.maximum_lift_coefficient = 2.
    
    configs.append(config)
    
    
    # done!
    return configs


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'embraer_e190ar test mission'

    # atmospheric model
    atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    planet = SUAVE.Attributes.Planets.Earth()

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments    


    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Throttle
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_1"

    # connect vehicle configuration
    segment.analyses.extend( analyses.takeoff )

    # define segment attributes
    segment.atmosphere     = atmosphere
    segment.planet         = planet

    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 0.353 * Units.km
    segment.air_speed      = 111.932
    segment.climb_rate     = 3000. * Units['ft/min']

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Throttle
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_2"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet

    segment.altitude_end = 1.157 * Units.km
    segment.air_speed    = 163.71
    segment.climb_rate   = 2500. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed, Constant Throttle
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_3"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet

    segment.altitude_end = 1.236 * Units.km
    segment.air_speed    = 193.25
    segment.climb_rate   = 1800. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Fourth Climb Segment: Constant Speed, Constant Throttle
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_4"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet

    segment.altitude_end = 1.807 * Units.km
    segment.air_speed    = 194.17
    segment.climb_rate   = 900. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)   
    
    # ------------------------------------------------------------------
    #   Fifth Climb Segment: Constant Speed, Constant Throttle
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_5"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet

    segment.altitude_end = 6.255 * Units.km
    segment.air_speed    = 176.03
    segment.climb_rate   = 300. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)       

    # ------------------------------------------------------------------
    #   Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude()
    segment.tag = "cruise"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere = atmosphere
    segment.planet     = planet

    segment.air_speed  = 230.
    segment.distance   = 1947. * Units.nmi

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: consant speed, constant segment rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate()
    segment.tag = "descent_1"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet

    segment.altitude_end = 6.26  * Units.km
    segment.air_speed    = 440.0 * Units.knots
    segment.descent_rate = 2400. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: consant speed, constant segment rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate()
    segment.tag = "descent_2"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet

    segment.altitude_end = 2.132 * Units.km
    segment.air_speed    = 365.0 * Units.knots
    segment.descent_rate = 2000. * Units['ft/min']

    # append to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment: consant speed, constant segment rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate()
    segment.tag = "descent_3"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere   = atmosphere
    segment.planet       = planet

    segment.altitude_end = 0.0   * Units.km
    segment.air_speed    = 250.0 * Units.knots
    segment.descent_rate = 1500. * Units['ft/min']

    # append to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Mission definition complete
    # ------------------------------------------------------------------

    return mission

#: def define_mission()

# ----------------------------------------------------------------------
#   Evaluate the Mission
# ----------------------------------------------------------------------
def evaluate_mission(configs,mission):
    
    # ------------------------------------------------------------------    
    #   Run Mission
    # ------------------------------------------------------------------
    
    results = mission.evaluate()
    
    #determine energy characteristiscs
    e_current_min=1E14
    Pmax=0.
    for i in range(len(results.segments)):
            if np.min(results.segments[i].conditions.propulsion.battery_energy[:,0])<e_current_min:
                e_current_min=np.min(results.segments[i].conditions.propulsion.battery_energy[:,0])
            if np.max(np.abs(results.segments[i].conditions.propulsion.battery_draw[:,0]))>Pmax:
                Pmax=np.max(np.abs(results.segments[i].conditions.propulsion.battery_draw[:,0]))         
    results.e_total=results.segments[0].conditions.propulsion.battery_energy[0,0]-e_current_min
    results.Pmax=Pmax
    print 'e_current_min=',e_current_min
    print "e_total=", results.e_total
    print "Pmax=", Pmax
    print "e_current_min=", e_current_min
    return results
# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,configs,line_style='bo-'):
    
    if line_style == 'k-':
        line_width = 2.
    else:
        line_width = 1.


    # ------------------------------------------------------------------
    #   Throttle
    # ------------------------------------------------------------------
    plt.figure("Throttle History")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        eta  = results.segments[i].conditions.propulsion.throttle[:,0]
        axes.plot(time, eta, line_style)
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Throttle')
    axes.grid(True)


    # ------------------------------------------------------------------
    #   Angle of Attack
    # ------------------------------------------------------------------

    plt.figure("Angle of Attack History")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        aoa = results.segments[i].conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        axes.plot(time, aoa, line_style)
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Angle of Attack (deg)')
    axes.grid(True)


    # ------------------------------------------------------------------
    #   Mass Rate
    # ------------------------------------------------------------------
    plt.figure("Mass Rate")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        mdot = -results.segments[i].conditions.weights.vehicle_mass_rate[:,0]
        axes.plot(time, mdot, line_style)
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Mass Rate (kg/s)')
    axes.grid(True)


    # ------------------------------------------------------------------
    #   Altitude
    # ------------------------------------------------------------------
    plt.figure("Altitude")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        altitude = results.segments[i].conditions.freestream.altitude[:,0] / Units.km
        axes.plot(time, altitude, line_style)
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Altitude (km)')
    axes.grid(True)


    # ------------------------------------------------------------------
    #   Vehicle Mass
    # ------------------------------------------------------------------
    plt.figure("Vehicle Mass")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        mass = results.segments[i].conditions.weights.total_mass[:,0]
        axes.plot(time, mass, line_style)
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Vehicle Mass (kg)')
    axes.grid(True)


    # ------------------------------------------------------------------
    #   Aerodynamics
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Forces")
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Lift   = -segment.conditions.frames.wind.lift_force_vector[:,2]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , Lift , line_style )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Lift (N)')
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , Drag , line_style )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Drag (N)')
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , Thrust , line_style )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Thrust (N)')
        axes.grid(True)


    # ------------------------------------------------------------------
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients")
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , CLift , line_style )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CL')
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , CDrag , line_style )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CD')
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , Drag   , line_style )
        axes.plot( time , Thrust , 'ro-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Drag and Thrust (N)')
        axes.grid(True)


    # ------------------------------------------------------------------
    #   Aerodynamics 3
    # ------------------------------------------------------------------
    fig = plt.figure("Drag Components")
    axes = plt.gca()
    for i, segment in enumerate(results.segments.values()):

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cdc = drag_breakdown.compressible.total[:,0]
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cd  = drag_breakdown.total[:,0]

        if line_style == 'bo-':
            axes.plot( time , cdp , 'ko-', label='CD_P' )
            axes.plot( time , cdi , 'bo-', label='CD_I' )
            axes.plot( time , cdc , 'go-', label='CD_C' )
            axes.plot( time , cdm , 'yo-', label='CD_M' )
            axes.plot( time , cd  , 'ro-', label='CD'   )
            if i == 0:
                axes.legend(loc='upper center')            
        else:
            axes.plot( time , cdp , line_style )
            axes.plot( time , cdi , line_style )
            axes.plot( time , cdc , line_style )
            axes.plot( time , cdm , line_style )
            axes.plot( time , cd  , line_style )            

    axes.set_xlabel('Time (min)')
    axes.set_ylabel('CD')
    axes.grid(True)
    
    
    # ------------------------------------------------------------------
    #   Flight Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Flight Conditions",figsize=(6.5,10))
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        altitude = segment.conditions.freestream.altitude[:,0] / Units.km
        mach     = segment.conditions.freestream.mach_number[:,0]
        distance = segment.conditions.frames.inertial.position_vector[:,0] / Units.km

        axes = fig.add_subplot(3,1,1)
        axes.plot( time, distance, line_style )
        axes.set_ylabel('Distance (km)')
        axes.grid(True)        

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , altitude , line_style , lw=line_width )
        axes.set_ylabel('Altitude (km)')
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , mach, line_style , lw=line_width )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Mach Number (-)')
        axes.grid(True)    
    
    # ------------------------------------------------------------------    
    #  Mass, State of Charge, Power
    # ------------------------------------------------------------------
    
    fig = plt.figure("Electric Aircraft Outputs",figsize=(6.5,10))
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        mass = segment.conditions.weights.total_mass[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , mass , line_style , lw=line_width )
        axes.set_ylabel('Vehicle Mass (kg)')
        axes.grid(True)
        
        try:
            battery=configs.base.energy_network['battery']
            state_of_charge=segment.conditions.propulsion.battery_energy[:,0]/battery.max_energy
            battery_power=-segment.conditions.propulsion.battery_draw[:,0]/Units.MW            
        except:
            continue        
        
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , state_of_charge , line_style , lw=line_width )
        axes.set_ylabel('State of Charge (-)')
        axes.set_ylim([-0.005,1.005])
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , battery_power , line_style , lw=line_width )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Discharge Power (MW)')
        axes.grid(True)    

    return


if __name__ == '__main__':
    main()
    plt.show()