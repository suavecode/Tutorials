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
from SUAVE.Core import Units, Data

import numpy as np
import copy, time

import matplotlib
import pylab as plt

import Vehicle
import Analyses
import Mission

from SUAVE.Optimization.Nexus import Nexus
from SUAVE.Analyses.Process import Process
from SUAVE.Methods.Performance import estimate_take_off_field_length
from SUAVE.Methods.Performance import estimate_landing_field_length 
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform
from SUAVE.Methods.Propulsion.ducted_fan_sizing import ducted_fan_sizing
from SUAVE.Sizing.Sizing_Loop import Sizing_Loop


from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Propulsion.compute_ducted_fan_geometry import compute_ducted_fan_geometry
matplotlib.interactive(True)

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    
    #initialize the problem
    nexus                        = Nexus()
    nexus.vehicle_configurations = Vehicle.setup()
    nexus.analyses               = Analyses.setup(nexus.vehicle_configurations)
    nexus.missions               = Mission.setup(nexus.analyses)
    

   
    #problem = Data()
    #nexus.optimization_problem       = problem
    nexus.procedure                  = setup()
    nexus.sizing_loop                = Sizing_Loop()
    nexus.total_number_of_iterations = 0
    
    evaluate_problem(nexus)
    results = nexus.results
    plot_mission(results, nexus.vehicle_configurations)
    #output=nexus._really_evaluate() #run; use optimization setup without inputs
    return
    
def evaluate_problem(nexus):
    for key,step in nexus.procedure.items():
        if hasattr(step,'evaluate'):
            self = step.evaluate(nexus)
        else:
            nexus = step(nexus)
        self = nexus
    return nexus
# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    
    
    procedure = Process()
    procedure.run_sizing_loop       = run_sizing_loop #size aircraft and run mission
    procedure.evaluate_field_length = evaluate_field_length
    procedure.evaluate_constraints  = evaluate_constraints
    
    return procedure


   
   
def run_sizing_loop(nexus):
    vehicle    = nexus.vehicle_configurations.base
    configs    = nexus.vehicle_configurations
    analyses   = nexus.analyses
    mission    = nexus.missions.base
    
    
    
    #initial guesses
    m_guess    = 60000.       
    Ereq_guess = 100000000000.  
    Preq_guess=  200000. 
 
  
    scaling       = np.array([1E4,1E9,1E6])
    y             = np.array([m_guess, Ereq_guess, Preq_guess])/scaling
    min_y         = [.05, 1E-5,10.]
    max_y         = [10., 10., 10.]
    
    
    #create sizing loop object
    sizing_loop = nexus.sizing_loop
    #assign to sizing loop
    
    sizing_loop.tolerance                                      = 1E-4 #percentage difference in mass and energy between iterations
    sizing_loop.initial_step                                   = 'Default' #Default, Table, SVR
    sizing_loop.update_method                                  = 'successive_substitution' #'successive_substitution','newton-raphson', 'broyden'
    sizing_loop.default_y                                      = y
    sizing_loop.min_y                                          = min_y
    sizing_loop.max_y                                          = max_y
    sizing_loop.default_scaling                                = scaling
    sizing_loop.sizing_evaluation                              = sizing_evaluation
    
    sizing_loop.maximum_iterations                             = 50
    sizing_loop.write_threshhold                               = 50.
   
    sizing_loop.output_filename                                = 'sizing_outputs.txt' #used if you run optimization
    
   
    sizing_loop.iteration_options.h                            = 1E-6
    sizing_loop.iteration_options.min_fix_point_iterations     = 2
    sizing_loop.iteration_options.initialize_jacobian          = 'newton-raphson' #approximate #option for Broyden's Method, not used here
    
    nexus.max_iter                  = sizing_loop.maximum_iterations  #used to pass it to constraints
  
  
    #nexus.sizing_loop               = sizing_loop
    nexus = sizing_loop(nexus)
    return nexus   
   
   
# ----------------------------------------------------------------------        
#   Sizing
# ----------------------------------------------------------------------    

def simple_sizing(nexus):
    #unpack nexus
    configs    = nexus.vehicle_configurations
    analyses   = nexus.analyses
    mission    =  nexus.missions.base.segments
    airport    = nexus.missions.base.airport
    atmo       = airport.atmosphere
    base       = configs.base
    ducted_fan =base.propulsors['ducted_fan']
    battery    = base.energy_network['battery']
    fuselage   = base.fuselages['fuselage']
    
    #unpack guesses
    m_guess = base.m_guess
    Ereq    = base.Ereq
    Preq    = base.Preq
    
    #make it so all configs handle the exact same battery object
    configs.cruise.energy_network['battery']   = battery 
    configs.takeoff.energy_network['battery']  = battery
    configs.landing.energy_network['battery']  = battery
    
    
    # ------------------------------------------------------------------
    #   Define New Gross Takeoff Weight
    # ------------------------------------------------------------------
    
    base.mass_properties.max_takeoff   = m_guess
    base.mass_properties.max_zero_fuel = m_guess  #just used for weight calculation
    design_thrust                      = base.thrust_loading*m_guess*9.81 
    Sref                               = m_guess/base.wing_loading
    
    #assign area
    base.reference_area                     = Sref
    base.wings['main_wing'].areas.reference = base.reference_area
    
    #determine geometry of fuselage, wing, and tail
    SUAVE.Methods.Geometry.Two_Dimensional.Planform.fuselage_planform(fuselage)
    fuselage.areas.side_projected       = fuselage.heights.maximum*fuselage.lengths.cabin*1.1 #guess
    
    #create wing platforms based on geometry
    base.wings['main_wing']             = wing_planform(base.wings['main_wing'])
    base.wings['horizontal_stabilizer'] = wing_planform(base.wings['horizontal_stabilizer']) 
    base.wings['vertical_stabilizer']   = wing_planform(base.wings['vertical_stabilizer'])
    
    #calculate position of horizontal stabilizer
    base.wings['horizontal_stabilizer'].aerodynamic_center[0] = base.w2h- \
    (base.wings['horizontal_stabilizer'].origin[0] + \
    base.wings['horizontal_stabilizer'].aerodynamic_center[0] - \
    base.wings['main_wing'].origin[0] - base.wings['main_wing'].aerodynamic_center[0])
    
    #wing areas based on correlations
    for wing in base.wings:
        wing.areas.wetted   = 2.00 * wing.areas.reference
        wing.areas.affected = 0.60 * wing.areas.reference
        wing.areas.exposed  = 0.75 * wing.areas.wetted
  
    #compute conditions for aircraft propulsor sizing
    cruise_altitude     = mission['climb_3'].altitude_end
    conditions          = atmo.compute_values(cruise_altitude)
    conditions.velocity = mission['cruise'].air_speed
    mach_number         = conditions.velocity/conditions.speed_of_sound   
    
    # assign conditions
    conditions.mach_number         = mach_number
    conditions.gravity             = 9.81
    prop_conditions                = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()   #assign conditions in form for propulsor sizing
    prop_conditions.freestream     = conditions 
    conditions0                    = atmo.compute_values(12500.*Units.ft) #cabin pressure
    p0                             = conditions0.pressure
    fuselage_diff_pressure         = max(conditions0.pressure-conditions.pressure,0)
    fuselage.differential_pressure = fuselage_diff_pressure
   
    # battery calcs
    SUAVE.Methods.Power.Battery.Sizing.initialize_from_energy_and_power(battery,Ereq,Preq, max='soft')
    battery.current_energy = [battery.max_energy] #initialize list of current energy
    m_air                  =SUAVE.Methods.Power.Battery.Variable_Mass.find_total_mass_gain(battery)
    
    #now add the electric motor weight
    motor_mass                      = ducted_fan.number_of_engines*SUAVE.Methods.Weights.Correlations.Propulsion.air_cooled_motor((Preq)*Units.watts/ducted_fan.number_of_engines,.1783, .9884 )
    propulsion_mass                 = SUAVE.Methods.Weights.Correlations.Propulsion.integrated_propulsion(motor_mass/ducted_fan.number_of_engines,ducted_fan.number_of_engines)
    ducted_fan.mass_properties.mass = propulsion_mass
    ducted_fan.thrust.total_design  = design_thrust
    
    #compute geometries
    ducted_fan_sizing(ducted_fan, mach_number,cruise_altitude)  
    compute_ducted_fan_geometry(ducted_fan, conditions = prop_conditions)
    
    #evaluate total mass breakdown
    breakdown                             = analyses.base.weights.evaluate()
    breakdown.battery                     = battery.mass_properties.mass
    breakdown.air                         = m_air
    base.mass_properties.breakdown        = breakdown
    m_fuel                                = 0.
    base.mass_properties.operating_empty  = breakdown.empty 
    
    #m_full = GTOW, m_end = GLW
    m_full                       = breakdown.empty+battery.mass_properties.mass+breakdown.payload
    m_end                        = m_full+m_air
    base.mass_properties.takeoff = m_full
    base.store_diff()

    # Update all configs with new base data    
    for config in configs:
        config.pull_base()

    

    return nexus
    
def sizing_evaluation(y,nexus, scaling):

    #unpack inputs
    m_guess     = y[0]*scaling[0]
    Ereq_guess  = y[1]*scaling[1]
    Preq_guess  = y[2]*scaling[2]
    
    #print guesses
    print 'm_guess      =', m_guess              
    print 'Ereq_guess   =', Ereq_guess   
    print 'Preq_guess   =', Preq_guess           
    
    #unpack guesses
    configs           = nexus.vehicle_configurations
    analyses          = nexus.analyses
    mission           = nexus.missions.base
    battery           = configs.base.energy_network['battery']
    
    #assign guesses to aircraft
    configs.base.m_guess = m_guess
    configs.base.Ereq    = Ereq_guess
    configs.base.Preq    = Preq_guess
   
    #run size aircraft geometry/mass based on guess
    simple_sizing(nexus)
    analyses.finalize() #wont run without this
    results = evaluate_mission(configs,mission)
    
    
    
    #handle outputs
    
    mass_out = results.segments[-1].conditions.weights.total_mass[-1,0]  #actual landing weight
    Ereq_out = results.e_total
    Preq_out = results.Pmax
             
    #errors  
    dm       = (mass_out-m_guess)/m_guess
    dE_total = (Ereq_out-Ereq_guess)/Ereq_guess
    dPower   = (Preq_out-Preq_guess)/Preq_guess
    
    #pack up results
    nexus.results = results 
    
    # return it to sizing loop (within SUAVE/Sizing/Sizing_Loop.py
    f     = np.array([dm, dE_total, dPower])
    y_out = np.array([mass_out, Ereq_out, Preq_out ])/scaling
    
    #print sizing
    print 'y=', y
    print 'y_out=', y_out
    print 'f=', f
    return f, y_out

# ----------------------------------------------------------------------
#   Finalizing Function (make part of optimization nexus)[needs to come after simple sizing doh]
# ----------------------------------------------------------------------    

def finalize(nexus):
    
    #nexus.vehicle_configurations.finalize()
    nexus.analyses.finalize()   
    
    return nexus         
       


def evaluate_mission(configs,mission):
    
    # ------------------------------------------------------------------    
    #   Run Mission
    # ------------------------------------------------------------------
    
    results = mission.evaluate()
    
    #determine energy characteristiscs
    e_current_min=1E14
    Pmax=0.
    max_alpha=np.zeros(len(results.segments))
    min_alpha=np.zeros(len(results.segments))
    
    for i in range(len(results.segments)):
           
            if np.min(results.segments[i].conditions.propulsion.battery_energy)<e_current_min:
                e_current_min=np.min(results.segments[i].conditions.propulsion.battery_energy)
            if np.max(np.abs(results.segments[i].conditions.propulsion.battery_draw))>Pmax:
                Pmax=np.max(np.abs(results.segments[i].conditions.propulsion.battery_draw))         
            aoa=results.segments[i].conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
            max_alpha[i]=max(aoa)
            min_alpha[i]=min(aoa)
            
    max_alpha           = max(max_alpha)
    min_alpha           = min(min_alpha)         
    total_range         = results.segments[-1].conditions.frames.inertial.position_vector[-1,0]
    
    #pack up results
    results.total_range = total_range
    results.e_total     = results.segments[0].conditions.propulsion.battery_energy[0,0]-e_current_min
    results.Pmax        = Pmax
    results.max_alpha   = max_alpha
    results.min_alpha   = min_alpha
    
    #print to make sure it's working correctly
    print 'e_current_min=',e_current_min
    print "e_total=", results.e_total
    print "Pmax=", Pmax
    
  
    return results

    
    


def evaluate_constraints(nexus):
    #optimizer may not be able to handle constraints from inputs
    vehicle=nexus.vehicle_configurations.base
    analyses=nexus.analyses
    mission=nexus.missions.base.segments
    
    results     = nexus.results
    motor_power = vehicle.Preq
    #make sure motor can handle power requirements

    results.power_margin=motor_power-results.Pmax
    #results.segments[-1].conditions.weights.total_mass[-1,0]+=abs(min(0, motor_power-results.Pmax))
    #make sure there is some washout
    alpha_rc=vehicle['wings'].main_wing.twists.root    
    alpha_tc=vehicle['wings'].main_wing.twists.tip
    results.washout=alpha_rc-alpha_tc
    
    results.max_alpha_constraint =15.-results.max_alpha
    results.min_alpha_constraint=15+results.min_alpha
    


    #now make sure there is consistency in the mission profile
    climb_alt_1  =mission['climb_1'].altitude_end
    climb_alt_2  =mission['climb_2'].altitude_end
    climb_alt_3  =mission['climb_3'].altitude_end
  
    descent_alt_1=mission['descent_1'].altitude_end
    descent_alt_2=mission['descent_2'].altitude_end
    results.climb_constraint_1=climb_alt_2-climb_alt_1
    results.climb_constraint_2=climb_alt_3-climb_alt_2

    
    
    results.descent_constraint_1=climb_alt_3-descent_alt_1
    results.descent_constraint_2=descent_alt_1-descent_alt_2
    
    climb_velocity_1=mission['climb_1'].air_speed
    climb_velocity_2=mission['climb_2'].air_speed
    climb_velocity_3=mission['climb_3'].air_speed
    
    results.climb_velocity_constraint_1=climb_velocity_2-climb_velocity_1
    results.climb_velocity_constraint_2=climb_velocity_3-climb_velocity_2
    
    
    results.takeoff_field_constraint= 2500.- results.field_length.takeoff
    results.landing_field_constraint= 2500.- results.field_length.landing
                                               
    max_throttle = 0.
    for segment in results.segments.values():
        max_segment_throttle=np.max(segment.conditions.propulsion.throttle[:,0])
        if max_segment_throttle>max_throttle:
            max_throttle=max_segment_throttle
    
    results.max_throttle_constraint=1-max_throttle
    
    nexus.iteration_constraint= nexus.sizing_loop.tolerance-nexus.sizing_loop.norm_error#((nexus.max_iter-5)-nexus.number_of_iterations)/(1.*nexus.max_iter) #give it some slack to help constraints converge
   
    return nexus
 

 
def evaluate_field_length(nexus):
    configs = nexus.vehicle_configurations
    analyses = nexus.analyses
    mission = nexus.missions.base
    results = nexus.results
    
    # unpack
    airport = mission.airport
    
    takeoff_config = configs.takeoff
    landing_config = configs.landing
   
    # evaluate
    TOFL = estimate_take_off_field_length(takeoff_config,analyses,airport)
    LFL = estimate_landing_field_length (landing_config, analyses,airport)
    
    # pack
    field_length = SUAVE.Core.Data()
    field_length.takeoff = TOFL[0]
    field_length.landing = LFL[0]
    
    results.field_length = field_length
    nexus.results = results
    
    return nexus
    

    
    
# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,configs,line_style='bo-'):
    
    if line_style == 'k-':
        line_width = 2.
    else:
        line_width = 1.



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
            state_of_charge=segment.conditions.propulsion.battery_energy/battery.max_energy
            battery_power=-segment.conditions.propulsion.battery_draw/Units.MW            
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
    
    
    raw_input('Press Enter To Quit')
    return


if __name__ == '__main__':
    main()
    plt.show()