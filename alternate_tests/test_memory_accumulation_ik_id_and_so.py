import opensim 
import os
import psutil
import numpy as np
import gc
import argparse

#####################################
#Define functions for IK, ID, and SO. 
#####################################

def ik_analysis(model, location_gait_model_files, ik_setup_filename, 
                gait_marker_filename, save_ik_name, location_saved_results,
                run_tool=False
                ):
    #Perform IK analysis
    #If run_tool == True, then "run" the ik tool.
    #otherwise, just setup the ik_tool. 
    ik_tool = opensim.InverseKinematicsTool(os.path.join(location_gait_model_files, ik_setup_filename))
    ik_tool.setModel(model)
    ik_tool.setName(gait_marker_filename)
    ik_tool.setMarkerDataFileName(os.path.join(location_gait_model_files, gait_marker_filename))
    ik_tool.setOutputMotionFileName(os.path.join(location_saved_results, save_ik_filename))
    #Use run_tool
    if run_tool is True: 
        ik_tool.run()

def id_analysis(model, location_saved_results, location_gait_model_files, ik_filename,
                save_id_filename, external_loads_filename, save_id_setting_filename, 
                kinematic_filter_frequency=6.0, run_id=False, start_time=0,
                end_time=-1):
    id_tool = opensim.InverseDynamicsTool()
    id_tool.setModel(model)
    id_tool.setCoordinatesFileName(os.path.join(location_saved_results, ik_filename))
    id_tool.setLowpassCutoffFrequency(kinematic_filter_frequency)
    id_tool.setResultsDir(location_saved_results)
    id_tool.setOutputGenForceFileName(save_id_filename)

    # id_tool.setStartTime(start_time)
    # id_tool.setEndTime(end_time)

    excludedForces = opensim.ArrayStr()
    excludedForces.append('Muscles')
    id_tool.setExcludedForces(excludedForces)
    id_tool.setExternalLoadsFileName(os.path.join(location_gait_model_files, external_loads_filename))
    id_tool.printToXML(os.path.join(location_saved_results, save_id_setting_filename))
    
    if run_id is True:
        id_tool.run()
def createTorqueActuator(bodyA, bodyB, axis, name, optimal_force=10, 
                         torque_global=True, min_control=-float('inf'), 
                         max_control=float('inf')):
    torque_actuator = opensim.TorqueActuator()
    torque_actuator.setBodyA(bodyA)
    torque_actuator.setBodyB(bodyB)
    torque_actuator.setAxis(opensim.Vec3(axis[0], axis[1], axis[2]))
    torque_actuator.setOptimalForce(optimal_force)
    torque_actuator.setTorqueIsGlobal(torque_global)
    torque_actuator.setMinControl(min_control)
    torque_actuator.setMaxControl(max_control)
    torque_actuator.setName(name)
    return(torque_actuator)

def createPointActuator(body, point, direction, name, optimal_force=10, 
                        min_control=-float('inf'), max_control=float('inf'), 
                        point_is_global=False, force_is_global=True):
    point_actuator = opensim.PointActuator()
    point_actuator.set_body(body)
    point_actuator.set_point(opensim.Vec3(point[0], point[1], point[2]))
    point_actuator.set_direction(opensim.Vec3(direction[0], direction[1], direction[2]))
    point_actuator.setOptimalForce(optimal_force)
    point_actuator.setMinControl(min_control)
    point_actuator.setMaxControl(max_control)
    point_actuator.setName(name)
    point_actuator.set_point_is_global(point_is_global)
    point_actuator.set_force_is_global(force_is_global)
    return(point_actuator)

def model_add_actuators(model):
    # Add actuators to allow Static Optimization. 

    # Add torque actuators
    pelvis_body = model.getBodySet().get('pelvis')
    ground_body = model.getGround()
    Mx_axis = (1,0,0)
    Mx_name = 'pelvis_MX'
    My_axis = (0,1,0)
    My_name = 'pelvis_MY'
    Mz_axis = (0,0,1)
    Mz_name = 'pelvis_MZ'
    pelvis_Mx_torque_actuator = createTorqueActuator(pelvis_body, ground_body, Mx_axis, Mx_name)
    pelvis_My_torque_actuator = createTorqueActuator(pelvis_body, ground_body, My_axis, My_name)
    pelvis_Mz_torque_actuator = createTorqueActuator(pelvis_body, ground_body, Mz_axis, Mz_name)
    model.getForceSet().append(pelvis_Mx_torque_actuator)
    model.getForceSet().append(pelvis_My_torque_actuator)
    model.getForceSet().append(pelvis_Mz_torque_actuator)

    # Add point actuators
    pelvis_Fx_point_actuator = createPointActuator('pelvis', (-0.034,0.0,0), (1,0,0), 'pelvis_FX')
    pelvis_Fy_point_actuator = createPointActuator('pelvis', (-0.034,0.0,0), (0,1,0), 'pelvis_FY')
    pelvis_Fz_point_actuator = createPointActuator('pelvis', (-0.034,0.0,0), (0,0,1), 'pelvis_FZ')

    model.getForceSet().append(pelvis_Fx_point_actuator)
    model.getForceSet().append(pelvis_Fy_point_actuator)
    model.getForceSet().append(pelvis_Fz_point_actuator)

    return(model)



def so_analysis(model, location_saved_results, location_gait_model_files, 
                so_analyze_filename, ik_filename, save_analyze_tool_settings_so_name,
                external_loads_filename,
                kinematic_filter_frequency=6.0, run_so=False):

    ############ PERFORM STATIC OPTIMIZATION ################
    analyze_tool = opensim.AnalyzeTool(os.path.join(location_gait_model_files, so_analyze_filename), False)
    analyze_tool.setModel(model)
    # analyze_tool.setInitialTime(so_start_time)
    # analyze_tool.setFinalTime(so_end_time)
    # analyze_tool.getAnalysisSet().get(0).setStartTime(so_start_time)
    # analyze_tool.getAnalysisSet().get(0).setEndTime(so_end_time)
    analyze_tool.setLowpassCutoffFrequency(kinematic_filter_frequency)
    analyze_tool.setCoordinatesFileName(os.path.join(location_saved_results, ik_filename))
    analyze_tool.setExternalLoadsFileName(os.path.join(location_gait_model_files, external_loads_filename))
    analyze_tool.setLoadModelAndInput(True)
    analyze_tool.setResultsDir(location_saved_results)

    # These are parameters that can be used to programatically alter the optimization parameters
    analyze_tool.setReplaceForceSet(False)
    # analyze_tool.setForceSetFiles(gait2392_SO_Residual_Actuators.xml
    analyze_tool.setOutputPrecision(6)
    analyze_tool.setSolveForEquilibrium(False)
    analyze_tool.setMaximumNumberOfSteps(20000)
    analyze_tool.setMaxDT(1)
    analyze_tool.setMinDT(1e-008)
    analyze_tool.setErrorTolerance(1e-005)

    analyze_tool.printToXML(os.path.join(location_saved_results, save_analyze_tool_settings_so_name))
    if run_so is True:
        analyze_tool.run()

#Use parser to input the number of loops to iterate over. 
parser = argparse.ArgumentParser(description='Number of Loops')
parser.add_argument("loops", help="Enter number of loops")
args = parser.parse_args()
loops = int(args.loops)
#set number of loops to test memory accumulation
# loops = 100

process = psutil.Process(os.getpid()) #Used to print memory usage on each loop. 

#Create results folder to save outputs
cwd = os.getcwd()
results_foldername = 'Results'
location_saved_results = os.path.join(cwd, results_foldername)
if os.path.isdir(location_saved_results) is False:
    os.mkdir(location_saved_results)

#Set variables for locations of folders & files needed for script
gait_model_foldername = 'Gait2392_Simbody'
location_gait_model_files = os.path.join(cwd, gait_model_foldername)
model_name = 'gait2392_millard2012muscle.osim'
subject_mass = 72.0
markerset_name = 'gait2392_Scale_MarkerSet.xml'
scale_file = 'subject01_Setup_Scale.xml'
static_marker_filename = 'subject01_static.trc'
scaled_model_filename = 'subject01_scaled.osim'
scaled_reserve_model_filename = 'subject01_scaled_reserve.osim'
ik_setup_filename = 'subject01_Setup_IK.xml'
gait_marker_filename = 'subject01_walk1.trc'
save_ik_filename = 'ik_results.mot'
save_id_filename = 'id_results.sto'
external_loads_filename = 'subject01_walk1_grf.xml'
save_id_setting_filename = 'id_settings.xml'
save_analyze_tool_settings_so_name = 'analyze_settings_so.xml'
so_analyze_filename = 'setup_static_optimization.xml'

############ LOAD MARKERSET ################
markerset = opensim.MarkerSet(os.path.join(location_gait_model_files, markerset_name))
############ LOAD MODEL ################
model = opensim.Model(os.path.join(location_gait_model_files, model_name))
model.updateMarkerSet(markerset)
model.initSystem()
state = model.initSystem()
############ SCALE MODEL ################
scale_tool = opensim.ScaleTool(os.path.join(location_gait_model_files, scale_file))
scale_tool.setSubjectMass(subject_mass)
scale_tool.setPathToSubject('')

scale_tool.getGenericModelMaker().setModelFileName(os.path.join(location_gait_model_files, model_name))
scale_tool.getGenericModelMaker().setMarkerSetFileName(os.path.join(location_gait_model_files, markerset_name))
scale_tool.getGenericModelMaker().processModel()

scale_tool.getModelScaler().setMarkerFileName(os.path.join(location_gait_model_files, static_marker_filename))
scale_tool.getModelScaler().processModel(model, '', subject_mass)

scale_tool.getMarkerPlacer().setMarkerFileName(os.path.join(location_gait_model_files, static_marker_filename))
scale_tool.getMarkerPlacer().processModel(model)

scale_tool.printToXML(os.path.join(location_saved_results, 'scale_settings.xml'))
model.printToXML(os.path.join(location_saved_results, scaled_model_filename))

############ LOOP OVER CREATING OSIM FUNCTIONS WITHOUT "RUNNING" ################
#Test memory build up by running loop without running ik_tool.run() 
#Create list to append memory usage of python script on each loop. 
memory_usage_per_loop = np.zeros((loops,2)) 

for loop in range(loops):
    #Loading model each loop - becasue SO & ID add forces to it
    model = opensim.Model(os.path.join(location_saved_results, scaled_model_filename))
    model.updateMarkerSet(markerset)
    model.initSystem()
    state = model.initSystem()

    ik_analysis(model, location_gait_model_files, ik_setup_filename, 
                gait_marker_filename, save_ik_filename, location_saved_results,
                run_tool=False)
    
    id_analysis(model, location_saved_results, location_gait_model_files, save_ik_filename,
                save_id_filename, external_loads_filename, save_id_setting_filename,
                run_id=False)
    
    ############ RELOAD SCALED MODEL ################
    #This is done because the id_analysis appends forces to model, as does SO
    #including both id & so on same model will double the applied forces. 
    model = opensim.Model(os.path.join(location_saved_results, scaled_model_filename))
    model.updateMarkerSet(markerset)
    model.initSystem()
    state = model.initSystem()

    model = model_add_actuators(model)
    model.printToXML(os.path.join(location_saved_results, scaled_reserve_model_filename))

    so_analysis(model, location_saved_results, location_gait_model_files, 
                so_analyze_filename, save_ik_filename, save_analyze_tool_settings_so_name,
                external_loads_filename, 
                run_so=False)

    memory_usage_per_loop[loop,0] = process.memory_info().rss*1e-6

############ LOOP OVER CREATING OSIM FUNCTIONS AND "RUN" EACH ################
#Test memory build up by running loop WITH running ik_tool.run() 
for loop in range(loops):
    #Loading model each loop - becasue SO & ID add forces to it
    model = opensim.Model(os.path.join(location_saved_results, scaled_model_filename))
    model.updateMarkerSet(markerset)
    model.initSystem()
    state = model.initSystem()

    ik_analysis(model, location_gait_model_files, ik_setup_filename, 
                gait_marker_filename, save_ik_filename, location_saved_results,
                run_tool=True)
    
    id_analysis(model, location_saved_results, location_gait_model_files, save_ik_filename,
                save_id_filename, external_loads_filename, save_id_setting_filename,
                run_id=True)
    
    ############ RELOAD SCALED MODEL ################
    #This is done because the id_analysis appends forces to model, as does SO
    #including both id & so on same model will double the applied forces. 
    model = opensim.Model(os.path.join(location_saved_results, scaled_model_filename))
    model.updateMarkerSet(markerset)
    model.initSystem()
    state = model.initSystem()

    model = model_add_actuators(model)
    model.printToXML(os.path.join(location_saved_results, scaled_reserve_model_filename))

    so_analysis(model, location_saved_results, location_gait_model_files, 
                so_analyze_filename, save_ik_filename, save_analyze_tool_settings_so_name,
                external_loads_filename,
                run_so=True)

    memory_usage_per_loop[loop,1] = process.memory_info().rss*1e-6

np.savetxt(os.path.join(location_saved_results, 'memory_usage_log_ik_id_and_so.csv'), memory_usage_per_loop, delimiter=',')

average_memory_accumulation_per_loop = np.mean(np.diff(memory_usage_per_loop, axis=0), axis=0)
print('Average memory accumulation per loop WITHOUT run: {:7.3f} MB'.format(average_memory_accumulation_per_loop[0]))
print('Average memory accumulation per loop WITH run:    {:7.3f} MB'.format(average_memory_accumulation_per_loop[1]))
