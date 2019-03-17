import opensim 
import os
import psutil
import numpy as np
import gc

def ik_analysis(model, location_gait_model_files, ik_setup_filename, 
                gait_marker_filename, save_ik_name,
                run_tool=False
                ):
    #Perform IK analysis
    #If run_tool == True, then "run" the ik tool.
    #otherwise, just setup the ik_tool. 
    ik_tool = opensim.InverseKinematicsTool(os.path.join(location_gait_model_files, ik_setup_filename))
    ik_tool.setModel(model)
    ik_tool.setName(gait_marker_filename)
    ik_tool.setMarkerDataFileName(os.path.join(location_gait_model_files, gait_marker_filename))
    ik_tool.setOutputMotionFileName(os.path.join(location_gait_model_files, save_ik_name))
    #Use run_tool
    if run_tool is True: 
        ik_tool.run()

#set number of loops to test
loops = 100

process = psutil.Process(os.getpid()) #Used to print memory usage on each loop. 
cwd = os.getcwd()

#Set variables for locations of folders & files needed for script
gait_model_foldername = 'Gait2392_Simbody'
location_gait_model_files = os.path.join(cwd, gait_model_foldername)
model_name = 'gait2392_millard2012muscle.osim'
subject_mass = 72.0
markerset_name = 'gait2392_Scale_MarkerSet.xml'
scale_file = 'subject01_Setup_Scale.xml'
static_marker_filename = 'subject01_static.trc'
scaled_model_filename = 'subject01_scaled.osim'
ik_setup_filename = 'subject01_Setup_IK.xml'
gait_marker_filename = 'subject01_walk1.trc'
save_ik_name = 'ik_results.mot'

#Load markerset
markerset = opensim.MarkerSet(os.path.join(location_gait_model_files, markerset_name))

#Load model 
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

scale_tool.printToXML(os.path.join(location_gait_model_files, 'scale_settings.xml'))

model.printToXML(os.path.join(location_gait_model_files, scaled_model_filename))


#Test memory build up by running loop without running ik_tool.run() 
#Create list to append memory usage of python script on each loop. 
memory_usage_per_loop = np.zeros((loops,2))

for loop in range(loops):
    ik_analysis(model, location_gait_model_files, ik_setup_filename, 
                gait_marker_filename, save_ik_name,
                run_tool=False)

    memory_usage_per_loop[loop,0] = process.memory_info().rss*1e-6

#Test memory build up by running loop with running ik_tool.run() 
#Create list to append memory usage of python script on each loop. 

for loop in range(loops):
    ik_analysis(model, location_gait_model_files, ik_setup_filename, 
                gait_marker_filename, save_ik_name,
                run_tool=True)

    memory_usage_per_loop[loop,1] = process.memory_info().rss*1e-6

np.savetxt(os.path.join(cwd, 'memory_usage_log_ik_only.csv'), memory_usage_per_loop, delimiter=',')

average_memory_accumulation_per_loop = np.mean(np.diff(memory_usage_per_loop, axis=0), axis=0)
print('Average memory accumulation per loop WITHOUT run: {:7.3f} MB'.format(average_memory_accumulation_per_loop[0]))
print('Average memory accumulation per loop WITH run:    {:7.3f} MB'.format(average_memory_accumulation_per_loop[1]))
