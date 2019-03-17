### Script to test memory consumption for opensim with python API

To run the test, run `python test.py n` from within the folder where `n` is the number of loops you 
want to test over. This script will save interim files with results from the processes in the Results 
folder.  <br>

The script runs two loops for each of the main opensim processing pipelines (IK, ID, SO, and JR). The
first loop simply constructs everything to do the processing, but doesnt actually <i><b>run</b></i> the 
processing. The second loop does the same thing, but it actually <i><b>runs</b></i> it - the difference
being whether the `.run()` command is entered for each of the pipelines. The memory is recorded at the 
end of each of these loops. Memory is recorded seperately for the `.run()`/no-`.run()` loops. <br>

At the end of the script, the results of the average memory accumulated for the RUN vs. NO-RUN loops for 
each of the processes are printed to the terminal. <br>

The raw results of the memory accumulation for each of the processes are also saved to .csv files in the
Results folder. These files can be used to plot and see how the memory increases for each of the processes. <br>

From preliminary testing, the main culprit seems to be the Static Optimization which accumulates 40MB of 
memory for every loop (the results are printed below). It should be noted that only 10 time-steps are 
being analyzed during this SO script. <br>

#### Below is the average accumulated memory over 20 loops. 

```
IK - Average memory accumulation per loop WITHOUT run:   0.294 MB
IK - Average memory accumulation per loop WITH run:      0.263 MB

ID - Average memory accumulation per loop WITHOUT run:   0.055 MB
ID - Average memory accumulation per loop WITH run:      0.781 MB

SO - Average memory accumulation per loop WITHOUT run:   0.003 MB
SO - Average memory accumulation per loop WITH run:     37.513 MB

JR - Average memory accumulation per loop WITHOUT run:   1.644 MB
JR - Average memory accumulation per loop WITH run:      2.164 MB
```