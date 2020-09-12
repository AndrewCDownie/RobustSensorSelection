# ECE 686 Project Code

This is a project to explore a state of the art robust sensor algorithm proposed in a paper called Robust and Adaptive Sequential Submodular Optimization by Vasileios Tzoumas, Ali Jadbabaie and  George J. Pappas. Here is the arXiv citation link for the paper.

@misc{tzoumas2019robust,
    title={Robust and Adaptive Sequential Submodular Optimization},
    author={Vasileios Tzoumas and Ali Jadbabaie and George J. Pappas},
    year={2019},
    eprint={1909.11783},
    archivePrefix={arXiv},
    primaryClass={math.OC}
}

The RAM algorithm utilizes Submodular Optimization to selected sensors that minimize the Kalman error at each time stage. The Algorithm also selects sensors to defend against the worst case sensor failures.  The plots show the experimental results from simulations.


To execute simply run the project.m file and the simulation will execute.

What you should expect
1. The code will run simulations for each beta with specified number of trials in the code
   there will be output indicating the progress of the simulations
2. 12 plots should be made comparing the performance of the greedy strategy vs the Ram Algorithm based on cumulative error over time

3. An example plot showing the estimates of the position of a over time

Dependencies
Uses standard matlab libraries


Extras
1. If you want to compare error at each time step instead of cumulative error you can uncomment lines 206-216
   in project.m and then the plots will show a comparison of the greedy v.s. RAM error at each time step.

2. You can save the plots to the plots folders if you uncommment lines 240,260 and 278 in project.m

File Descriptions
Project.m - runs the main simulation and does the plotting

sim_worst_case_failure.m - Runs simulations for selecting sensors using random selection,
                           greedy selection and Ram selection then removes the worst case sensors.

sim_greedy_failure.m - Same as sim_worst_case_failure.m but with greedy sensor removal

sim_random_failure.m - Same as previous two but with random sensor removal

RAM_select_sensor.m - Selects sensors using the ram algorithm from paper

greedy_select_sensors.m - Selects sensors using greedy algorithm

remove_worst_case.m - Determines the worst case sensors removal

greedy_remove_sensors.m - Determines the greedy removal of sensors

objective_function.m - calculates the objective value give the kalman filter matrices and a set of sensors

select_sensors.m - generates the H matrix to use in the kalman filter given a list of sensors

select_measurements.m - generates the measurement covariance matrix V based on the selected sensors
