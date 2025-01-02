# Frc-Robot-Template

This template is for FRC robot code.

## Run Configuration

The project has several pre-made run configurations for your convenience. These configurations will be loaded automatically 
when you open the project.

To use a run configuration, select it from the configurations drop down box and click on _run_

![image](https://github.com/user-attachments/assets/c49b8c62-2be3-4996-97cf-8ce6d9209020)

### Deploying to Robot

Select the `Robot Deploy` run configuration and click on _run_. This will build and deploy the code to the robot.

![image](https://github.com/user-attachments/assets/abdb9590-35b1-4022-863e-26398b77984d)

#### Deploying with Debugger

Select the `Robot Deploy Debug` run configuration to deploy the code with debugger enabled.

![image](https://github.com/user-attachments/assets/95d5c8b4-e470-421e-90af-2a7d646d7e97)

To attach a debugger, after a successful deployment (i.e. wait for the deployment to finish successfully), select and run the `RoboRIO Debugger` (run with debug, the bug button, not the run button).

![image](https://github.com/user-attachments/assets/f69ad561-6c7c-4bff-b864-725a2f602f2f)

### Running Simulation

Select the `Simulate` run configuration and click on _run_. This will build and run the code in the simulator.

![image](https://github.com/user-attachments/assets/c6d534f5-fd32-4ad9-a6ab-7feff2d39753)

#### Running with Debugger

Select the `Simulate Debug` run configuration and click on _run_. This will run the simulation with debugger enabled. 

![image](https://github.com/user-attachments/assets/3d8f9f75-cbcb-463d-a2a8-5e77e3177bee)

Once the console prints `Listening for transport dt_socket at address` select and run the `Simulation Debugger` run configuration (run with debug, the bug button, not the run button).

![image](https://github.com/user-attachments/assets/77df3f2e-a93e-42cf-8e35-3323f5b869d3)

![image](https://github.com/user-attachments/assets/603da2af-1b5f-4b73-b597-85b058550378)

Note that doing all these will run two configuration simultaneously. This is fine, but when you wish to stop the simulation you will need to stop both the simulation and the debugger.

![image](https://github.com/user-attachments/assets/6f37f594-2031-401f-89a3-0c1cebd04825)
