1. copy darias_description file into 
    cd .mujoco/mujoco200/bin

2. copy all (.stl) file from the subfolders(arms,hand) of meshes into meshes folder

3. open file darias.urdf and add those lines below <robot name="darias" ...>:

<mujoco>
        <compiler meshdir="/home/username/.mujoco/mujoco200/bin/darias_description/meshes/"balanceinertia="true"/>
</mujoco>


4. save the urdf file and copy it to .mujoco/mujoco200/bin

5. cd .mujoco/mujoco200/bin
    ./compile darias.urdf darias.mjb
    
6. ./simulate 
    and then drag darias.mjb into window

