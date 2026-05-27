import mujoco
import mujoco.viewer

# r = 0.65 h = 0.98 c = [1 0.74 0.65] z
# r = 1 h = 0.51 c = [1 1 1.4]
# r = 0.47 h = 0.64 c = [0.43 1.01 2.7]
# 2.45 cm = 1 inch

xml = f"""
<mujoco>
    <compiler angle="radian" meshdir="REU_Snake/Mesh/" />
  <asset>
    <mesh name="ReU_mesh" file="ReU_Mesh.STL" scale="0.0254 0.0254 0.0254"/>
    <material name="red" rgba="1 0 0 1"/>
    <material name="blue" rgba="0 0 1 1"/>
    <material name="green_dark" rgba="0 0.6 0 1"/>  
    <material name="green_light" rgba="0 0.4 0 1"/>
    <material name="debug_col" rgba="1 0.5 0.5 0.3"/> 
  </asset>

    <worldbody>
        <light pos="0 0 3"/>
        <geom type="plane" size="1 1 0.1"/>
        <body name = "Refer" pos = "0 0 0">
         <geom type="sphere" size="0.01" material="red" mass = "0.1"/>
        </body>

        <body name="mod_0_in" pos="0 0 0">
                <geom name = "mod_0_in" type="cylinder" size="{0.67*0.0254} {0.0254/2}" pos ="0.0254 {0.74*0.0254} {0.65*0.0254}" euler="1.57079632679 0 0" material="debug_col" 
                contype="2" conaffinity="1" group="0"/>


        <body name="mod_0_body" pos="0 0 0">
        <geom name = "mesh1" type="mesh" mesh="ReU_mesh" pos = "0 0 0" material="green_dark"/>

        <geom name = "mod_0_link" type="cylinder" pos ="0.0254 0.0258 {0.0254*1.4} " size="0.0258 0.006" material="debug_col" /> 
        <body name="mod_0_out" pos="{0.43*0.0254} {1.01*0.0254} {2.7*0.0254}" euler="0 1.57079632679 0">
                <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
                <geom name = "mod_0_out" type="cylinder" size="0.0125 0.008" pos ="0.001 0 0" euler="0 0 0" material="debug_col" 
                contype="2" conaffinity="1" group="0"/>
        </body>
        </body></body>
    </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
model.opt.gravity[:] = [0, 0, 0]
mujoco.viewer.launch(model,data)