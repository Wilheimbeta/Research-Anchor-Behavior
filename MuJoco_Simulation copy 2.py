import mujoco
import mujoco.viewer
import numpy as np
import time
import prototype
import matplotlib.pyplot as plt
# r = 0.65 h = 0.98 c = [1 0.74 0.65] z
# r = 1 h = 0.51 c = [1 1 1.4]
# r = 0.47 h = 0.64 c = [0.43 1.01 2.7]
# 2.45 cm = 1 inch 
# [0.01 2.09 2.01]
# [2.09 0.01 2.01]


REU_snake_xml =  f"""
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
  """
NUM_MODULES = 3

REU_snake_xml += f"""
  <worldbody>
  <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    

        <body name="mod_0_in" pos="{-0.0253} {-0.74*0.0254*0} 0.05">
        <joint name="mod_0_free" type="free"/>   
        
        


        <body name="mod_0_body" pos="0 0 0">
        <geom name = "mesh0" type="mesh" mesh="ReU_mesh" pos = "0 0 0" material="green_dark" contype="2" conaffinity="1" group="0"/>
        <inertial pos="0.02326 0.02258 0.03501" 
          mass="0.24" 
          fullinertia="1.061933e-04 1.055403e-04 6.550863e-05 -1.62875e-06 -1.473256e-05 1.460056e-05"/>
        
        <body name="mod_0_out" pos="0 0 0" euler="0 0 0 ">
              <geom name = "mod_0_out" type="cylinder" size="0.001 0.001" pos="{0.43*0.0254+0.001} {1.01*0.0254} {2.7*0.0254}" euler="0 1.57079632679 0 " material="debug_col" 
                contype="2" conaffinity="1" group="0"/>
    
                <inertial pos="0 0 0" mass="0.001" diaginertia="0.001 0.001 0.001"/>
                

"""

for i in range(NUM_MODULES):
    if i<16:
        REU_snake_xml += f"""
  
        <body name="mod_{i+1}_in" pos="{2.09*0.0254} {0.01*0.0254} {2.01*0.0254}" euler="0 0 1.57079632679">
       
        
  
        <joint name="joint_{i+1}" type="hinge" pos="0.0254 {0.74*0.0254} {0.65*0.0254}" axis="0 1 0" range="-1.57079632679 1.57079632679" 
                limited="true" damping="3.18" frictionloss="0.2"/>


        <body name="mod_{i+1}_body" pos="0 0 0">
        <geom name = "mesh{i+1}" type="mesh" mesh="ReU_mesh" pos = "0 0 0" material="green_dark" contype="2" conaffinity="1" group="0"/>
          <inertial pos="0.02326 0.02258 0.03501" 
          mass="0.24" 
          fullinertia="1.061933e-04 1.055403e-04 6.550863e-05 -1.62875e-06 -1.473256e-05 1.460056e-05"/>
        
        <body name="mod_{i+1}_out" pos="0 0 0" euler="0 0 0 ">
                <inertial pos="{0.43*0.0254+0.001} {1.01*0.0254} {2.7*0.0254}" mass="0.0001" diaginertia="1e-9 1e-9 1e-9"/>
                <geom name = "mod_{i+1}_out" type="cylinder" size="0.001 0.001" pos="{0.43*0.0254+0.001} {1.01*0.0254} {2.7*0.0254}" euler="0 1.57079632679 0 " material="debug_col" 
                contype="2" conaffinity="1" group="0"/>
    
        """
    else:
       REU_snake_xml += f"""
  
        <body name="mod_{i+1}_in" pos="{2.09*0.0254} {0.01*0.0254} {2.01*0.0254}" euler="0 0 1.57079632679">
       
        
        <geom name = "mod_{i+1}_in" type="cylinder" size="{0.67*0.0254} {0.0254/2}" pos ="0.0254 {0.74*0.0254} {0.65*0.0254}" euler="1.57079632679 0 0" material="debug_col" 
                contype="2" conaffinity="1" group="0"/>
        <joint name="joint_{i+1}" type="hinge" pos="0.0254 {0.74*0.0254} {0.65*0.0254}" axis="0 0 1" range="-1.57079632679 1.57079632679" 
                limited="true" damping="3.18" frictionloss="0.2"/>


        <body name="mod_{i+1}_body" pos="0 0 0">
        <geom name = "mesh{i+1}" type="mesh" mesh="ReU_mesh" pos = "0 0 0" material="green_dark"/>

        <geom name = "mod_{i+1}_link" type="cylinder" pos ="0.0254 0.0258 {0.0254*1.4} " size="0.0258 0.006" material="debug_col" /> 

        <body name="mod_{i+1}_out" pos="0 0 0" euler="0 0 0 ">
                <inertial pos="{0.43*0.0254+0.001} {1.01*0.0254} {2.7*0.0254}" mass="0.0001" diaginertia="1e-9 1e-9 1e-9"/>
                <geom name = "mod_{i+1}_out" type="cylinder" size="0.0125 0.008" pos="{0.43*0.0254+0.001} {1.01*0.0254} {2.7*0.0254}" euler="0 1.57079632679 0 " material="debug_col" 
                contype="2" conaffinity="1" group="0"/>
                    
    
        """

for i in range(NUM_MODULES+1):
    REU_snake_xml += f"""
    
    </body> </body> </body>
    """


REU_snake_xml += f"""


</worldbody>
"""
REU_snake_xml += f"""

<actuator>

"""
for i in range(NUM_MODULES):
    if i!=0 and i!=2:
      REU_snake_xml  += f"""
      <position name="servo_{i+1}_pos" joint="joint_{i+1}" kp="100" ctrlrange="-1.5708 1.5708" ctrllimited="true" forcerange="-10.0 10.0" forcelimited="true"/>
      """
    else:
      REU_snake_xml  += f"""
      <position name="servo_{i+1}_pos" joint="joint_{i+1}" kp="100" ctrlrange="-1.5708 1.5708" ctrllimited="true" forcerange="-10.0 10.0" forcelimited="true"/>
      """ 

#REU_snake_xml+= f"""
 #<motor name="servo_1_motor" joint="joint_1" ctrlrange="-10.0 10.0" ctrllimited="true"/>

 #<motor name="servo_3_motor" joint="joint_3" ctrlrange="-10.0 10.0" ctrllimited="true"/>
#"""

REU_snake_xml += f"""

</actuator>

"""

REU_snake_xml += f"""<sensor>"""

for i in range(NUM_MODULES):
  REU_snake_xml += f"""
    <jointactuatorfrc joint="joint_{i+1}" name="joint_{i+1}_torque_sensor"/>"""

REU_snake_xml +="""</sensor>"""

REU_snake_xml += f"""

</mujoco>

"""
#print(REU_snake_xml)
with open("REU_snake_robot.xml", "w", encoding="utf-8") as f:
    f.write(REU_snake_xml)



xml_anchor_scene = f"""
<mujoco model="anchoring_workspace_scene">
<visual>
        <scale contactheight="0.05" contactwidth="0.01"/>
    </visual>
  <option cone="elliptic" noslip_iterations="5"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <asset>
     
    <texture type="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="32" height="32"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
  </asset>
  
  <include file="REU_snake_robot.xml"/>

  
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="5 5 .05" type="plane" material="grid" group="2"/>
    ``0.0635 wall width''
    <body name="Wall1" pos="{0.0635/2+0.01} 0 0.2">
        <geom name="Wall_1" type="box" size="0.01 0.11 0.5" contype="1" conaffinity="2" group="2" rgba="0.8 0.5 0.2 0.2"/>
    </body>    

    <body name="Wall2" pos="{-0.0635/2-0.01} 0 0.2">
        <geom name="Wall_2" type="box" size="0.01 0.11 0.5" contype="1" conaffinity="2" group="2" rgba="0.8 0.5 0.2 0.2"/>    
    
    </body>
  </worldbody>
  """
xml_anchor_scene += """
<contact>
"""
for i in range(NUM_MODULES+1):
    xml_anchor_scene += f"""
    
    
    <pair name="f_mod_{i}_w1_link"  geom1="Wall_1" geom2="mesh{i}" condim="3" friction="0.5 0.5 0.5"  solimp="0.8 0.9 0.001" solref="0.004 1"/>
    
    
    <pair name="f_mod_{i}_w2_link"  geom1="Wall_2" geom2="mesh{i}" condim="3" friction="0.5 0.5 0.5"  solimp="0.8 0.9 0.001" solref="0.004 1"/>

    """
xml_anchor_scene += f"""
  </contact>
  </mujoco>
  """

#print(xml_anchor_scene)

with open("xml_anchor_scene.xml", "w", encoding="utf-8") as f:
    f.write(xml_anchor_scene)


#model = mujoco.MjModel.from_xml_string(xml_anchor_scene)
#data = mujoco.MjData(model)
#model.opt.gravity[:] = [0, 0, 0]
#mujoco.viewer.launch(model,data)

model = mujoco.MjModel.from_xml_string(xml_anchor_scene)
model.opt.gravity[:] = [0, 0, 0]
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)


desired_tau = np.array([0, 1.5, 0]) 
data.ctrl[:] = desired_tau
n = 4000
for j in range(n): mujoco.mj_step(model, data)#;print(data.geom_xpos[geom_id])
contacts = []

Mark_1 = []

for i in range(4):
   geom_name = f"mod_{i}_out"
   geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
   geom_world_pos = data.geom_xpos[geom_id].copy()
   Mark_1.append(geom_world_pos)


for i in range(data.ncon):
    contact = data.contact[i]
    #print( f"Contact {i}",contact)
    print( f"Contact {i} position",contact.pos)
    #print( f"Contact {i} normal Direction",contact.frame[:3])
    contact_index = i 
    c_force = np.zeros(6, dtype=np.float64)
    mujoco.mj_contactForce(model, data, contact_index, c_force)
    print(f"Contact{i}",c_force[0])    

    contacts.append({
            'p': contact.pos,   # Right foot position relative to CoM under Spatial Frame 
            'n': contact.frame[:3],    # Normal vector pointing upwards
            'mu': 0.5,               # Coefficient of friction
            'f_max': 200            # Maximum normal force capacity for this contact
    })

joint_name = "joint_2"  
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
print("J2",data.xanchor[joint_id])

def check_is_sliding(Mark_1,data, slip_threshold=1e-3):
  
    
    for i in range(4):
      geom_name = f"mod_{i}_out"
      geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
      geom_world_pos = data.geom_xpos[geom_id]
      
      for slip in np.abs(Mark_1[i]-geom_world_pos):
        is_sliding = slip > slip_threshold
        if is_sliding:
          print("New:",i,geom_world_pos)
          print("Old:",Mark_1[i])
          print(slip)
          print("--------")
    
          break
    
    
    return is_sliding, slip



def test_wrench(wrench,time_step,model,data,base_data):
    body_name = "mod_3_out"
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    data.xfrc_applied[body_id, :3] = wrench[:3]#force
    data.xfrc_applied[body_id, 3:] = wrench[3:]#moment

    
    flag_stop = False
    for j in range(time_step): 
      if flag_stop:
        break
      mujoco.mj_step(model, data)
     
      if j > 0:
        is_sliding, slip = check_is_sliding(Mark_1,data)
          
        if is_sliding:
            print("sliding")
            flag_stop = True
            print("slip distance:",slip)
            break

   
    mujoco.mj_copyData(data, model, base_data)




    return wrench,flag_stop


#Save and load    
base_data = mujoco.MjData(model)
mujoco.mj_copyData(base_data, model, data)


geom_name = "mod_3_out"
geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
geom_world_pos = data.geom_xpos[geom_id]
body_name = "mod_3_out"
body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
"""
Wrenches_slip = []
Wrenches_nonslip = []

for n in range(100):
  mujoco.mj_copyData(data, model, base_data)
  force = np.random.uniform(-200,200,size = 3)
  moment = np.random.uniform(-5,5,size = 3)
  wrench = np.array([0,0,0,0,0,0])
  wrench[:3] = force
  #wrench[3:] = moment
  time_step = 5000
  wrench,slip = test_wrench(wrench,time_step,model,data,base_data)
  if slip:
     print("wrench",wrench)
     Wrenches_slip.append(wrench)
  else:
     Wrenches_nonslip.append(wrench)
  

wrenches_slip = np.array(Wrenches_slip)
wrenches_nonslip = np.array(Wrenches_nonslip)
print(wrenches_slip)
print(wrenches_nonslip)

generator = prototype.WrenchPrototypeGenerator(cone_edges=32)
f_proto, m_proto = generator.compute_grasp_wrench_hull(contacts,geom_world_pos)
#generator.visualize_prototype(f_proto[0], f_proto[1], "3D Force Subspace of Wrench Hull",color = 'green')
#generator.visualize_prototype(m_proto[0], m_proto[1], "3D Moment Subspace of Wrench Hull",color = 'blue')
generator.visualize_contact_setup(contacts,geom_world_pos)

generator.visualize_prototype2(f_proto[0], f_proto[1],wrenches_nonslip[:,:3],wrenches_slip[:,:3], "3D Force Subspace of Wrench Hull",color = 'green')
generator.visualize_prototype2(m_proto[0], m_proto[1],wrenches_nonslip[:,3:],wrenches_slip[:,3:], "3D Moment Subspace of Wrench Hull",color = 'blue')

"""

data.xfrc_applied[body_id, :3] = np.array([200,0,0])#force
data.xfrc_applied[body_id, 3:] = np.array([0,0,0])#moment



Forces = []
Poses = []

geom_name = f"mod_{3}_out"
geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
geom_world_pos = data.geom_xpos[geom_id]

print(geom_world_pos)
#print(model.nsensor)
n = 5000
flag_stop = False
for j in range(n): 
   
        
      


    Poses.append(data.geom_xpos[geom_id].copy())
    mujoco.mj_step(model, data)
    F = np.array([0,0,0,0])
    for i in range(data.ncon):
      
      contact = data.contact[i]
 
      c_force = np.zeros(6, dtype=np.float64)
      mujoco.mj_contactForce(model, data, i, c_force)
      F[i] = c_force[0]
    Forces.append(F)

Forces = np.array(Forces)
Poses = np.array(Poses)
print(Forces[:,0].shape)

plt.figure(figsize=(10, 7))
for i in range (4):
  plt.plot(np.arange(Forces[:,i].shape[0]),Forces[:,i],'o',label = f"contact {i+1}")
plt.legend()
plt.show()


for i in range (3):
  plt.plot(np.arange(Poses[:,i].shape[0]),Poses[:,i],'-',label = f"D {i}")
plt.legend()
plt.show()
#mujoco.mj_copyData(data, model, base_data)

#print(model.nsensor)
print(geom_world_pos)
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    
    while viewer.is_running():
       
        #mujoco.mj_step(model, data)
        viewer.sync()       
        time.sleep(0.02)  


 
