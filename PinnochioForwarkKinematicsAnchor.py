import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

Joint_axises = []
Joint_axises.append(np.array([0,0,1]))
Joint_axises.append(np.array([0,1,0]))
Joint_axises.append(np.array([0,0,1]))
Joint_axises.append(np.array([0,1,0]))

def Link(axis,joint_id,model,link_id,translation):
    T_link = pin.SE3.Identity()
    T_link.translation = translation
    joint_axis = pin.JointModelRevoluteUnaligned(axis)
    T_link_id = model.addJoint(link_id, joint_axis, T_link, f"joint{joint_id+1}")
    return T_link_id


def main():
    
    R = []
    model = pin.Model()
    i = 0
    link_id = 0
    for axis in Joint_axises:
        if i == 0:
            link_id = Link(axis,i,model,link_id,np.array([0,0,0]))
        else:

            link_id = Link(axis,i,model,link_id,np.array([0.0506,0,0]))
        i+=1
    T_ee = pin.SE3.Identity()
    T_ee.translation = np.array([0.0506, 0.0, 0.0])
    parent_frame_id = model.getFrameId(f"joint{len(Joint_axises)}")
    ee_frame = pin.Frame("end_effector", 
                         link_id, 
                         parent_frame_id, 
                         T_ee, 
                         pin.FrameType.OP_FRAME)
    
    ee_frame_id = model.addFrame(ee_frame)

    data = model.createData()
    q = np.array([np.pi/2-1.35,0,1.35,0])
    #q = np.array([0,0,0,0])
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    for k in range(len(Joint_axises)):
        print(f"Homogenous transfermation matrix {k},{k+1}: {data.oMi[k+1]}")
        P_ = data.oMi[k+1].translation.reshape(-1,1)
        if k == 0:
            P = P_
        else:
            P = np.hstack([P,P_])
        R.append(data.oMi[k+1].rotation)
        print(f"Homogenous transformation matrix of End Effector: \n{data.oMf[ee_frame_id]}")
    P_ee = data.oMf[ee_frame_id].translation.reshape(-1, 1)
    P = np.hstack([P, P_ee])
    R.append(data.oMf[ee_frame_id].rotation)
    verify_kinematics_plot(P,R,Joint_axises)
    pin.computeJointJacobians(model, data, q)
    J_ee = pin.getFrameJacobian(model, data, ee_frame_id, pin.ReferenceFrame.WORLD)
    print("Spatial Jacobian", J_ee)

def verify_kinematics_plot(P,R,Joint_axises):
    
    print("\n--- Test 1: Kinematics 3D Visualization ---")
    max_x = max(P[0,:])
    min_x = min(P[0,:])
    max_y = max(P[1,:])
    min_y = min(P[1,:])
    max_z = max(P[2,:])
    min_z = min(P[2,:])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(0,0,0,'ro',markersize = 10, label = 'Fixed frame')
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', length=0.01, arrow_length_ratio=0.2, normalize=True, label = "xs")
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', length=0.01,  arrow_length_ratio=0.2, normalize=True, label = "ys")
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', length=0.01,  arrow_length_ratio=0.2, normalize=True, label = "zs")

    ax.plot(P[0,:], P[1,:],P[2,:], 'yo-', linewidth=2, markersize=8, label='Joints/Links')
    i=0
    for Rot in R:
        if i < len(Joint_axises):
            if Joint_axises[i][0] == 1:
                ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,0], Rot[1,0], Rot[2,0], color='r', length=0.03, arrow_length_ratio=0.2, normalize=True)
                ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,1], Rot[1,1], Rot[2,1], color='g', length=0.01,  arrow_length_ratio=0.2, normalize=True)
                ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,2], Rot[1,2], Rot[2,2], color='b', length=0.01,  arrow_length_ratio=0.2, normalize=True)
            elif Joint_axises[i][1] == 1:
                ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,0], Rot[1,0], Rot[2,0], color='r', length=0.01, arrow_length_ratio=0.2, normalize=True)
                ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,1], Rot[1,1], Rot[2,1], color='g', length=0.03,  arrow_length_ratio=0.2, normalize=True)
                ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,2], Rot[1,2], Rot[2,2], color='b', length=0.01,  arrow_length_ratio=0.2, normalize=True)
            elif Joint_axises[i][2] == 1:
                ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,0], Rot[1,0], Rot[2,0], color='r', length=0.01, arrow_length_ratio=0.2, normalize=True)
                ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,1], Rot[1,1], Rot[2,1], color='g', length=0.01,  arrow_length_ratio=0.2, normalize=True)
                ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,2], Rot[1,2], Rot[2,2], color='b', length=0.03,  arrow_length_ratio=0.2, normalize=True)
        else:
            ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,0], Rot[1,0], Rot[2,0], color='r', length=0.01, arrow_length_ratio=0.2, normalize=True)
            ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,1], Rot[1,1], Rot[2,1], color='g', length=0.01,  arrow_length_ratio=0.2, normalize=True)
            ax.quiver(P[0,i], P[1,i], P[2,i], Rot[0,2], Rot[1,2], Rot[2,2], color='b', length=0.01,  arrow_length_ratio=0.2, normalize=True)
      
        i=i+1
    


    #ax.plot([1,0,0], [0,0,1], 'bo-', linewidth=2, markersize=8, label='Joints/Links')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('Robot Kinematic Chain Verification')
    ax.legend()
    ax.set_xlim(-0.05,0.05)
    ax.set_ylim(-0.05,0.05)
    ax.set_zlim(-0.05,0.05)
    ax.set_box_aspect([1, 1, 1])
    
    plt.show()
    


main()    