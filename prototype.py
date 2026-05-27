W, L = 0.0635-0.0254*2, 0.0506

import numpy as np
from scipy.spatial import ConvexHull
import itertools
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class WrenchPrototypeGenerator:
    def __init__(self, cone_edges=8):
        """
        :param cone_edges: Number of edges for the linearized friction cone (m).
        """
        self.m = cone_edges

    def _get_orthonormal_basis(self, normal):
        """
        Construct a local contact frame using the cross-product method.
        
        Returns:
            n  : unit normal vector.
            t1 : first unit tangent, orthogonal to n.
            t2 : second unit tangent, orthogonal to both n and t1.
        """
        n = np.array(normal, dtype=float)
        norm = np.linalg.norm(n)
        if norm < 1e-12:
            raise ValueError("Normal vector cannot be zero.")
        n = n / norm

        # --- Core improvement: pick the component of n with the smallest absolute 
        #     value to build a guaranteed non-parallel auxiliary vector. ---
        abs_n = np.abs(n)
        min_idx = np.argmin(abs_n)  # 0, 1, or 2

        aux = np.zeros(3)
        aux[min_idx] = 1.0          # e.g. if y is the smallest component, aux = [0,1,0]

        # t1 = n × aux. It is guaranteed orthogonal to n, and numerically most 
        # stable because aux is the least parallel to n.
        t1 = np.cross(n, aux)
        t1_norm = np.linalg.norm(t1)
        
        # Guard check: theoretically t1_norm cannot be zero since aux is not 
        # parallel to n, but we keep this for safety.
        if t1_norm < 1e-12:
            raise ValueError("Failed to construct orthogonal basis.")
        t1 = t1 / t1_norm

        # t2 = n × t1, automatically orthogonal to both n and t1.
        t2 = np.cross(n, t1)
        t2 = t2 / np.linalg.norm(t2)

        return n, t1, t2

    def get_contact_wrench_vertices(self, pos, normal, mu, f_n_max,ref_point = [0,0,0]):
        """
        Generate the set of Wrench vertices in 6D space for a single contact point.
        """
        n, t1, t2 = self._get_orthonormal_basis(normal)
        p = np.array(pos)-np.array(ref_point)
        
        # Include the zero-force point (representing no contact force applied)
        wrench_vertices = [np.zeros(6)]
        
        # Generate edge forces of the linearized friction cone and convert to Wrenches
        for j in range(self.m):
            angle = 2 * np.pi * j / self.m
            # Friction cone edge direction vector (normal component is 1)
            f_dir = n + mu * (np.cos(angle) * t1 + np.sin(angle) * t2)
            
            # Scale to the maximum normal force limit
            f_vector = f_n_max * f_dir
            
            # Map to 6D Wrench: [Force_x, Force_y, Force_z, Moment_x, Moment_y, Moment_z]
            moment_vector = np.cross(p, f_vector)
            wrench = np.concatenate([f_vector, moment_vector])
            wrench_vertices.append(wrench)
            
        return wrench_vertices
    
    def Friction_vertrices(self, pos, normal, mu, f_n_max,ref_point):
        """
        Generate the set of Wrench vertices in 6D space for a single contact point.
        """
        n, t1, t2 = self._get_orthonormal_basis(normal)
        p = np.array(pos)-np.array(ref_point)
        
        # Include the zero-force point (representing no contact force applied)
        f_vertices = [np.zeros(3)]
        
        # Generate edge forces of the linearized friction cone and convert to Wrenches
        for j in range(self.m):
            angle = 2 * np.pi * j / self.m
            # Friction cone edge direction vector (normal component is 1)
            f_dir = n + mu * (np.cos(angle) * t1 + np.sin(angle) * t2)
            
            # Scale to the maximum normal force limit
            f_vector = f_n_max * f_dir
            f_vertices.append(f_vector)
            # Map to 6D Wrench: [Force_x, Force_y, Force_z, Moment_x, Moment_y, Moment_z]
            
        return f_vertices
    
    def compute_grasp_wrench_space(self, contact_data):
        all_wrenches_per_point = []
        for c in contact_data:
            v = self.get_contact_wrench_vertices(c['p'], c['n'], c['mu'], c['f_max'])
            all_wrenches_per_point.append(v)

        combinations = itertools.product(*all_wrenches_per_point)
        total_wrench_vertices = np.array([sum(combo) for combo in combinations])

        force_points = total_wrench_vertices[:, 0:3]
        force_hull = ConvexHull(force_points)

        moment_points = total_wrench_vertices[:, 3:6]
        moment_hull = ConvexHull(moment_points)

        return (force_hull, force_points), (moment_hull, moment_points)



    def compute_grasp_wrench_hull(self, contact_data,ref_point):
        """
        Compute the Force and Moment prototypes using Minkowski sum and spatial projection.
        :param contact_data: List of dicts {'p': [x,y,z], 'n': [nx,ny,nz], 'mu': float, 'f_max': float}
        """
        all_wrenches = []
        for c in contact_data:
            v = self.get_contact_wrench_vertices(c['p'], c['n'], c['mu'], c['f_max'],ref_point)
            all_wrenches.extend(v)

        # 1. Compute mapping of fi,j to wi,j
       
        total_wrench_vertices = np.array(all_wrenches) # {Wi,j}

        # 2. Project onto Force space (extract the first 3 dimensions)
        force_points = total_wrench_vertices[:, 0:3]
        force_hull = ConvexHull(force_points)

        # 3. Project onto Moment space (extract the last 3 dimensions)
        moment_points = total_wrench_vertices[:, 3:6]
        moment_hull = ConvexHull(moment_points)

        return (force_hull, force_points), (moment_hull, moment_points)

    def visualize_prototype1(self, hull, points, title="Polytope Prototype"):
        """
        Visualize the resulting 3D convex polytope.
        """
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot the internal/boundary vertices

        ax.scatter(points[:,0], points[:,1], points[:,2], color='r',s =15, alpha=0.5)
        
        # Plot the convex hull surface
        for simplex in hull.simplices:
            ax.plot(points[simplex, 0], points[simplex, 1], points[simplex, 2], 'b-')
            
        ax.set_title(title)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    def visualize_prototype(self, hull, points, title="Polytope Prototype",color = 'green'):
        """
        Visualize the resulting 3D convex polytope (Upgraded Surface).
        """
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')
        
        # 1. Plot points: slightly reduce point size (s=15) and increase transparency
        ax.scatter(points[:,0], points[:,1], points[:,2], color='y', s=15, alpha=0.5)
        
        # 2. Plot the solid surface (Replaces your original for-loop)
        # Extract all triangular faces from the convex hull
        faces = [points[simplex] for simplex in hull.simplices]
        
        # Generate 3D surface: facecolors controls fill, edgecolors controls border, alpha controls transparency
        poly3d = Poly3DCollection(faces, alpha=0.4, facecolors=color, edgecolors=color, linewidths=0.5)
        ax.add_collection3d(poly3d)
        
        # 3. Enforce equal aspect ratio for 3D axes (Crucial! Prevents matplotlib from automatically stretching/distorting the polytope)
        max_range = np.array([points[:,0].max()-points[:,0].min(),
                              points[:,1].max()-points[:,1].min(),
                              points[:,2].max()-points[:,2].min()]).max() / 2.0
        
        mid_x = (points[:,0].max()+points[:,0].min()) * 0.5
        mid_y = (points[:,1].max()+points[:,1].min()) * 0.5
        mid_z = (points[:,2].max()+points[:,2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # Set labels and title
        ax.set_title(title, fontweight='bold')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        plt.show()

    def visualize_prototype2(self, hull, points,test_stable, test_slip, title="Polytope Prototype",color = 'green', ):
        """
        Visualize the resulting 3D convex polytope (Upgraded Surface).
        """
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')
        
        # 1. Plot points: slightly reduce point size (s=15) and increase transparency
        ax.scatter(points[:,0], points[:,1], points[:,2], color='y', s=15, alpha=0.5,label = 'Boundary Point')
        ax.legend()
        ax.scatter(test_stable[:,0], test_stable[:,1], test_stable[:,2], color='orange', s=15, alpha=0.95, label = 'Non-slip')
        ax.legend()
        ax.scatter(test_slip[:,0], test_slip[:,1], test_slip[:,2], color='grey', s=15, alpha=0.95,label = 'Slip')
        ax.legend()
        # 2. Plot the solid surface (Replaces your original for-loop)
        # Extract all triangular faces from the convex hull
        faces = [points[simplex] for simplex in hull.simplices]
        
        # Generate 3D surface: facecolors controls fill, edgecolors controls border, alpha controls transparency
        poly3d = Poly3DCollection(faces, alpha=0.05, facecolors=color, edgecolors=color, linewidths=0.5)
        ax.add_collection3d(poly3d)
        
        # 3. Enforce equal aspect ratio for 3D axes (Crucial! Prevents matplotlib from automatically stretching/distorting the polytope)
        max_range = np.array([points[:,0].max()-points[:,0].min(),
                              points[:,1].max()-points[:,1].min(),
                              points[:,2].max()-points[:,2].min()]).max() / 2.0
        
        mid_x = (points[:,0].max()+points[:,0].min()) * 0.5
        mid_y = (points[:,1].max()+points[:,1].min()) * 0.5
        mid_z = (points[:,2].max()+points[:,2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # Set labels and title
        ax.set_title(title, fontweight='bold')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        plt.show()


    def visualize_contact_setup(self,contact_data, ref_point=[0, 0, 0], force_arrow_scale=0.1, title="Contact Setup Visualization"):
        """
        Visualize the geometric setup in 3D:
        - Contact Points
        - Normal Forces (Quiver arrows)
        - Reference Point (e.g., CoM or Spatial Frame origin)
        """
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # 1. Plot Reference Point (Origin of Spatial Frame/CoM)
        # Marked as a large red 'X'
        ref_p = np.array(ref_point)
        ax.scatter(ref_p[0], ref_p[1], ref_p[2], color='r', marker='X', s=200, label='Reference Point (Origin)')

        # Extract data for plotting contact points and normals
        p_list = []
        n_list = []
        f_max_list = []
        
        for c in contact_data:
            p_list.append(c['p'])
            n_list.append(c['n'])
            # We use f_max to visually distinguish the arrows, though they are all scaled by force_arrow_scale
            f_max_list.append(c['f_max']) 

        p_array = np.array(p_list)
        n_array = np.array(n_list)
        
        # Scale normals by f_max to give a hint of different capacities (optional, adjusted by scale factor)
        # If all f_max are same, arrows are same length.
        arrows = n_array * (np.array(f_max_list)[:, np.newaxis] * force_arrow_scale)

        # 2. Plot Contact Points
        # Marked as large green dots
        ax.scatter(p_array[:, 0], p_array[:, 1], p_array[:, 2], color='g', s=100, depthshade=False, label='Contact Points')

        # 3. Plot Normal Vectors as Quivers (arrows)
        # Blue arrows pointing in the direction of normals, starting from contact points
        ax.quiver(p_array[:, 0], p_array[:, 1], p_array[:, 2],  # Starting points
                  arrows[:, 0], arrows[:, 1], arrows[:, 2],     # Vector components
                  color='b', linewidth=0.5, length=0.01, pivot='tail', label='Normal Forces')

        # 4. Enforce Equal Aspect Ratio (Extremely Important for geometry viz)
        # Calculate bounds from all points (contacts + reference point)
        all_geometrical_points = np.vstack([p_array, ref_p])
        max_range = np.array([all_geometrical_points[:,0].max()-all_geometrical_points[:,0].min(),
                              all_geometrical_points[:,1].max()-all_geometrical_points[:,1].min(),
                              all_geometrical_points[:,2].max()-all_geometrical_points[:,2].min()]).max() / 2.0
        
        mid_x = (all_geometrical_points[:,0].max()+all_geometrical_points[:,0].min()) * 0.5
        mid_y = (all_geometrical_points[:,1].max()+all_geometrical_points[:,1].min()) * 0.5
        mid_z = (all_geometrical_points[:,2].max()+all_geometrical_points[:,2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # Set labels and title
        ax.set_title(title, fontweight='bold', pad=20)
        ax.set_xlabel('X', labelpad=10)
        ax.set_ylabel('Y', labelpad=10)
        ax.set_zlabel('Z', labelpad=10)
        
        ax.grid(color='lightgray', linestyle='--', linewidth=0.5)
        
        # Change pane colors to match standard academic looks
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        
        ax.legend(loc='best', framealpha=0.5)
        plt.tight_layout()
        plt.show()

# ================= Usage Example =================

if __name__ == "__main__":
    # 1. Define input parameters
    # Example: A robot with two feet on the ground (Origin is at the Center of Mass)
    contacts = [
        #{
         #   'p': [0.0, 0.0, 0.0],   # Right foot position relative to CoM under Spatial Frame 
         #   'n': [1.0, 0.0, 0.0],    # Normal vector pointing upwards
          #  'mu': 1.35,               # Coefficient of friction
           # 'f_max': 50.0            # Maximum normal force capacity for this contact
        #},
        {
            'p': [0.03175, 0.0, 0.02874], # Left foot position relative to CoM
            'n': [-1.0, 0.0, 0.0],    
            'mu': 0.5, 
            'f_max': 100.0
        },
                {
            'p': [-0.03175, 0.0, 0.07537], # Left foot position relative to CoM
            'n': [1.0, 0.0, 0.0],    
            'mu': 0.5, 
            'f_max': 100.0
        },
                 {
            'p': [0.03175, 0.0, -0.02874], # Left foot position relative to CoM
            'n': [-1.0, 0.0, 0.0],    
            'mu': 0.5, 
            'f_max': 100.0
        },
                 {
            'p': [-0.03175, 0.0, -0.07537], # Left foot position relative to CoM
            'n': [1.0,0.0,  0.0],    
            'mu': 0.5, 
            'f_max': 100.0
        }
    ]

    # 2. Instantiate and compute
    ref_point=[0.0,0.0,0.102]
    generator = WrenchPrototypeGenerator(cone_edges=32)
    f_proto, m_proto = generator.compute_grasp_wrench_hull(contacts,ref_point)

    # 3. Print output metrics
    print(f"Force Prototype Vertices: {len(f_proto[0].vertices)}")
    print(f"Force Prototype Volume (Force Capacity): {f_proto[0].volume:.4f}")
    print(f"Moment Prototype Vertices: {len(m_proto[0].vertices)}")
    print(f"Moment Prototype Volume (Balance Capacity): {m_proto[0].volume:.4f}")

    # 4. Visualize the results
    generator.visualize_prototype(f_proto[0], f_proto[1], "3D Force Subspace of Wrench Hull",color = 'green')
    generator.visualize_prototype(m_proto[0], m_proto[1], "3D Moment Subspace of Wrench Hull",color = 'blue')
    
    c = contacts[0]
    friction_cone = generator.Friction_vertrices(c['p'], c['n'], c['mu'], c['f_max'],ref_point)
    #print(friction_cone)
    friction_hull = ConvexHull(friction_cone)
    friction_vertrices  = np.array(friction_cone)
    generator.visualize_prototype1(friction_hull,friction_vertrices,"Friction Cone Linearized")
    generator.visualize_contact_setup(contacts,ref_point)
    
    nonslip = np.random.uniform(-10,10,size = (15,3))
    slip = np.random.uniform(-10,10,size = (15,3))
    generator.visualize_prototype2(friction_hull,friction_vertrices,nonslip,slip,"Friction Cone Linearized")