import numpy as np

class ConvexMpc:
    def __init__(
        self,
        body_mass,
        body_inertia_list,
        num_legs,
        planning_horizon_steps,
        planning_timestep,
        weights_list,
        regularization_weight=1e-5,
        qp_solver=None
    ):
        """
        Convex MPC controller for legged locomotion.
        
        Args:
            body_mass: Mass of the robot body (kg)
            body_inertia_list: List of inertia tensor components [Ixx, Iyy, Izz] or full 3x3 matrix
            num_legs: Number of legs on the robot
            planning_horizon_steps: Number of steps in the planning horizon
            planning_timestep: Time duration of each planning step (seconds)
            weights_list: List of weights for cost function terms
            regularization_weight: Regularization weight for QP problem
            qp_solver: QP solver instance (if None, will need to be set later)
        """
        self._body_mass = body_mass
        self._body_inertia = np.array(body_inertia_list)
        self._num_legs = num_legs
        self._PLANNING_HORIZON_STEPS = planning_horizon_steps
        self._PLANNING_TIMESTEP = planning_timestep
        self._weights = weights_list
        self._regularization_weight = regularization_weight
        self._qp_solver = qp_solver
        
        # Initialize other necessary variables
        self._setup_mpc_problem()
    
    def _setup_mpc_problem(self):
        """Setup the MPC problem matrices and constraints."""
        # This would typically involve:
        # 1. Creating system dynamics matrices (A, B)
        # 2. Setting up cost matrices (Q, R)
        # 3. Initializing constraint matrices
        # 4. Preparing QP problem formulation
        
        # Placeholder implementation
        self._state_dim = 13  # [position(3), orientation(4), linear_vel(3), angular_vel(3)]
        self._control_dim = 3 * self._num_legs  # 3D force per leg
        
        print(f"MPC initialized with {self._num_legs} legs, {self._PLANNING_HORIZON_STEPS} horizon steps")
    
    def compute_contact_forces(
        self,
        com_position,  # [x,y,z] in world frame
        com_velocity,  # [x_dot,y_dot,z_dot] in body frame
        com_roll_pitch_yaw,  # [roll, pitch, yaw] in world frame
        com_angular_velocity,  # [wx, wy, wz] in body frame
        foot_contact_state,  # [c1, c2, c3, c4] contact states for 4 legs
        foot_positions_base_frame,  # [p1_x,p1_y,p1_z,...,p4_x,p4_y,p4_z] in body frame
        foot_friction_coeffs,  # [k1,k2,k3,k4]
        desired_com_position,  # desired_com_position in world frame
        desired_com_velocity,  # desired_linear_velocity in body frame
        desired_com_roll_pitch_yaw,  # desired_com_roll_pitch_yaw
        desired_com_angular_velocity  # desired_com_angular_velocity
    ):
        """
        Compute optimal contact forces using convex MPC.
        
        Returns:
            contact_forces: Array of contact forces for each leg in world frame
                            Shape: (num_legs * 3,) - [f1_x, f1_y, f1_z, f2_x, ...]
        """
        # Convert inputs to numpy arrays
        com_pos = np.array(com_position)
        com_vel = np.array(com_velocity)
        com_rpy = np.array(com_roll_pitch_yaw)
        com_ang_vel = np.array(com_angular_velocity)
        foot_contacts = np.array(foot_contact_state)
        foot_positions = np.array(foot_positions_base_frame)
        friction_coeffs = np.array(foot_friction_coeffs)
        desired_pos = np.array(desired_com_position)
        desired_vel = np.array(desired_com_velocity)
        desired_rpy = np.array(desired_com_roll_pitch_yaw)
        desired_ang_vel = np.array(desired_com_angular_velocity)
        
        # Validate input dimensions
        self._validate_inputs(
            com_pos, com_vel, com_rpy, com_ang_vel,
            foot_contacts, foot_positions, friction_coeffs,
            desired_pos, desired_vel, desired_rpy, desired_ang_vel
        )
        
        # 1. Compute rotation matrix from body to world frame
        R_body_to_world = self._rpy_to_rotation_matrix(com_rpy)
        
        # 2. Formulate the current state vector
        current_state = self._formulate_state_vector(
            com_pos, com_vel, com_rpy, com_ang_vel
        )
        
        # 3. Formulate the desired state vector
        desired_state = self._formulate_state_vector(
            desired_pos, desired_vel, desired_rpy, desired_ang_vel
        )
        
        # 4. Build the MPC problem
        qp_problem = self._build_mpc_problem(
            current_state,
            desired_state,
            foot_contacts,
            foot_positions,
            friction_coeffs,
            R_body_to_world
        )
        
        # 5. Solve the QP problem
        if self._qp_solver is None:
            # Use a simple fallback if no solver provided
            contact_forces = self._solve_simple_mpc(
                current_state, desired_state, foot_contacts, R_body_to_world
            )
        else:
            # Use the provided QP solver
            contact_forces = self._qp_solver.solve(qp_problem)
        
        return contact_forces
    
    def _validate_inputs(self, com_pos, com_vel, com_rpy, com_ang_vel,
                        foot_contacts, foot_positions, friction_coeffs,
                        desired_pos, desired_vel, desired_rpy, desired_ang_vel):
        """Validate input dimensions and values."""
        assert len(com_pos) == 3, "COM position must be 3D"
        assert len(com_vel) == 3, "COM velocity must be 3D"
        assert len(com_rpy) == 3, "COM RPY must be 3D"
        assert len(com_ang_vel) == 3, "COM angular velocity must be 3D"
        assert len(foot_contacts) == self._num_legs, f"Foot contacts must have {self._num_legs} elements"
        assert len(foot_positions) == 3 * self._num_legs, f"Foot positions must have {3 * self._num_legs} elements"
        assert len(friction_coeffs) == self._num_legs, f"Friction coefficients must have {self._num_legs} elements"
    
    def _rpy_to_rotation_matrix(self, rpy):
        """Convert roll, pitch, yaw to rotation matrix."""
        roll, pitch, yaw = rpy
        
        # ZYX rotation (common in robotics)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        return Rz @ Ry @ Rx
    
    def _formulate_state_vector(self, position, velocity, rpy, angular_velocity):
        """Formulate state vector from components."""
        # For simplicity, using RPY representation
        # In a full implementation, you might use quaternions
        state = np.concatenate([
            position,        # 3D position
            rpy,            # 3D orientation (roll, pitch, yaw)
            velocity,       # 3D linear velocity
            angular_velocity # 3D angular velocity
        ])
        return state
    
    def _build_mpc_problem(self, current_state, desired_state, foot_contacts,
                          foot_positions, friction_coeffs, R_body_to_world):
        """Build the MPC optimization problem."""
        # This is a simplified placeholder implementation
        # A real implementation would:
        # 1. Create prediction matrices for the horizon
        # 2. Set up dynamics constraints
        # 3. Set up friction cone constraints
        # 4. Set up cost function
        
        problem = {
            'current_state': current_state,
            'desired_state': desired_state,
            'foot_contacts': foot_contacts,
            'foot_positions': foot_positions,
            'friction_coeffs': friction_coeffs,
            'rotation_matrix': R_body_to_world,
            'horizon_steps': self._PLANNING_HORIZON_STEPS,
            'timestep': self._PLANNING_TIMESTEP
        }
        
        return problem
    
    def _solve_simple_mpc(self, current_state, desired_state, foot_contacts, R_body_to_world):
        """Simple fallback MPC solver for demonstration."""
        # This is a very simplified implementation
        # A real implementation would solve a proper QP problem
        
        num_legs = self._num_legs
        contact_forces = np.zeros(3 * num_legs)
        
        # Simple gravity compensation
        gravity_force = self._body_mass * 9.81 / np.sum(foot_contacts) if np.sum(foot_contacts) > 0 else 0
        
        for i in range(num_legs):
            if foot_contacts[i]:
                # Apply vertical force for gravity compensation
                force_world = np.array([0, 0, gravity_force])
                # Transform to body frame if needed, or keep in world frame
                contact_forces[3*i:3*i+3] = force_world
        
        return contact_forces