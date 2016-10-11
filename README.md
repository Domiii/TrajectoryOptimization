# Trajectroy Optimization Project [2013]

## Introduction

This has been a project for the 2013 Numerical Optimization course of the NTU Math department.

[This is my final presentation](https://goo.gl/cBhfHg).

In this project, I aim to optimize the trajectory for an agent by controlling its vector of actuators. The trajectory needs to bring the object from some start state to some goal state, subject to a set of general constraints (in this case, the constraints being basic physics). This is a common problem in path planning and robotics.

I use [a direct method as explained by Russ Tedrake in this lecture video](https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/video-lectures/lecture-9-trajectory-optimization/) and [this 2013 paper](http://groups.csail.mit.edu/robotics-center/public_papers/Posa13.pdf). The method starts out with an initial guess (a really bad trajectory spanning from start to goal and might not even satisfy all constraints), and then, by defining the physics as explicit constraints, solve and eventually (hopefully) ease into an actual solution. The entire trajectory is pre-defined by `N` steps, where each step represents a subset of constraints. By increasing the amount of steps, you increase the solution's accuracy.

The heavy lifting is done by Matlab's own nonlinear constraint solver [fmincon](https://www.mathworks.com/help/optim/ug/fmincon.html) set to the [SQP algorithm](https://www.mathworks.com/help/optim/ug/constrained-nonlinear-optimization-algorithms.html#f26622) for this.

NOTE: I polluted the repository a little in an effort to generalize things, and add more projects to it, so to find your way around:

* [The Final Matlab Code of the project](https://github.com/Domiii/TrajectoryOptimization/tree/master/matl/num%20opt%20proj)
* [C# Matlab Code Creation](https://github.com/Domiii/TrajectoryOptimization/tree/master/Squishy.Matlab)

The project has two test-cases:

## Test Case #1: 2D Pathfinding

The first is simple pathfinding in a 2.5D space where height indicates cost. I wrote the code by hand, which was Ok, since the constraint gradients (or: constraint Jacobian) was relatively simple to build.

[You can find the code here](https://github.com/Domiii/TrajectoryOptimization/blob/master/matl/num%20opt%20proj/test1/test1run.m). It is bug-free (to the best of my knowledge).

You can see results with all kinds of different settings at the end of the presentation (because it is so nice and visual, I added more later on).

The blue line is the initial guess. The red line is the optimized trajectory. As you can see, it won't easily fall into local minima.

![test case #1-1](http://i.imgur.com/wKrbNa9.png)

![test case #1-2](http://i.imgur.com/bmlTWmT.png)

## Test Case #2: Jumping Spring

In this second example, I used a motor that controlled the contraction of a jumping spring (the contraction of the spring being the actuator). This is much more complex, since the state space is a lot bigger, we need to account for collision dynamics (effectively making it a system of hybrid dynamics) and (because of that) our constraints have non-trivial (not to say very complicated) gradients. Those constraint gradients are a huge matrix where each column needs to map correctly to the other inputs and outputs, making it rather tedious to compute.

I remember, the results looked pretty good, but had a few bugs.

[Here is the final Matlab script](https://github.com/Domiii/TrajectoryOptimization/blob/master/matl/num%20opt%20proj/test2/test2run.m). As you can see it is just a hell of a lot of numbers. I did not write those myself, instead I used a C# program to write this script.

[Here is the C# program that created the final Matlab script](https://github.com/Domiii/TrajectoryOptimization/blob/master/Squishy.Matlab/Dyn1Program.cs).

I explain some of the ideas behind the Matlab-writing-C# code in the [presentation](https://goo.gl/cBhfHg).

## Random Notes

### Papers
    - Survey of Numerical Methods for Trajectory Optimization [1998]
        - http://arc.aiaa.org/doi/abs/10.2514/2.4231
    
    

### Optimization
    -> Trust Region Method is complementary problem (see Chapter 4, p.5)
    
    -> Chapter 12 Notes: Contrained Optimization
        - Feasible set: Set of all points for which all constraints are satisfied
        - Active set: Set of all constraints that might not be satisfied
            - Always contains all equality constraints, and only some inequality constraints
            - Inactive constraints will be satisfied no matter where we go, for some neigborhood around current x
                - The Lagrangian multiplier of an inactive constraint is always 0 because of the complementary condition
        - Define Lagrangian function: L(x, lambda) = f(x) - lambda^T C
        - Necessary (but not sufficient) condition: grad_x (L) = 0 => grad(f) = lambda^T grad(C)
        - Inequality constraints:
            - ... also have the complementary condition: lambda^T C = 0
                -> If C is active: lambda > 0
                -> Else:           lambda = 0 => grad(f) = 0
        - Lagrangian Multipliers are easily derived using the 2 first terms of the Taylor series (i.e. a linear approximation of f)
            -> That only works if "linearized approximation captures the essential geometric features of the feasible set near the point x in question"
        - Tangent Cone & Constraint qualification
            - Define feasible sequence {z_k} and some sequence of positive scalars {t_k} where t_k -> 0
            - Then any tangent of the feasibility set satisfies: d = limit k->inf (z_k - x) / t_k
            - Constraint qualifications: 
                - The set of linearized feasible directions F(x) = {d | d^T nabla ci(x)=0 forall i in E, d^T nabla ci(x)>=0, forall in A intersect I }
                - "Constraint qualifications are conditions under which the linearized feasible set F(x) is similar to the tangent cone"
                - Commonly used Constraint Qualification: LICQ
                    - The gradients of all currently active constraints must be linearly independent
        - KKT: First-order optimality conditions
            - At a solution x^* the following holds:
                - grad_x (L) = 0
                    => grad(f) = lambda^T grad(C)
                - Constraints satisfied
                - All lambdas >= 0
                - lambda_i c_i = 0 for all constraints c_i
            - Note:
                nabla f = sum_{i \in A} lambda_i nabla c_i
            - Strict complementarity:
                - Def.: lambda_i > 0 forall active constraints
                - Makes things easier
            
    -> Chapter 13 Notes: Linear Programming & The Simplex Method
        - Problem:
            min c^T x, s.t. Ax = b, x >= 0
            - A is m x n, usually full row-rank
            - x has len n, b has len m
        - KKT Formulation:
            A^T lambda + s = c
            A x = b
            x >= 0, s >= 0, x^T s = 0
        - Two sets of multipliers:
            lambda (len = m) for equality constraints Ax = b
            s (len = n) for bound constraints x >= 0
        - Simplex Method strategy: Only examine basic feasible points
            - x is a basic feasible point if it is feasible and there exists a subset of A (called the basis) which:
                - contains exactly m indices
                - x_i > 0 (x_i is inactive) only if i in B
                - has nonsingular basis matrix B = [A_i], for all i in the basis of the problem
            - All basic feasible points are vertices of the feasible polytype {x | Ax = b, x>=0} (and vice versa)
            - Let N be the complement of B in A, then A x = B x_B + N x_N = b
                - where x_S is [x_i] forall i in S
                - Note: x_N = 0
            - Algo idea:
                - Start at some vertex of the polytype (i.e. a point in B)
                - Then move along an edge to the next vertex
                    - When transistioning from edge to vertex, some constraint that was active has become inactive and another one that was inactive has become active
                - Then swap those two constraints between B and N
                - Find new vertex: x^+_B = x_B - B^-1 A_q x^+_q
            
        
        
        

### Underactuated Robotics
    -> Dynamic Programming
    -> Partial Feedback Linearization (PFL)
        -> Use active joints to control passive joints
        -> Collocated  = active joints
        -> Non-collocated = passive joints
        - Notes
            -> "Pumps in a somewhat arbitrary amount of energy to squish the dynamics"
            -> "Collocated means: We use the action to linearize the state that is associated with that action; e.g. we gonna linearize the dynamics of the cart"
            -> See: http://www.clemson.edu/ces/crb/ece496/spring2004/GroupA/index_files/C59Spong.pdf
            -> Need at least as many active as passive joints to control all of the passive joints
            -> There must be "inertial coupling" between controlling and passive joint (Called: Strong inertial coupling)
        -> General PFL: Task Space PFL
            - m passive and l active joints
            - http://web.mit.edu/shkolnik/www/publications/Iros08.pdf
                -> "In a real sense, the control mapping we provide from task space to (underactuated) joint space is a mechanism for
                        exploiting known properties of the dynamics of the robot in order to dramatically improve the efficiency of search"

    -> Energy shaping
        - homoclinic orbit (goes from one unstable fixed point to the same u.f.p.)
        -> Combine with Collocated PFL: Paper  by Spong, 1996
        
        
    Walking & Running
        -> Center of Pressure = Centroid of the vertical contact force distribution
        -> For flat horizontal ground surfaces, ZMP = CoP
        
        
    Papers
        SIGGRAPH 2010: Terrain-Adaptive Bipedal Locomotion Control
        ACM SIGGRAPH Asia 2011: Controlling Physics-Based Characters Using Soft Contacts
        Physically Plausible Simulation for Character Animation
            -> PD Control = Proportional Derivative (PD) Control
                - PD gains control frequency (omega) and damping (zeta)
                - Critical damping means zeta = 1
            
