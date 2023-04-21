#ifndef SBMPO_MOMENTUM_MODEL_UNICYCLE_KINEMATICS_HPP
#define SBMPO_MOMENTUM_MODEL_UNICYCLE_KINEMATICS_HPP

#include <sbmpo/model.hpp>
#include<iostream>

namespace my_namespace{

using namespace sbmpo;

class MomentumModelUnicycleKinematics : public Model {

    public:

    // enum States {X,V};  
    enum States {X,Y,th}; 

    // sample accel
    //enum Controls {A}; // Forward acceleration
 
    // sample forward and angular velocities
    enum Controls {V,Omega}; //
    
    MomentumModelUnicycleKinematics(){

       // threshold_X_ = 0.01f;
       // threshold_Y_ = 0.01f;

        threshold_X_ = 0.3f;
        threshold_Y_ = 0.3f;

        threshold_V_ = 1.0f;
              
        min_vel_ = -2.0f;
        max_vel_ = 10.0f;
        mass_ = 500.0f;
        I_ = 5.0f;
        g_ = 9.8f;
        r_ = 0.2;

     }


    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next_state = state;

          
        // read the terrain parameters
        float patch_start = -1.0; 
        float patch_end = 0.0;
        float b = 0;
        float bz = 0;



        float dx,dy;
        dx = control[V]*cos(state[th]);
        dy = control[V]*sin(state[th]);

        
    
        next_state[X] += dx*time_span; 
        next_state[Y] += dy*time_span; 

        next_state[th] += control[Omega]*time_span;      
        next_state[th] = atan2(sin(next_state[th]),cos(next_state[th])); 

    
    
        return next_state;

    }


    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {
        return time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {

        /*
        *   Time Optimal Heuristic
        */

        float h, d;

        d = (state[X]- goal[X])*(state[X]- goal[X]) + (state[Y]- goal[Y])*(state[Y]- goal[Y]);
        d = std::pow(d,0.5); 
       
        h = d/max_vel_; 

        return h;

    }


    // Determine if node is valid
    virtual bool is_valid(const State& state) {

       
        /*
            Does this state meet the model constraints?
            i.e Boundary constraints, Obstacles, State limits
        */

      /*
        if( (state[X]>= -5.0) && (state[X] <= 5.0) && (state[Y]>= -5.0) && (state[Y] <= 5.0)  ){

            return true;
        }

        else{
            
            return false;
        }
*/
    float d1;
    float r1 = 2.0;
    float obs1_x = 6.0;
    float obs1_y = 3.0;
    d1 = (state[X] - obs1_x)*(state[X] - obs1_x) + (state[Y] - obs1_y)*(state[Y] - obs1_y);
    if ( d1 < r1*r1 ){
        return false;
    }
    else{

        return true;
    }

    }


     // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {
        return std::abs(goal[X] - state[X]) <= threshold_X_
            && std::abs(goal[Y] - state[Y]) <= threshold_Y_;
            
    }

   
    virtual ~MomentumModelUnicycleKinematics() {}

    /// @brief Set the goal threshold X value
    /// @param goal_threshold_X Value to set as goal threshold X
    void set_goal_threshold_X(float goal_threshold_X) {
        threshold_X_ = goal_threshold_X;
    }

    /// @brief Set the goal threshold Y value
    /// @param goal_threshold_Y Value to set as goal threshold Y
    void set_goal_threshold_Y(float goal_threshold_Y) {
        threshold_Y_ = goal_threshold_Y;
    }

    /// @brief Set the goal threshold V value
    /// @param goal_threshold_V Value to set as goal threshold V
    void set_goal_threshold_V(float goal_threshold_V) {
        threshold_V_ = goal_threshold_V;
    }

    /// @brief Set the minimum acceleration value
    /// @param min_acceleration Value to set as the minimum acceleration
    void set_min_acceleration(float min_acceleration) {
        min_acc_ = min_acceleration;
    }

    /// @brief Set the maximum acceleration value
    /// @param max_acceleration Value to set as the maximum acceleration
    void set_max_acceleration(float max_acceleration) {
        max_acc_ = max_acceleration;
    }

    /// @brief Set the minimum acceleration value
    /// @param min_velocity Value to set as the minimum velocity
    void set_min_velocity(float min_velocity) {
        min_vel_ = min_velocity;
    }

    /// @brief Set the maximum velocity value
    /// @param max_velocity Value to set as the maximum velocity
    void set_max_velocity(float max_velocity) {
        max_vel_ = max_velocity;
    }


    /// @brief Set the mass value
    /// @param mass Value to set as the mass of the vehicle
    void set_mass(float mass) {
        mass_ = mass;
    }

    /// @brief Set the gravitational value
    /// @param gravity Value to set as the gravity
    void set_gravity(float gravity) {
        g_ = gravity;
    }


    /// @brief Set the radius value
    /// @param radius Value to set as the radius
    void set_radius(float radius) {
        r_ = radius;
    }

    /// @brief Set the moment of inertia value
    /// @param I Value to set as the moment of inertia
    void set_I(float I) {
        I_ = I;
    }



    protected:

   float threshold_X_;
   float threshold_Y_; 
   float threshold_V_;
   float min_acc_;
   float max_acc_;
   float min_vel_;
   float max_vel_;
   float mass_; 
   float I_;
   float r_;
   float g_;


};

}

#endif


