#ifndef SBMPO_MOMENTUM_MODEL_HPP
#define SBMPO_MOMENTUM_MODEL_HPP

#include <sbmpo/model.hpp>
#include<iostream>

namespace my_namespace{

using namespace sbmpo;

class MomentumModel : public Model {

    public:

    enum States {X,V}; // X: global X position, V: 
 
    // sample accel
    //enum Controls {A}; // Forward acceleration
 
    // sample torque
    enum Controls {Tau}; // Torque input
    
    MomentumModel(){
        threshold_x_ = 0.01f;
        threshold_v_ = 0.01f;

        min_acc_ = -1.0f;
        max_acc_ = 1.0f;
              
        min_vel_ = -2.0f;
        max_vel_ = 2.0f;
        mass_ = 1.0f;
        r_ = 0.2f; 
        g_ = 9.8f;

     }


    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next_state = state;

        float Izz = 0.5 * mass_* r_ * r_;
        float coeff = r_/(mass_*r_*r_ + Izz);
        float mu, mu_patch;
        float normal;
        float friction, friction_max;
        float acc;      
        float fload,fload_patch;
        bool  is_force_valid;

        float load_to_traction = 1.0;

        float patch_start = -1.0; 
        float patch_end = 0.0;
       

        
        // friction depends on location
        normal = mass_ * g_;
        mu_patch = 1.0;

        mu = ( (state[X] >= patch_start)  && (state[X] <= patch_end) ) ? mu_patch : 1.0;
        friction_max = mu*normal;
       
        fload_patch = load_to_traction*friction_max; 
        
        fload = ( (state[X] >= patch_start)  && (state[X] <= patch_end) ) ? fload_patch : 0.0;
      
        acc = ( control[Tau] - fload * r_) * coeff;

        if(fabs(state[X]+1.0)<0.001){
            std::cout<<"x, acc, fload: " << state[X] << "     " << acc << "   " << fload <<  std::endl;
        }

        friction = mass_ * acc + fload; // required friction  

        is_force_valid = ( (friction >= -friction_max) && (friction <= friction_max) ) ? true : false;

        if (is_force_valid){
            next_state[X] += state[V] * time_span;        
            next_state[V] += acc * time_span;
        }
       
                    
       
        // --- sample accel  
        /*
        // Get terrain properties

        float damping;
        float Fres;
        float F_max = 0.7;
        float mu;      

        Fres = ( (state[X] > 0)  && (state[X] < 1) ) ? 1.0 : 0.0;
 

        // required force
        float F;
        bool is_force_valid;    

        //bool force_within_limits;

        
        F = mass_*control[A] + Fres; 
        is_force_valid = ( (F >= -F_max)  && (F <= F_max) ) ? true: false;

       // is_force_valid = true;

       // only propagate state if force is valid
       if (is_force_valid){
        next_state[X] += state[V] * time_span;        
        next_state[V] += control[A] * time_span;
       }
       */

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

        //max_acc_ = 0.2;
        //min_acc_ = -1.0;
        float h;

        float q_10 = state[X] - goal[X];
        float q_20 = state[V] - goal[V];

        float b = 0, c = 0;
        if (q_10 + 0.5f*q_20*std::abs(q_20) / max_acc_ >= 0) {
            b = 2.0f * q_20 / min_acc_;
            c = (q_20*q_20 + 2.0f*(max_acc_ - min_acc_)*q_10) / (max_acc_ * min_acc_);
        } else if (q_10 - 0.5f*q_20*std::abs(q_20) / min_acc_ < 0) {
            b = 2.0f * q_20 / max_acc_;
            c = (q_20*q_20 - 2.0f*(max_acc_ - min_acc_)*q_10) / (max_acc_ * min_acc_);
        }

        h = 0.5f * (-b + sqrtf(b*b - 4*c));
        return h;
    }


    // Determine if node is valid
    virtual bool is_valid(const State& state) {

        /*
            Does this state meet the model constraints?
            i.e Boundary constraints, Obstacles, State limits
        */

       bool tmp;

       if( ((state[X]>= -1.05) && (state[X] <=-0.8)) && (state[V] < 2.0)  ){

            tmp = false;
       }

       else{
        tmp = true;
       }

       // I can add here a constraint so if the vehicle is within
       // the patch, we make sure it doesnt cross the min velocity boundary
       // required to get to the end of patch with positive velocity 
                  
       //bool meets_vel_limits;

       //meets_vel_limits = ( (state[V] >= min_vel_) && (state[V] <= max_vel_) ) ? true : false;

       //return meets_vel_limits;
       return tmp;

    }


     // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {
        return std::abs(goal[X] - state[X]) <= threshold_x_
            && std::abs(goal[V] - state[V]) <= threshold_v_;
    }

   
    virtual ~MomentumModel() {}

    /// @brief Set the goal threshold X value
    /// @param goal_threshold_x Value to set as goal threshold X
    void set_goal_threshold_x(float goal_threshold_x) {
        threshold_x_ = goal_threshold_x;
    }

    /// @brief Set the goal threshold V value
    /// @param goal_threshold_v Value to set as goal threshold V
    void set_goal_threshold_v(float goal_threshold_v) {
        threshold_v_ = goal_threshold_v;
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



    protected:

   float threshold_x_;
   float threshold_v_;
   float min_acc_;
   float max_acc_;
   float min_vel_;
   float max_vel_;
   float mass_; 
   float r_;
   float g_;


};

}

#endif


