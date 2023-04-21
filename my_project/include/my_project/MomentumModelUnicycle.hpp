#ifndef SBMPO_MOMENTUM_MODEL_UNICYCLE_HPP
#define SBMPO_MOMENTUM_MODEL_UNICYCLE_HPP

#include <sbmpo/model.hpp>
#include<iostream>

namespace my_namespace{

using namespace sbmpo;

class MomentumModelUnicycle : public Model {

    public:

    // enum States {X,V};  
    enum States {X,Y,th,dX,dY,dth,V}; 

    // sample accel
    //enum Controls {A}; // Forward acceleration
 
    // sample tractive force and torque
    enum Controls {Fx,Tau}; //
    
    MomentumModelUnicycle(){

       // threshold_X_ = 0.01f;
       // threshold_Y_ = 0.01f;

        threshold_X_ = 1.0f;
        threshold_Y_ = 1.0f;

        threshold_V_ = 1.0f;
              
        max_acc_ = 2.0f;
        min_acc_ = -2.0f;

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

        float accX, accY; // acceleration in global coordinates
        float accx, accy; // acceleration in local coordinates

        float dvx,d2th;
          
        // read the terrain parameters
        float patch_start = -1.0; 
        float patch_end = 0.0;
        float b = 0;
        float bz = 0;


        // based on the map and robot location. Use only meaningful samples
        // could use a function to choose samples meaningful to each terrain. use a function with indices
        // the indices will be dependent on terrain info

        float tractive_force, steering_torque;
        tractive_force = control[Fx];
        steering_torque = control[Tau];

        dvx = (tractive_force - b*state[V])/mass_;
        d2th = (steering_torque - bz*state[dth])/I_;
      
        accx = dvx;
        accy = state[dth]*state[V]; // assume no side slip (vy = 0)

        float heading = state[th];
        accX = accx*cos(heading) - accy*sin(heading);
        accY = accx*sin(heading) + accy*cos(heading);

        next_state[X] += state[dX]*time_span + 0.5*accX*time_span*time_span; 
        next_state[Y] += state[dY]*time_span + 0.5*accY*time_span*time_span; 
        next_state[th] += state[dth]*time_span + 0.5*d2th*time_span*time_span;      
        next_state[th] = atan2(sin(next_state[th]),cos(next_state[th])); 

        next_state[dX] += accX*time_span;
        next_state[dY] += accY*time_span;
        next_state[dth] += d2th*time_span;
    
        next_state[V] += dvx*time_span;


        //std::cout<<"X:"<<next_state[X]<<std::endl;                    
       
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
        float h;

        float q_10;
        float q_20;

        // approximate the heuristic using minimum time to the farther
        // X or Y

        float distX, distY;
        distX = std::abs(state[X] - goal[X]);
        distY = std::abs(state[Y] - goal[Y]);


        if(distX > distY){
            q_10 = state[X] - goal[X];
            q_20 = state[V] - goal[V];
        }
        else{
            q_10 = state[Y] - goal[Y];
            q_20 = state[V] - goal[V];
        }

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

      /*
        if( (state[X]>= -5.0) && (state[X] <= 5.0) && (state[Y]>= -5.0) && (state[Y] <= 5.0)  ){

            return true;
        }

        else{
            
            return false;
        }
*/
    return true;
 

    }


     // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {
        return std::abs(goal[X] - state[X]) <= threshold_X_
            && std::abs(goal[Y] - state[Y]) <= threshold_Y_
            && std::abs(goal[V] - state[V]) <= threshold_V_; // maybe dont need the zero vel
    }

   
    virtual ~MomentumModelUnicycle() {}

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


