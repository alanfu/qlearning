//
//  qlearner.hpp
//  qlearning
//
//  Created by FuYongrui on 12/4/15.
//  Copyright © 2015 FuYongrui. All rights reserved.
//

#ifndef qlearner_h
#define qlearner_h

#include "parse.hpp"
#include <random>

namespace qlearning
{
    using qlearning::Matrix;
    
    template<typename T, typename D = double, typename R = double>
    class qlearner// abstract class for qlearner
    {
        
    public:
        
        using SizeType = typename Matrix<T>::size_type;
        using RewardType = R;
        using ActionType = unsigned int;
        using StateType = unsigned int;
        
        qlearner() :
        num_states(100),
        num_actions(4),
        alpha(0.2),
        gamma(0.9),
        rar(0.5),
        radr(0.99)
        {
            qtable = initial_qtable();
            rng = std::mt19937(rd());
            int_uni = std::uniform_int_distribution<ActionType>(0, num_actions - 1);
            real_uni = std::uniform_real_distribution<double>(0, 1);
            //qlearning::print(origin_map);
        }
        
        void learn()
        {
            do_learn();
        }
        
        virtual ~qlearner() {}
        
    protected:
        
        enum action {UP = 0, RIGHT, DOWN, LEFT};
        std::random_device rd;// only used once to initialise (seed) engine
        std::mt19937 rng;// random-number engine used (Mersenne-Twister in this case)
        
        std::uniform_int_distribution<ActionType> int_uni;
        std::uniform_real_distribution<double> real_uni;
        
        double alpha;
        double gamma;
        
        double rar; //random action rate
        double radr; //random action decay rate
        
        StateType num_states;
        ActionType num_actions;
        Matrix<D> qtable;
        
        inline ActionType get_rand()
        {
            ActionType random_integer = int_uni(rng);
            return random_integer;
        }
        
        inline ActionType action_generator(const StateType current_state)
        {
            auto random_num = real_uni(rng);
            ActionType act;
            
            if(random_num > rar) // optimal action
                act = distance(qtable[current_state].begin(),
                               max_element(qtable[current_state].begin(),
                                           qtable[current_state].end()));
            else
                act = get_rand();
            
            return act;
        }
        
        inline void update_rar()
        {
            rar *= radr;
        }
        
        inline ActionType initial_query()
        {
            return get_rand();
        }
        
        inline void update_qtable(ActionType action,
                                  RewardType reward,
                                  StateType current_state,
                                  StateType old_state)
        {
            auto old_value = qtable[old_state][action];
            auto max_q = max_element(qtable[current_state].begin(),
                                     qtable[current_state].end());
            auto new_value = (1 - alpha) * old_value + alpha * (reward + gamma * (*max_q));
            
            qtable[old_state][action] = new_value;
        }
        
        Matrix<D> initial_qtable()
        {
            Matrix<D> mat;
            vector<D> vec;
            for(SizeType i = 0; i != num_actions; i++)
                vec.push_back(0);
            for(SizeType j = 0; j != num_states; j++)
                mat.push_back(vec);
            return mat;
        }
        
    private:
        
        virtual void do_learn() = 0;
        
        inline virtual ActionType query(ActionType, RewardType, StateType, StateType) = 0;

    };// class qlearner
    
}// namaspace qlearning


#endif /* qlearner_h */
