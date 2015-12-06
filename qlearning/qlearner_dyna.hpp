//
//  qlearner_dyna.hpp
//  qlearning
//
//  Created by FuYongrui on 12/5/15.
//  Copyright © 2015 FuYongrui. All rights reserved.
//

#ifndef qlearner_dyna_h
#define qlearner_dyna_h

#include <limits>
#include <boost/range/numeric.hpp>
#include <numeric>
#include "qlearner_simple.hpp"

using namespace std;

namespace qlearning
{
    using qlearning::Matrix;
    
    template<typename D = double>
    using ThreeD_Vec = vector<vector<vector<D>>>;
    
    //template<typename D = double>
    /*
    void print(vector<D>& vec)
    {
        for(int i = 0; i != vec.size(); i++)
            cout << vec[i] << " ";
        cout << endl;
    }
    */
    template<typename T, typename D = double, typename R = double>
    class qlearner_dyna : public qlearner_simple<T>
    {
        using qlearner = qlearning::qlearner_simple<T>;
        using typename qlearner::SizeType;
        using typename qlearner::StateType;
        using typename qlearner::ActionType;
        using typename qlearner::RewardType;
        
        
    public:
        qlearner_dyna(string file) : qlearner(file), dyna(100)
        {
            r = qlearning::qlearner_simple<T>::initial_qtable();
            t = init_threed_vec(1.0 / num_states);
            tc = init_threed_vec(0.00001);
            int_uni_states = std::uniform_int_distribution<int>(0,num_states - 1);
            real_uni_zero_one = std::uniform_real_distribution<double>(0,1);
        }
        virtual ~qlearner_dyna() {}
        
    protected:
        std::uniform_int_distribution<int> int_uni_states;
        std::uniform_real_distribution<double> real_uni_zero_one;
        
        using qlearner::update_rar;
        using qlearner::update_qtable;
        using qlearner::action_generator;
        using qlearning::qlearner<T>::num_states;
        using qlearning::qlearner<T>::num_actions;
        using qlearning::qlearner<T>::alpha;
        using qlearning::qlearner<T>::get_rand;
        using qlearning::qlearner<T>::rng;
        
        inline virtual ActionType query(ActionType action,
                                        RewardType reward,
                                        StateType current_state,
                                        StateType old_state) override
        {
            return dyna_query(action, reward, current_state, old_state);
        }
        
        unsigned int dyna;
        Matrix<D> r;
        ThreeD_Vec<D> t, tc;
        
    private:
        inline ActionType dyna_query(ActionType action,
                                     RewardType reward,
                                     StateType current_state,
                                     StateType old_state)
        {
            update_qtable(action, reward, current_state, old_state);
            
            do_dyna(action, reward, current_state, old_state);
            
            auto act = action_generator(current_state);
            update_rar();
            return act;
        }
        
        inline void do_dyna(ActionType action,
                            RewardType reward,
                            StateType current_state,
                            StateType old_state)
        {
            tc[old_state][action][current_state] += 1.0;
            
            for(auto i = 0; i != num_states; i++)
                t[old_state][action][i] = tc[old_state][action][i] / boost::accumulate(tc[old_state][action], 0.0);

            r[old_state][action] = (1 - alpha) * r[old_state][action] + alpha * reward;
            
            ActionType act;
            StateType state, state_prime;
            RewardType re;
            vector<D> cumsum(t[0][0].size());
            
            for(auto i = 0; i != dyna; i++)
            {
                act = get_rand();
                state = int_uni_states(rng);
                std::partial_sum(t[state][act].begin(), t[state][act].end(), cumsum.begin()); // cumsum
                auto temp_rand = real_uni_zero_one(rng);
                
                for(auto i = 0; i != cumsum.size(); i++)
                {
                    if(temp_rand < cumsum[i])
                    {
                        state_prime = i;
                        break;
                    }
                }
                re = r[state][act];
                update_qtable(act, re, state_prime, state);
            }
        }
        
        ThreeD_Vec<D> init_threed_vec(const D& init_val)
        {
            ThreeD_Vec<D> threed_vec;
            Matrix<D> mat;
            vector<D> vec;
            
            for(auto i = 0; i != num_states; i++)
                vec.push_back(init_val);
            for(auto j = 0; j != num_actions; j++)
                mat.push_back(vec);
            for(auto k = 0; k != num_states; k++)
                threed_vec.push_back(mat);
            
            return threed_vec;
        }
    };// class qlearner_dyna

}// namespace qlearning

#endif /* qlearner_dyna_h */
