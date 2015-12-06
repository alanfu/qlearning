//
//  qlearner_simple.hpp
//  qlearning
//
//  Created by FuYongrui on 12/5/15.
//  Copyright © 2015 FuYongrui. All rights reserved.
//

#ifndef qlearner_simple_h
#define qlearner_simple_h

#include "qlearner.hpp"
#include <tuple>
using namespace std;

namespace qlearning
{
    using qlearning::Matrix;
    
    template<typename T, typename D = double, typename R = double>
    class qlearner_simple : public qlearner<T>// qlearner implementation without Dyna (for map path)
    {
    public:
        using qlearner = typename qlearning::qlearner<T>;
        using SizeType = typename Matrix<T>::size_type;
        using Position = typename std::tuple<SizeType,SizeType>;
        using typename qlearner::StateType;
        using typename qlearner::ActionType;
        using typename qlearner::RewardType;
        
        qlearner_simple(string file) : qlearner()
        {
            origin_map = qlearning::parse<T>(file);
        }
        
        virtual ~qlearner_simple() {}
        
        void optimal_path()
        {
            auto new_map = origin_map;
            Position goal_pos = get_goal_pos(origin_map);
            StateType goal_state = get_state(goal_pos);
            Position current_pos = get_current_pos(new_map);
            StateType current_state = get_state(current_pos);
            int steps = 0;
            
            while(current_state != goal_state)
            {
                auto act = distance(qtable[current_state].begin(),
                                    max_element(qtable[current_state].begin(),
                                                qtable[current_state].end()));
                current_pos = move_robot(new_map, act);
                steps += 1;
                
                current_state = get_state(current_pos);
            }
            cout << "steps: " << steps << endl;
            //qlearning::print(new_map);
        }
        
    protected:
        Matrix<T> origin_map;
        
        using qlearner::qtable;
        
        using qlearner::rd;
        using qlearner::rng;
        
        using qlearner::int_uni;
        using qlearner::real_uni;
        
        using qlearner::alpha;
        using qlearner::gamma;
        
        using qlearner::rar;
        using qlearner::radr;
        
        using qlearner::num_states;
        using qlearner::num_actions;
        
        using qlearner::initial_query;
        using qlearner::update_rar;
        using qlearner::action_generator;
        using qlearner::update_qtable;
        
        virtual void do_learn() override
        {
            Position goal_pos = get_goal_pos(origin_map);
            StateType goal_state = get_state(goal_pos);
            
            RewardType reward;
            
            for(int i = 0; i < 500; i++)
            {
                int steps = 0;
                auto new_map = origin_map;
                Position current_pos = get_current_pos(new_map);
                StateType current_state = get_state(current_pos);
                StateType old_state;
                auto act = initial_query();
                
                while(current_state != goal_state)
                {
                    old_state = current_state;
                    current_pos = move_robot(new_map,act);
                    current_state = get_state(current_pos);
                    
                    if(current_state == goal_state)
                        reward = 1.0;
                    else
                        reward = -1.0;
                    
                    act = query(act, reward, current_state, old_state);
                    
                    steps += 1;
                }
                cout << "iteration " << i << " : " << steps << endl;
            }
        }
        
        inline virtual ActionType query(ActionType action,
                                        RewardType reward,
                                        StateType current_state,
                                        StateType old_state) override
        {
            return simple_query(action, reward, current_state, old_state);
        }
        
    private:
        
        inline ActionType simple_query(ActionType action,
                                       RewardType reward,
                                       StateType current_state,
                                       StateType old_state)
        {
            update_qtable(action,
                          reward,
                          current_state,
                          old_state);
            
            auto act = action_generator(current_state);
            update_rar();
            return act;
        }
        
        inline Position move_robot(Matrix<T>& map,
                                   ActionType act)
        {
            Position current_pos = get_current_pos(map);
            SizeType row, col;
            tie(row,col) = current_pos;
            SizeType old_row = row;
            SizeType old_col = col;
            
            switch (act)
            {
                case qlearner::action::UP: row += 1; break;
                case qlearner::action::RIGHT: col += 1; break;
                case qlearner::action::DOWN: row -= 1; break;
                case qlearner::action::LEFT: col -= 1; break;
            }
            
            if(row < 0 || row >= map.size() ||
               col < 0 || col >= map[0].size() ||
               map[row][col] == 1)
                return current_pos;
            else
            {
                map[row][col] = 2;
                map[old_row][old_col] = 8;
                current_pos = std::make_tuple(row,col);
            }
            return current_pos;
        }
        
        inline StateType get_state(const Position& pos) const
        {
            return get<0>(pos) * 10 + get<1>(pos);
        }
        
        inline Position get_current_pos(const Matrix<T>& mat)
        {
            Position pos;
            
            for(SizeType row = 0; row != mat.size(); row++)
                for(SizeType col = 0; col != mat[0].size(); col++)
                    if(mat[row][col] == 2)
                    {
                        pos = make_tuple(row,col);
                        return pos;
                    }
            cout << "no current position found!" << endl;
            return pos;
        }
        
        inline Position get_goal_pos(const Matrix<T>& mat)
        {
            Position pos;
            
            for(SizeType row = 0; row != mat.size(); row++)
                for(SizeType col = 0; col != mat[0].size(); col++)
                    if(mat[row][col] == 3)
                    {
                        pos = make_tuple(row,col);
                        return pos;
                    }
            cout << "no goal position found!" << endl;
            return pos;
        }
        
    };// class qlearner_simple
    
}// namespace qlearning

#endif /* qlearner_simple_h */
