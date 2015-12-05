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
#include <tuple>
#include <random>

namespace qlearning
{
    using qlearning::Matrix;
    
    
    template<typename T, typename D = double>
    class qlearner
    {
        enum action {UP = 0, RIGHT, DOWN, LEFT};
        using SizeType = typename Matrix<T>::size_type;
        using Position = typename std::tuple<SizeType,SizeType>;
        
    public:
        qlearner(string file) :
        num_states(100),
        num_actions(4),
        alpha(0.2),
        gamma(0.9),
        rar(0.98),
        radr(0.99)
        {
            origin_map = qlearning::parse<T>(file);
            qtable = initial_qtable();
            
            rng = std::mt19937(rd());
            int_uni = std::uniform_int_distribution<int>(0,num_actions - 1);
            real_uni = std::uniform_real_distribution<double>(0,1);
            //qlearning::print(origin_map);
        }
        
        void learn()
        {
            Position goal_pos = get_goal_pos(origin_map);
            SizeType goal_state = get_state(goal_pos);
            
            int reward;
            
            for(int i = 0; i < 500; i++)
            {
                int steps = 0;
                auto new_map = origin_map;
                Position current_pos = get_current_pos(new_map);
                SizeType current_state = get_state(current_pos);
                SizeType old_state;
                auto act = initial_query();
                
                while(current_state != goal_state)
                {
                    old_state = current_state;
                    current_pos = move_robot(new_map,act);
                    current_state = get_state(current_pos);
                    
                    if(current_state == goal_state)
                        reward = 1;
                    else
                        reward = -1;
                    
                    act = query(act, reward, current_state, old_state);
                    
                    steps += 1;
                }
                cout << "iteration " << i << " : " << steps << endl;
            }
        }
        
        void optimal_path()
        {
            auto new_map = origin_map;
            Position goal_pos = get_goal_pos(origin_map);
            SizeType goal_state = get_state(goal_pos);
            Position current_pos = get_current_pos(new_map);
            SizeType current_state = get_state(current_pos);
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
        
    private:
        inline int initial_query()
        {
            return get_rand();
        }
        
        inline void update_rar()
        {
            rar *= radr;
        }
        
        inline int action_generator(const SizeType current_state)
        {
            auto random_num = real_uni(rng);
            int act;
            
            if(random_num > rar) // optimal action
                act = distance(qtable[current_state].begin(),
                                max_element(qtable[current_state].begin(),
                                            qtable[current_state].end()));
            else
                act = get_rand();
            
            return act;
        }
        
        inline int query(int action,
                         int reward,
                         SizeType current_state,
                         SizeType old_state)
        {
            auto old_value = qtable[old_state][action];
            auto max_q = max_element(qtable[current_state].begin(),
                                     qtable[current_state].end());
            auto new_value = (1 - alpha) * old_value + alpha * (reward + gamma * (*max_q));
            
            qtable[old_state][action] = new_value;
            
            auto act = action_generator(current_state);
            update_rar();
            return act;
        }
        
        inline Position move_robot(Matrix<T>& map,
                                   SizeType act)
        {
            Position current_pos = get_current_pos(map);
            SizeType row, col;
            tie(row,col) = current_pos;
            SizeType old_row = row;
            SizeType old_col = col;
            
            switch (act)
            {
                case action::UP: row += 1; break;
                case action::RIGHT: col += 1; break;
                case action::DOWN: row -= 1; break;
                case action::LEFT: col -= 1; break;
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
        
        inline int get_rand()
        {
            auto random_integer = int_uni(rng);
            return random_integer;
        }
        
        inline SizeType get_state(const Position& pos) const
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
        
        // only used once to initialise (seed) engine
        std::random_device rd;
        // random-number engine used (Mersenne-Twister in this case)
        std::mt19937 rng;
        
        std::uniform_int_distribution<int> int_uni;
        std::uniform_real_distribution<double> real_uni;
        
        double alpha;
        double gamma;
        
        double rar; //random action rate
        double radr; //random action decay rate
        
        SizeType num_states;
        SizeType num_actions;
        Matrix<T> origin_map;
        Matrix<D> qtable;
    };
}


#endif /* qlearner_h */
