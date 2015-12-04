//
//  parse.hpp
//  qlearning
//
//  Created by FuYongrui on 12/4/15.
//  Copyright © 2015 FuYongrui. All rights reserved.
//

#ifndef parse_h
#define parse_h

#include <iostream>     // cout, endl
#include <fstream>      // fstream
#include <vector>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

namespace qlearning
{
    template<typename T>
    using Matrix = typename std::vector<std::vector<T>>;
    
    template<typename T>
    Matrix<T> parse(string file)
    {
        typedef boost::tokenizer<boost::char_separator<char>> Tokenizer;
        boost::char_separator<char> sep(",");
        vector<T> vec;
        Matrix<T> mat;
        
        ifstream in(file.c_str());
        
        string line;
        
        while (getline(in,line))
        {
            Tokenizer tokens(line, sep);
            
            vec.clear();
            BOOST_FOREACH(string t, tokens)
                vec.push_back(boost::lexical_cast<int>(t));
            
            mat.push_back(vec);
        }
        
        return mat;
    }
    
    template<typename T>
    void print(const Matrix<T>& mat)
    {
        using SizeType = typename Matrix<T>::size_type;
        
        for(SizeType row = 0; row != mat.size(); row++)
        {
            for(SizeType col = 0; col != mat[0].size(); col++)
            {
                cout << mat[row][col] << " ";
            }
            cout << endl;
        }
    }
    
}// namespace qlearning

#endif /* parse_h */
