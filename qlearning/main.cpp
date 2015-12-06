//
//  main.cpp
//  qlearning
//
//  Created by FuYongrui on 12/4/15.
//  Copyright © 2015 FuYongrui. All rights reserved.
//

#include <iostream>
//#include "qlearner.hpp"
#include "qlearner_simple.hpp"
#include "qlearner_dyna.hpp"
#include <ctime>
#include <memory>
using namespace std;

int main(int argc, const char * argv[]) {
    using qlearner = qlearning::qlearner_simple<int>;
    using qlearner_dyna = qlearning::qlearner_dyna<int>;
    
    clock_t start = clock();
    double duration;
    
    auto ql = make_shared<qlearner>("world02.csv");
    auto ql_dyna = make_shared<qlearner_dyna>("world01.csv");
    //ql->learn();
    ql_dyna->learn();
    //ql->optimal_path();
    
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << duration << " second" << endl;
}
