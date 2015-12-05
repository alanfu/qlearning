//
//  main.cpp
//  qlearning
//
//  Created by FuYongrui on 12/4/15.
//  Copyright © 2015 FuYongrui. All rights reserved.
//

#include <iostream>
#include "qlearner.hpp"
#include <ctime>
using namespace std;

int main(int argc, const char * argv[]) {
    clock_t start = clock();
    double duration;
    
    qlearning::qlearner<int> qq("world02.csv");
    qq.learn();
    //qq.optimal_path();
    
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << duration << " second" << endl;
}
