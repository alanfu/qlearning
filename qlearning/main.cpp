//
//  main.cpp
//  qlearning
//
//  Created by FuYongrui on 12/4/15.
//  Copyright © 2015 FuYongrui. All rights reserved.
//

#include <iostream>
#include "qlearner.hpp"
using namespace std;

int main(int argc, const char * argv[]) {
    qlearning::qlearner<int> qq("world02.csv");
    qq.learn();
    qq.optimal_path();
}
