#ifndef CPPFACTORY_WhiteAgent_H
#define CPPFACTORY_WhiteAgent_H

#include "Agent.h"
#include <iostream>
using namespace std;

class WhiteAgent : public Agent
{
public:
    WhiteAgent(){};
    virtual ~WhiteAgent(){};

    virtual void doSomething() { cout << "I am the white one" << endl; }
};
#endif // CPPFACTORY_DERIVEDCLASSONE_H