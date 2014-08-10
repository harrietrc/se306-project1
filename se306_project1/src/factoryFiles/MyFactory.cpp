#include "MyFactory.h"




Registrar::Registrar(string name, function<Agent*(void)> classFactoryFunction)
{
    // register the class factory function
    MyFactory::Instance()->RegisterFactoryFunction(name, classFactoryFunction);
}


MyFactory * MyFactory::Instance()
{
    static MyFactory factory;
    return &factory;
}


void MyFactory::RegisterFactoryFunction(string name, function<Agent*(void)> classFactoryFunction)
{
    // register the class factory function 
    factoryFunctionRegistry[name] = classFactoryFunction;
}


shared_ptr<Agent> MyFactory::Create(string name)
{
    Agent * instance = nullptr;

    // find name in the registry and call factory method.
    auto it = factoryFunctionRegistry.find(name);
    if(it != factoryFunctionRegistry.end())
        instance = it->second();
    
    // wrap instance in a shared ptr and return
    if(instance != nullptr)
        return std::shared_ptr<Agent>(instance);
    else
        return nullptr;
}