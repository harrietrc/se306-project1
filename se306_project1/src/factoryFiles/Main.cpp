#include "MyFactory.h"

int main(int argc, char** argv)
{
    auto instanceOne = MyFactory::Instance()->Create("one");
    
    instanceOne->doSomething();
    while(true) {
	}
    return 0;
}