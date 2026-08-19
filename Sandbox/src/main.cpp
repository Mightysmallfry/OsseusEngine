#include <iostream>
#include <memory>
#include <vector>

#include <Osseus/Osseus.h>

void RunIdenticalBodiesSimulation()
{
    
    std::cout << "Identical-body simulation complete.\n";
}


void RunUniqueBodiesSimulation()
{

    std::cout << "Unique-body simulation complete.\n";
}


int main()
{
    std::cout << "Running identical-body simulation...\n";
    RunIdenticalBodiesSimulation();

    std::cout << "Running unique-body simulation...\n";
    RunUniqueBodiesSimulation();

    return 0;
}