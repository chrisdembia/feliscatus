#include <iostream>

#include <OpenSim/OpenSim.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using OpenSim::Model;

int main()
{
    Model cat = Model();
    cat.setName("feliscatus");

    cat.print("feliscatus.osim");
    return EXIT_SUCCESS;
}
