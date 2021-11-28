// Kids_Simulator.cpp
//

#include <iostream>




int main()
try
{
    
}

catch (std::exception& e) {
    std::cerr << "exception: " << e.what() << std::endl;
    return 1;
}
catch (...) {
    std::cerr << "exception\n";
    return 2;

}
