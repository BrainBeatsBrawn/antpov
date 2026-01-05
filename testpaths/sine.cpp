#include <iostream>
#include <cmath>

int main()
{
    float f_st = 0.02f;
    for (int i = 0; i < 2000; ++i) {
        float f = f_st * i;
        std::cout << f << "," << std::sin(f) << std::endl;
    }
}
