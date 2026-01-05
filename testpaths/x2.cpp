#include <iostream>

int main()
{
    float f_st = 0.01f;
    for (int i = 0; i < 200; ++i) {
        float f = f_st * i;
        std::cout << f << "," << f * f << std::endl;
    }
}
