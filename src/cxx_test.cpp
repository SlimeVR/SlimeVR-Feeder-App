#include <iostream>
#include "rust_part.h"
#include <chrono>

int cpp_echo(int val)
{
    return val;
}

int test_fun()
{
    int sum = 0;
    for (int i = 0; i < 1000000; i += 1)
    {
        sum += rust_part::rust_echo(i);
    }
    return sum;
}

int test_inline()
{
    int sum = 0;
    for (int i = 0; i < 1000000; i += 1)
    {
        sum += cpp_echo(i);
    }
    return sum;
}

void test_lto()
{
    auto t1 = std::chrono::high_resolution_clock::now();
    auto sum = test_fun();
    auto t2 = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();

    std::cout << "Calling rust function"
              << ", time elapsed: " << duration << " ns." << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    sum = test_inline();
    t2 = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();

    std::cout << "Calling c++ function"
              << ", time elapsed: " << duration << " ns." << std::endl;
}