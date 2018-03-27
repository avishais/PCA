
#include <stdio.h>      /* printf */
#include <iostream>
#include <time.h>
#include <unistd.h>

using namespace std;

#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

int main() {
	// clock_t sT = clock();
	// clock_t sE;
	// for (int i = 0; i < 10; i++) {
	// 	sleep(2);
	// 	sE = clock();
	// 	cout << (i+1)*2 << " " << double(sE-sT) << " " << sE << " " << sT << " " << double(sE-sT) / (double)CLOCKS_PER_SEC  << endl;
	// }
	// cout << double(clock()-sT) << " " << CLOCKS_PER_SEC << " " << double(clock()-sT) / CLOCKS_PER_SEC  << endl;

    auto t1 = Clock::now();
    for (int i = 0; i < 5; i++) {
    sleep(1);
    auto t2 = Clock::now();
    std::cout << (i+1)*2 << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e9 << " seconds" << std::endl;
    }
    auto t2 = Clock::now();
    std::cout << "Delta t2-t1: " 
              << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count()
              << " nanoseconds" << std::endl;
    cout << std::chrono::duration<double>(t2 - t1).count() << endl;
}
