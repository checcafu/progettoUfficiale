#include "boid.hpp"

#include <iostream>
#include <stdexcept>
int main() {
  std::cout << "Insert the number of boids (a whole number between 5 and 500)"
            << '\n';
  try {
    int n;
    std::cin >> n;
    if (n < 5 || n > 500) {
      throw std::runtime_error{"The number of boids inserted is not valid"};
    }
    double s;
    std::cout
        << "Insert the 'separation' parameter s (a number between 0. and 1.)"
        << '\n';
    std::cin >> s;
    if (0 > s || s > 1) {
      throw std::runtime_error{"The s value inserted is not valid"};
    }
    double a;
    std::cout
        << "Insert the 'alignment' parameter a (a number between 0. and 1.)"
        << '\n';
    std::cin >> a;
    if (0 > a || a > 1) {
      throw std::runtime_error{"The a value inserted is not valid"};
    }
    double c;
    std::cout
        << "Insert the 'cohesion' parameter c (a number between 0. and 1.)"
        << '\n';
    std::cin >> c;
    if (0 > c || c > 1) {
      throw std::runtime_error{"The c value inserted is not valid"};
    }
    Boids simulation;
    simulation.create_boids(n);
    for (int t = 0; t < 181; t++) {
      if (t % 10 == 0) {
        std::cout << '\n'
                  << "Time:" << trunc(t / 60) << " "
                  << "min"
                  << " " << t - trunc(t / 60) * 60 << "s" << '\n'
                  << "Mean speed:" << simulation.meanspeed().meanvalue << "+/-"
                  << simulation.meanspeed().stdmeanvalue << '\n'
                  << "Mean distance:" << simulation.meandistance().meanvalue
                  << "+/-" << simulation.meandistance().stdmeanvalue << '\n'
                  << "Max distance:" << simulation.maxdistance() << '\n';
      }
      simulation.evolve(s, a, c);
      simulation.apply_evolve();
    }
  } catch (std::runtime_error const& e) {
    std::cerr << e.what() << '\n';
  }
}
