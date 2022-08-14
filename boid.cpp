#include "boids.tor.hpp"
 
#include <iostream>
#include <stdexcept>
 
int main() {
  srand(static_cast<unsigned>(time(0)));
  std::cout << "Insert the number of boids (a whole number between 2 and 100)"
            << '\n';
  try{
  int n;
  std::cin >> n;  // inserimento in input del numero di boid
  if(n<2 || n>100){
    throw std::runtime_error{"The number of boids inserted is not valid"};
  }
  Boids simulation;
  std::cout << "sto creando i boids"<< '\n';
  simulation.create_boids(n);
  std::cout << "boids creati"<< '\n';
  double s;
  double a;
  double c;
  
  std::cout << "Insert the 'separation' parameter s, a number between 0. and 1."
            << '\n';
  std::cin >> s;
  if(0>s || s>1){ 
    throw std::runtime_error{"The s value inserted is not valid"};
  }
  std::cout << "Insert the 'alignment' parameter a, a number between 0. and 1."
            << '\n';
  std::cin >> a;
   if(0>a || a>1){ 
    throw std::runtime_error{"The a value inserted is not valid"};
  }
  std::cout << "Insert the 'cohesion' parameter c, a number between 0. and 1."
            << '\n';
  std::cin >> c;
   if(0>c || c>1){ 
    throw std::runtime_error{"The c value inserted is not valid"};
  }
  for (int t = 0; t < 10; t++)  // ciclo for che simula tempo (qui va da 0s a
                                // 10s, è provvisorio come valore per adesso)
  {
    std::cout << "Velocità media dopo "<<t<< " secondi: " << simulation.meanspeed() << '\n';
    simulation.evolve(s, a, c);  // chiama metodo che calcola l'evoluzione
    simulation.apply_evolve();   // chiama metodo che applica l'evoluzione
  }
 } catch (std::runtime_error const& e){
        std::cerr<<e.what()<<'\n';
    }  
}
