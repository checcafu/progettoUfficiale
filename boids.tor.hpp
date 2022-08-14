#ifndef BOIDS_TOR_HPP
#define BOIDS_TOR_HPP
#define _USE_MATH_DEFINES
 
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <vector>
 
struct Boid {
  double x;
  double y;
  double v_x;
  double v_y;
  double alpha; 

  Boid(double x_, double y_, double v_x_, double v_y_, double alpha_){
    x = x_;
    y = y_;
    v_x = v_x_;
    v_y = v_y_;
    alpha = alpha_;
  }
};
 
class Boids {
  std::vector<Boid> m_boids;
  std::vector<Boid> m_evolve;
 
  double distance(int const& i, int const& j, int const& k, int const& l) {  // calcolo distanza tra boid i-esimo e j-esimo
    return sqrt(k * pow(m_boids[i].x - m_boids[j].x, 2) + l * pow(m_boids[i].y - m_boids[j].y, 2));
  }

  double speed(int const& i) {  // calcolo modulo velocità
    return sqrt(pow(m_boids[i].v_x, 2) + pow(m_boids[i].v_y, 2));
  }

double radius_boid(int h){ 
        return sqrt(pow(m_boids[h].x,2) + pow(m_boids[h].y,2));
        }

 void edges(int h){ 
  double x_to_sum;
  double y_to_sum;
   for(h=0; h < (int)m_boids.size(); h++){
    x_to_sum = 270*cos(m_boids[h].alpha);//300-30=270
    y_to_sum = 270*cos(m_boids[h].alpha);
    if(radius_boid(h)<30){
                 m_evolve[h].x += x_to_sum;
                 m_evolve[h].y += y_to_sum;
            }
    if(radius_boid(h)>300){
                m_evolve[h].x -= x_to_sum;
                m_evolve[h].y -= y_to_sum;
             
            }
        
        }
      }  

  void separation(int const& i, double const& s) {  // calcolo dell'effetto di separazione sulla velocità // double s = 0.03;
    double sum_x;
    double sum_y;
    for (int j = 0; j < (int)m_boids.size(); j++) {
      double m = (m_boids[i].y - m_boids[j].y)/(m_boids[i].x - m_boids[j].x); //coefficiente angolare della distanz
      double q = ((m_boids[j].x*m_boids[i].y) - (m_boids[i].y*m_boids[j].x))/(m_boids[j].x- m_boids[i].x); //intercetta della distanza
      double r = -q/sqrt(pow(m,2) + 1);//distanza retta-origine
      if(r>=0){ //valore assoluto
      r = r;
      } else {
      r = -r;
      }
      if (i != j && distance(i, j, 1, 1) < 20 && r>30) {
        sum_x += distance(i, j, 1, 0);
        sum_y += distance(i, j, 0, 1);
      }
    }
    m_evolve[i].v_x += (-s * sum_x);
    m_evolve[i].v_y += (-s * sum_y);
  }
  void alignment(int const& i, double const& a) {  // calcolo dell'effetto di allineamento sulla velocità
    // double a = 0.06;
    double sum_v_x;
    double sum_v_y;
    int counter;
    for (int j = 0; j < (int)m_boids.size(); j++) {
      double m = (m_boids[i].y - m_boids[j].y)/(m_boids[i].x - m_boids[j].x); //coefficiente angolare della distanz
      double q = ((m_boids[j].x*m_boids[i].y) - (m_boids[i].y*m_boids[j].x))/(m_boids[j].x- m_boids[i].x); //intercetta della distanza
      double r = -q/sqrt(pow(m,2) + 1);//distanza retta-origine
      if(r>=0){ //valore assoluto
      r = r;
      } else {
      r = -r;
      }
      if (i != j && distance(i, j, 1, 1) < 50 && r>30) {
        sum_v_x += m_boids[j].v_x;
        sum_v_y += m_boids[j].v_y;
        counter++;
      }
    }
    m_evolve[i].v_x += a * (sum_v_x / (counter - 1) - m_boids[i].v_x);
    m_evolve[i].v_y += a * (sum_v_y / (counter - 1) - m_boids[i].v_y);
  }
  void cohesion(int const& i, double const& c) {  // calcolo dell'effetto di coesione sulla velocità
    // double c = 0.08;
    double center_mass_x;
    double center_mass_y;
    int counter;
    for (int j = 0; j < (int)m_boids.size(); j++) {
      double m = (m_boids[i].y - m_boids[j].y)/(m_boids[i].x - m_boids[j].x); //coefficiente angolare della distanz
      double q = ((m_boids[j].x*m_boids[i].y) - (m_boids[i].y*m_boids[j].x))/(m_boids[j].x- m_boids[i].x); //intercetta della distanza
      double r = -q/sqrt(pow(m,2) + 1);//distanza retta-origine
      if(r>=0){ //valore assoluto
      r = r;
      } else {
      r = -r;
      }
      if (distance(i, j, 1, 1) < 50 && r>30) {
        center_mass_x += m_boids[j].x;
        center_mass_y += m_boids[j].y;
        counter++;
      }
      center_mass_x /= (counter);
      center_mass_y /= (counter);
      m_evolve[i].v_x += c * (center_mass_x - m_boids[i].x);
      m_evolve[i].v_y += c * (center_mass_y - m_boids[i].y);
    }
  }
 
 public:

  void create_boids(int const& n){
    for (int i=0; i<n; i++){
        double radius_i = round(30 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (300))));
        std::cout << "radius_i settato"<< '\n';
        double alpha_ = round(static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))));
        std::cout << "alpha settato"<< '\n';
        double x_ = radius_i * cos(alpha_);
        std::cout << "x settato"<< '\n';
        double y_ = radius_i * sin(alpha_);
        std::cout << "y settato"<< '\n';

        double v_x_ = round(-15 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (15))));
        std::cout << "v_x settato"<< '\n';
        double v_y_ = round(-15 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (15))));
        std::cout << "v_y settato"<< '\n';

        m_boids.push_back(Boid(x_, y_, v_x_, v_y_, alpha_));
        m_evolve.push_back(Boid(0., 0., 0., 0., 0.));
        std::cout << "fine ciclo numero" << i <<'\n';
    }
  }
  void evolve(double const& s, double const& a,double const& c) {  // calcola valore degli elementi di m_evolve,
    // ovvero di quanto
    // varia la velocità ogni secondo, usando i metodi privati
    for (int i = 0; i < (int)m_boids.size(); i++) {
      separation(i, s);
      alignment(i, a);
      cohesion(i, c);
      edges(i);
 
    }
  }
  void apply_evolve() {  // applica i cambiamenti di m_evolve in modo da far
                         // avanzare il sistema di 1 secondo
    for (int i = 0; i < (int)m_boids.size(); i++) {
      m_boids[i].v_x += m_evolve[i].v_x;
      m_boids[i].v_y += m_evolve[i].v_y;
      m_boids[i].x += m_evolve[i].v_x * 1;  // dt=1
      m_boids[i].y += m_evolve[i].v_x * 1;
      m_boids[i].alpha += atan(m_boids[i].y/m_boids[i].x);
      m_evolve[i].v_x = 0;
      m_evolve[i].v_y = 0;  // annulla ciò che si è fatto in modo da poter
                            // eseguire da capo per il secondo successivo
    }
  }
  double meanspeed() {
    double meanspeed = 0;
    for (int i = 0; i < (int)m_boids.size(); i++) {
      meanspeed += speed(i);
    }
    return meanspeed / m_boids.size();
  }
};
 
#endif
