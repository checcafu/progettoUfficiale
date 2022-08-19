#ifndef BOIDS_TOR_HPP
#define BOIDS_TOR_HPP

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <vector>

//manca il calcolo della distanza
struct Boid {  //definisce un boid, che è caratterizzato dalla posizione e dalla velocità
  double x;
  double y;
  double v_x;
  double v_y;
  double radius;
};

 class Boids { //definisce uno stormo 
  std::vector<Boid> m_boids; //è un vettore che contiene le informazioni di ciascun boid. ogni elemento è costituito dai dati sulle posizioni e sulle velocità di un boid
  std::vector<Boid> m_evolve; //è un vettore che tiene conto dei cambiamenti di posizione e delle velocità sommate nel corso del volo di un boid

  double distance(int const& i, int const& j) {  // calcolo distanza tra boid i-esimo e j-esimo
    return sqrt(pow(m_boids[i].x - m_boids[j].x, 2) +
                pow(m_boids[i].y - m_boids[j].y, 2));
  }

  double signeddistance(int const& i, int const& j, int const& k, int const& l) {  // calcolo distanza tra boid i-esimo e j-esimo, serve per determinare angolo di 'vista'
    return k * (m_boids[j].x - m_boids[i].x) + l * (m_boids[j].y - m_boids[i].y);
  }

  double angle(int const& i, int const& j) {
    return acos((m_boids[i].v_x * signeddistance(i, j, 1, 0) + m_boids[i].v_y * signeddistance(i, j, 0, 1)) / (speed(i) * distance(i, j)));
  }

  double speed(int const& i) {  // calcolo modulo velocità
    return sqrt(pow(m_boids[i].v_x, 2) + pow(m_boids[i].v_y, 2));
  }

  void radius_boid(int h){
    for(int h = 0; h < (int)m_boids.size(); h++){
        m_evolve[h].radius += sqrt(pow(m_boids[h].x,2) + pow(m_boids[h].y,2));
        }
  }

  void separation(int const& i,double const& s) {  // calcolo dell'effetto di separazione sulla velocità
    double sum_x;
    double sum_y;
    for (int j = 0; j < (int)m_boids.size(); j++) {
      if (i != j && distance(i, j) < 50 && angle(i, j) < 1.04) {  // 1.04 rad, circa 60°
        sum_x += signeddistance(j, i, 1, 0);
        sum_y += signeddistance(j, i, 0, 1);
      }
    }
    m_evolve[i].v_x += (-s * sum_x);
    m_evolve[i].v_y += (-s * sum_y);
  }

  void alignment(int const& i,double const& a) {  // calcolo dell'effetto di allineamento sulla velocità
    double sum_v_x;
    double sum_v_y;
    int counter;
    for (int j = 0; j < (int)m_boids.size(); j++) {
      if (i != j && distance(i, j) < 150 && angle(i, j) < 1.04) {
        sum_v_x += m_boids[j].v_x;
        sum_v_y += m_boids[j].v_y;
        counter++;
      }
    }
    if (counter > 0) {
      m_evolve[i].v_x += a * (sum_v_x / counter - m_boids[i].v_x);
      m_evolve[i].v_y += a * (sum_v_y / counter - m_boids[i].v_y);
    }
  }

  void cohesion(int const& i, double const& c) {  // calcolo dell'effetto di coesione sulla velocità
    double center_mass_x;
    double center_mass_y;
    int counter;
    for (int j = 0; j < (int)m_boids.size(); j++) {
      if (i != j && distance(i, j) < 150 && angle(i, j) < 1.04) {
        center_mass_x += m_boids[j].x;
        center_mass_y += m_boids[j].y;
        counter++;
      }
    }
    if (counter > 0) {
      center_mass_x /= counter;
      center_mass_y /= counter;
      m_evolve[i].v_x += c * (center_mass_x - m_boids[i].x);
      m_evolve[i].v_y += c * (center_mass_y - m_boids[i].y);
    }
  }

    public:
    void create_boids(int const& n){
    for (int i=0; i<n; i++){ //ma è utile questo for?
        double radius_ = round(30 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (300))));
        double alpha = round(static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))));
        double x_ = radius_ * cos(alpha);
        double y_ = radius_ * sin(alpha);

        double v_x_ = -10 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (20)));
        double v_y_ = -10 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (20)));
        // creano numeri casuali per coordinate e velocità, così partono in
        // condizioni casuali
        m_boids.push_back({x_, y_, v_x_, v_y_, radius_});
        m_evolve.push_back({0., 0., 0., 0., 0.});
        }
    }

    void evolve(double const& s, double const& a, double const& c){  // calcola valore degli elementi di m_evolve,
    // ovvero di quanto
    // varia la velocità ogni secondo, usando i metodi privati
    for (int i = 0; i < (int)m_boids.size(); i++) {
       separation(i, s);
       alignment(i, a);
       cohesion(i, c);
       radius_boid(i);
    }
  }

  void apply_evolve() {  // applica i cambiamenti di m_evolve in modo da far
                         // avanzare il sistema di 1 secondo
    for (int i = 0; i < (int)m_boids.size(); i++) {
      m_boids[i].v_x += m_evolve[i].v_x;
      m_boids[i].v_y += m_evolve[i].v_y;
      m_boids[i].radius+= m_evolve[i].radius;
      
      if (m_boids[i].radius<40) {  //se boid si trova vicino a bordi si crea velocità che lo respinge
        m_boids[i].v_x *= (m_boids[i].x / 20 - 1);
      } else if (m_boids[i].radius<290) {
        m_boids[i].v_x *= (m_boids[i].x / (-20) + 24);
      }
      if (m_boids[i].radius<40) {
        m_boids[i].v_y *= (m_boids[i].y / 20 - 1);
      } else if (m_boids[i].radius<290) {
        m_boids[i].v_x *= (m_boids[i].y / (-20) + 24);
      }

      if (speed(i) > 10) {
        m_boids[i].v_x = 15 * m_boids[i].v_x / speed(i);
        m_boids[i].v_y = 15 * m_boids[i].v_y / speed(i);
      }
      m_boids[i].x += m_evolve[i].v_x * 1;  // dt=1
      m_boids[i].y += m_evolve[i].v_x * 1;
      m_evolve[i].v_x = 0;
      m_evolve[i].v_y = 0;
      m_evolve[i].radius = 0;
     // annulla ciò che si è fatto in modo da poter eseguire da capo per il secondo successivo
    }
  }
  double meanspeed() {
    double meanspeed;
    for (int i = 0; i < (int)m_boids.size(); i++) {
      meanspeed += speed(i);
    }
    return meanspeed / m_boids.size();
  }
  double stdmeanspeed() {
    double meansquaredspeed;
    double stdspeed;
    for (int i = 0; i < (int)m_boids.size(); i++) {
      meansquaredspeed += pow(speed(i), 2);
    }
    meansquaredspeed /= m_boids.size();
    stdspeed =
        sqrt((meansquaredspeed - pow(meanspeed(), 2)) / (m_boids.size() - 1));
    return stdspeed;
  }
  double meandistance() {
    double meandist;
    for (int i = 0; i < (int)m_boids.size(); i++) {
      for (int j = 0; j < (int)m_boids.size(); j++) {
        meandist += distance(i, j);
      }
    }
    meandist /= ((m_boids.size() - 1) * m_boids.size());
    return meandist;
  }
  double stdmeandistance() {
    double meansquareddist;
    double stddist;
    for (int i = 0; i < (int)m_boids.size(); i++) {
      for (int j = 0; j < (int)m_boids.size(); j++) {
        meansquareddist += pow(distance(i, j), 2);
      }
    }
    if (m_boids.size() > 2) {
      meansquareddist /= ((m_boids.size() - 1) * m_boids.size());
      stddist = sqrt((meansquareddist - pow(meandistance(), 2)) /
                     (((m_boids.size() - 1) * m_boids.size()) / 2 - 1));
      return stddist;
    } else {
      return 0;
    }
  }
  //serve a visualizzare i cambiamenti(non ancora chiamata)
  double maxdistance() {
    double maxdist;
    for (int i = 0; i < (int)m_boids.size(); i++) {
      for (int j = 0; j < (int)m_boids.size(); j++) {
        if (maxdist < distance(i, j)) {
          maxdist = distance(i, j);
        }
      }
    }
    return maxdist;
  } 

 };  
#endif 