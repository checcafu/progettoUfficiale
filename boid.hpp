#ifndef BOID_HPP
#define BOID_HPP

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

struct Boid { 
  double x; //posizione del boid sull'asse x
  double y; //posizione del boid sull'asse y
  double z; //posizione del boid sull'asse z
  double v_x; //componente x della velocità propria del boid
  double v_y; //componente y della velocità propria del boid
  double v_z; //componente z della velocità propria del boid
};

struct Meanvalue { //output:
  double meanvalue; //media delle velocità dei boids
  double stdmeanvalue; //deviazione standard della media delle velocità dei boids
};

class Boids { //stormo
  std::vector<Boid> m_boids; //vettore di cui ogni elemento contiene i dati 
                             //sulla posizione e sulle velocità di ciascun boid
  std::vector<Boid> m_evolve; //vettore di cui ogni elemento contiene i dati
                              //sull'evoluzione nel tempo della posizione della velocità di ciascun boid

  double radius(int const& i) { 
    return sqrt(pow(m_boids[i].x, 2) + pow(m_boids[i].y, 2) + pow(m_boids[i].z, 2));
  } 
  //distanza del boid i-esimo dall'origine degli assi

  double distance(int const& i, int const& j) { 
    return sqrt(pow(m_boids[i].x - m_boids[j].x, 2) +
                pow(m_boids[i].y - m_boids[j].y, 2) +
                pow(m_boids[i].z - m_boids[j].z, 2));
  } 
  //distanza tra il boid i-esimo e il boid j-esimo

  double signeddistance(int const& i, int const& j, int const& k, int const& l, int const& m) {
    return k * (m_boids[j].x - m_boids[i].x) +
           l * (m_boids[j].y - m_boids[i].y) +
           m * (m_boids[j].z - m_boids[i].z);
  } 
  //metodo utile a calcolare la componente della distanza d'interesse:
  //ponendo a turno k, l, m uguali a 1 e le altre costanti a 0, si considera solo una delle componenti

  double angle(int const& i, int const& j) {
    return acos((m_boids[i].v_x * signeddistance(i, j, 1, 0, 0) +
                 m_boids[i].v_y * signeddistance(i, j, 0, 1, 0) +
                 m_boids[i].v_y * signeddistance(i, j, 0, 0, 1)) /
                (speed(i) * distance(i, j)));
  } 
  //angolo solido(?) formato tra il boid i-esimo e il boid j-esimo rispetto alla normale del boid i-esimo

  double speed(int const& i) {
    return sqrt(pow(m_boids[i].v_x, 2) + pow(m_boids[i].v_y, 2) + pow(m_boids[i].v_z, 2));
  } 
  //calcolo della velocità risultante

  int size() { 
    return m_boids.size(); 
    } 
  //numero di elementi del vettore m-boid
  //che equivale al numero di boids presenti nello stormo meno uno
  //(poichè il conteggio del metodo "size" parte da 0)

  void separation(int const& i, double const& s) { //parametri: numero del boid scelto(i), costante di separazione
    double sum_x; 
    double sum_y;
    double sum_z;
    for (int j = 0; j < size(); j++) { //per ognuno dei restanti boids
      if (i != j && distance(i, j) < 20 && angle(i, j) < 2.618) { //se la distanza tra il boid i-esimo e j-esimo è minore di 20m 
                                                                  //e l'angolo formato tra i due è minore di 150°
        sum_x += signeddistance(j, i, 1, 0, 0); //somma delle componenti x delle distanze tra il boid i-esimo e gli altri boid
        sum_y += signeddistance(j, i, 0, 1, 0); //somma delle componenti y delle distanze tra il boid i-esimo e gli altri boid
        sum_z += signeddistance(j, i, 0, 0, 1); //somma delle componenti z delle distanze tra il boid i-esimo e gli altri boid
      }
    }
    m_evolve[i].v_x += (-s * sum_x); //aggiungere una velocità opposta alla traiettoria che porta al boid j-esimo sull'asse x
    m_evolve[i].v_y += (-s * sum_y); //aggiungere una velocità opposta alla traiettoria che porta al boid j-esimo sull'asse x
    m_evolve[i].v_z += (-s * sum_z); //aggiungere una velocità opposta alla traiettoria che porta al boid j-esimo sull'asse z
  }
  //calcolo della velocità da sommare al fine di allonanare
  //i boid gli uni dagli altri se la distanza è al di sotto di 20m

  void alignment(int const& i, double const& a) { //parametri: numero del boid scelto(i), costante di allineamento
    double sum_v_x;
    double sum_v_y;
    double sum_v_z;
    int counter;
    for (int j = 0; j < size(); j++) {
      if (i != j && distance(i, j) < 150 && angle(i, j) < 2.618) {
        sum_v_x += m_boids[j].v_x; //somma delle componenti x delle velocità degli altri boid
        sum_v_y += m_boids[j].v_y; //somma delle componenti y delle velocità degli altri boid
        sum_v_z += m_boids[j].v_z; //somma delle componenti z delle velocità degli altri boid
        counter++; //conteggio del numero di boid 
      }
    }
    if (counter > 0) {
      m_evolve[i].v_x += a * (sum_v_x / counter - m_boids[i].v_x); //aggiunge una velocità nella direzione dello stormo sull'asse x
      m_evolve[i].v_y += a * (sum_v_y / counter - m_boids[i].v_y); //aggiunge una velocità nella direzione dello stormo sull'asse y
      m_evolve[i].v_z += a * (sum_v_z / counter - m_boids[i].v_z); //aggiunge una velocità nella direzione dello stormo sull'asse z
    }
  }
  //calcolo celle velocità da sommare al fine di omologare le traiettorie di ciuscun boid a quella dello stormo

  void cohesion(int const& i, double const& c) {
    double center_mass_x; //posizione del centro di massa dello stormo sull'asse x
    double center_mass_y; //posizione del centro di massa dello stormo sull'asse y
    double center_mass_z; //posizione del centro di massa dello stormo sull'asse z
    int counter;
    for (int j = 0; j < size(); j++) {
      if (i != j && distance(i, j) < 150 && angle(i, j) < 2.618) {
        center_mass_x += m_boids[j].x; //somma delle posizioni sull'asse x di tutti i boid
        center_mass_y += m_boids[j].y; //somma delle posizioni sull'asse y di tutti i boid
        center_mass_z += m_boids[j].z; //somma delle posizioni sull'asse z di tutti i boid
        counter++; //conteggio del numero di boid 
      }
    }
    if (counter > 0) {
      center_mass_x /= counter; //calcolo della posizione del centro di passa sull'asse x
      center_mass_y /= counter; //calcolo della posizione del centro di passa sull'asse y
      center_mass_z /= counter; //calcolo della posizione del centro di passa sull'asse z
      m_evolve[i].v_x += c * (center_mass_x - m_boids[i].x); //aggiunge una velocità nella direzione del centro di massa dello stormo sull'asse x
      m_evolve[i].v_y += c * (center_mass_y - m_boids[i].y); //aggiunge una velocità nella direzione del centro di massa dello stormo sull'asse y
      m_evolve[i].v_z += c * (center_mass_z - m_boids[i].z); //aggiunge una velocità nella direzione del centro di massa dello stormo sull'asse z
    }
  }
  //calcolo della velocità da sommare al fine di direzionare la traiettoria del boid 
  //verso il centro di massa dello stormo se il boid tende ad allontanarsi

 public:

  void create_boids(int const& n) { 
    srand(static_cast<unsigned>(time(0))); //??
    int i = 0;
    while (i < n) {
      double x = -200 + round(static_cast<float>(rand()) / 
                              (static_cast<float>(RAND_MAX / (400)))); //generazione pseudo-casuale della coordinata x del boid i-esimo
      double y = -200 + round(static_cast<float>(rand()) /
                              (static_cast<float>(RAND_MAX / (400)))); //generazione pseudo-casuale della coordinata y del boid i-esimo
      double z = -200 + round(static_cast<float>(rand()) /
                              (static_cast<float>(RAND_MAX / (400)))); //generazione pseudo-casuale della coordinata z del boid i-esimo
      double v_x = -10 + static_cast<float>(rand()) /
                             (static_cast<float>(RAND_MAX / (20))); //generazio pseudo.casuale della componente x della velocità del boid i-esimo
      double v_y = -10 + static_cast<float>(rand()) /
                             (static_cast<float>(RAND_MAX / (20))); //generazio pseudo.casuale della componente y della velocità del boid i-esimo
      double v_z = -10 + static_cast<float>(rand()) /
                             (static_cast<float>(RAND_MAX / (20))); //generazio pseudo.casuale della componente z della velocità del boid i-esimo
      if (pow(x, 2) + pow(y, 2) + pow(z, 2) < pow(200, 2)) { //se le coordinate rientrano all'interno di una sfera di raggio 40000m
        m_boids.push_back({x, y, z, v_x, v_y, v_z}); //assegnarle come valori delle posizioni e delle velocità dei boids in ordine
        m_evolve.push_back({0., 0., 0., 0., 0., 0.}); //inizializza le componenti aggiuntive dei valori nel tempo a 0
        i++;
      }
    }
  }
  //metodo utile a generare random la posizione iniziale del boid
  //all'interno di una sfera di raggio 40000m
  //e la velocità in un range tra -15m/s a 15m/s


  void evolve(double const& s, double const& a, double const& c) {
    for (int i = 0; i < size(); i++) {
      separation(i, s);
      alignment(i, a);
      cohesion(i, c);
    }
  }
  //chiama i metodi di separazione, allineamento e coesione

  void apply_evolve() { 
    for (int i = 0; i < size(); i++) {
      m_boids[i].v_x += m_evolve[i].v_x;
      m_boids[i].v_y += m_evolve[i].v_y;
      m_boids[i].v_z += m_evolve[i].v_z;
      if (speed(i) > 20) {
        m_boids[i].v_x *= (10 / speed(i));
        m_boids[i].v_y *= (10 / speed(i));
        m_boids[i].v_z *= (10 / speed(i));
      }
      m_boids[i].x += m_boids[i].v_x;
      m_boids[i].y += m_boids[i].v_y;
      m_boids[i].z += m_boids[i].v_z;
      if (radius(i) > 250) {
        double d0 = radius(i);
        m_boids[i].x *= (250 / d0);
        m_boids[i].y *= (250 / d0);
        m_boids[i].z *= (250 / d0);
        m_boids[i].v_x *= (10 / 250);
        m_boids[i].v_x *= (10 / 250);
        m_boids[i].v_x *= (10 / 250);
      }
      m_evolve[i].v_x = 0;
      m_evolve[i].v_y = 0;
      m_evolve[i].v_z = 0;
    }
  }


  Meanvalue meanspeed() {
    double meanspeed;
    double meansquaredspeed;
    double stdspeed;
    for (int i = 0; i < size(); i++) {
      meanspeed += speed(i);
      meansquaredspeed += pow(speed(i), 2);
    }
    meanspeed /= size();
    meansquaredspeed /= size();
    stdspeed = sqrt((meansquaredspeed - pow(meanspeed, 2)) / (size() - 1));
    return {meanspeed, stdspeed};
  }

  Meanvalue meandistance() {
    double meandist;
    double meansquareddist;
    double stddist;
    for (int i = 0; i < size(); i++) {
      for (int j = 0; j < size(); j++) {
        meandist += distance(i, j);
        meansquareddist += pow(distance(i, j), 2);
      }
    }
    meandist /= ((size() - 1) * size());
    if (size() > 2) {
      meansquareddist /= ((size() - 1) * size());
      stddist = sqrt((meansquareddist - pow(meandist, 2)) /
                     (((size() - 1) * size()) / 2 - 1));
    } else {
      stddist = 0;
    }
    return {meandist, stddist};
  }
  
  double maxdistance() {
    double maxdist;
    for (int i = 0; i < size(); i++) {
      for (int j = 0; j < size(); j++) {
        if (maxdist < distance(i, j)) {
          maxdist = distance(i, j);
        }
      }
    }
    return maxdist;
  }
};

#endif
