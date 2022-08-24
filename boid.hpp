#ifndef BOID_HPP
#define BOID_HPP

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

struct Boid {
  double x;
  double y;
  double z;
  double v_x;
  double v_y;
  double v_z;
};

struct Meanvalue {
  double meanvalue;
  double stdmeanvalue;
};

class Boids {
  std::vector<Boid> m_boids;
  std::vector<Boid> m_evolve;

  double radius(int const& i) {
    return sqrt(pow(m_boids[i].x, 2) + pow(m_boids[i].y, 2) + pow(m_boids[i].z, 2));
  }

  double distance(int const& i, int const& j) {
    return sqrt(pow(m_boids[i].x - m_boids[j].x, 2) +
                pow(m_boids[i].y - m_boids[j].y, 2) +
                pow(m_boids[i].z - m_boids[j].z, 2));
  }

  double signeddistance(int const& i, int const& j, int const& k, int const& l, int const& m) {
    return k * (m_boids[j].x - m_boids[i].x) +
           l * (m_boids[j].y - m_boids[i].y) +
           m * (m_boids[j].z - m_boids[i].z);
  }

  double angle(int const& i, int const& j) {
    return acos((m_boids[i].v_x * signeddistance(i, j, 1, 0, 0) +
                 m_boids[i].v_y * signeddistance(i, j, 0, 1, 0) +
                 m_boids[i].v_y * signeddistance(i, j, 0, 0, 1)) /
                (speed(i) * distance(i, j)));
  }

  double speed(int const& i) {
    return sqrt(pow(m_boids[i].v_x, 2) + pow(m_boids[i].v_y, 2) + pow(m_boids[i].v_z, 2));
  }

  int size() { 
    return m_boids.size(); 
    }

  void separation(int const& i, double const& s) {
    double sum_x;
    double sum_y;
    double sum_z;
    for (int j = 0; j < size(); j++) {
      if (i != j && distance(i, j) < 20 && angle(i, j) < 2.618) {
        sum_x += signeddistance(j, i, 1, 0, 0);
        sum_y += signeddistance(j, i, 0, 1, 0);
        sum_z += signeddistance(j, i, 0, 0, 1);
      }
    }
    m_evolve[i].v_x += (-s * sum_x);
    m_evolve[i].v_y += (-s * sum_y);
    m_evolve[i].v_z += (-s * sum_z);
  }

  void alignment(int const& i, double const& a) {
    double sum_v_x;
    double sum_v_y;
    double sum_v_z;
    int counter;
    for (int j = 0; j < size(); j++) {
      if (i != j && distance(i, j) < 150 && angle(i, j) < 2.618) {
        sum_v_x += m_boids[j].v_x;
        sum_v_y += m_boids[j].v_y;
        sum_v_z += m_boids[j].v_z;
        counter++;
      }
    }
    if (counter > 0) {
      m_evolve[i].v_x += a * (sum_v_x / counter - m_boids[i].v_x);
      m_evolve[i].v_y += a * (sum_v_y / counter - m_boids[i].v_y);
      m_evolve[i].v_z += a * (sum_v_z / counter - m_boids[i].v_z);
    }
  }

  void cohesion(int const& i, double const& c) {
    double center_mass_x;
    double center_mass_y;
    double center_mass_z;
    int counter;
    for (int j = 0; j < size(); j++) {
      if (i != j && distance(i, j) < 150 && angle(i, j) < 2.618) {
        center_mass_x += m_boids[j].x;
        center_mass_y += m_boids[j].y;
        center_mass_z += m_boids[j].z;
        counter++;
      }
    }
    if (counter > 0) {
      center_mass_x /= counter;
      center_mass_y /= counter;
      center_mass_z /= counter;
      m_evolve[i].v_x += c * (center_mass_x - m_boids[i].x);
      m_evolve[i].v_y += c * (center_mass_y - m_boids[i].y);
      m_evolve[i].v_z += c * (center_mass_z - m_boids[i].z);
    }
  }

 public:

  void create_boids(int const& n) {
    srand(static_cast<unsigned>(time(0)));
    int i = 0;
    while (i < n) {
      double x = -200 + round(static_cast<float>(rand()) /
                              (static_cast<float>(RAND_MAX / (400))));
      double y = -200 + round(static_cast<float>(rand()) /
                              (static_cast<float>(RAND_MAX / (400))));
      double z = -200 + round(static_cast<float>(rand()) /
                              (static_cast<float>(RAND_MAX / (400))));
      double v_x = -10 + static_cast<float>(rand()) /
                             (static_cast<float>(RAND_MAX / (20)));
      double v_y = -10 + static_cast<float>(rand()) /
                             (static_cast<float>(RAND_MAX / (20)));
      double v_z = -10 + static_cast<float>(rand()) /
                             (static_cast<float>(RAND_MAX / (20)));
      if (pow(x, 2) + pow(y, 2) + pow(z, 2) < pow(200, 2)) {
        m_boids.push_back({x, y, z, v_x, v_y, v_z});
        m_evolve.push_back({0., 0., 0., 0., 0., 0.});
        i++;
      }
    }
  }

  void evolve(double const& s, double const& a, double const& c) {
    for (int i = 0; i < size(); i++) {
      separation(i, s);
      alignment(i, a);
      cohesion(i, c);
    }
  }

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
