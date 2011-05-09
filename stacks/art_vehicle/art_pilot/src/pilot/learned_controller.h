#ifndef __LEARNED_SPEED_H_
#define __LEARNED_SPEED_H_

#include "speed.h"
#include <ros/ros.h>


/** Acceleration matrix speed controller class */
class LearnedSpeedControl: public SpeedControl
{
 public:

  LearnedSpeedControl();
  virtual ~LearnedSpeedControl();
  virtual void adjust(float speed, float error,
                      float *brake_req, float *throttle_req);
  virtual void configure(art_pilot::PilotConfig &newconfig);
  virtual void reset(void);
  int getAction(const std::vector<float> &s);

 private:

  std::vector<float> s;

  /** The implementation maps all sensations to a set of canonical
      pointers, which serve as the internal representation of
      environment state. */
  typedef const std::vector<float> *state_t;

  /** Produces a canonical representation of the given sensation.
      \param s The current sensation from the environment.
      \return A pointer to an equivalent state in statespace. */
  state_t canonicalize(const std::vector<float> &s);

  /** Set of all distinct sensations seen.  Pointers to elements of
      this set serve as the internal representation of the environment
      state. */
  std::set<std::vector<float> > statespace;

  /** The primary data structure of the learning algorithm, the value
      function Q.  For state_t s and int a, Q[s][a] gives the
      learned maximum expected future discounted reward conditional on
      executing action a in state s. */
  std::map<state_t, std::vector<float> > Q;

  void loadPolicy(const char* filename);

  const int numactions;

  bool LOADDEBUG;
  bool loaded;

};

#endif
