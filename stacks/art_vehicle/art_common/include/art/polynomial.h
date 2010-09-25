/* -*- mode: C++ -*-
 *
 *  Description: Polynomial class
 *
 *  Copyright (C) 2009 Austin Robot Technology                    
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _POLYNOMIAL_HH_
#define _POLYNOMIAL_HH_

/**  @file
   
     @brief Polynomial class interface
*/

#include <math.h>
#include <vector>


#define POLYNOMIAL_NAME_SIZE 32

/** @brief polynomial interface */
class Polynomial
{
 public:

  /** @brief Constructor
   * @param pname polynomial name
   */
  Polynomial(const char *pname)
    {
      strncpy(name, pname, sizeof(name));
      coef.clear();
    }

  virtual ~Polynomial() {};

  /** @brief Add default coefficient
   *
   *  first call adds constant term
   *  each successive call adds increasing powers of x
   */
  void inline add_coef(float term)
  {
    coef.push_back(term);
  }

#if 0
  /** @brief Configure coefficients from tuple */
  void inline configure()
  {
    unsigned len = cf->GetTupleCount(section, name);
    if (len > 0)			// tuple defined
      {
	coef.clear();
	for (unsigned i = 0; i < len; ++i)
	  {
	    float val = cf->ReadTupleFloat(section, name, i, 0.0);
	    coef.push_back(val);
	  }
      }

    // log the coefficients
    for (unsigned i = 0; i < coef.size(); ++i)
      ROS_INFO("Polynomial constructor: %s[%d] = %.7f", name, i, coef.at(i));
  };
#endif

  /** @brief evaluate polynomial
   * @param x point at which to evaluate
   */
  float inline value(const float x)
  {
    float retval = 0.0;
    float pow = 1.0;
    for (unsigned i = 0; i < coef.size(); ++i)
      {
	retval += pow * coef.at(i);
	pow *= x;
      }
    return retval;
  }

  // coef[i] applies to the x**i term
  std::vector<float> coef;             /**< polynomial coefficients */

protected:

  char name[POLYNOMIAL_NAME_SIZE];	/**< polynomial name */

};

#endif // _POLYNOMIAL_HH_ //
