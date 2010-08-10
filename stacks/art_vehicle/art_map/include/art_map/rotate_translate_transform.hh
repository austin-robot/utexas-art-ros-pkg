//////////////////////////////////////////////////////////////////////
/** \file 
Written and maintained by Patrick Beeson (pbeeson@cs.utexas.edu)
**/
//////////////////////////////////////////////////////////////////////

#ifndef rotate_translate_transform_hh
#define rotate_translate_transform_hh


// TODO: move posetype to <art/types.hh> and replace MapPose with it
/**
   Many things have x,y or x,y,theta.
**/
class posetype {
public:
  posetype(float _x=0.0, float _y=0.0, float _t=0.0): x(_x), y(_y), theta(_t){}
  float x,y,theta;
};


/**
   Rotate_Translate transformations class
**/
class rotate_translate_transform {
private:
  posetype actual_transform; //<! Holds the transform information
  float actual_cs, actual_sn;
public:
  rotate_translate_transform();
  void find_transform(const posetype&,
		      const posetype&);
  posetype apply_transform(const posetype&) const;
  posetype apply_inverse_transform(const posetype&) const;
};

#endif
