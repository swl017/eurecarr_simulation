/**
 * @file   model_param.h
 * @author Seungwook Lee
 * @date   2021-12-10
 * @brief  Vehicle dynamics model parameters.
 * @todo   Rewrite this(writing params in headerfiles) into reading YAML
 */

#ifndef ES_MODEL_PARAM_H_
#define ES_MODEL_PARAM_H_

const double AXIS_DISTANCE = 3.0; // unit: m
const double COG_TO_FRONT_AXIS = 1.5; // unit: m
const double COG_TO_REAR_AXIS = AXIS_DISTANCE - COG_TO_FRONT_AXIS;

const double Cm1        = 2853.6789;  //0.287
const double Cm1_brake  = 356.1283; 
const double Cm2        = 0.0; //0.0545
const double Cr0        = 71.272; //0.0518
const double Cr2        = 0.4440625; //0.00035
const double Br     	= 7.3689; //3.385
const double Cr 	    = 1.9589; //1.2691
const double Dr     	= 4702.8301; //0.173
const double Bf	        = 7.0215; //2.579
const double Cf     	= 1.9148; //1.2
const double Df     	= 3621.6843; //0.192
const double m       	= 481.6; //0.041
const double Iz     	= 550; //27.8E-6
const double length     = 2.971;  // 0.15 
const double width      = 2.03;  // 0.08 
const double lf         = 1.738476549225652;  // 0.15/2.0
const double lr         = AXIS_DISTANCE - COG_TO_FRONT_AXIS;




#endif // ES_MODEL_PARAM_H_