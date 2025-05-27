// 두번째 커밋 실습위해 첫출 수정
/*****************************************************************************
* Copyright 2013 Delphi Technologies, Inc., All Rights Reserved
*
* CM Synergy Header
* %full_name: kok_css2#23/csrc/CTA.c/2.4.16 %
* %created_by: fz7nzw %
* %date_modified: Fri Mar  3 04:04:03 2017 %
*
*****************************************************************************/


/**\file
*
* CTA: This feature is designed to warn the driver when traffic that is approaching from
* the side presents a danger.  Initially, this is focused on traffic crossing behind
* the host vehicle while in reverse.  It may also be extended to support traffic
* crossing in front of the host vehicle while moving forward.
* \b Project:            ADAS FEATURE \n
* \b Application:        cta \n
* \b Module:             %name: CTA.c % \n
* \b Classification:     CONFIDENTIAL \n
*/


#include "customer_iface.h" 
#include "CTA.h"
#include "cta_types.h"
#include <mathf.h>
#include "optimised_basic_ops.h"
#include <assert.h>
#include "cta_pre_run.h"
#include "cta_post_run.h"
#include "CTA_PERSISTENT_T.h"
#include "Vector_2d.h"
#include "Vector_Alg.h"
#include "CTA_Iterator.h"
#include "CTA_Object_Struct.h"
#include "CTA_Factory.h"
#include "CTA_Debug_Internal.h"
#include "cta_cal.h"
#include "Geometric_2d_Structs.h"
#include "vehicle_can_common_Iface.h"
#include "Basic_Macros.h"
#include "rsds_rccw_cal.h"
#include "RCCW_TI.h"
#include "Math_Selector.h"


/*===========================================================================*\
* Local Defines
\*===========================================================================*/
#define USM_SENSITIVITY_CTA_DEFAULT   (0)
#define USM_SENSITIVITY_CTA_LATE      (1)
#define USM_SENSITIVITY_CTA_NORMAL    (2)
#define USM_SENSITIVITY_CTA_EARLY     (3)

#define PI_0_5 (1.5707963267949F)

#define CTA_DIAGONAL_MAX_DYNY (36.F)

#ifdef PC_RESIM
#define KPH2MPS(a) ((a) * 1000.0f/3600.0f)
#define MPS2KPH(a) ((a) * 3.6f)
#endif

#define PI_1_2           (PI*0.5f)

/**
 * Number of support points of characteristic curve for steering-angle-based
 * adaptation of the conflict zone length. Must comply with corresponding CAL array size.
 */
#define CTA_STEER_SUPPORT_POINTS_CNT 5
#define GET_INDEX_OF_PERIODIC_ARRAY(index,periodizity) (index)%(periodizity)
#define SIGN(number) (((number) > 0) - ((number) < 0))

#ifndef REPLACE_SQRT_IN_TRACKER
#	define SQRT(arg) FAST_SQRT(arg)
#else
#	define SQRT(arg) mrr_base_ops_sqrt(arg)
#endif

 /*===========================================================================*\
 * File Scope variables
 \*===========================================================================*/
 /*% file-scope to CTA module, initialized in initCTA.m*/
/* BEGIN: % Added For Front CTA */
static CTA_Position_T CTA_FrontOrRear; /* Tells the model whether it is Front or rear CTA */
static Tracker_Output_Object_T tracker_output_object_high_crit[2]; /* data array to store tracker output for objects with highest criticality level on both sides */

/*===========================================================================*\
* Local Functions Prototypes
\*===========================================================================*/

static void CheckCTA_Single_Object(const CTA_INPUT_T *p_CTA_input,
	CTA_OUTPUT_T *p_CTA_output,
	CTA_PERSISTENT_T* p_CTA_persistent,
	const CALIBRATION_CTA_T *p_cals,
	CTA_Object_Data_T *p_object,
	const cta_heading_angle_t *p_heading_angle,
	Level_Logic_Calibration_T *p_level_cals,
	const Level_Logic_Calibration_T *p_level_cals_backup,
	const float32_T sin_k_cta_min_park_angle,
	const float32_T cos_k_cta_min_park_angle);

static void checkCTA(const CTA_INPUT_T *p_CTA_input,
	CTA_OUTPUT_T *p_CTA_output,
	CTA_PERSISTENT_T* p_CTA_persistent,
	const CALIBRATION_CTA_T* p_cals);

static void flipCTAzone(const CTA_Side_T side,
	cta_zone_point_t *cta_zone,
	const CALIBRATION_CTA_T* p_cals,
	const VEHICLE_DATA_FLT_T* p_vehicle_data);

static boolean_T Is_Object_In_Field_of_Interest(const CTA_Target_Reference *p_ref_point,
    const CTA_Advanced_Object_Corner *p_object_corner,
    const Vector_2d_T* cta_zone,
    const uint8_T num_polygon_corners,
    const CALIBRATION_CTA_T *p_cals);

static Vector_2d_T getIntersectionPointWithXandYAxis(const CTA_Target_Reference ref_point,
	const CTA_Object_Data_T *p_object,
	const Vector_2d_T obj_relative_velocity);

//static float32_T calculateTimeToConflict(const Vector_2d_T* p_obj_relative_velocity,
static float32_T calculateTimeToConflict(float32_T rel_lat_vel,
	const CTA_Target_Reference* p_obj_ref_point,
	const CTA_Object_Data_T *p_object,
	const VEHICLE_DATA_FLT_T *p_vehicle_data,
	const CALIBRATION_CTA_T* p_cals);

static float32_T calculateRadialDistance(const CTA_Target_Reference* p_obj_ref_point,
	const CTA_PERSISTENT_T* p_CTA_persistent);

static void calculateTargetReferencePoint(const CTA_Object_Data_T* p_object,
	const float32_T tracker_cs_to_reference_cs,
	const CTA_Advanced_Object_Corner *p_target_object_corners,
	CTA_Target_Reference *p_target_reference,
	const CALIBRATION_CTA_T *p_cals);
	
static void calculateTargetCornersPoint(const CTA_Object_Data_T* p_object,
	CTA_Advanced_Object_Corner * p_target_corners);

static float32_T calculate_dyn_ttc(unsigned16_T mode, float32_T relvel_y, const CALIBRATION_CTA_T *p_cals);

static boolean_T Check_Single_Level(uint8_T last_cycle_krit_level,
	                                   uint8_T* p_waiting_counter,
									   uint8_T* p_holding_counter,
									   const uint8_T krit_level,
									   const uint8_T in_zone,
									   const TargetAttributes_T* p_target_attributes,
									   const float32_T max_ttc,
									   const float32_T max_radial_distance,
									   const float32_T min_intersec_point,
									   const float32_T max_intersec_point,
									   const float32_T intersec_accuracy_thres,
									   const uint8_T suppress_cycle_count,
									   const uint8_T hold_cycle_count,
									   const uint8_T f_prevent_fall_back_level_1,
									   uint8_T* f_level_1_is_active);
									   
static void Check_All_Levels(CTA_Crit_Level_T* max_level,
	CTA_Object_Data_T *p_object_highest_crit,
	const CTA_Object_Data_T *p_object,
	const uint8_t f_target_in_zone,
	const Level_Logic_Calibration_T* p_level_cals,
	const CALIBRATION_CTA_T* p_cals);

static void Set_Object_highest_crit(CTA_Object_Data_T* p_object_highest_crit,
	const CTA_Object_Data_T* p_object);

static boolean_T Check_Object_Validity(const CTA_Object_Data_T* p_object,
	const CALIBRATION_CTA_T* p_cals,
	const cta_heading_angle_t heading_angle);

static uint8_T getHysteresisLevel(const CTA_Object_Data_T *p_object);

static void adaptIntersectLinesToSteeringAngle(const CALIBRATION_CTA_T* p_cals,
	Level_Logic_Calibration_T* p_level_cals,
	const float32_T steering_angle);

static float32_T getSteeringBasedIntersectLinesAdaptionFactor(const float32_T steering_angle,
	const float32_T* steer_angle_table,
	const float32_T* steer_factor_table);

static void adaptIntersectLinesToObjectHeading(Level_Logic_Calibration_T* p_level_cals,
	const CALIBRATION_CTA_T* p_cals,
	const float32_T obj_heading,
	const float32_T sin_min_park_angle);

static void adaptIntersectPointToObjectHeading(CTA_Object_Data_T* p_object,
	const VEHICLE_DATA_FLT_T *p_vehicle_data,
	const float32_T sin_min_park_angle,
	const float32_T cos_min_park_angle);

static void Fill_Debug_Output_With_Default_Values(const CTA_OUTPUT_T *p_CTA_output,
	const CALIBRATION_CTA_T *p_cals);
	
static float32_T getRelativeLongitudinalVelocityObject(const Tracker_Output_Object_T* p_target_object_data,
	const VEHICLE_DATA_FLT_T *p_vehicle_data,
	const CALIBRATION_CTA_T* p_cals);

static float32_T getRelativeLateralVelocityObject(const Tracker_Output_Object_T* p_target_object_data,
	const CALIBRATION_CTA_T* p_cals);

static Vector_2d_T getRelativeVelocityObject(const Tracker_Output_Object_T* p_target_object_data,
	const VEHICLE_DATA_FLT_T *p_vehicle_data,
	const CALIBRATION_CTA_T* p_cals);

static float32_T getIntersectionPointAccuracy(const Tracker_Output_Object_T* p_target_object_data,
	const Vector_2d_T obj_relative_velocity,
	const CALIBRATION_CTA_T* p_cals);

static boolean_T Is_Point_In_Polygon(const Vector_2d_T* cta_zone,
	const uint8_T num_polygon_corners,
	const Vector_2d_T* p_point);

static Line_Normal_T Create_Line_Normal(const Vector_2d_T *p_p0, const Vector_2d_T *p_p1);

static CTA_Line_Side_T Get_Side_of_Point(const Line_Normal_T *p_line, const Vector_2d_T *p_point);

/*===========================================================================*\
* Global Functions	Definition
\*===========================================================================*/
float32_T getCTA_OffLatLine(const CALIBRATION_CTA_T *p_cals, float32_T host_width, float32_T length_obj, int dir)/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
{
    float32_T ypos;
    //coverity[misra_c_2012_rule_10_4_violation]  LEFT_TO_RIGHT's type casting is normally doing
    if(LEFT_TO_RIGHT == dir)               
    {
        //coverity[misra_c_2012_rule_10_4_violation]  host_width's type casting is normally doing    
        if((u16p0_T)0 == (p_cals->k_cta_ZoneOffCond_Is_TargetCenter)) // new revision
        {
            /*when value is false, warning zone end criteria is that target's rear bumper is away from below ypos */
            ypos = 1+(host_width*0.5F);   // new , 
        }
        else
        {
            /*when value is true, warning zone end criteria is that target's center is away from below ypos*/
            ypos = 1+(host_width*0.5F) - ((0.5F)*length_obj);//old revision
        }
    }
    else
    {        
        //coverity[misra_c_2012_rule_10_4_violation]  host_width's type casting is normally doing            
        if((u16p0_T)0 == (p_cals->k_cta_ZoneOffCond_Is_TargetCenter)) // new revision
        {
            /*when value is false, warning zone end criteria is that target's rear bumper is away from below ypos */
            ypos = (-1.0F*(1+(host_width*0.5F))); //new
        }
        else
        {
            /*when value is true, warning zone end criteria is that target's center is away from below ypos*/
            ypos = (-1.0F*(1+(host_width*0.5F))) + ((0.5F)*length_obj); //org   
        }
    }

    return ypos;
}

#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE
/************LPF***************
  <weak filter>
  COEF  = 0.1;    

  <middle filter>
  COEF  = 0.5;    

  <strong filter>
  COEF  = 0.9;    
*********************************/
uint8_T   k_cta_LPF_USE_dynamiczone_heading = 0;
float32_T k_cta_LPF_dynamiczone_heading_COEF= 0.99F; /* <weak filter> COEF  = 0.1 <middle filter> COEF  = 0.5 <strong filter> COEF  = 0.9 */
float32_T filtered_cta_LTOR_dynamiczone_heading=90.0F*Degree2Radian;
float32_T filtered_cta_RTOL_dynamiczone_heading=-90.0F*Degree2Radian;


float32_T cta_dynamiczone_heading_filteredLPF(CTA_Side_T side, float32_T heading)
{
    if(LeftSide == side)
    {
        filtered_cta_LTOR_dynamiczone_heading = (k_cta_LPF_dynamiczone_heading_COEF*filtered_cta_LTOR_dynamiczone_heading) + ((1.0F-k_cta_LPF_dynamiczone_heading_COEF)*heading);/*polyspace RTE:OVFL [Justified:Low] "will not be overflowing by considered input"*/

        return filtered_cta_LTOR_dynamiczone_heading;
    }
    else
    {
        filtered_cta_RTOL_dynamiczone_heading = (k_cta_LPF_dynamiczone_heading_COEF*filtered_cta_RTOL_dynamiczone_heading) + ((1.0F-k_cta_LPF_dynamiczone_heading_COEF)*heading);/*polyspace RTE:OVFL [Justified:Low] "will not be overflowing by considered input"*/

        return filtered_cta_RTOL_dynamiczone_heading;
    }
}

float32_T get_dynamic_minus_offset_by_heading(RCCA_DIR_State_T side, float32_T heading, float32_T host_width)
{
    float32_T abs_comp_offset   =0.0F;
    float32_T abs_heading_rad   =0.0F;
    float32_T abs_heading_degree=0.0F;

    switch(side)
    {
        case LEFT_TO_RIGHT:
        {       
            /*****************************************/
            /* lower approaching angle(obtuse) : 둔각*/
            if((heading>=(0.0F*Degree2Radian)) && (heading<=(90.0F*Degree2Radian)) && ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_LOWER_USE_offsetEndzone)
            {
                abs_heading_rad    = FAST_ABS((0.5F*PI) - FAST_ABS(heading));     
                abs_heading_degree = Radian2Degree*(abs_heading_rad);
                
                if(abs_heading_degree > ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_LOWER_absolute_angthresDegree_for_offsetEndzone)               
                {                    
                    abs_comp_offset = FAST_ABS(FAST_SIN(abs_heading_rad) * host_width);
                }else{/* empty */}
                
                abs_comp_offset *= ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_LOWER_scalefactor_offsetEndzone;
                abs_comp_offset = min(abs_comp_offset, ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_LOWER_max_offsetEndzone); // limit offset to 1m   
            }
            /*****************************************/
            /* upper approaching  angle(acute)  : 예각*/
            else if((heading>=(90.0F*Degree2Radian)) && (heading<(180.0F*Degree2Radian)) && ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_UPPER_USE_offsetEndzone)
            {                
                abs_heading_rad    = FAST_ABS(FAST_ABS(heading) - (0.5F*PI));     
                abs_heading_degree = Radian2Degree*(abs_heading_rad);
                
                if(abs_heading_degree > ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_UPPER_absolute_angthresDegree_for_offsetEndzone)               
                {                    
                    abs_comp_offset = FAST_ABS(FAST_SIN(abs_heading_rad) * host_width);
                }else{/* empty */}
                
                abs_comp_offset *= ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_UPPER_scalefactor_offsetEndzone;
                abs_comp_offset = min(abs_comp_offset, ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_UPPER_max_offsetEndzone); // limit offset to 1m   
            }            
            else{/* empty */}
            
            break;
        }
        
        case RIGHT_TO_LEFT:
        {
            /*****************************************/
            /* lower approaching angle(obtuse) : 둔각*/
            if((heading>=(-90.0F*Degree2Radian)) && (heading<(0.0F*Degree2Radian)) && ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_LOWER_USE_offsetEndzone)
            {   
                abs_heading_rad    = FAST_ABS((0.5F*PI) - FAST_ABS(heading));     
                abs_heading_degree = Radian2Degree*(abs_heading_rad);
                
                if(abs_heading_degree > ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_LOWER_absolute_angthresDegree_for_offsetEndzone)               
                {                    
                    abs_comp_offset = FAST_ABS(FAST_SIN(abs_heading_rad) * host_width);
                }else{/* empty */}
                
                abs_comp_offset *= ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_LOWER_scalefactor_offsetEndzone;
                abs_comp_offset = min(abs_comp_offset, ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_LOWER_max_offsetEndzone); // limit offset to 1m   
            }
            /*****************************************/
            /* upper approaching angle(acute)  : 예각*/
            else if((heading>=(-180.0F*Degree2Radian)) && (heading<(-90.0F*Degree2Radian)) && ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_UPPER_USE_offsetEndzone)
            {                
                abs_heading_rad    = FAST_ABS(FAST_ABS(heading) - (0.5F*PI));     
                abs_heading_degree = Radian2Degree*(abs_heading_rad);
                
                if(abs_heading_degree > ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_UPPER_absolute_angthresDegree_for_offsetEndzone)               
                {
                    abs_comp_offset = FAST_ABS(FAST_SIN(abs_heading_rad) * host_width);
                }else{/* empty */}
                
                abs_comp_offset *= ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_UPPER_scalefactor_offsetEndzone;
                abs_comp_offset = min(abs_comp_offset, ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_UPPER_max_offsetEndzone); // limit offset to 1m   
            }
            else{/* empty */}
            
            break;
        }
    }
        
    return abs_comp_offset;
}
#endif

#ifdef RCCW_DYNAMIC_ZONE_REFPNT_LAT_CENTER
float32_T set_cta_off_zone_LT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                         float32_T            target_angle,
                         float32_T            target_length,
                         const CALIBRATION_CTA_T *p_cals,
                         float32_T            *off_zone_pt_x,
                         float32_T            *off_zone_pt_y)
{
    float32_T        host_length;
    float32_T        host_width;
    float32_T        zone_x_length;
    float32_T        zone_y_length;    
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;
    float32_T        approach_ang;
    float32_T        DYN_HYS_LR_INNER;
    float32_T        DYN_HYS_LR_OUTER;
    float32_T        DYN_HYS_UD;

    p_vehicle_data = p_CTA_input->vehicle_data;

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    DYN_HYS_LR_INNER = p_cals->k_cta_min_dyny; // 4m
    DYN_HYS_LR_OUTER = p_cals->k_cta_zone_hys_lr; // 2m
    DYN_HYS_UD       = p_cals->k_cta_zone_hys_ud; // 2

#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE
    minus_offset = get_dynamic_minus_offset_by_heading(LEFT_TO_RIGHT, target_angle, p_vehicle_data->host_vehicle_width);
    zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room - minus_offset;
#else
    zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room;    
#endif

    zone_y_length = p_cals->k_cta_max_dyny-(DYN_HYS_LR_INNER-host_width*0.5F);

    /*********************************/
    /*  OFF zone coordinate setting  */    
    ref_pt_x = (-1.0F) * (host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = getCTA_OffLatLine(p_cals, host_width, target_length, LEFT_TO_RIGHT);

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = p_cals->k_cta_perpendicular_angle-p_cals->k_cta_perpendicular_angle_hys; /* 80  deg */
    upper_thr_90_angle = p_cals->k_cta_perpendicular_angle+p_cals->k_cta_perpendicular_angle_hys; /* 100 deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }
    
    if((target_angle > 0.F) && (target_angle <= p_cals->k_cta_perpendicular_angle)) /* 0 ~90 deg */
    {        
        approach_ang = FAST_ABS(0.5F*PI-FAST_ABS(target_angle));
            
        off_zone_pt_x[3] = ref_pt_x + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_UD);
        off_zone_pt_y[3] = ref_pt_y - FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_UD);
        
        off_zone_pt_x[2] = off_zone_pt_x[3] - FAST_ABS( FAST_COS(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        off_zone_pt_y[2] = off_zone_pt_y[3] + FAST_ABS( FAST_SIN(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        
        off_zone_pt_x[1] = off_zone_pt_x[2] - FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER+DYN_HYS_LR_OUTER));
        off_zone_pt_y[1] = off_zone_pt_y[2] - FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER+DYN_HYS_LR_OUTER));
        
        off_zone_pt_x[0] = off_zone_pt_x[3] - FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER+DYN_HYS_LR_OUTER));
        off_zone_pt_y[0] = off_zone_pt_y[3] - FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER+DYN_HYS_LR_OUTER));
    }
    else
    {
        approach_ang = FAST_ABS(FAST_ABS(target_angle)-0.5F*PI);
        
        off_zone_pt_x[3] = ref_pt_x + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_UD);
        off_zone_pt_y[3] = ref_pt_y + FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_UD);
        
        off_zone_pt_x[2] = off_zone_pt_x[3] - FAST_ABS( FAST_COS(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        off_zone_pt_y[2] = off_zone_pt_y[3] - FAST_ABS( FAST_SIN(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        
        off_zone_pt_x[1] = off_zone_pt_x[2] + FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[1] = off_zone_pt_y[2] - FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        
        off_zone_pt_x[0] = off_zone_pt_x[3] + FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[0] = off_zone_pt_y[3] - FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));

    }

    return minus_offset;
}

float32_T set_cta_on_zone_LT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                        float32_T            target_angle,
                        const CALIBRATION_CTA_T *p_cals,
                        float32_T            *on_zone_pt_x,
                        float32_T            *on_zone_pt_y)
{
    float32_T        host_length;    
    float32_T        host_width;    
    float32_T        zone_x_length;    
    float32_T        zone_y_length;    
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;
    float32_T        approach_ang;
    float32_T        DYN_HYS_LR_INNER;

    p_vehicle_data = p_CTA_input->vehicle_data;
    
    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    DYN_HYS_LR_INNER = p_cals->k_cta_min_dyny; // 4m

#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE    
    minus_offset = get_dynamic_minus_offset_by_heading(LEFT_TO_RIGHT, target_angle, p_vehicle_data->host_vehicle_width);
    zone_x_length = p_cals->k_cta_zone_x - minus_offset;
#else
    zone_x_length = p_cals->k_cta_zone_x;/*AQB-271*/
#endif

    zone_y_length = p_cals->k_cta_max_dyny-(DYN_HYS_LR_INNER-host_width*0.5F);

    /*********************************/
    /*  ON zone coordinate setting  */  
    ref_pt_x = (-1.0F) * (host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = 0;

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = p_cals->k_cta_perpendicular_angle-p_cals->k_cta_perpendicular_angle_hys; /* 80  deg */
    upper_thr_90_angle = p_cals->k_cta_perpendicular_angle+p_cals->k_cta_perpendicular_angle_hys; /* 100 deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }
    
    if((target_angle > 0) && (target_angle <= p_cals->k_cta_perpendicular_angle)) /* 0 ~90 deg */
    {
        approach_ang = FAST_ABS(0.5F*PI-FAST_ABS(target_angle));
        
        on_zone_pt_x[3] = ref_pt_x - FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_LR_INNER );
        on_zone_pt_y[3] = ref_pt_y - FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_LR_INNER );
        
        on_zone_pt_x[2] = on_zone_pt_x[3] - FAST_ABS( FAST_COS(approach_ang) * zone_x_length);
        on_zone_pt_y[2] = on_zone_pt_y[3] + FAST_ABS( FAST_SIN(approach_ang) * zone_x_length);
        
        on_zone_pt_x[1] = on_zone_pt_x[2] - FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[1] = on_zone_pt_y[2] - FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
        
        on_zone_pt_x[0] = on_zone_pt_x[3] - FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[0] = on_zone_pt_y[3] - FAST_ABS( FAST_COS(approach_ang) * zone_y_length);

    }
    else
    {        
        approach_ang = FAST_ABS(FAST_ABS(target_angle)-0.5F*PI);
        
        on_zone_pt_x[3] = ref_pt_x + FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_LR_INNER );
        on_zone_pt_y[3] = ref_pt_y - FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_LR_INNER );

        on_zone_pt_x[2] = on_zone_pt_x[3] - FAST_ABS( FAST_COS(approach_ang) * zone_x_length);
        on_zone_pt_y[2] = on_zone_pt_y[3] - FAST_ABS( FAST_SIN(approach_ang) * zone_x_length);

        on_zone_pt_x[1] = on_zone_pt_x[2] + FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[1] = on_zone_pt_y[2] - FAST_ABS( FAST_COS(approach_ang) * zone_y_length);

        on_zone_pt_x[0] = on_zone_pt_x[3] + FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[0] = on_zone_pt_y[3] - FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
    }
    
    return minus_offset;
}

float32_T set_cta_off_zone_RT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                         float32_T            target_angle,
                         float32_T            target_length,
                         const CALIBRATION_CTA_T *p_cals,
                         float32_T            *off_zone_pt_x,
                         float32_T            *off_zone_pt_y)
{
    float32_T        host_length; 
    float32_T        host_width;
    float32_T        zone_x_length;    
    float32_T        zone_y_length;    
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;
    float32_T        approach_ang;
    float32_T        DYN_HYS_LR_INNER;
    float32_T        DYN_HYS_LR_OUTER;
    float32_T        DYN_HYS_UD;

    p_vehicle_data = p_CTA_input->vehicle_data;

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    DYN_HYS_LR_INNER = p_cals->k_cta_min_dyny; // 4m
    DYN_HYS_LR_OUTER = p_cals->k_cta_zone_hys_lr; // 2m
    DYN_HYS_UD       = p_cals->k_cta_zone_hys_ud; // 2

#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE  
    minus_offset = get_dynamic_minus_offset_by_heading(RIGHT_TO_LEFT, target_angle, p_vehicle_data->host_vehicle_width);
    zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room - minus_offset;
#else
    zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room;
#endif

    zone_y_length = p_cals->k_cta_max_dyny-(DYN_HYS_LR_INNER-host_width*0.5F);

    /*********************************/
    /*  OFF zone coordinate setting  */
    ref_pt_x = (-1.0F) * (host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = getCTA_OffLatLine(p_cals, host_width, target_length, RIGHT_TO_LEFT);


    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)-p_cals->k_cta_perpendicular_angle_hys; /* -100 deg */
    upper_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)+p_cals->k_cta_perpendicular_angle_hys; /* -80  deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }

    if((target_angle >= (-1.0F*p_cals->k_cta_perpendicular_angle)) && (target_angle < 0.F)) /* -90 ~ 0 */
    {        
        approach_ang = FAST_ABS(0.5F*PI - FAST_ABS(target_angle));
        
        off_zone_pt_x[0] = ref_pt_x + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_UD);
        off_zone_pt_y[0] = ref_pt_y + FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_UD);
        
        off_zone_pt_x[1] = off_zone_pt_x[0] - FAST_ABS( FAST_COS(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        off_zone_pt_y[1] = off_zone_pt_y[0] - FAST_ABS( FAST_SIN(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        
        off_zone_pt_x[2] = off_zone_pt_x[1] - FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[2] = off_zone_pt_y[1] + FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        
        off_zone_pt_x[3] = off_zone_pt_x[0] - FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[3] = off_zone_pt_y[0] + FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        
    }
    else
    {        
        approach_ang = FAST_ABS(FAST_ABS(target_angle) - 0.5F*PI);
        
        off_zone_pt_x[0] = ref_pt_x + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_UD);
        off_zone_pt_y[0] = ref_pt_y - FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_UD);
        
        off_zone_pt_x[1] = off_zone_pt_x[0] - FAST_ABS( FAST_COS(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        off_zone_pt_y[1] = off_zone_pt_y[0] + FAST_ABS( FAST_SIN(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        
        off_zone_pt_x[2] = off_zone_pt_x[1] + FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[2] = off_zone_pt_y[1] + FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        
        off_zone_pt_x[3] = off_zone_pt_x[0] + FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[3] = off_zone_pt_y[0] + FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
    }
    
    return minus_offset;
}

float32_T set_cta_on_zone_RT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                        float32_T            target_angle,
                        const CALIBRATION_CTA_T *p_cals,
                        float32_T            *on_zone_pt_x,
                        float32_T            *on_zone_pt_y)
{
    float32_T        host_length;    
    float32_T        host_width;    
    float32_T        zone_x_length;    
    float32_T        zone_y_length;    
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;
    float32_T        approach_ang;
    float32_T        DYN_HYS_LR_INNER;

    p_vehicle_data = p_CTA_input->vehicle_data;
    
    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;
    
    DYN_HYS_LR_INNER = p_cals->k_cta_min_dyny; // 4m

#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE  
    minus_offset = get_dynamic_minus_offset_by_heading(RIGHT_TO_LEFT, target_angle, p_vehicle_data->host_vehicle_width);
    zone_x_length = p_cals->k_cta_zone_x - minus_offset;
#else
    zone_x_length = p_cals->k_cta_zone_x;/*AQB-271*/
#endif

    zone_y_length = p_cals->k_cta_max_dyny-(DYN_HYS_LR_INNER-host_width*0.5F);

    /*********************************/
    /*  ON zone coordinate setting  */       
    ref_pt_x = (-1.0F) * (host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = 0;

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)-p_cals->k_cta_perpendicular_angle_hys; /* -100 deg */
    upper_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)+p_cals->k_cta_perpendicular_angle_hys; /* -80  deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }

    if((target_angle >= (-1.0F*p_cals->k_cta_perpendicular_angle)) && (target_angle < 0.F)) /* -90 ~ 0 */
    {        
        approach_ang = FAST_ABS(0.5F*PI - FAST_ABS(target_angle));
        
        on_zone_pt_x[0] = ref_pt_x - FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_LR_INNER );
        on_zone_pt_y[0] = ref_pt_y + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_LR_INNER );
        
        on_zone_pt_x[1] = on_zone_pt_x[0]  - FAST_ABS( FAST_COS(approach_ang) * zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0]  - FAST_ABS( FAST_SIN(approach_ang) * zone_x_length);
        
        on_zone_pt_x[2] = on_zone_pt_x[1] - FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[2] = on_zone_pt_y[1] + FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
        
        on_zone_pt_x[3] = on_zone_pt_x[0] - FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[3] = on_zone_pt_y[0] + FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
    }
    else
    {        
        approach_ang = FAST_ABS(FAST_ABS(target_angle) - 0.5F*PI);

        
        on_zone_pt_x[0] = ref_pt_x + FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_LR_INNER );
        on_zone_pt_y[0] = ref_pt_y + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_LR_INNER );
        
        on_zone_pt_x[1] = on_zone_pt_x[0]  - FAST_ABS( FAST_COS(approach_ang) * zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0]  + FAST_ABS( FAST_SIN(approach_ang) * zone_x_length);
        
        on_zone_pt_x[2] = on_zone_pt_x[1] + FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[2] = on_zone_pt_y[1] + FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
        
        on_zone_pt_x[3] = on_zone_pt_x[0] + FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[3] = on_zone_pt_y[0] + FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
        
    }
    
    return minus_offset;

}

#ifdef PC_RESIM
void RESIM_set_cta_off_zone_LT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                         float32_T            target_angle,
                         float32_T            target_length,
                         const CALIBRATION_CTA_T *p_cals,
                         float32_T            *off_zone_pt_x,
                         float32_T            *off_zone_pt_y)
{
    float32_T        host_length;
    float32_T        host_width;
    float32_T        zone_x_length;
    float32_T        zone_y_length;    
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;
    float32_T        approach_ang;
    float32_T        DYN_HYS_LR_INNER;
    float32_T        DYN_HYS_LR_OUTER;
    float32_T        DYN_HYS_UD;

    p_vehicle_data = p_CTA_input->vehicle_data;

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    DYN_HYS_LR_INNER = p_cals->k_cta_min_dyny; // 4m
    DYN_HYS_LR_OUTER = p_cals->k_cta_zone_hys_lr; // 2m
    DYN_HYS_UD       = p_cals->k_cta_zone_hys_ud; // 2

//#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE
//    minus_offset = get_dynamic_minus_offset_by_heading(LEFT_TO_RIGHT, target_angle, p_vehicle_data->host_vehicle_width);
//    zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room - minus_offset;
//#else
    zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room;    
//#endif

    zone_y_length = p_cals->k_cta_max_dyny-(DYN_HYS_LR_INNER-host_width*0.5F);

    /*********************************/
    /*  OFF zone coordinate setting  */    
    ref_pt_x = (-1.0F) * (host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = getCTA_OffLatLine(p_cals, host_width, target_length, LEFT_TO_RIGHT);

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = p_cals->k_cta_perpendicular_angle-p_cals->k_cta_perpendicular_angle_hys; /* 80  deg */
    upper_thr_90_angle = p_cals->k_cta_perpendicular_angle+p_cals->k_cta_perpendicular_angle_hys; /* 100 deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }
    
    if((target_angle > 0.F) && (target_angle <= p_cals->k_cta_perpendicular_angle)) /* 0 ~90 deg */
    {        
        approach_ang = FAST_ABS(0.5F*PI-FAST_ABS(target_angle));
            
        off_zone_pt_x[3] = ref_pt_x + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_UD);
        off_zone_pt_y[3] = ref_pt_y - FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_UD);
        
        off_zone_pt_x[2] = off_zone_pt_x[3] - FAST_ABS( FAST_COS(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        off_zone_pt_y[2] = off_zone_pt_y[3] + FAST_ABS( FAST_SIN(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        
        off_zone_pt_x[1] = off_zone_pt_x[2] - FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER+DYN_HYS_LR_OUTER));
        off_zone_pt_y[1] = off_zone_pt_y[2] - FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER+DYN_HYS_LR_OUTER));
        
        off_zone_pt_x[0] = off_zone_pt_x[3] - FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER+DYN_HYS_LR_OUTER));
        off_zone_pt_y[0] = off_zone_pt_y[3] - FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER+DYN_HYS_LR_OUTER));
    }
    else
    {
        approach_ang = FAST_ABS(FAST_ABS(target_angle)-0.5F*PI);
        
        off_zone_pt_x[3] = ref_pt_x + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_UD);
        off_zone_pt_y[3] = ref_pt_y + FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_UD);
        
        off_zone_pt_x[2] = off_zone_pt_x[3] - FAST_ABS( FAST_COS(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        off_zone_pt_y[2] = off_zone_pt_y[3] - FAST_ABS( FAST_SIN(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        
        off_zone_pt_x[1] = off_zone_pt_x[2] + FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[1] = off_zone_pt_y[2] - FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        
        off_zone_pt_x[0] = off_zone_pt_x[3] + FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[0] = off_zone_pt_y[3] - FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));

    }

    return minus_offset;
}



void RESIM_set_cta_on_zone_LT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                        float32_T            target_angle,
                        const CALIBRATION_CTA_T *p_cals,
                        float32_T            *on_zone_pt_x,
                        float32_T            *on_zone_pt_y)
{
    float32_T        host_length;    
    float32_T        host_width;    
    float32_T        zone_x_length;    
    float32_T        zone_y_length;    
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;
    float32_T        approach_ang;
    float32_T        DYN_HYS_LR_INNER;

    p_vehicle_data = p_CTA_input->vehicle_data;
    
    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    DYN_HYS_LR_INNER = p_cals->k_cta_min_dyny; // 4m

//#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE    
//    minus_offset = get_dynamic_minus_offset_by_heading(LEFT_TO_RIGHT, target_angle, p_vehicle_data->host_vehicle_width);
//    zone_x_length = p_cals->k_cta_zone_x - minus_offset;
//#else
    zone_x_length = p_cals->k_cta_zone_x;/*AQB-271*/
//#endif

    zone_y_length = p_cals->k_cta_max_dyny-(DYN_HYS_LR_INNER-host_width*0.5F);

    /*********************************/
    /*  ON zone coordinate setting  */  
    ref_pt_x = (-1.0F) * (host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = 0;

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = p_cals->k_cta_perpendicular_angle-p_cals->k_cta_perpendicular_angle_hys; /* 80  deg */
    upper_thr_90_angle = p_cals->k_cta_perpendicular_angle+p_cals->k_cta_perpendicular_angle_hys; /* 100 deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }
    
    if((target_angle > 0) && (target_angle <= p_cals->k_cta_perpendicular_angle)) /* 0 ~90 deg */
    {
        approach_ang = FAST_ABS(0.5F*PI-FAST_ABS(target_angle));
        
        on_zone_pt_x[3] = ref_pt_x - FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_LR_INNER );
        on_zone_pt_y[3] = ref_pt_y - FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_LR_INNER );
        
        on_zone_pt_x[2] = on_zone_pt_x[3] - FAST_ABS( FAST_COS(approach_ang) * zone_x_length);
        on_zone_pt_y[2] = on_zone_pt_y[3] + FAST_ABS( FAST_SIN(approach_ang) * zone_x_length);
        
        on_zone_pt_x[1] = on_zone_pt_x[2] - FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[1] = on_zone_pt_y[2] - FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
        
        on_zone_pt_x[0] = on_zone_pt_x[3] - FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[0] = on_zone_pt_y[3] - FAST_ABS( FAST_COS(approach_ang) * zone_y_length);

    }
    else
    {        
        approach_ang = FAST_ABS(FAST_ABS(target_angle)-0.5F*PI);
        
        on_zone_pt_x[3] = ref_pt_x + FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_LR_INNER );
        on_zone_pt_y[3] = ref_pt_y - FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_LR_INNER );

        on_zone_pt_x[2] = on_zone_pt_x[3] - FAST_ABS( FAST_COS(approach_ang) * zone_x_length);
        on_zone_pt_y[2] = on_zone_pt_y[3] - FAST_ABS( FAST_SIN(approach_ang) * zone_x_length);

        on_zone_pt_x[1] = on_zone_pt_x[2] + FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[1] = on_zone_pt_y[2] - FAST_ABS( FAST_COS(approach_ang) * zone_y_length);

        on_zone_pt_x[0] = on_zone_pt_x[3] + FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[0] = on_zone_pt_y[3] - FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
    }
    
    return minus_offset;
}



void RESIM_set_cta_off_zone_RT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                         float32_T            target_angle,
                         float32_T            target_length,
                         const CALIBRATION_CTA_T *p_cals,
                         float32_T            *off_zone_pt_x,
                         float32_T            *off_zone_pt_y)
{
    float32_T        host_length; 
    float32_T        host_width;
    float32_T        zone_x_length;    
    float32_T        zone_y_length;    
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;
    float32_T        approach_ang;
    float32_T        DYN_HYS_LR_INNER;
    float32_T        DYN_HYS_LR_OUTER;
    float32_T        DYN_HYS_UD;

    p_vehicle_data = p_CTA_input->vehicle_data;

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    DYN_HYS_LR_INNER = p_cals->k_cta_min_dyny; // 4m
    DYN_HYS_LR_OUTER = p_cals->k_cta_zone_hys_lr; // 2m
    DYN_HYS_UD       = p_cals->k_cta_zone_hys_ud; // 2

//#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE  
//    minus_offset = get_dynamic_minus_offset_by_heading(RIGHT_TO_LEFT, target_angle, p_vehicle_data->host_vehicle_width);
//    zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room - minus_offset;
//#else
    zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room;
//#endif

    zone_y_length = p_cals->k_cta_max_dyny-(DYN_HYS_LR_INNER-host_width*0.5F);

    /*********************************/
    /*  OFF zone coordinate setting  */
    ref_pt_x = (-1.0F) * (host_length - p_cals->k_cta_rear_offset);
    //ref_pt_y = 0;
    ref_pt_y = getCTA_OffLatLine(p_cals, host_width, target_length, RIGHT_TO_LEFT);


    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)-p_cals->k_cta_perpendicular_angle_hys; /* -100 deg */
    upper_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)+p_cals->k_cta_perpendicular_angle_hys; /* -80  deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }

    if((target_angle >= (-1.0F*p_cals->k_cta_perpendicular_angle)) && (target_angle < 0.F)) /* -90 ~ 0 */
    {        
        approach_ang = FAST_ABS(0.5F*PI - FAST_ABS(target_angle));
        
        off_zone_pt_x[0] = ref_pt_x + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_UD);
        off_zone_pt_y[0] = ref_pt_y + FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_UD);
        
        off_zone_pt_x[1] = off_zone_pt_x[0] - FAST_ABS( FAST_COS(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        off_zone_pt_y[1] = off_zone_pt_y[0] - FAST_ABS( FAST_SIN(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        
        off_zone_pt_x[2] = off_zone_pt_x[1] - FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[2] = off_zone_pt_y[1] + FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        
        off_zone_pt_x[3] = off_zone_pt_x[0] - FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[3] = off_zone_pt_y[0] + FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        
    }
    else
    {        
        approach_ang = FAST_ABS(FAST_ABS(target_angle) - 0.5F*PI);
        
        off_zone_pt_x[0] = ref_pt_x + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_UD);
        off_zone_pt_y[0] = ref_pt_y - FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_UD);
        
        off_zone_pt_x[1] = off_zone_pt_x[0] - FAST_ABS( FAST_COS(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        off_zone_pt_y[1] = off_zone_pt_y[0] + FAST_ABS( FAST_SIN(approach_ang) * (zone_x_length + DYN_HYS_UD*2.0F));
        
        off_zone_pt_x[2] = off_zone_pt_x[1] + FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[2] = off_zone_pt_y[1] + FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        
        off_zone_pt_x[3] = off_zone_pt_x[0] + FAST_ABS( FAST_SIN(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
        off_zone_pt_y[3] = off_zone_pt_y[0] + FAST_ABS( FAST_COS(approach_ang) * (zone_y_length + DYN_HYS_LR_INNER + DYN_HYS_LR_OUTER));
    }
    
    return minus_offset;
}



void RESIM_set_cta_on_zone_RT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                        float32_T            target_angle,
                        const CALIBRATION_CTA_T *p_cals,
                        float32_T            *on_zone_pt_x,
                        float32_T            *on_zone_pt_y)
{
    float32_T        host_length;    
    float32_T        host_width;    
    float32_T        zone_x_length;    
    float32_T        zone_y_length;    
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;
    float32_T        approach_ang;
    float32_T        DYN_HYS_LR_INNER;

    p_vehicle_data = p_CTA_input->vehicle_data;
    
    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;
    
    DYN_HYS_LR_INNER = p_cals->k_cta_min_dyny; // 4m

//#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE  
//    minus_offset = get_dynamic_minus_offset_by_heading(RIGHT_TO_LEFT, target_angle, p_vehicle_data->host_vehicle_width);
//    zone_x_length = p_cals->k_cta_zone_x - minus_offset;
//#else
    zone_x_length = p_cals->k_cta_zone_x;/*AQB-271*/
//#endif

    zone_y_length = p_cals->k_cta_max_dyny-(DYN_HYS_LR_INNER-host_width*0.5F);

    /*********************************/
    /*  ON zone coordinate setting  */       
    ref_pt_x = (-1.0F) * (host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = 0;

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)-p_cals->k_cta_perpendicular_angle_hys; /* -100 deg */
    upper_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)+p_cals->k_cta_perpendicular_angle_hys; /* -80  deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }

    if((target_angle >= (-1.0F*p_cals->k_cta_perpendicular_angle)) && (target_angle < 0.F)) /* -90 ~ 0 */
    {        
        approach_ang = FAST_ABS(0.5F*PI - FAST_ABS(target_angle));
        
        on_zone_pt_x[0] = ref_pt_x - FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_LR_INNER );
        on_zone_pt_y[0] = ref_pt_y + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_LR_INNER );
        
        on_zone_pt_x[1] = on_zone_pt_x[0]  - FAST_ABS( FAST_COS(approach_ang) * zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0]  - FAST_ABS( FAST_SIN(approach_ang) * zone_x_length);
        
        on_zone_pt_x[2] = on_zone_pt_x[1] - FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[2] = on_zone_pt_y[1] + FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
        
        on_zone_pt_x[3] = on_zone_pt_x[0] - FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[3] = on_zone_pt_y[0] + FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
    }
    else
    {        
        approach_ang = FAST_ABS(FAST_ABS(target_angle) - 0.5F*PI);

        
        on_zone_pt_x[0] = ref_pt_x + FAST_ABS( FAST_SIN(approach_ang) * DYN_HYS_LR_INNER );
        on_zone_pt_y[0] = ref_pt_y + FAST_ABS( FAST_COS(approach_ang) * DYN_HYS_LR_INNER );
        
        on_zone_pt_x[1] = on_zone_pt_x[0]  - FAST_ABS( FAST_COS(approach_ang) * zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0]  + FAST_ABS( FAST_SIN(approach_ang) * zone_x_length);
        
        on_zone_pt_x[2] = on_zone_pt_x[1] + FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[2] = on_zone_pt_y[1] + FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
        
        on_zone_pt_x[3] = on_zone_pt_x[0] + FAST_ABS( FAST_SIN(approach_ang) * zone_y_length);
        on_zone_pt_y[3] = on_zone_pt_y[0] + FAST_ABS( FAST_COS(approach_ang) * zone_y_length);
        
    }
    
    return minus_offset;

}


#endif

#else
float32_T set_cta_off_zone_LT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                         float32_T            target_angle,
                         float32_T            target_length,
                         const CALIBRATION_CTA_T *p_cals,
                         float32_T            *off_zone_pt_x,
                         float32_T            *off_zone_pt_y)
{
    float32_T        host_length;
    float32_T        host_width;
    float32_T        on_zone_x_length;
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    float32_T        compensated_dyn_Y;
	const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;

    p_vehicle_data = p_CTA_input->vehicle_data;

#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE
    minus_offset = get_dynamic_minus_offset_by_heading(LEFT_TO_RIGHT, target_angle, p_vehicle_data->host_vehicle_width);
    on_zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room - minus_offset;
#else
    on_zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room;    
#endif

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    /*********************************/
    /*  OFF zone coordinate setting  */    
    ref_pt_x = (-1.0F)*(host_length - p_cals->k_cta_rear_offset);
    //coverity[misra_c_2012_rule_10_3_violation]  LEFT_TO_RIGHT's type casting is normally doing
    ref_pt_y = getCTA_OffLatLine(p_cals, host_width, target_length, LEFT_TO_RIGHT);

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = p_cals->k_cta_perpendicular_angle-p_cals->k_cta_perpendicular_angle_hys; /* 80  deg */
    upper_thr_90_angle = p_cals->k_cta_perpendicular_angle+p_cals->k_cta_perpendicular_angle_hys; /* 100 deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }
    
    compensated_dyn_Y = min(p_cals->k_cta_max_dyny, max(p_cals->k_cta_min_dyny, CTA_DIAGONAL_MAX_DYNY));/*AQB-271*/ /*EBE-157*/
    compensated_dyn_Y = compensated_dyn_Y - p_cals->k_cta_zone_hys_lr; /* zone max : 25m + (host_width/2) */

    //coverity[misra_c_2012_rule_10_3_violation]  target_angle's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
    if((target_angle > 0.F) && (target_angle <= p_cals->k_cta_perpendicular_angle)) /* 0 ~90 deg */
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_SIN(target_angle))*p_cals->k_cta_zone_hys_ud);
        off_zone_pt_y[0] = ref_pt_y - (FAST_ABS(FAST_COS(target_angle))*p_cals->k_cta_zone_hys_ud);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[1] = off_zone_pt_x[0] - (FAST_ABS(FAST_SIN(target_angle))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[1] = off_zone_pt_y[0] + (FAST_ABS(FAST_COS(target_angle))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[2] = off_zone_pt_x[1] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));
        off_zone_pt_y[2] = off_zone_pt_y[1] - (FAST_ABS(FAST_COS(PI_0_5-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[3] = off_zone_pt_x[2] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[3] = off_zone_pt_y[2] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
    }
    else
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_ud);
        off_zone_pt_y[0] = ref_pt_y + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_ud);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[1] = off_zone_pt_x[0] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[1] = off_zone_pt_y[0] - (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[2] = off_zone_pt_x[1] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));
        off_zone_pt_y[2] = off_zone_pt_y[1] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[3] = off_zone_pt_x[2] + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[3] = off_zone_pt_y[2] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
    }

    return minus_offset;
}

float32_T set_cta_on_zone_LT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                        float32_T            target_angle,
                        const CALIBRATION_CTA_T *p_cals,
                        float32_T            *on_zone_pt_x,
                        float32_T            *on_zone_pt_y)
{
    float32_T        host_length;    
    float32_T        host_width;    
    float32_T        on_zone_x_length;
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    float32_T        compensated_dyn_Y;
	const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;

    p_vehicle_data = p_CTA_input->vehicle_data;

#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE    
    minus_offset = get_dynamic_minus_offset_by_heading(LEFT_TO_RIGHT, target_angle, p_vehicle_data->host_vehicle_width);
    on_zone_x_length = p_cals->k_cta_zone_x - minus_offset;
#else
    on_zone_x_length = p_cals->k_cta_zone_x;/*AQB-271*/
#endif

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    /*********************************/
    /*  ON zone coordinate setting  */  
    ref_pt_x = (-1.0F)*(host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = (-1.0F)*(0.5F*host_width);

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = p_cals->k_cta_perpendicular_angle-p_cals->k_cta_perpendicular_angle_hys; /* 80  deg */
    upper_thr_90_angle = p_cals->k_cta_perpendicular_angle+p_cals->k_cta_perpendicular_angle_hys; /* 100 deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }
    
    compensated_dyn_Y = min(p_cals->k_cta_max_dyny, max(p_cals->k_cta_min_dyny, CTA_DIAGONAL_MAX_DYNY));/*AQB-271*/ /*EBE-157*/
    compensated_dyn_Y = compensated_dyn_Y - p_cals->k_cta_zone_hys_lr; /* zone max : 25m + (host_width/2) */
    //coverity[misra_c_2012_rule_10_3_violation]  target_angle's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
    if((target_angle > 0) && (target_angle <= p_cals->k_cta_perpendicular_angle)) /* 0 ~90 deg */
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[0] = ref_pt_x - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*p_cals->k_cta_zone_hys_lr);
        on_zone_pt_y[0] = ref_pt_y - (FAST_ABS(FAST_COS(PI_0_5-target_angle))*p_cals->k_cta_zone_hys_lr);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[1] = on_zone_pt_x[0] - (FAST_ABS(FAST_SIN(target_angle))*on_zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0] + (FAST_ABS(FAST_COS(target_angle))*on_zone_x_length);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[2] = on_zone_pt_x[1] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*compensated_dyn_Y);
        on_zone_pt_y[2] = on_zone_pt_y[1] - (FAST_ABS(FAST_COS(PI_0_5-target_angle))*compensated_dyn_Y);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[3] = on_zone_pt_x[2] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*on_zone_x_length);
        on_zone_pt_y[3] = on_zone_pt_y[2] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*on_zone_x_length);
    }
    else
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_lr);
        on_zone_pt_y[0] = ref_pt_y - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_lr);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[1] = on_zone_pt_x[0] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*on_zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0] - (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*on_zone_x_length);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[2] = on_zone_pt_x[1] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*compensated_dyn_Y);
        on_zone_pt_y[2] = on_zone_pt_y[1] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*compensated_dyn_Y);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[3] = on_zone_pt_x[2] + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*on_zone_x_length);
        on_zone_pt_y[3] = on_zone_pt_y[2] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*on_zone_x_length);
    }
    
    return minus_offset;
}

float32_T set_cta_off_zone_RT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                         float32_T            target_angle,
                         float32_T            target_length,
                         const CALIBRATION_CTA_T *p_cals,
                         float32_T            *off_zone_pt_x,
                         float32_T            *off_zone_pt_y)
{
    float32_T        host_length; 
    float32_T        host_width;
    float32_T        on_zone_x_length;
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    float32_T        compensated_dyn_Y;
	const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;

    p_vehicle_data = p_CTA_input->vehicle_data;

#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE  
    minus_offset = get_dynamic_minus_offset_by_heading(RIGHT_TO_LEFT, target_angle, p_vehicle_data->host_vehicle_width);
    on_zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room - minus_offset;
#else
    on_zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room;
#endif

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    /*********************************/
    /*  OFF zone coordinate setting  */
    ref_pt_x = (-1.0F)*(host_length - p_cals->k_cta_rear_offset);
    //coverity[misra_c_2012_rule_10_3_violation]  RIGHT_TO_LEFT's type casting is normally doing
    ref_pt_y = getCTA_OffLatLine(p_cals, host_width, target_length, RIGHT_TO_LEFT);

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)-p_cals->k_cta_perpendicular_angle_hys; /* -100 deg */
    upper_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)+p_cals->k_cta_perpendicular_angle_hys; /* -80  deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }

    compensated_dyn_Y = min(p_cals->k_cta_max_dyny, max(p_cals->k_cta_min_dyny, CTA_DIAGONAL_MAX_DYNY));/*AQB-271*/ /*EBE-157*/
    compensated_dyn_Y = compensated_dyn_Y - p_cals->k_cta_zone_hys_lr; /* zone max : 25m + (host_width/2) */

    //coverity[misra_c_2012_rule_10_3_violation]  target_angle's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
    if((target_angle >= (-1.0F*p_cals->k_cta_perpendicular_angle)) && (target_angle < 0.F)) /* -90 ~ 0 */
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_SIN(target_angle))*p_cals->k_cta_zone_hys_ud);
        off_zone_pt_y[0] = ref_pt_y + (FAST_ABS(FAST_COS(target_angle))*p_cals->k_cta_zone_hys_ud);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[1] = off_zone_pt_x[0] - (FAST_ABS(FAST_SIN(target_angle))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[1] = off_zone_pt_y[0] - (FAST_ABS(FAST_COS(target_angle))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[2] = off_zone_pt_x[1] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));
        off_zone_pt_y[2] = off_zone_pt_y[1] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[3] = off_zone_pt_x[2] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[3] = off_zone_pt_y[2] + (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
    }
    else
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_ud);
        off_zone_pt_y[0] = ref_pt_y - (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_ud);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[1] = off_zone_pt_x[0] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[1] = off_zone_pt_y[0] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[2] = off_zone_pt_x[1] + (FAST_ABS(FAST_COS(PI-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));
        off_zone_pt_y[2] = off_zone_pt_y[1] + (FAST_ABS(FAST_SIN(PI-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[3] = off_zone_pt_x[2] + (FAST_ABS(FAST_SIN(PI-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[3] = off_zone_pt_y[2] - (FAST_ABS(FAST_COS(PI-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
    }
    
    return minus_offset;
}

float32_T set_cta_on_zone_RT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                        float32_T            target_angle,
                        const CALIBRATION_CTA_T *p_cals,
                        float32_T            *on_zone_pt_x,
                        float32_T            *on_zone_pt_y)
{
    float32_T        host_length;    
    float32_T        host_width;    
    float32_T        on_zone_x_length;
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    float32_T        compensated_dyn_Y;
	const VEHICLE_DATA_FLT_T* p_vehicle_data;
    float32_T        minus_offset=0.0F;

    p_vehicle_data = p_CTA_input->vehicle_data;

#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE  
    minus_offset = get_dynamic_minus_offset_by_heading(RIGHT_TO_LEFT, target_angle, p_vehicle_data->host_vehicle_width);
    on_zone_x_length = p_cals->k_cta_zone_x - minus_offset;
#else
    on_zone_x_length = p_cals->k_cta_zone_x;/*AQB-271*/
#endif

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    /*********************************/
    /*  ON zone coordinate setting  */       
    ref_pt_x = (-1.0F)*(host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = 0.5F*host_width;

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)-p_cals->k_cta_perpendicular_angle_hys; /* -100 deg */
    upper_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)+p_cals->k_cta_perpendicular_angle_hys; /* -80  deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }

    compensated_dyn_Y = min(p_cals->k_cta_max_dyny, max(p_cals->k_cta_min_dyny, CTA_DIAGONAL_MAX_DYNY));/*AQB-271*/ /*EBE-157*/
    compensated_dyn_Y = compensated_dyn_Y - p_cals->k_cta_zone_hys_lr; /* zone max : 25m + (host_width/2) */

    //coverity[misra_c_2012_rule_10_3_violation]  target_angle's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
    if((target_angle >= (-1.0F*p_cals->k_cta_perpendicular_angle)) && (target_angle < 0.F))
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[0] = ref_pt_x - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*p_cals->k_cta_zone_hys_lr);
        on_zone_pt_y[0] = ref_pt_y + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*p_cals->k_cta_zone_hys_lr);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[1] = on_zone_pt_x[0] - (FAST_ABS(FAST_SIN(target_angle))*on_zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0] - (FAST_ABS(FAST_COS(target_angle))*on_zone_x_length);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[2] = on_zone_pt_x[1] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*compensated_dyn_Y);
        on_zone_pt_y[2] = on_zone_pt_y[1] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*compensated_dyn_Y);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[3] = on_zone_pt_x[2] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*on_zone_x_length);
        on_zone_pt_y[3] = on_zone_pt_y[2] + (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*on_zone_x_length);
    }
    else
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_lr);
        on_zone_pt_y[0] = ref_pt_y + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_lr);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[1] = on_zone_pt_x[0] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*on_zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*on_zone_x_length);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[2] = on_zone_pt_x[1] + (FAST_ABS(FAST_COS(PI-target_angle))*compensated_dyn_Y);
        on_zone_pt_y[2] = on_zone_pt_y[1] + (FAST_ABS(FAST_SIN(PI-target_angle))*compensated_dyn_Y);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[3] = on_zone_pt_x[2] + (FAST_ABS(FAST_SIN(PI-target_angle))*on_zone_x_length);
        on_zone_pt_y[3] = on_zone_pt_y[2] - (FAST_ABS(FAST_COS(PI-target_angle))*on_zone_x_length);
    }
    
    return minus_offset;

}

#ifdef PC_RESIM
void RESIM_set_cta_off_zone_LT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                         float32_T            target_angle,
                         float32_T            target_length,
                         const CALIBRATION_CTA_T *p_cals,
                         float32_T            *off_zone_pt_x,
                         float32_T            *off_zone_pt_y)
{
    float32_T        host_length;
    float32_T        host_width;
    float32_T        on_zone_x_length;
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    float32_T        compensated_dyn_Y;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;

    p_vehicle_data = p_CTA_input->vehicle_data;

//#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE
//    on_zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room - get_dynamic_minus_offset_by_heading(LEFT_TO_RIGHT, target_angle, p_vehicle_data->host_vehicle_width);
//#else
    on_zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room;    
//#endif

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    /*********************************/
    /*  OFF zone coordinate setting  */    
    ref_pt_x = (-1.0F)*(host_length - p_cals->k_cta_rear_offset);
    //coverity[misra_c_2012_rule_10_3_violation]  LEFT_TO_RIGHT's type casting is normally doing
    ref_pt_y = getCTA_OffLatLine(p_cals, host_width, target_length, LEFT_TO_RIGHT);

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = p_cals->k_cta_perpendicular_angle-p_cals->k_cta_perpendicular_angle_hys; /* 80  deg */
    upper_thr_90_angle = p_cals->k_cta_perpendicular_angle+p_cals->k_cta_perpendicular_angle_hys; /* 100 deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }
    
    compensated_dyn_Y = min(p_cals->k_cta_max_dyny, max(p_cals->k_cta_min_dyny, CTA_DIAGONAL_MAX_DYNY));/*AQB-271*/ /*EBE-157*/
    compensated_dyn_Y = compensated_dyn_Y - p_cals->k_cta_zone_hys_lr; /* zone max : 25m + (host_width/2) */

    //coverity[misra_c_2012_rule_10_3_violation]  target_angle's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
    if((target_angle > 0.F) && (target_angle <= p_cals->k_cta_perpendicular_angle)) /* 0 ~90 deg */
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_SIN(target_angle))*p_cals->k_cta_zone_hys_ud);
        off_zone_pt_y[0] = ref_pt_y - (FAST_ABS(FAST_COS(target_angle))*p_cals->k_cta_zone_hys_ud);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[1] = off_zone_pt_x[0] - (FAST_ABS(FAST_SIN(target_angle))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[1] = off_zone_pt_y[0] + (FAST_ABS(FAST_COS(target_angle))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[2] = off_zone_pt_x[1] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));
        off_zone_pt_y[2] = off_zone_pt_y[1] - (FAST_ABS(FAST_COS(PI_0_5-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[3] = off_zone_pt_x[2] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[3] = off_zone_pt_y[2] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
    }
    else
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_ud);
        off_zone_pt_y[0] = ref_pt_y + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_ud);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[1] = off_zone_pt_x[0] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[1] = off_zone_pt_y[0] - (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[2] = off_zone_pt_x[1] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));
        off_zone_pt_y[2] = off_zone_pt_y[1] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[3] = off_zone_pt_x[2] + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[3] = off_zone_pt_y[2] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
    }

}

void RESIM_set_cta_on_zone_LT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                        float32_T            target_angle,
                        const CALIBRATION_CTA_T *p_cals,
                        float32_T            *on_zone_pt_x,
                        float32_T            *on_zone_pt_y)
{
    float32_T        host_length;    
    float32_T        host_width;    
    float32_T        on_zone_x_length;
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    float32_T        compensated_dyn_Y;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;

    p_vehicle_data = p_CTA_input->vehicle_data;

//#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE    
//    on_zone_x_length = p_cals->k_cta_zone_x - get_dynamic_minus_offset_by_heading(LEFT_TO_RIGHT, target_angle, p_vehicle_data->host_vehicle_width);
//#else
    on_zone_x_length = p_cals->k_cta_zone_x;/*AQB-271*/
//#endif

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    /*********************************/
    /*  ON zone coordinate setting  */  
    ref_pt_x = (-1.0F)*(host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = (-1.0F)*(0.5F*host_width);

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = p_cals->k_cta_perpendicular_angle-p_cals->k_cta_perpendicular_angle_hys; /* 80  deg */
    upper_thr_90_angle = p_cals->k_cta_perpendicular_angle+p_cals->k_cta_perpendicular_angle_hys; /* 100 deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }
    
    compensated_dyn_Y = min(p_cals->k_cta_max_dyny, max(p_cals->k_cta_min_dyny, CTA_DIAGONAL_MAX_DYNY));/*AQB-271*/ /*EBE-157*/
    compensated_dyn_Y = compensated_dyn_Y - p_cals->k_cta_zone_hys_lr; /* zone max : 25m + (host_width/2) */
    //coverity[misra_c_2012_rule_10_3_violation]  target_angle's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
    if((target_angle > 0) && (target_angle <= p_cals->k_cta_perpendicular_angle)) /* 0 ~90 deg */
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[0] = ref_pt_x - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*p_cals->k_cta_zone_hys_lr);
        on_zone_pt_y[0] = ref_pt_y - (FAST_ABS(FAST_COS(PI_0_5-target_angle))*p_cals->k_cta_zone_hys_lr);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[1] = on_zone_pt_x[0] - (FAST_ABS(FAST_SIN(target_angle))*on_zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0] + (FAST_ABS(FAST_COS(target_angle))*on_zone_x_length);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[2] = on_zone_pt_x[1] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*compensated_dyn_Y);
        on_zone_pt_y[2] = on_zone_pt_y[1] - (FAST_ABS(FAST_COS(PI_0_5-target_angle))*compensated_dyn_Y);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[3] = on_zone_pt_x[2] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*on_zone_x_length);
        on_zone_pt_y[3] = on_zone_pt_y[2] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*on_zone_x_length);
    }
    else
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_lr);
        on_zone_pt_y[0] = ref_pt_y - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_lr);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[1] = on_zone_pt_x[0] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*on_zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0] - (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*on_zone_x_length);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[2] = on_zone_pt_x[1] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*compensated_dyn_Y);
        on_zone_pt_y[2] = on_zone_pt_y[1] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*compensated_dyn_Y);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[3] = on_zone_pt_x[2] + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*on_zone_x_length);
        on_zone_pt_y[3] = on_zone_pt_y[2] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*on_zone_x_length);
    }
}

void RESIM_set_cta_off_zone_RT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                         float32_T            target_angle,
                         float32_T            target_length,
                         const CALIBRATION_CTA_T *p_cals,
                         float32_T            *off_zone_pt_x,
                         float32_T            *off_zone_pt_y)
{
    float32_T        host_length; 
    float32_T        host_width;
    float32_T        on_zone_x_length;
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    float32_T        compensated_dyn_Y;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;

    p_vehicle_data = p_CTA_input->vehicle_data;

//#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE    
//    on_zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room - get_dynamic_minus_offset_by_heading(RIGHT_TO_LEFT, target_angle, p_vehicle_data->host_vehicle_width);
//#else
    on_zone_x_length = p_cals->k_cta_zone_x + p_cals->k_cta_zone_x_room;
//#endif

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    /*********************************/
    /*  OFF zone coordinate setting  */
    ref_pt_x = (-1.0F)*(host_length - p_cals->k_cta_rear_offset);
    //coverity[misra_c_2012_rule_10_3_violation]  RIGHT_TO_LEFT's type casting is normally doing
    ref_pt_y = getCTA_OffLatLine(p_cals, host_width, target_length, RIGHT_TO_LEFT);

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)-p_cals->k_cta_perpendicular_angle_hys; /* -100 deg */
    upper_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)+p_cals->k_cta_perpendicular_angle_hys; /* -80  deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }

    compensated_dyn_Y = min(p_cals->k_cta_max_dyny, max(p_cals->k_cta_min_dyny, CTA_DIAGONAL_MAX_DYNY));/*AQB-271*/ /*EBE-157*/
    compensated_dyn_Y = compensated_dyn_Y - p_cals->k_cta_zone_hys_lr; /* zone max : 25m + (host_width/2) */

    //coverity[misra_c_2012_rule_10_3_violation]  target_angle's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
    if((target_angle >= (-1.0F*p_cals->k_cta_perpendicular_angle)) && (target_angle < 0.F)) /* -90 ~ 0 */
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_SIN(target_angle))*p_cals->k_cta_zone_hys_ud);
        off_zone_pt_y[0] = ref_pt_y + (FAST_ABS(FAST_COS(target_angle))*p_cals->k_cta_zone_hys_ud);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[1] = off_zone_pt_x[0] - (FAST_ABS(FAST_SIN(target_angle))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[1] = off_zone_pt_y[0] - (FAST_ABS(FAST_COS(target_angle))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[2] = off_zone_pt_x[1] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));
        off_zone_pt_y[2] = off_zone_pt_y[1] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[3] = off_zone_pt_x[2] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[3] = off_zone_pt_y[2] + (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
    }
    else
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_ud);
        off_zone_pt_y[0] = ref_pt_y - (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_ud);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[1] = off_zone_pt_x[0] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[1] = off_zone_pt_y[0] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*(on_zone_x_length+(p_cals->k_cta_zone_hys_ud*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[2] = off_zone_pt_x[1] + (FAST_ABS(FAST_COS(PI-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));
        off_zone_pt_y[2] = off_zone_pt_y[1] + (FAST_ABS(FAST_SIN(PI-target_angle))*(compensated_dyn_Y + (p_cals->k_cta_zone_hys_lr*2.0F)));

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        off_zone_pt_x[3] = off_zone_pt_x[2] + (FAST_ABS(FAST_SIN(PI-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
        off_zone_pt_y[3] = off_zone_pt_y[2] - (FAST_ABS(FAST_COS(PI-target_angle))*(on_zone_x_length + (p_cals->k_cta_zone_hys_ud*2.0F)));
    }
}

void RESIM_set_cta_on_zone_RT(const CTA_INPUT_T    *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
                        float32_T            target_angle,
                        const CALIBRATION_CTA_T *p_cals,
                        float32_T            *on_zone_pt_x,
                        float32_T            *on_zone_pt_y)
{
    float32_T        host_length;    
    float32_T        host_width;    
    float32_T        on_zone_x_length;
    float32_T        ref_pt_x;
    float32_T        ref_pt_y;
    float32_T        upper_thr_90_angle;
    float32_T        lower_thr_90_angle;
    float32_T        compensated_dyn_Y;
    const VEHICLE_DATA_FLT_T* p_vehicle_data;

    p_vehicle_data = p_CTA_input->vehicle_data;

//#ifdef RCCW_NEW_SLOPE_TDP_IMPROVE    
//    on_zone_x_length = p_cals->k_cta_zone_x - get_dynamic_minus_offset_by_heading(RIGHT_TO_LEFT, target_angle, p_vehicle_data->host_vehicle_width);
//#else
    on_zone_x_length = p_cals->k_cta_zone_x;/*AQB-271*/
//#endif

    host_length = p_vehicle_data->host_vehicle_length;
    host_width  = p_vehicle_data->host_vehicle_width;

    /*********************************/
    /*  ON zone coordinate setting  */       
    ref_pt_x = (-1.0F)*(host_length - p_cals->k_cta_rear_offset);
    ref_pt_y = 0.5F*host_width;

    /* angle hysteresis : -85~+95 ==> 90 deg */
    lower_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)-p_cals->k_cta_perpendicular_angle_hys; /* -100 deg */
    upper_thr_90_angle = ((-1.0F)*p_cals->k_cta_perpendicular_angle)+p_cals->k_cta_perpendicular_angle_hys; /* -80  deg */
    if((target_angle > lower_thr_90_angle) && (target_angle < upper_thr_90_angle))
    {
        target_angle = p_cals->k_cta_perpendicular_angle;
    }

    compensated_dyn_Y = min(p_cals->k_cta_max_dyny, max(p_cals->k_cta_min_dyny, CTA_DIAGONAL_MAX_DYNY));/*AQB-271*/ /*EBE-157*/
    compensated_dyn_Y = compensated_dyn_Y - p_cals->k_cta_zone_hys_lr; /* zone max : 25m + (host_width/2) */

    //coverity[misra_c_2012_rule_10_3_violation]  target_angle's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
    if((target_angle >= (-1.0F*p_cals->k_cta_perpendicular_angle)) && (target_angle < 0.F))
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[0] = ref_pt_x - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*p_cals->k_cta_zone_hys_lr);
        on_zone_pt_y[0] = ref_pt_y + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*p_cals->k_cta_zone_hys_lr);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[1] = on_zone_pt_x[0] - (FAST_ABS(FAST_SIN(target_angle))*on_zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0] - (FAST_ABS(FAST_COS(target_angle))*on_zone_x_length);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[2] = on_zone_pt_x[1] - (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*compensated_dyn_Y);
        on_zone_pt_y[2] = on_zone_pt_y[1] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*compensated_dyn_Y);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[3] = on_zone_pt_x[2] + (FAST_ABS(FAST_COS(PI_0_5-target_angle))*on_zone_x_length);
        on_zone_pt_y[3] = on_zone_pt_y[2] + (FAST_ABS(FAST_SIN(PI_0_5-target_angle))*on_zone_x_length);
    }
    else
    {
        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[0] = ref_pt_x + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_lr);
        on_zone_pt_y[0] = ref_pt_y + (FAST_ABS(FAST_COS(target_angle-PI_0_5))*p_cals->k_cta_zone_hys_lr);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[1] = on_zone_pt_x[0] - (FAST_ABS(FAST_COS(target_angle-PI_0_5))*on_zone_x_length);
        on_zone_pt_y[1] = on_zone_pt_y[0] + (FAST_ABS(FAST_SIN(target_angle-PI_0_5))*on_zone_x_length);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[2] = on_zone_pt_x[1] + (FAST_ABS(FAST_COS(PI-target_angle))*compensated_dyn_Y);
        on_zone_pt_y[2] = on_zone_pt_y[1] + (FAST_ABS(FAST_SIN(PI-target_angle))*compensated_dyn_Y);

        //coverity[misra_c_2012_rule_10_4_violation]  target_angle's type casting is normally doing
        on_zone_pt_x[3] = on_zone_pt_x[2] + (FAST_ABS(FAST_SIN(PI-target_angle))*on_zone_x_length);
        on_zone_pt_y[3] = on_zone_pt_y[2] - (FAST_ABS(FAST_COS(PI-target_angle))*on_zone_x_length);
    }

}
#endif
#endif

/**
* FUNCTION: initCTA
* This function is used to initialize the outputs, calibrations, and persistent values used in the CTA feature.
*
* \param[out] p_CTA_output
*
* \param[in]  p_CTA_cals
* \param[in]  cta_radar_position
*
* \return     void
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/
void initCTA(CTA_OUTPUT_T *p_CTA_output,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const CALIBRATION_CTA_T *p_CTA_cals)
{
	
	if (NULL == p_CTA_output)
	{/*polyspace RTE:UNR [Justified:Low] "intended"*/
        //coverity[misra_c_2012_rule_10_1_violation]  assert macro's type casting is normally doing
		assert(NULL == p_CTA_output);
	}
	else
	{
		memset(&p_CTA_output->error_flags, 0, sizeof(CTA_ERRORS_T));
	}

}/* end of initCTA function*/

void CTA_CheckActivation(const CTA_INPUT_T *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
                         CTA_OUTPUT_T *p_CTA_output,
                         CTA_PERSISTENT_T *p_CTA_persistent)
{ 
    //coverity[misra_c_2012_rule_10_4_violation]  CTA_OP_STATE_ACTIVE's type casting is normally doing
	if (CTA_OP_STATE_ACTIVE == p_CTA_input->FF_OP_State_CTA)
	{/*polyspace RTE:UNR [Justified:Low] "intended"*/
        //coverity[misra_c_2012_rule_10_3_violation]  f_cta_enabled's type casting is normally doing
		p_CTA_output->f_cta_enabled = TRUE;
	}
	else
	{
        //coverity[misra_c_2012_rule_10_3_violation]  f_cta_enabled's type casting is normally doing
		p_CTA_output->f_cta_enabled = FALSE;
	}

    switch(p_CTA_input->FF_OP_State_CTA)
    {
        //coverity[misra_c_2012_rule_10_3_violation]  CTA_OP_STATE_OFF's type casting is normally doing
        case CTA_OP_STATE_OFF:/*polyspace RTE:UNR [Justified:Low] "intended"*/
        //coverity[misra_c_2012_rule_10_3_violation]  CTA_OP_STATE_FAULT's type casting is normally doing
        case CTA_OP_STATE_FAULT:/*polyspace RTE:UNR [Justified:Low] "intended"*/
        {
            break;
        }
        //coverity[misra_c_2012_rule_10_3_violation]  CTA_OP_STATE_INACTIVE's type casting is normally doing
        case CTA_OP_STATE_INACTIVE:/*polyspace RTE:UNR [Justified:Low] "intended"*/
        {
            break;
        }
        //coverity[misra_c_2012_rule_10_3_violation]  CTA_OP_STATE_ACTIVE's type casting is normally doing
        case CTA_OP_STATE_ACTIVE:/*polyspace RTE:UNR [Justified:Low] "intended"*/
        {   
            if(0 == ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_rcca_forcibly_expand_by_n_gear)
            {
                /* [RCCA rev7] If value is set to '0', ff_op_rccw and ff_op_rcca will be turned off immediately by N gear(0:rcca revision 7 / 1:rcca revision is below 7) */
                p_CTA_output->feature_activated_rccw_bymdloutput = TRUE;
            }
            else
            {
                /* [below RCCA rev7] if value is set to '1', ff_op_rccw and ff_op_rcca will be expanded until all core2 rcca singal is turned off for N gear(0:rcca revision 7 / 1:rcca revision is below 7) */
                //coverity[misra_c_2012_rule_10_4_violation]  f_rcca_warn_lh's type casting is normally doing
                p_CTA_output->feature_activated_rccw_bymdloutput = (boolean_T)((FALSE==RCCA_output.f_rcca_braking) && (FALSE==RCCA_output.f_rcca_warn_lh) && (FALSE==RCCA_output.f_rcca_warn_rh) && (0==RCCA_output.rcca_func_state));
            }        
            break;
        }
        default:/* polyspace RTE:UNR [Justified:Low] " no affection" */
        {
            //coverity[misra_c_2012_rule_16_4_violation] intented empty code
            break;
        }
    }
}

/**
* FUNCTION: CTA
* This main function calls checkCTA subrountine which implements CTA feature
* - get feature enable states \n
* - reset CTA params when function disabled \n
* - check CTA when function enabled \n
*
* \param[in,out] p_CTA_output
* \param[in,out] CTA_PERSISTENT_T *p_CTA_persistent
*
* \param[in]  p_CTA_input
* \param[in]  p_cals
*
* \return	  void
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/
void CTA(const CTA_INPUT_T *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	CTA_OUTPUT_T *p_CTA_output,
	const CALIBRATION_CTA_T *p_cals,
	CTA_PERSISTENT_T *p_CTA_persistent)
{
    CTA_FrontOrRear = p_CTA_input->cta_radar_position;

    CTA_CheckActivation(p_CTA_input, p_CTA_output, p_CTA_persistent);
    //coverity[misra_c_2012_rule_10_4_violation]  f_cta_enabled's type casting is normally doing    
	if (FALSE == p_CTA_output->f_cta_enabled)
	{
        //coverity[misra_c_2012_rule_10_4_violation]  f_cta_prev_reset's type casting is normally doing    
		if (FALSE == p_CTA_persistent->f_cta_prev_reset)
		{
			resetCTA(p_CTA_output,
				p_CTA_persistent);

		}
		else
		{
			/* do nothing */
		}

		Fill_Debug_Output_With_Default_Values(p_CTA_output, p_cals);
	}
	else
	{/*polyspace RTE:UNR [Justified:Low] "intended"*/
     //coverity[misra_c_2012_rule_10_3_violation]  f_cta_prev_reset's type casting is normally doing
		p_CTA_persistent->f_cta_prev_reset = FALSE;
		checkCTA(p_CTA_input,
			p_CTA_output,
			p_CTA_persistent,
			p_cals);
			
	}
}/*end of CTA function*/

/**
* FUNCTION: Fill_Debug_Output_With_Default_Values
*
* This function fills the debug output with default values.
*
* \param[in] p_CTA_output
* \param[in]  p_cals
*
* \return         	void
*
* \remark
* \ServID         	xx
* \Reentrancy     	non-reentrant
* \Synchronism    	synchronous
* \Precondition   	none
* \Caveats        	none
* \Requirements
* \reqtrace{}
*/
static void Fill_Debug_Output_With_Default_Values(const CTA_OUTPUT_T *p_CTA_output,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const CALIBRATION_CTA_T *p_cals)
{
	ASGUI_DBG_WRITE_CTA_CAL_MAC(p_cals);
	ASGUI_DBG_WRITE_CTA_OUTPUT_MAC(p_CTA_output);
	ASGUI_DBG_WRITE_CTA_DEBUG_MAC();
}

/*
    원점(origin)        : 0,0
    회전할 대상(taget)  : target_x,y
    회전중심점(center)  : rotate_org_
x,y
    회전후의점(new pt)  : new_
x,y
*/
 void CTA_Track_rotate(float32_T* new_x, float32_T* new_y, float32_T target_x, float32_T target_y, float32_T rotate_org_x, float32_T rotate_org_y,  float32_T rotate_angle_rad )
 {
     float32_T cos_ang = FAST_COS( rotate_angle_rad );
     float32_T sin_ang = FAST_SIN( rotate_angle_rad );
 
     // 회전중심점 C가 원점  O와 일치하도록 회전할 점 T를 함께 평행이동
     target_x -= rotate_org_x, target_y -= rotate_org_y;
 
     // 원점 O에 대하여 회전할 점 T를 q라디안 만큼 회전
     *new_x  =  target_x *  cos_ang - target_y * sin_ang;
     *new_y =  target_y * cos_ang + target_x * sin_ang;
 
     // 원점 O가 원래의 회전 중심점 C와 일치하도록 회전된 점 N과 함께 이동
     *new_x += rotate_org_x, *new_y += rotate_org_y;
 } 


 /*===========================================================================*\
 * Local Functions Definitions
 \*===========================================================================*/
 /**
 * FUNCTION: CheckCTA_Single_Object
 * This function is used to check if a CTA alert should be given.  Basic conditions
 * are: Target OTG heading is within a calibratible range (ensures target is approaching
 * from the side)
 * Target is expected to cross host path within a certain time
 * Target is within the defined alert zone
 * Target is expected to pass near the rear of the host (i.e. the target will enter
 * the conflict zone)
 * Target has been mature for a certain number of counts while meeting these
 * conditions
 *
 * The Function only looks at a single object.
 *
 *  \param[in,out] max_level,
 *  \param[in,out] p_level_cals,
 *  \param[in,out] p_target_attributes,
 *  \param[in,out] p_CTA_persistent,
 *  \param[in,out] loop_idx_object_highest_criticality,
 *  \param[in,out] f_any_cta_alert_lvl,
 *  \param[in,out] min_cta_ttc_warn,
 *  \param[in,out] min_cta_ttc_alert
 *
 *  \param[out] loop_idx_relevant_object_objectList_warn,
 *  \param[out] p_CTA_output,
 *  \param[out] loop_idx_relevant_object_objectList_alert,
 *
 *  \param[in] p_target_attributes_array,
 *  \param[in] p_cals,
 *  \param[in] loop_index,
 *  \param[in] p_target_object_data,
 *  \param[in] p_vehicle_data,
 *  \param[in] p_heading_angle,
 *  \param[in] p_level_cals_backup,
 *  \param[in] sin_k_cta_min_park_angle,
 *  \param[in] cos_k_cta_min_park_angle,
 *  \param[in] p_transistion_vector_to_middle_of_bumper,
 *
 * \return	  void
 *
 * \remark
 * \ServID         xx
 * \Reentrancy     non-reentrant
 * \Synchronism    synchronous
 * \Precondition   none
 * \Caveats        none
 * \Requirements
 * \reqtrace{}{}
 */

static void CheckCTA_Single_Object(const CTA_INPUT_T *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/ /*polyspace CODE-METRIC:CALLS [Justified:Low] "acceptable called number"*/
	CTA_OUTPUT_T *p_CTA_output,
	CTA_PERSISTENT_T* p_CTA_persistent,
	const CALIBRATION_CTA_T *p_cals,
	CTA_Object_Data_T *p_object,
	const cta_heading_angle_t *p_heading_angle,
	Level_Logic_Calibration_T *p_level_cals,
	const Level_Logic_Calibration_T *p_level_cals_backup,
	const float32_T sin_k_cta_min_park_angle,
	const float32_T cos_k_cta_min_park_angle)
{
	/* constant number refering to the number of corners of the polygon above */
	const uint8_T num_corners_polygon = 4;
	const VEHICLE_DATA_FLT_T *p_vehicle_data = p_CTA_input->vehicle_data;

	uint8_T f_matured;
	uint8_T f_in_zone;
    uint8_T f_ttc_cond;    
	uint8_T f_rel_lat_vel;
	uint8_T is_warning;
	Vector_2d_T obj_relative_velocity;
	cta_zone_point_t cta_zone_obj;
	CTA_Target_Reference obj_ref_point;
	CTA_Advanced_Object_Corner target_corners;
    CTA_Advanced_Object_Corner rotated_target_corners;
    float32_T track_rotate_angle=0.F;
    float32_T abs_min_speed;
    float32_T lat_min_speed;    
	float32_T dyn_ttc=25.5F;
#ifdef BINARY_DEBUG 
	float32_T dyn_Y = 25.5F;    
#endif
	float32_T dyn_hys_Y;
    float32_T rel_vel_y=0.F;
    float32_T comp_relVelY_withAbsSpeed = 0.F;
    float32_T target_angle=0.F;
    float32_T target_length;
    float32_T off_zone_pt_x[4];
    float32_T off_zone_pt_y[4];
    float32_T on_zone_pt_x[4];
    float32_T on_zone_pt_y[4];
    float32_T minus_offset;
#ifdef PC_RESIM
    cta_zone_point_t RESIM_cta_zone_obj;

    float32_T RESIM_off_zone_pt_x[4];
    float32_T RESIM_off_zone_pt_y[4];
    float32_T RESIM_on_zone_pt_x[4];
    float32_T RESIM_on_zone_pt_y[4];
#endif

    boolean_T flag_track_latVel_vulner = FALSE;
    boolean_T flag_track_heading_vulner = FALSE;

    obj_relative_velocity.x = 0.F;
    obj_relative_velocity.y = 0.F;

    obj_ref_point.point.x = 0;
    obj_ref_point.point.y = 0;
    obj_ref_point.index = 0;
    obj_ref_point.neighbor1 = 0;
    obj_ref_point.neighbor2 = 0;
    obj_ref_point.neighbor3 = 0;
    
	/* check if target is valid candidate for cta alert */
	if (TRUE == Check_Object_Validity(p_object,
		p_cals,
		*p_heading_angle))
	{
		obj_relative_velocity = getRelativeVelocityObject(p_object->tracker_output,
			p_vehicle_data,
			p_cals);

        #if 0 // HKMC does'nt use
		/* recalculate relative velocity when pathHeading applys */
		if (TRUE == (boolean_T)p_cals->k_cta_f_apply_path_tracking)
		{
			obj_relative_velocity.x = (p_object->tracker_output->speed * FAST_COS(p_object->attributes->pathHeading)) - p_vehicle_data->speed;
			obj_relative_velocity.y = (p_object->tracker_output->speed * FAST_SIN(p_object->attributes->pathHeading));
		}
        #endif

		if (p_CTA_persistent->object_with_highest_crit[p_object->attributes->side].attributes == NULL)
		{
			Set_Object_highest_crit(&(p_CTA_persistent->object_with_highest_crit[p_object->attributes->side]), p_object);
		}

		/* calculation of the eight reference points of the current target and determenation of the most relevant reference point */
		calculateTargetCornersPoint(p_object,
			&target_corners);
		
		calculateTargetReferencePoint(p_object,
			p_vehicle_data->host_vehicle_length,
			&target_corners,
			&obj_ref_point,
			p_cals);
            
    #if defined(MAX_K_CTA_TRACKLATVEL_COMPUSE_IN_VULNERAREA)
        if(TRUE == p_cals->k_cta_TrackLatVel_CompUse_In_VulnerArea)
        {            
            /*************************************************************************************************************************************/
            /* 취약구역에서 생성된 트랙이고 && 트랙의 횡위치가 3.5m안쪽이고 && 이전에 경고하던 트랙이면 tracker->latvel대신 트랙의 절대속을 사용 */
            //if((TRUE == p_object->attributes->track_created_vulnerable_area) && 
            //    (FAST_ABS(p_object->tracker_output->vcs.position.y) < p_cals->k_cta_TrackLatVel_Comp_LatDistance) &&
            //    (0<getHysteresisLevel(p_object)))
            if((TRUE == p_object->attributes->track_created_vulnerable_area) && 
                (FAST_ABS(p_object->tracker_output->vcs.position.y) < p_cals->k_cta_TrackLatVel_Comp_LatDistance)) // < 3m
            {                
                flag_track_latVel_vulner = TRUE;
                
                comp_relVelY_withAbsSpeed = p_object->tracker_output->speed;
                
                if(obj_relative_velocity.y<0)
                {
                    comp_relVelY_withAbsSpeed = p_object->tracker_output->speed * -1.0F;
                }

                p_object->attributes->ttc = calculateTimeToConflict(comp_relVelY_withAbsSpeed,
                                                                    &obj_ref_point,
                                                                    p_object,
                                                                    p_vehicle_data,
                                                                    p_cals);
                rel_vel_y = comp_relVelY_withAbsSpeed;                
            }
            else
            {
                p_object->attributes->ttc = calculateTimeToConflict(obj_relative_velocity.y,
                                                                    &obj_ref_point,
                                                                    p_object,
                                                                    p_vehicle_data,
                                                                    p_cals);                
                rel_vel_y = obj_relative_velocity.y;
            }            
        }
        else
        {
            p_object->attributes->ttc = calculateTimeToConflict(obj_relative_velocity.y,
                                                                &obj_ref_point,
                                                                p_object,
                                                                p_vehicle_data,
                                                                p_cals);  
            
            /*  LATE  > CLU19.NValueSet == 1
                        Switch_status_final_USM=??
                        Short TTC = Rev_vel*(-0.144) + 3.3 
                NORMAL> CLU19.NValueSet == 2
                        Switch_status_final_USM=??
                        long TTC = Rev_vel*(-0.249) + 4.25
                EARLY > CLU19.NValueSet == 3
                        Switch_status_final_USM=??
                        long TTC = Rev_vel*(-0.249) + 4.25 
            */
            rel_vel_y = obj_relative_velocity.y;

        }    
    #else
        {
            p_object->attributes->ttc = calculateTimeToConflict(obj_relative_velocity.y,
                                                                &obj_ref_point,
                                                                p_object,
                                                                p_vehicle_data,
                                                                p_cals);  
            
            /*  LATE  > CLU19.NValueSet == 1
                        Switch_status_final_USM=??
                        Short TTC = Rev_vel*(-0.144) + 3.3 
                NORMAL> CLU19.NValueSet == 2
                        Switch_status_final_USM=??
                        long TTC = Rev_vel*(-0.249) + 4.25
                EARLY > CLU19.NValueSet == 3
                        Switch_status_final_USM=??
                        long TTC = Rev_vel*(-0.249) + 4.25 
            */
            rel_vel_y = obj_relative_velocity.y;

        }
    #endif
    
        dyn_ttc = calculate_dyn_ttc(p_CTA_input->Switch_status_final_USM, rel_vel_y, p_cals);
        
        if (rel_vel_y < 0.F)
        {
           rel_vel_y = -rel_vel_y;
        }
            //coverity[misra_c_2012_rule_10_1_violation]  f_cta_warn_right's type casting is normally doing
        if(p_CTA_output->f_cta_warn_left || p_CTA_output->f_cta_alert_left || p_CTA_output->f_cta_warn_right || p_CTA_output->f_cta_alert_right)
        {
            /* TTC threshold hysterisis */
            //coverity[misra_c_2012_rule_10_4_violation]  rel_vel_y's type casting is normally doing
            if(FAST_ABS(rel_vel_y) < p_cals->k_cta_dynttc_hys_thres_lowspd)
            {
              dyn_ttc += p_cals->k_cta_dynttc_hys_under_lowspd;
            }
            else
            {
              dyn_ttc += p_cals->k_cta_dynttc_hys_over_lowspd;
            }
        }

    #ifdef BINARY_DEBUG 
        dyn_Y = dyn_ttc * rel_vel_y;
    #endif
        
		/* Store prev cta alert suppress flag in order to avoid toggling */
        //coverity[misra_c_2012_rule_10_4_violation]  prev_cta_alert_suppress_counter's type casting is normally doing
		if ((p_object->attributes->ttc < p_cals->k_cta_stop_alert_ttc) && (0 == p_object->persistent->prev_cta_alert_suppress_counter))
		{
            //coverity[misra_c_2012_rule_10_3_violation]  prev_cta_alert_suppress's type casting is normally doing
			p_object->persistent->prev_cta_alert_suppress = TRUE;
            //coverity[misra_c_2012_rule_10_4_violation]  prev_cta_alert_suppress_counter's type casting is normally doing
			p_object->persistent->prev_cta_alert_suppress_counter += 1;
		}
        //coverity[misra_c_2012_rule_10_4_violation]  prev_cta_alert_suppress's type casting is normally doing
		if ((p_object->attributes->ttc >= p_cals->k_cta_stop_alert_ttc) && (FALSE == p_object->persistent->prev_cta_alert_suppress)) // maybe hysteresis is needed to prevent toggling of warnings/alerts - needs to be added yet
		{
            //coverity[misra_c_2012_rule_10_4_violation]  k_cta_f_adapt_intersect_lines_by_obj_heading's type casting is normally doing
			if (TRUE == p_cals->k_cta_f_adapt_intersect_lines_by_obj_heading)
			{
				/* restore backed up level logic cals, as they have been modified by
				previous loop cycle for previous object */
				memcpy(p_level_cals, p_level_cals_backup, sizeof(Level_Logic_Calibration_T));

                #if 0 // HKMC does'nt use
                 //coverity[misra_c_2012_rule_10_4_violation]  k_cta_f_apply_path_tracking's type casting is normally doing
				if (TRUE == p_cals->k_cta_f_apply_path_tracking)
				{
					adaptIntersectLinesToObjectHeading(p_level_cals, p_cals, p_object->attributes->pathHeading, sin_k_cta_min_park_angle);
				}
				else
                #endif
				{
					adaptIntersectLinesToObjectHeading(p_level_cals, p_cals, p_object->tracker_output->heading, sin_k_cta_min_park_angle);
				}
			}

			cta_zone_obj = p_CTA_persistent->cta_zone;
            #if 0 /* HKMC zone is dymaic Y left and right, so dont need Y flilp zone */
			flipCTAzone(p_object->attributes->side,
				&cta_zone_obj,
				p_cals,
				p_vehicle_data);
            #endif
        
        #if defined(MAX_K_CTA_TRACKHEADING_COMPUSE_IN_VULNERAREA)
            if(TRUE == p_cals->k_cta_TrackHeading_CompUse_In_VulnerArea)
            {
                /*******************************************************************************************************/
                /* 취약구역에서 생성된 트랙이고 && 트랙의 age가 20이상이면 트래커 헤딩대신 vx, vy로 계산한 헤딩을 사용 */
                if((TRUE==p_object->attributes->track_created_vulnerable_area) && (p_object->tracker_output->age>p_cals->k_cta_VULNERABLE_CRITERIA_AGE)) // k_cta_VULNERABLE_CRITERIA_AGE=5
                {
                    flag_track_heading_vulner = TRUE;
                    
                    target_angle = p_object->attributes->heading_rad_by_vxvy_LPF;

                    track_rotate_angle = p_object->attributes->heading_rad_by_vxvy_LPF - p_object->tracker_output->heading;
                    
                    CTA_Track_rotate(&rotated_target_corners.points[0].x, &rotated_target_corners.points[0].y, target_corners.points[0].x, target_corners.points[0].y, 
                                     p_object->tracker_output->vcs.position.x, p_object->tracker_output->vcs.position.y, track_rotate_angle);
                    CTA_Track_rotate(&rotated_target_corners.points[1].x, &rotated_target_corners.points[1].y, target_corners.points[1].x, target_corners.points[1].y, 
                                     p_object->tracker_output->vcs.position.x, p_object->tracker_output->vcs.position.y, track_rotate_angle);
                    CTA_Track_rotate(&rotated_target_corners.points[2].x, &rotated_target_corners.points[2].y, target_corners.points[2].x, target_corners.points[2].y, 
                                     p_object->tracker_output->vcs.position.x, p_object->tracker_output->vcs.position.y, track_rotate_angle);
                    CTA_Track_rotate(&rotated_target_corners.points[3].x, &rotated_target_corners.points[3].y, target_corners.points[3].x, target_corners.points[3].y, 
                                     p_object->tracker_output->vcs.position.x, p_object->tracker_output->vcs.position.y, track_rotate_angle);
                    CTA_Track_rotate(&rotated_target_corners.points[4].x, &rotated_target_corners.points[4].y, target_corners.points[4].x, target_corners.points[4].y, 
                                     p_object->tracker_output->vcs.position.x, p_object->tracker_output->vcs.position.y, track_rotate_angle);
                    CTA_Track_rotate(&rotated_target_corners.points[5].x, &rotated_target_corners.points[5].y, target_corners.points[5].x, target_corners.points[5].y, 
                                     p_object->tracker_output->vcs.position.x, p_object->tracker_output->vcs.position.y, track_rotate_angle);
                    CTA_Track_rotate(&rotated_target_corners.points[6].x, &rotated_target_corners.points[6].y, target_corners.points[6].x, target_corners.points[6].y, 
                                     p_object->tracker_output->vcs.position.x, p_object->tracker_output->vcs.position.y, track_rotate_angle);
                    CTA_Track_rotate(&rotated_target_corners.points[7].x, &rotated_target_corners.points[7].y, target_corners.points[7].x, target_corners.points[7].y, 
                                     p_object->tracker_output->vcs.position.x, p_object->tracker_output->vcs.position.y, track_rotate_angle);
                }
                else
                {
                    target_angle = p_object->tracker_output->heading;
                }
            }
            else
            {
                target_angle = p_object->tracker_output->heading;
            }
        #else
            {
                target_angle = p_object->tracker_output->heading;
            }
        #endif
        
        #ifdef RCCW_NEW_SLOPE_TDP_IMPROVE
            if(1==k_cta_LPF_USE_dynamiczone_heading)
            {
                target_angle = cta_dynamiczone_heading_filteredLPF(p_object->attributes->side, target_angle);
            }else{/* empty */}
        #endif

            target_length = p_object->tracker_output->length;

             /*  cta_zone_obj
                 [1]--------[0]--------[1]
                  |  left    |  right   |
                 [2]--------[3]--------[2] */   
            
            /* dynamic y setting */ 
            if(p_object->attributes->side == LeftSide)
            { 
                //coverity[misra_c_2012_rule_10_1_violation]  f_cta_warn_left's type casting is normally doing
                if(p_CTA_output->f_cta_alert_left || p_CTA_output->f_cta_warn_left)
                {                    
                    /**************************(*****************
                       off_zone_pt(hysterisis) coordinate setting
                       [0]--------[3]   [0]--------[3]
                        |  left    |     |   right  |
                       [1]--------[2]   [1]--------[2]                */   
                    minus_offset = set_cta_off_zone_LT(p_CTA_input, target_angle, target_length, p_cals, off_zone_pt_x, off_zone_pt_y);

                    cta_zone_obj.points[1].x = off_zone_pt_x[0];                    
                    cta_zone_obj.points[1].y = off_zone_pt_y[0];
                    cta_zone_obj.points[2].x = off_zone_pt_x[1];  
                    cta_zone_obj.points[2].y = off_zone_pt_y[1];
                    cta_zone_obj.points[3].x = off_zone_pt_x[2];  
                    cta_zone_obj.points[3].y = off_zone_pt_y[2];
                    cta_zone_obj.points[0].x = off_zone_pt_x[3];                      
                    cta_zone_obj.points[0].y = off_zone_pt_y[3];

                    /*For ASGUI*/
                    p_CTA_output->cta_off_zone_left.points[0].x = cta_zone_obj.points[0].x;
                    p_CTA_output->cta_off_zone_left.points[0].y = cta_zone_obj.points[0].y;
                    p_CTA_output->cta_off_zone_left.points[1].x = cta_zone_obj.points[1].x;
                    p_CTA_output->cta_off_zone_left.points[1].y = cta_zone_obj.points[1].y;
                    p_CTA_output->cta_off_zone_left.points[2].x = cta_zone_obj.points[2].x;
                    p_CTA_output->cta_off_zone_left.points[2].y = cta_zone_obj.points[2].y;
                    p_CTA_output->cta_off_zone_left.points[3].x = cta_zone_obj.points[3].x;
                    p_CTA_output->cta_off_zone_left.points[3].y = cta_zone_obj.points[3].y;

                    p_CTA_output->minusoffset_cta_off_zone_left = minus_offset;

                #ifdef PC_RESIM //ORG 6M zone
                    RESIM_set_cta_off_zone_LT(p_CTA_input, target_angle, target_length, p_cals, RESIM_off_zone_pt_x, RESIM_off_zone_pt_y);
                
                    RESIM_cta_zone_obj.points[1].x = RESIM_off_zone_pt_x[0];                    
                    RESIM_cta_zone_obj.points[1].y = RESIM_off_zone_pt_y[0];
                    RESIM_cta_zone_obj.points[2].x = RESIM_off_zone_pt_x[1];  
                    RESIM_cta_zone_obj.points[2].y = RESIM_off_zone_pt_y[1];
                    RESIM_cta_zone_obj.points[3].x = RESIM_off_zone_pt_x[2];  
                    RESIM_cta_zone_obj.points[3].y = RESIM_off_zone_pt_y[2];
                    RESIM_cta_zone_obj.points[0].x = RESIM_off_zone_pt_x[3];                      
                    RESIM_cta_zone_obj.points[0].y = RESIM_off_zone_pt_y[3];
                    
                    /*For ASGUI*/
                    p_CTA_output->RESIM_cta_off_zone_left.points[0].x = RESIM_cta_zone_obj.points[0].x;
                    p_CTA_output->RESIM_cta_off_zone_left.points[0].y = RESIM_cta_zone_obj.points[0].y;
                    p_CTA_output->RESIM_cta_off_zone_left.points[1].x = RESIM_cta_zone_obj.points[1].x;
                    p_CTA_output->RESIM_cta_off_zone_left.points[1].y = RESIM_cta_zone_obj.points[1].y;
                    p_CTA_output->RESIM_cta_off_zone_left.points[2].x = RESIM_cta_zone_obj.points[2].x;
                    p_CTA_output->RESIM_cta_off_zone_left.points[2].y = RESIM_cta_zone_obj.points[2].y;
                    p_CTA_output->RESIM_cta_off_zone_left.points[3].x = RESIM_cta_zone_obj.points[3].x;
                    p_CTA_output->RESIM_cta_off_zone_left.points[3].y = RESIM_cta_zone_obj.points[3].y;
                #endif
                }
                else
                {
                    /*********************************
                      on_zone_pt coordinate setting
                       [0]--------[3]   [0]--------[3]
                        |  left    |     |   right  |
                       [1]--------[2]   [1]--------[2]                */   
                    minus_offset = set_cta_on_zone_LT(p_CTA_input, target_angle, p_cals, on_zone_pt_x, on_zone_pt_y); 

                    cta_zone_obj.points[1].x = on_zone_pt_x[0];                    
                    cta_zone_obj.points[1].y = on_zone_pt_y[0];
                    cta_zone_obj.points[2].x = on_zone_pt_x[1];  
                    cta_zone_obj.points[2].y = on_zone_pt_y[1];
                    cta_zone_obj.points[3].x = on_zone_pt_x[2];  
                    cta_zone_obj.points[3].y = on_zone_pt_y[2];
                    cta_zone_obj.points[0].x = on_zone_pt_x[3];                      
                    cta_zone_obj.points[0].y = on_zone_pt_y[3];

                    /*For ASGUI*/
                    p_CTA_output->cta_on_zone_left.points[0].x = cta_zone_obj.points[0].x;
                    p_CTA_output->cta_on_zone_left.points[0].y = cta_zone_obj.points[0].y;
                    p_CTA_output->cta_on_zone_left.points[1].x = cta_zone_obj.points[1].x;
                    p_CTA_output->cta_on_zone_left.points[1].y = cta_zone_obj.points[1].y;
                    p_CTA_output->cta_on_zone_left.points[2].x = cta_zone_obj.points[2].x;
                    p_CTA_output->cta_on_zone_left.points[2].y = cta_zone_obj.points[2].y;
                    p_CTA_output->cta_on_zone_left.points[3].x = cta_zone_obj.points[3].x;
                    p_CTA_output->cta_on_zone_left.points[3].y = cta_zone_obj.points[3].y;  
                    
                    p_CTA_output->minusoffset_cta_on_zone_left = minus_offset;

                #ifdef PC_RESIM //ORG 6M zone
                    RESIM_set_cta_on_zone_LT(p_CTA_input, target_angle, p_cals, RESIM_on_zone_pt_x, RESIM_on_zone_pt_y);
                
                    RESIM_cta_zone_obj.points[1].x = RESIM_on_zone_pt_x[0];                    
                    RESIM_cta_zone_obj.points[1].y = RESIM_on_zone_pt_y[0];
                    RESIM_cta_zone_obj.points[2].x = RESIM_on_zone_pt_x[1];  
                    RESIM_cta_zone_obj.points[2].y = RESIM_on_zone_pt_y[1];
                    RESIM_cta_zone_obj.points[3].x = RESIM_on_zone_pt_x[2];  
                    RESIM_cta_zone_obj.points[3].y = RESIM_on_zone_pt_y[2];
                    RESIM_cta_zone_obj.points[0].x = RESIM_on_zone_pt_x[3];                      
                    RESIM_cta_zone_obj.points[0].y = RESIM_on_zone_pt_y[3];
                    
                    /*For ASGUI*/
                    p_CTA_output->RESIM_cta_on_zone_left.points[0].x = RESIM_cta_zone_obj.points[0].x;
                    p_CTA_output->RESIM_cta_on_zone_left.points[0].y = RESIM_cta_zone_obj.points[0].y;
                    p_CTA_output->RESIM_cta_on_zone_left.points[1].x = RESIM_cta_zone_obj.points[1].x;
                    p_CTA_output->RESIM_cta_on_zone_left.points[1].y = RESIM_cta_zone_obj.points[1].y;
                    p_CTA_output->RESIM_cta_on_zone_left.points[2].x = RESIM_cta_zone_obj.points[2].x;
                    p_CTA_output->RESIM_cta_on_zone_left.points[2].y = RESIM_cta_zone_obj.points[2].y;
                    p_CTA_output->RESIM_cta_on_zone_left.points[3].x = RESIM_cta_zone_obj.points[3].x;
                    p_CTA_output->RESIM_cta_on_zone_left.points[3].y = RESIM_cta_zone_obj.points[3].y;
                #endif
                    
                }
            }

            if (p_object->attributes->side == RightSide)
            { 
                //coverity[misra_c_2012_rule_10_1_violation]  f_cta_warn_right's type casting is normally doing
                if(p_CTA_output->f_cta_alert_right || p_CTA_output->f_cta_warn_right)
                {
                    /*********************************/
                    /*  OFF zone coordinate setting  */                        
                    minus_offset = set_cta_off_zone_RT(p_CTA_input, target_angle, target_length, p_cals, off_zone_pt_x, off_zone_pt_y); 

                    cta_zone_obj.points[0].x = off_zone_pt_x[0];                    
                    cta_zone_obj.points[0].y = off_zone_pt_y[0];
                    cta_zone_obj.points[3].x = off_zone_pt_x[1];  
                    cta_zone_obj.points[3].y = off_zone_pt_y[1];
                    cta_zone_obj.points[2].x = off_zone_pt_x[2];  
                    cta_zone_obj.points[2].y = off_zone_pt_y[2];
                    cta_zone_obj.points[1].x = off_zone_pt_x[3];
                    cta_zone_obj.points[1].y = off_zone_pt_y[3];                    

                    /*For ASGUI*/
                    p_CTA_output->cta_off_zone_right.points[0].x = cta_zone_obj.points[0].x;
                    p_CTA_output->cta_off_zone_right.points[0].y = cta_zone_obj.points[0].y;
                    p_CTA_output->cta_off_zone_right.points[1].x = cta_zone_obj.points[1].x;
                    p_CTA_output->cta_off_zone_right.points[1].y = cta_zone_obj.points[1].y;
                    p_CTA_output->cta_off_zone_right.points[2].x = cta_zone_obj.points[2].x;
                    p_CTA_output->cta_off_zone_right.points[2].y = cta_zone_obj.points[2].y;
                    p_CTA_output->cta_off_zone_right.points[3].x = cta_zone_obj.points[3].x;
                    p_CTA_output->cta_off_zone_right.points[3].y = cta_zone_obj.points[3].y;

                    p_CTA_output->minusoffset_cta_off_zone_right = minus_offset;

                #ifdef PC_RESIM //ORG 6M zone
                    RESIM_set_cta_off_zone_RT(p_CTA_input, target_angle, target_length, p_cals, RESIM_off_zone_pt_x, RESIM_off_zone_pt_y);
                
                    RESIM_cta_zone_obj.points[1].x = RESIM_off_zone_pt_x[0];                    
                    RESIM_cta_zone_obj.points[1].y = RESIM_off_zone_pt_y[0];
                    RESIM_cta_zone_obj.points[2].x = RESIM_off_zone_pt_x[1];  
                    RESIM_cta_zone_obj.points[2].y = RESIM_off_zone_pt_y[1];
                    RESIM_cta_zone_obj.points[3].x = RESIM_off_zone_pt_x[2];  
                    RESIM_cta_zone_obj.points[3].y = RESIM_off_zone_pt_y[2];
                    RESIM_cta_zone_obj.points[0].x = RESIM_off_zone_pt_x[3];                      
                    RESIM_cta_zone_obj.points[0].y = RESIM_off_zone_pt_y[3];
                    
                    /*For ASGUI*/
                    p_CTA_output->RESIM_cta_off_zone_right.points[0].x = RESIM_cta_zone_obj.points[0].x;
                    p_CTA_output->RESIM_cta_off_zone_right.points[0].y = RESIM_cta_zone_obj.points[0].y;
                    p_CTA_output->RESIM_cta_off_zone_right.points[1].x = RESIM_cta_zone_obj.points[1].x;
                    p_CTA_output->RESIM_cta_off_zone_right.points[1].y = RESIM_cta_zone_obj.points[1].y;
                    p_CTA_output->RESIM_cta_off_zone_right.points[2].x = RESIM_cta_zone_obj.points[2].x;
                    p_CTA_output->RESIM_cta_off_zone_right.points[2].y = RESIM_cta_zone_obj.points[2].y;
                    p_CTA_output->RESIM_cta_off_zone_right.points[3].x = RESIM_cta_zone_obj.points[3].x;
                    p_CTA_output->RESIM_cta_off_zone_right.points[3].y = RESIM_cta_zone_obj.points[3].y;
                #endif
                    
                }
                else
                {
                    /*********************************/
                    /*  ON zone coordinate setting  */ 
                    minus_offset = set_cta_on_zone_RT(p_CTA_input, target_angle, p_cals, on_zone_pt_x, on_zone_pt_y);

                    cta_zone_obj.points[0].x = on_zone_pt_x[0];                    
                    cta_zone_obj.points[0].y = on_zone_pt_y[0];
                    cta_zone_obj.points[3].x = on_zone_pt_x[1];  
                    cta_zone_obj.points[3].y = on_zone_pt_y[1];
                    cta_zone_obj.points[2].x = on_zone_pt_x[2];  
                    cta_zone_obj.points[2].y = on_zone_pt_y[2];
                    cta_zone_obj.points[1].x = on_zone_pt_x[3];  
                    cta_zone_obj.points[1].y = on_zone_pt_y[3];                                        

                    /*For ASGUI*/
                    p_CTA_output->cta_on_zone_right.points[0].x = cta_zone_obj.points[0].x;
                    p_CTA_output->cta_on_zone_right.points[0].y = cta_zone_obj.points[0].y;
                    p_CTA_output->cta_on_zone_right.points[1].x = cta_zone_obj.points[1].x;
                    p_CTA_output->cta_on_zone_right.points[1].y = cta_zone_obj.points[1].y;
                    p_CTA_output->cta_on_zone_right.points[2].x = cta_zone_obj.points[2].x;
                    p_CTA_output->cta_on_zone_right.points[2].y = cta_zone_obj.points[2].y;
                    p_CTA_output->cta_on_zone_right.points[3].x = cta_zone_obj.points[3].x;
                    p_CTA_output->cta_on_zone_right.points[3].y = cta_zone_obj.points[3].y;

                    p_CTA_output->minusoffset_cta_on_zone_right = minus_offset;
                    
                #ifdef PC_RESIM //ORG 6M zone
                    RESIM_set_cta_on_zone_RT(p_CTA_input, target_angle, p_cals, RESIM_on_zone_pt_x, RESIM_on_zone_pt_y);

                    RESIM_cta_zone_obj.points[1].x = RESIM_on_zone_pt_x[0];                    
                    RESIM_cta_zone_obj.points[1].y = RESIM_on_zone_pt_y[0];
                    RESIM_cta_zone_obj.points[2].x = RESIM_on_zone_pt_x[1];  
                    RESIM_cta_zone_obj.points[2].y = RESIM_on_zone_pt_y[1];
                    RESIM_cta_zone_obj.points[3].x = RESIM_on_zone_pt_x[2];  
                    RESIM_cta_zone_obj.points[3].y = RESIM_on_zone_pt_y[2];
                    RESIM_cta_zone_obj.points[0].x = RESIM_on_zone_pt_x[3];                      
                    RESIM_cta_zone_obj.points[0].y = RESIM_on_zone_pt_y[3];
                    
                    /*For ASGUI*/
                    p_CTA_output->RESIM_cta_on_zone_right.points[0].x = RESIM_cta_zone_obj.points[0].x;
                    p_CTA_output->RESIM_cta_on_zone_right.points[0].y = RESIM_cta_zone_obj.points[0].y;
                    p_CTA_output->RESIM_cta_on_zone_right.points[1].x = RESIM_cta_zone_obj.points[1].x;
                    p_CTA_output->RESIM_cta_on_zone_right.points[1].y = RESIM_cta_zone_obj.points[1].y;
                    p_CTA_output->RESIM_cta_on_zone_right.points[2].x = RESIM_cta_zone_obj.points[2].x;
                    p_CTA_output->RESIM_cta_on_zone_right.points[2].y = RESIM_cta_zone_obj.points[2].y;
                    p_CTA_output->RESIM_cta_on_zone_right.points[3].x = RESIM_cta_zone_obj.points[3].x;
                    p_CTA_output->RESIM_cta_on_zone_right.points[3].y = RESIM_cta_zone_obj.points[3].y;
                #endif
                    
                }
            }
            
			/* Use Path Heading to calculate intersection point */
			p_object->attributes->intersect_points = getIntersectionPointWithXandYAxis(obj_ref_point,
				p_object,
				obj_relative_velocity);
            //coverity[misra_c_2012_rule_10_4_violation]  k_cta_f_apply_heading_compensation_on_intersection_point's type casting is normally doing
        #if 0 // hkmc doesnt use
			if (TRUE == p_cals->k_cta_f_apply_heading_compensation_on_intersection_point) 
			{
				/* intersection point is adapted by a value depending on target heading and ego width -> leads to higher feature sensitivity */
				adaptIntersectPointToObjectHeading(p_object,
					p_vehicle_data,
					sin_k_cta_min_park_angle,
					cos_k_cta_min_park_angle);
			}
        #endif

			p_object->attributes->radial_distance = calculateRadialDistance(&obj_ref_point,
				p_CTA_persistent);
            is_warning = getHysteresisLevel(p_object);
            //coverity[misra_c_2012_rule_10_4_violation]  0 and is_warning's comparison type casting is normally doing    
            if(0==is_warning)
            {                
            #if defined(MAX_K_CTA_TRACKLATVEL_COMPUSE_IN_VULNERAREA)    
                if(TRUE == p_cals->k_cta_TrackLatVel_CompUse_In_VulnerArea)
                {
                    if(TRUE==p_object->attributes->track_created_vulnerable_area)
                    {
                        abs_min_speed = p_cals->k_cta_min_AbsSpeed_During_NoWarning_for_VulnerObj; // abs_speed > 1.2mps (4.3kph)
                        lat_min_speed = p_cals->k_cta_min_latVel_DuringNoWarning_for_VulnerObj;    // latvel > 0.8 mps (2.83kph)
                    }
                    else
                    {
                        abs_min_speed = p_cals->k_cta_min_AbsSpeed_During_NoWarning_for_GeneraObj; // abs_speed > 1.389mps (5kph)
                        lat_min_speed = p_cals->k_cta_min_latVel_DuringNoWarning_for_GeneraObj;    // latVel > 1mps (3.6kp)
                    }
                }
                else
                {
                    abs_min_speed = p_cals->k_cta_min_AbsSpeed_During_NoWarning_for_GeneraObj; // abs_speed > 1.389mps (5kph)
                    lat_min_speed = p_cals->k_cta_min_latVel_DuringNoWarning_for_GeneraObj;    // latVel > 1mps (3.6kp)
                }
            #else
                abs_min_speed = 1.389F; // abs_speed > 1.389mps (5kph)
                lat_min_speed = 1.0F;   // latVel > 1mps (3.6kp)
            #endif
            }
            else
            {
            #if defined(MAX_K_CTA_MIN_LATERAL_APPROACH_SPEED_DURINGWARNING)
                if(TRUE ==flag_track_latVel_vulner)
                {
                    abs_min_speed = p_cals->k_cta_min_AbsSpeed_During_Warning_for_VulnerObj; // abs_speed  > 0.833 mps(3kph)
                    lat_min_speed = p_cals->k_cta_min_latVel_DuringWarning_for_VulnerObj;    // latvel > 0.65mps (2.34kph)
                }
                else
                {
                    abs_min_speed = p_cals->k_cta_min_AbsSpeed_During_Warning_for_GeneraObj; // abs_speed  > 0.833 mps(3kph)
                    lat_min_speed = p_cals->k_cta_min_latVel_DuringWarning_for_GeneraObj;    // latvel > 0.65mps (2.34kph)
                }
            #else
                abs_min_speed = 0.833F; // latvel > 0.833 mps(3kph)
                lat_min_speed = 0.65F;  // latvel > 0.65mps (2.34kph)
            #endif
            }
            /*  LtoR                          RtoL
                ref6------ref7------ref0      ref2------ref3------ref4
                |                    |        |                    |
                ref5                ref1      ref1                ref5
                |                    |        |                    |
                ref4------ref3------ref2      ref0------ref7------ref6
            */
            if(p_object->attributes->side == LeftSide)
            {
                obj_ref_point.point = target_corners.points[0];            
                obj_ref_point.index = 0;
                obj_ref_point.neighbor1 = 7;
                obj_ref_point.neighbor2 = 1;

                /* 타겟의 뒷범퍼가 1+(자차폭/2)를 지나는순간 오프 */
                /* new warning zone : off line = 1+(host_width/2) */
                obj_ref_point.neighbor3 = 6;

                
                if(TRUE ==flag_track_latVel_vulner)
                {
                     /*                       
                       flag_track_latVel_vulner==true일때만 latvel을 절대속으로 치환사용
                    */
                    f_rel_lat_vel = (uint8_T)(comp_relVelY_withAbsSpeed >= abs_min_speed); // > 0.833mps (3kph)
                }
                else
                {
                    f_rel_lat_vel = (uint8_T)(obj_relative_velocity.y >= lat_min_speed); // 미경고중 : latVel > 1mps (3.6kp)   //경고중 :  latVel > 0.65mps (2.34kph)
                }
            }
            else
            {
                obj_ref_point.point = target_corners.points[2];            
                obj_ref_point.index = 2;
                obj_ref_point.neighbor1 = 3;
                obj_ref_point.neighbor2 = 1;

                /* 타겟의 뒷범퍼가 1+(자차폭/2)를 지나는순간 오프 */
                /* new warning zone : off line = -1*(1+(host_width/2)) */
                obj_ref_point.neighbor3 = 4;
                
                if(TRUE ==flag_track_latVel_vulner)
                {
                     /*
                       flag_track_latVel_vulner==true일때만 latvel을 절대속으로 치환사용
                    */
                    f_rel_lat_vel = (uint8_T)(comp_relVelY_withAbsSpeed <= ((-1.0)*abs_min_speed)); // > 0.833mps (3kph)
                }
                else
                {
                    f_rel_lat_vel = (uint8_T)(obj_relative_velocity.y <= ((-1.0)*lat_min_speed)); // 미경고중 : latVel > 1mps (3.6kp)   //경고중 : latVel > 0.65mps (2.34kph)
                }
            }

            f_ttc_cond = (uint8_T)(p_object->attributes->ttc < dyn_ttc);
            
            /* When mature_count >= 2, can start warning */
            if(getHysteresisLevel(p_object)) /* if warning status, alway recognize it as matured status */
            {
                //coverity[misra_c_2012_rule_10_3_violation]  mature_count's type casting is normally doing
                f_matured = TRUE;
            }
            else
            {
                //coverity[misra_c_2012_rule_10_4_violation]  mature_count's type casting is normally doing
                f_matured = (uint8_T)(p_object->attributes->mature_count >= p_cals->k_cta_matured_count);
            }

            if(TRUE == flag_track_heading_vulner)
            {
                //coverity[misra_c_2012_rule_10_3_violation]  Is_Object_In_Field_of_Interest's internal parameters casting is normally doing            
                f_in_zone = Is_Object_In_Field_of_Interest(&obj_ref_point,
                    &rotated_target_corners,
                    cta_zone_obj.points,
                    num_corners_polygon,
                    p_cals);
            }
            else
            {
                //coverity[misra_c_2012_rule_10_3_violation]  Is_Object_In_Field_of_Interest's internal parameters casting is normally doing            
                f_in_zone = Is_Object_In_Field_of_Interest(&obj_ref_point,
                    &target_corners,
                    cta_zone_obj.points,
                    num_corners_polygon,
                    p_cals);
            }
            
            //coverity[misra_c_2012_rule_10_1_violation]  f_matured's type casting is normally doing
            f_in_zone = (uint8_T)(f_in_zone && f_ttc_cond && f_matured && f_rel_lat_vel);
        
			p_object->attributes->intersect_point_accuracy = getIntersectionPointAccuracy(p_object->tracker_output,
				obj_relative_velocity,
				p_cals);

        #if 0 /* don't use in HKMC */
            markObjectAsRelevant(p_CTA_output, (uint32_T)p_object->attributes->loop_index, f_in_zone);
		#endif
        
			/*######################################################## BEGIN CHECK ALL CRITICALITY LEVELS ########################################################*/
			Check_All_Levels(&(p_CTA_persistent->max_level.level[p_object->attributes->side]),
				p_CTA_persistent->object_with_highest_crit,
				p_object,				
                //coverity[misra_c_2012_rule_10_3_violation]  f_in_zone's type casting is normally doing
				f_in_zone,
				p_level_cals,
				p_cals);
            
			/*######################################################### END CHECK ALL CRITICALITY LEVELS #########################################################*/
		} /* End Check TTC > TTC_Stop */        
	}/* End Check Object List Entry */
	else
	{
		Reset_CTA_Object_Persistent(p_object->persistent);
	}
    
#ifdef BINARY_DEBUG        
    ASGUI_DBG_STORE_CTA_DEBUG_DATA(CTA_FrontOrRear, p_object, obj_ref_point, obj_relative_velocity, p_level_cals, dyn_Y, rotated_target_corners);
#endif
}




/*===========================================================================*\
* Local Functions Definitions
\*===========================================================================*/
/**
* FUNCTION: checkCTA
* This function is used to check if a CTA alert should be given.  Basic conditions
* are: Target OTG heading is within a calibratible range (ensures target is approaching
* from the side)
* Target is expected to cross host path within a certain time
* Target is within the defined alert zone
* Target is expected to pass near the rear of the host (i.e. the target will enter
* the conflict zone)
* Target has been mature for a certain number of counts while meeting these
* conditions
*
* \param[in,out] p_CTA_output
*
* \param[in]  p_CTA_input
* \param[in]  p_CTA_persistent
* \param[in]  p_cals
*
* \return	  void
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/

static void checkCTA(const CTA_INPUT_T *p_CTA_input,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/ /*polyspace CODE-METRIC:CALLS [Justified:Low] "acceptable called number"*/
	CTA_OUTPUT_T *p_CTA_output,
	CTA_PERSISTENT_T* p_CTA_persistent,
	const CALIBRATION_CTA_T* p_cals)
{
	/* ################# Declaration of local varibles ############################# */

	const VEHICLE_DATA_FLT_T *p_vehicle_data = p_CTA_input->vehicle_data;
	CTA_Object_Data_T p_object;

	/************* Level Logic Calibration parameters ************/
	Level_Logic_Calibration_T level_cals;
	/* AUDI actuator requirements */
	/************* AUDI Calibration parameters ************/
	Level_Logic_Calibration_T level_cals_backup;
	cta_heading_angle_t heading_angle;
	/************* Loop specific parameter ************/       
	float32_T sin_k_cta_min_park_angle = (float32_T)FAST_SIN(p_cals->k_cta_min_park_angle);
	float32_T cos_k_cta_min_park_angle = (float32_T)FAST_COS(p_cals->k_cta_min_park_angle);
	p_CTA_persistent->transistion_vector_to_middle_of_bumper.x = p_cals->k_cta_distance_Rbumper_To_Origine - p_vehicle_data->host_vehicle_length;
	p_CTA_persistent->transistion_vector_to_middle_of_bumper.y = 0.F;
	Init_CTA_Object(&(p_CTA_persistent->object_with_highest_crit[LeftSide]));
	p_CTA_persistent->object_with_highest_crit[LeftSide].tracker_output = &(tracker_output_object_high_crit[LeftSide]);
	Init_CTA_Object(&(p_CTA_persistent->object_with_highest_crit[RightSide]));
	p_CTA_persistent->object_with_highest_crit[RightSide].tracker_output = &(tracker_output_object_high_crit[RightSide]);

	resetRelevantObjectMarkings(p_CTA_output); /* HKMC does not use CTA_output.obj_cnt member(only for AUDI) */
    
    //coverity[overrun-buffer-val] no issue because this is just debugging code
	ASGUI_DBG_RESET_CTA_DEBUG_DATA();
	ASGUI_DBG_WRITE_CTA_CAL_MAC(p_cals);

	p_CTA_persistent->max_level.level[0] = CTA_NO_CRIT; /* LtoR */
	p_CTA_persistent->max_level.level[1] = CTA_NO_CRIT; /* RtoL */

    /* LtoR : -35 ~ 55 based on Left LAT line */
    heading_angle.min[0] = p_cals->k_cta_min_approach_angle; /* 35 deg */
    heading_angle.max[0] = p_cals->k_cta_max_approach_angle; /* 135 deg */

    /* RtoL : -35 ~ 55 based on Right LAT line */
    heading_angle.min[1] = -p_cals->k_cta_max_approach_angle; /* -135 deg */
    heading_angle.max[1] = -p_cals->k_cta_min_approach_angle; /* -35 deg */
    
	level_cals = p_CTA_persistent->cta_level_logic_calib;
    //coverity[misra_c_2012_rule_10_4_violation]  k_cta_f_adapt_intersect_lines_by_steering_angle's type casting is normally doing
	if (TRUE == p_cals->k_cta_f_adapt_intersect_lines_by_steering_angle)
	{
		adaptIntersectLinesToSteeringAngle(p_cals, &level_cals, p_vehicle_data->steering_angle);
		/* create backup of level cals, as they will be modified individually for each
		   object later, but for each object, the original values need to be restored
		   before applying object-specific modifications */
		memcpy(&level_cals_backup, &level_cals, sizeof(Level_Logic_Calibration_T));
	}


	/* ################# Begin of CTA algorithm #############################*/
#if 0
	for (CTA_Iterator_First_Valid_Object(); FALSE == CTA_Iterator_Is_Done(); CTA_Iterator_Next_Valid_Object())
	{
		CTA_Iterator_Fill_With_Current_Item(&p_object);
		CheckCTA_Single_Object(p_CTA_input,
			p_CTA_output,
			p_CTA_persistent,
			p_cals,
			&p_object,
			&heading_angle,
			&level_cals,
			&level_cals_backup,
			sin_k_cta_min_park_angle,
			cos_k_cta_min_park_angle);
	}
#else // coverity fix for loop-> while
    CTA_Iterator_First_Valid_Object();
    while(FALSE == CTA_Iterator_Is_Done())
    {
		CTA_Iterator_Fill_With_Current_Item(&p_object);
		CheckCTA_Single_Object(p_CTA_input,
			p_CTA_output,
			p_CTA_persistent,
			p_cals,
			&p_object,
			&heading_angle,
			&level_cals,
			&level_cals_backup,
			sin_k_cta_min_park_angle,
			cos_k_cta_min_park_angle);

        CTA_Iterator_Next_Valid_Object();
    }
#endif

	/* AS Bin Writer */
	ASGUI_DBG_WRITE_CTA_DEBUG_MAC();
}

/**
* FUNCTION: Check_Object_Validity
* This function checks if target with object ID is a valid object.
* Return TRUE when all of the following criteria fulfills:
* - obj_stationary: false \n
* - age > 0 \n
* - obj_reflection: false \n
* - ego_speed > cta_off_speed
* - subfunction objectsRelevance is TRUE
*
* \param[in]  p_object
* \param[in]  p_cals
* \param[in]  heading_angle
*
* \return     flag indicating the usability of the requested object
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/

static boolean_T Check_Object_Validity(const CTA_Object_Data_T* p_object,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const CALIBRATION_CTA_T* p_cals,
	const cta_heading_angle_t heading_angle)
{
	float32_T k_cta_heading_angle_hys;
	float32_T min_speed_obj;
	float32_T min_lateral_approach_speed_obj;
	float32_T min_existence_probability_obj;
	float32_T min_heading_angle_obj;
	float32_T max_heading_angle_obj;
	uint8_T hysteresisLevel;
	uint8_T f_return;

	min_heading_angle_obj = heading_angle.min[p_object->attributes->side];
	max_heading_angle_obj = heading_angle.max[p_object->attributes->side];
    //coverity[misra_c_2012_rule_10_3_violation]  f_return's type casting is normally doing
	f_return = TRUE;
	k_cta_heading_angle_hys = p_cals->k_cta_cross_dist_max_lat_angle_diff*p_cals->k_cta_rel_warning_hysteresis;
	hysteresisLevel = getHysteresisLevel(p_object);
    
#if defined(MAX_K_CTA_TRACKLATVEL_COMPUSE_IN_VULNERAREA)    
    if(TRUE == p_cals->k_cta_TrackLatVel_CompUse_In_VulnerArea)
    {
        if(TRUE==p_object->attributes->track_created_vulnerable_area)
        {
        	min_speed_obj = p_cals->k_cta_min_AbsSpeed_During_NoWarning_for_VulnerObj; // abs_speed > 1.2mps(4.3kph)
        	min_lateral_approach_speed_obj = p_cals->k_cta_min_latVel_DuringNoWarning_for_VulnerObj; //  latvel > 0.8 mps (2.83kph)
        }
        else
        {
            min_speed_obj = p_cals->k_cta_min_AbsSpeed_During_NoWarning_for_GeneraObj; //abs_speed > 1.389mps(5kph)
            min_lateral_approach_speed_obj = p_cals->k_cta_min_latVel_DuringNoWarning_for_GeneraObj; //  latvel > 1.0 mps(3.6kph)
        }
    }
    else
    {
        min_speed_obj = p_cals->k_cta_min_AbsSpeed_During_NoWarning_for_GeneraObj; //abs_speed > 1.389mps(5kph)
        min_lateral_approach_speed_obj = p_cals->k_cta_min_latVel_DuringNoWarning_for_GeneraObj; //  latvel > 1.0 mps(3.6kph)
    }
#else
    min_speed_obj = p_cals->k_cta_min_AbsSpeed_During_NoWarning_for_GeneraObj; //abs_speed > 1.389mps(5kph)
    min_lateral_approach_speed_obj = p_cals->k_cta_min_latVel_DuringNoWarning_for_GeneraObj; //  latvel > 1.0 mps(3.6kph)
#endif

	min_existence_probability_obj = p_cals->k_cta_min_rel_existence_probability;
    //coverity[misra_c_2012_rule_10_4_violation]  hysteresisLevel's type casting is normally doing
	if (hysteresisLevel > 0)
	{
#if defined(MAX_K_CTA_TRACKLATVEL_COMPUSE_IN_VULNERAREA)
        if(TRUE == p_cals->k_cta_TrackLatVel_CompUse_In_VulnerArea)
		{
            //min_lateral_approach_speed_obj = p_cals->k_cta_min_lateral_approach_speed_duringWarning; // latvel > 0.65mps(2.34kph) 
            if(TRUE==p_object->attributes->track_created_vulnerable_area)
            {                
                min_speed_obj = p_cals->k_cta_min_AbsSpeed_During_Warning_for_VulnerObj; //abs_speed > 0.833mps( (3kph)
                min_lateral_approach_speed_obj = p_cals->k_cta_min_latVel_DuringWarning_for_VulnerObj; // latvel > 0.65mps (2.34kph)
            }
            else
            {                
                min_speed_obj = p_cals->k_cta_min_AbsSpeed_During_Warning_for_GeneraObj; //abs_speed > 0.833mps( (3kph)
                min_lateral_approach_speed_obj = p_cals->k_cta_min_latVel_DuringWarning_for_GeneraObj; // latvel > 0.65mps (2.34kph)
            }
		}
        else
        {
            min_speed_obj = p_cals->k_cta_min_AbsSpeed_During_Warning_for_GeneraObj; //abs_speed > 0.833mps( (3kph)
            min_lateral_approach_speed_obj = p_cals->k_cta_min_latVel_DuringWarning_for_GeneraObj; // latvel > 0.65mps (2.34kph)
        }    
#else
        {
            min_speed_obj = p_cals->k_cta_min_AbsSpeed_During_Warning_for_GeneraObj; //abs_speed > 0.833mps( (3kph)
            min_lateral_approach_speed_obj = p_cals->k_cta_min_latVel_DuringWarning_for_GeneraObj; // latvel > 0.65mps (2.34kph)
        }
#endif
        min_existence_probability_obj -= min_existence_probability_obj*p_cals->k_cta_rel_warning_hysteresis;/*polyspace DEFECT:USELESS_WRITE [Justified:Unset] "intended usage"*/
		min_heading_angle_obj -= k_cta_heading_angle_hys;
		max_heading_angle_obj += k_cta_heading_angle_hys;
	}

	//f_return &= !(p_object->tracker_output->f_stationary && p_cals->k_cta_f_check_stationary_signal);
    //coverity[misra_c_2012_rule_10_1_violation]  k_cta_min_object_age_in_cycles's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  k_cta_min_object_age_in_cycles's type casting is normally doing
	f_return &= (p_object->tracker_output->age > p_cals->k_cta_min_object_age_in_cycles);
    //coverity[misra_c_2012_rule_10_1_violation]  f_reflection's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  f_reflection's type casting is normally doing
    f_return &= !(p_object->tracker_output->f_reflection && p_cals->k_cta_f_check_reflection_signal);
    //coverity[misra_c_2012_rule_10_1_violation]  min_speed_obj's  type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  min_speed_obj's  type casting is normally doing
	f_return &= (p_object->tracker_output->speed >= min_speed_obj);
    //coverity[misra_c_2012_rule_10_1_violation]  min_heading_angle_obj's type casting is normally doing
    //coverity[misra_c_2012_rule_10_3_violation]  min_heading_angle_obj's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  min_heading_angle_obj's type casting is normally doing

#if defined(MAX_K_CTA_TRACKHEADING_COMPUSE_IN_VULNERAREA)
    if(TRUE == p_cals->k_cta_TrackHeading_CompUse_In_VulnerArea)
    {
        /* 취약구역에서 생성된 트랙이고 && 트랙의 age가 20이상이면 트래커 헤딩대신 vx, vy로 계산한 헤딩을 사용 */
        if((TRUE==p_object->attributes->track_created_vulnerable_area) && (p_object->tracker_output->age>p_cals->k_cta_VULNERABLE_CRITERIA_AGE)) //k_cta_VULNERABLE_CRITERIA_AGE=5
        {
            f_return &= (p_object->attributes->heading_rad_by_vxvy_LPF >= min_heading_angle_obj);
            //coverity[misra_c_2012_rule_10_1_violation]  min_heading_angle_obj's type casting is normally doing
            //coverity[misra_c_2012_rule_10_4_violation]  min_heading_angle_obj's type casting is normally doing
            f_return &= (p_object->attributes->heading_rad_by_vxvy_LPF <= max_heading_angle_obj);
        }
        else
        {
            f_return &= (p_object->tracker_output->heading >= min_heading_angle_obj);
            //coverity[misra_c_2012_rule_10_1_violation]  min_heading_angle_obj's type casting is normally doing
            //coverity[misra_c_2012_rule_10_4_violation]  min_heading_angle_obj's type casting is normally doing
            f_return &= (p_object->tracker_output->heading <= max_heading_angle_obj);
        }
    }
    else
    {
        f_return &= (p_object->tracker_output->heading >= min_heading_angle_obj);
        //coverity[misra_c_2012_rule_10_1_violation]  min_heading_angle_obj's type casting is normally doing
        //coverity[misra_c_2012_rule_10_4_violation]  min_heading_angle_obj's type casting is normally doing
        f_return &= (p_object->tracker_output->heading <= max_heading_angle_obj);
    }
#else
    {
        f_return &= (p_object->tracker_output->heading >= min_heading_angle_obj);
        //coverity[misra_c_2012_rule_10_1_violation]  min_heading_angle_obj's type casting is normally doing
        //coverity[misra_c_2012_rule_10_4_violation]  min_heading_angle_obj's type casting is normally doing
        f_return &= (p_object->tracker_output->heading <= max_heading_angle_obj);
    }
#endif

    //coverity[misra_c_2012_rule_10_1_violation]  f_behind_clutter's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  f_behind_clutter's type casting is normally doing
#if defined(MAX_K_CTA_GUARDRAILTYPE_SELECT) 
    if(0 == ((CALIBRATION_CTA_T *)&RSDS_CTA_Cal_DF_V)->k_cta_GuardrailType_Select)
    {
        /* k_cta_GuardrailType_Select=0 : warning supression by clutter determination */
        f_return &= (0==p_object->tracker_output->f_behind_clutter); /* ATH195 : to supress false warning by guarail */
    }
    else
    {    
        /* k_cta_GuardrailType_Select=1 : warning supression by barrier determination */
        if(LeftSide==p_object->attributes->side)
        {
            f_return &= (0==p_object->tracker_output->LSide_f_LAT_barrier); /* ATH195 : to supress false warning by barrier */
        }
        else
        {
            f_return &= (0==p_object->tracker_output->RSide_f_LAT_barrier); /* ATH195 : to supress false warning by barrier */            
        }
    }
#else
    f_return &= (0==p_object->tracker_output->f_behind_clutter); /* ATH195 : to supress false warning by guarail */
#endif
    //coverity[misra_c_2012_rule_10_4_violation]  min_lateral_approach_speed_obj's type casting is normally doing
    //coverity[misra_c_2012_rule_10_1_violation]  min_lateral_approach_speed_obj's type casting is normally doing
    f_return &= (FAST_ABS(getRelativeLateralVelocityObject(p_object->tracker_output, p_cals)) >= min_lateral_approach_speed_obj); // >0.65mps
	//f_return &= (p_object->tracker_output->existence_probability > min_existence_probability_obj); 
	//f_return &= (p_object->tracker_output->rangeRegionObstructed_probability <= range_region_obstructed_probability);

    //coverity[misra_c_2012_rule_10_3_violation]  f_return's return type casting is normally doing
	return f_return;
}

/**
* FUNCTION: getHysteresisLevel
* This function returns a hysteresis level based on current objects alert level.
* - Return 1: previous status > no warning \n
* - Return 2: previous status > warning \n
* - Return 0: default \n
*
* \param[in] p_object
*
* \return     hysteresisLevel
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/
static uint8_T getHysteresisLevel(const CTA_Object_Data_T *p_object)/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
{
	uint8_T hysteresisLevel;
    //coverity[misra_c_2012_rule_10_4_violation]  CTA_NO_CRIT's type casting is normally doing
	if (p_object->persistent->prev_RCTA_enLastCycleCritLevelOn.val[p_object->attributes->side] != CTA_NO_CRIT)
	{
		hysteresisLevel = 1;
	}
	else
	{
		hysteresisLevel = 0;
	}

	return hysteresisLevel;
}

/**
* FUNCTION: flipCTAzone
* This function flip CTAzne if needed based on the approaching direction
*
* \param[out] p_cta_zone
*
* \param[in] side
* \param[in] cta_zone
* \param[in] p_cals
* \param[in] p_vehicle_data
*
* \return void
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/

static void flipCTAzone(const CTA_Side_T side,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	cta_zone_point_t *cta_zone,
	const CALIBRATION_CTA_T* p_cals,
	const VEHICLE_DATA_FLT_T* p_vehicle_data)
{
    #if 0
	if (CTA_FrontOrRear == FRONT_CTA)
	{
		cta_zone->points[0].x = -(p_cals->k_cta_func_area_p1_x) - p_vehicle_data->host_vehicle_length; /* refers to P1x */
		cta_zone->points[1].x = -(p_cals->k_cta_func_area_p2_x) - p_vehicle_data->host_vehicle_length; /* refers to P2x */
		cta_zone->points[2].x = -(p_cals->k_cta_func_area_p4_x) - p_vehicle_data->host_vehicle_length; /* refers to P4x */
		cta_zone->points[3].x = -(p_cals->k_cta_func_area_p3_x) - p_vehicle_data->host_vehicle_length; /* refers to P3x */
	}
    #endif
    
	if (side == LeftSide)
	{
		cta_zone->points[0].y = -(cta_zone->points[0].y);
		cta_zone->points[1].y = -(cta_zone->points[1].y);
		cta_zone->points[2].y = -(cta_zone->points[2].y);
		cta_zone->points[3].y = -(cta_zone->points[3].y);
	}
}


/**
* FUNCTION: Is_Object_In_Field_of_Interest
* This function checks if object is within cta field of interest
* This check is done by checking whether the reference point or its neighbors are inside the field of view
* - compare the projection length of target and estimated road shape points in parallel to the estimated heading \n
*
* \param[in] ref_point
* \param[in] p_object_corner
* \param[in] cta_zone
* \param[in] num_polygon_corners
*
* \return boolean_T
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/ 
static boolean_T Is_Object_In_Field_of_Interest(const CTA_Target_Reference *p_ref_point,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
    const CTA_Advanced_Object_Corner *p_object_corner,
    const Vector_2d_T* cta_zone,
    const uint8_T num_polygon_corners,
    const CALIBRATION_CTA_T *p_cals)
{
    boolean_T ret;

    if((u16p0_T)0 == (p_cals->k_cta_ZoneOffCond_Is_TargetCenter)) // new revision
    {
        /*****************************************************************************************************/
        /* when para is false, warning zone end criteria is that target's rear bumper is away from end line */
        /*****************************************************************************************************/
        
        /*  LtoR                                  RtoL
            [[ref6]]------[[ref7]]------[[ref0]]      [[ref2]]------[[ref3]]------[[ref4]]
            |                              |           |                              |
            ref5                        [[ref1]]      [[ref1]]                       ref5
            |                              |           |                               |
            ref4-----------ref3----------ref2         ref0------------ref7-----------ref6
            
            obj_ref_point.index = 0;                  obj_ref_point.index = 2;
            obj_ref_point.neighbor1 = 7;              obj_ref_point.neighbor1 = 3;
            obj_ref_point.neighbor2 = 1;              obj_ref_point.neighbor2 = 1;
            
            obj_ref_point.neighbor3 = 6;              obj_ref_point.neighbor3 = 4;  
        */
            
        /****************************************************************************************************************************/
        /* 경고오프순간 : 타겟의 뒷범퍼가 getCTA_OffLatLine()에서 지정한 1+(자차폭/2)를 지나는순간 오프                             */
        /****************************************************************************************************************************/
        if ((Is_Point_In_Polygon(cta_zone,num_polygon_corners,&(p_ref_point->point))) ||
            (Is_Point_In_Polygon(cta_zone, num_polygon_corners, &(p_object_corner->points[p_ref_point->neighbor1]))) ||
            (Is_Point_In_Polygon(cta_zone, num_polygon_corners, &(p_object_corner->points[p_ref_point->neighbor2]))) ||
            (Is_Point_In_Polygon(cta_zone, num_polygon_corners, &(p_object_corner->points[p_ref_point->neighbor3]))))    
        {
            ret = TRUE;
        }
        else
        {
            ret = FALSE;
        }
        
        return ret;
    }
    else // old rccw revision
    {
        /*****************************************************************************************************/
        /* when para is true, warning zone end criteria is that target's center is away from end line       */
        /*****************************************************************************************************/
        
        /*  LtoR                                  RtoL
            ref6------[[ref7]]------[[ref0]]      [[ref2]]------[[ref3]]------ref4
            |                          |           |                           |
            ref5                    [[ref1]]      [[ref1]]                    ref5
            |                          |           |                           |
            ref4--------ref3----------ref2        ref0------------ref7--------ref6
        
            obj_ref_point.index = 0;               obj_ref_point.index = 2;
            obj_ref_point.neighbor1 = 7;           obj_ref_point.neighbor1 = 3;
            obj_ref_point.neighbor2 = 1;           obj_ref_point.neighbor2 = 1;
        */
        
        /****************************************************************************************************************************/
        /* 경고오프순간 : 타겟의 센서가 getCTA_OffLatLine()에서 지정한 1+(host_width*0.5F) - ((0.5F)*length_obj) 를 지나는순간 오프 */
        /****************************************************************************************************************************/
        if ((Is_Point_In_Polygon(cta_zone,num_polygon_corners,&(p_ref_point->point))) ||
            (Is_Point_In_Polygon(cta_zone, num_polygon_corners, &(p_object_corner->points[p_ref_point->neighbor1]))) ||
            (Is_Point_In_Polygon(cta_zone, num_polygon_corners, &(p_object_corner->points[p_ref_point->neighbor2]))))
        {
            ret = TRUE;
        }
        else
        {
            ret = FALSE;
        }

        return ret;
    }
}/* end of intersectZone function */

/**
* FUNCTION: Is_Point_In_Polygon
* This function checks whether an point is in an polygon. This polygon must be convex and without holes or stuff like that.
* The basic idea is that one checks whether the point is an the same side for all side lines of the polygon. 
* This can be done by just looking at the distance of the point to the line to be checked. The sign indicated the side of the line.
* It should be the same for all lines.
*
* \param[in]		const Vector_2d_T* polygon,
*					const uint8_T num_polygon_corners,
*					const Vector_2d_T* p_point,
*
* \param[in\out]
*
* \param[out]
*
* \return         	boolean_T
*
* \remark
* \ServID         	xx
* \Reentrancy     	non-reentrant
* \Synchronism    	synchronous
* \Precondition   	none
* \Caveats        	none
* \Requirements
* \reqtrace{}
*/
static boolean_T Is_Point_In_Polygon(const Vector_2d_T* cta_zone,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const uint8_T num_polygon_corners,
	const Vector_2d_T* p_point)
{
	boolean_T is_point_in_polygon = FALSE;
	CTA_Line_Side_T side_of_line;
	CTA_Line_Side_T side_of_line_tmp;
	Line_Normal_T line;
	uint8_T index;
	float32_T distance;

	side_of_line = CTA_LINE_SIDE_ONLINE;

	for ( index = 0; index < num_polygon_corners; index++)
	{
		is_point_in_polygon = TRUE;
		line = Create_Line_Normal(&(cta_zone[GET_INDEX_OF_PERIODIC_ARRAY(index, num_polygon_corners)]), 
            //coverity[misra_c_2012_rule_10_4_violation]  GET_INDEX_OF_PERIODIC_ARRAY macro's type casting is normally doing 
			&(cta_zone[GET_INDEX_OF_PERIODIC_ARRAY(index + 1, num_polygon_corners)]));
		side_of_line_tmp = Get_Side_of_Point(&line, p_point);

		if ((side_of_line == CTA_LINE_SIDE_ONLINE) || (side_of_line_tmp == side_of_line))
		{
			side_of_line = side_of_line_tmp;
		}
		else
			{
			is_point_in_polygon = FALSE;
			break;
		}
	}

	return is_point_in_polygon;
}


/**
* FUNCTION: Get_Side_of_Point
* This function evaluates on which side of the line the point is
*
* \param[in]		const Line_Normal_T *p_line, const Vector_2d_T *p_point
*
* \param[in\out]
*
* \param[out]
*
* \return         	CTA_Line_Side_T
*
* \remark
* \ServID         	xx
* \Reentrancy     	non-reentrant
* \Synchronism    	synchronous
* \Precondition   	none
* \Caveats        	none
* \Requirements
* \reqtrace{}
*/
static CTA_Line_Side_T Get_Side_of_Point(const Line_Normal_T *p_line, const Vector_2d_T *p_point)/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
{
	CTA_Line_Side_T retValue;
    //coverity[misra_c_2012_rule_10_1_violation]  SIGN macro's type casting is normally doing 
    //coverity[misra_c_2012_rule_10_4_violation]  SIGN macro's type casting is normally doing 
    //coverity[misra_c_2012_rule_10_8_violation]  SIGN's type casting is normally doing
	retValue = (CTA_Line_Side_T)SIGN(((p_line->p0.x - p_point->x)*p_line->n.x) + ((p_line->p0.y - p_point->y)*p_line->n.y));

	return retValue;
}

/**
* FUNCTION: Create_Line_Normal
*
* s.o.
*
* \param[in]		const Vector_2d_T *p_p0, const Vector_2d_T *p_p1
*
* \param[in\out]
*
* \param[out]
*
* \return         	Line_Normal_T
*
* \remark
* \ServID         	xx
* \Reentrancy     	non-reentrant
* \Synchronism    	synchronous
* \Precondition   	none
* \Caveats        	none
* \Requirements
* \reqtrace{}
*/
static Line_Normal_T Create_Line_Normal(const Vector_2d_T *p_p0, const Vector_2d_T *p_p1)/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
{
	Line_Normal_T retValue;
	
	/*
	ASSERT_NO_NAN_VECTOR(start_pt);
	ASSERT_NO_NAN_VECTOR(end_pt);

	ASSERT(p_p0 != NULL);
	ASSERT(p_p1 != NULL);
	*/

	retValue.p0 = *p_p0;

	retValue.n.x = -p_p0->y + p_p1->y;
	retValue.n.y = p_p0->x - p_p1->x;

	return retValue;
}

/**
* FUNCTION: getIntersectionPointWithXandYAxis
* This functions computes the intersection point of the target with x and y axis
* xy_IntersectionPoint.x = ref_point.x + ttc_host_path * x_vel_rel \n
* xy_IntersectionPoint.y = ref_point.y + (ref_point.x / x_vel_rel)*y_vel_rel \n
*
* \param[in] ref_point
* \param[in] p_object
* \param[in] relative_velocity
*
* \return  xy_IntersectionPoint
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/
static Vector_2d_T getIntersectionPointWithXandYAxis(const CTA_Target_Reference ref_point,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const CTA_Object_Data_T *p_object,
	const Vector_2d_T obj_relative_velocity)
{
	/* the intersection points of a target with x- and y-axis,
	for the right side the front right edge should be taken and
	for the left side the front left edge*/

	Vector_2d_T IntersectionPoint;

	IntersectionPoint.x = -INFTY;/*polyspace DEFECT:USELESS_WRITE [Justified:Unset] "intended usage"*/
	IntersectionPoint.y = -INFTY;/*polyspace DEFECT:USELESS_WRITE [Justified:Unset] "intended usage"*/

	IntersectionPoint.x = ref_point.point.x + (p_object->attributes->ttc * obj_relative_velocity.x);
    //coverity[misra_c_2012_rule_10_4_violation]  obj_relative_velocity's  type casting is normally doing	
	if (FAST_ABS(obj_relative_velocity.x) < 1.0e-5)
	{
		IntersectionPoint.y = INFTY;
	}
	else
	{
		IntersectionPoint.y = ref_point.point.y - ((ref_point.point.x / obj_relative_velocity.x) * obj_relative_velocity.y);
	}

	return IntersectionPoint;
}

/**
* FUNCTION: calculateTimeToConflict
* This functions computes the time to conflict between a target
* and the ego vehicle based on linear extrapolation
*
* \param[in] p_obj_relative_velocity
* \param[in] p_obj_ref_point
* \param[in] p_object
* \param[in] p_vehicle_data
* \param[in] p_cals
*
* \return  ttc
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{FERHR-SDD-C2RCTA-calculateTimeToConflict}{FERHR-SWS-C2RCTA-406-1, FERHR-SWS-C2RCTA-407-1, FERHR-SWS-C2RCTA-408-1}
*/
//static float32_T calculateTimeToConflict(const Vector_2d_T* p_obj_relative_velocity,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
//	const CTA_Target_Reference* p_obj_ref_point,
//	const CTA_Object_Data_T *p_object,
//	const VEHICLE_DATA_FLT_T *p_vehicle_data,
//	const CALIBRATION_CTA_T* p_cals)
static float32_T calculateTimeToConflict(float32_T rel_lat_vel,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
    const CTA_Target_Reference* p_obj_ref_point,
    const CTA_Object_Data_T *p_object,
    const VEHICLE_DATA_FLT_T *p_vehicle_data,
    const CALIBRATION_CTA_T* p_cals)

{
	float32_T ttc;
	float32_T sign;
    //coverity[misra_c_2012_rule_10_4_violation]  k_cta_f_calc_ttc_ego_side_enabled's type casting is normally doing
#if 0
	if (TRUE == p_cals->k_cta_f_calc_ttc_ego_side_enabled)
#endif
	{
		if (LeftSide == p_object->attributes->side)
		{
			sign = -1.0f;
		}
		else
		{
			sign = 1.0f;
		}        
        //coverity[misra_c_2012_rule_10_4_violation]  p_obj_relative_velocity's type casting is normally doing
		ttc = (FAST_ABS(rel_lat_vel) > 1.0e-3f) ? (-(p_obj_ref_point->point.y - (sign*0.5f*p_vehicle_data->host_vehicle_width)) / rel_lat_vel) : 60.0f;
        ttc = max(ttc, 0.0f);
	}
#if 0
	else
	{
        //coverity[misra_c_2012_rule_10_4_violation]  p_obj_relative_velocity's type casting is normally doing
		ttc = (FAST_ABS(rel_lat_vel) > 1.0e-3f) ? (-p_obj_ref_point->point.y / rel_lat_vel) : 60.0f;
	}
#endif
    
	return ttc;
}

/**
* FUNCTION: calculateRadialDistance
* This functions computes the euclidean distance between the current reference point of a target
* and a position on the longitudinal axis of the ego which can be shifted with respect to front
* bumper by persistent signal transistion_vector_to_middle_of_bumper
*
* \param[in] p_obj_ref_point
* \param[in] p_CTA_persistent
*
* \return  radial_distance
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{FERHR-SDD-C2RCTA-calculateRadialDistance}{FERHR-SWS-C2RCTA-411-1}
*/
static float32_T calculateRadialDistance(const CTA_Target_Reference* p_obj_ref_point,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const CTA_PERSISTENT_T* p_CTA_persistent)
{
	return Vector_Alg_Abs(&(p_obj_ref_point->point),
		&(p_CTA_persistent->transistion_vector_to_middle_of_bumper));
}

/**
* FUNCTION: calculateTargetCornersPoint
* This functions computes 4 corner points of the target
* - Corners_point.x = target.x + cos(heading) * rotation.x - sin(heading) * rotation.y \n
* - Corners_point.y = target.y + sin(heading) * rotation.x + cos(heading) * rotation.y
*
* \param[in] p_object
* \param[out] p_target_corners
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/
static void calculateTargetCornersPoint(const CTA_Object_Data_T* p_object,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:NCALLS [Justified:Low] "No issue"*/
	CTA_Advanced_Object_Corner * p_target_corners)
{
	/*Initialisiation of the reference point*/
	/*According to the AUDI req. ID 1193*/

	float32_T cos_heading = FAST_COS(p_object->tracker_output->heading);
	float32_T sin_heading = FAST_SIN(p_object->tracker_output->heading);

	float32_T half_length = p_object->tracker_output->length * 0.5f;
	float32_T half_width = p_object->tracker_output->width * 0.5f;
	Rectangle_T target_object_corners;

	/*% compute object corners*/
	target_object_corners.points[0].x = half_length;
	target_object_corners.points[0].y = -half_width;

	target_object_corners.points[1].x = half_length;
	target_object_corners.points[1].y = half_width;

	target_object_corners.points[2].x = -half_length;
	target_object_corners.points[2].y = half_width;

	target_object_corners.points[3].x = -half_length;
	target_object_corners.points[3].y = -half_width;


	p_target_corners->points[0].x = p_object->tracker_output->vcs.position.x + (cos_heading*target_object_corners.points[0].x) - (sin_heading*target_object_corners.points[0].y);
	p_target_corners->points[0].y = p_object->tracker_output->vcs.position.y + (sin_heading*target_object_corners.points[0].x) + (cos_heading*target_object_corners.points[0].y);
	p_target_corners->points[2].x = p_object->tracker_output->vcs.position.x + (cos_heading*target_object_corners.points[1].x) - (sin_heading*target_object_corners.points[1].y);
	p_target_corners->points[2].y = p_object->tracker_output->vcs.position.y + (sin_heading*target_object_corners.points[1].x) + (cos_heading*target_object_corners.points[1].y);
	p_target_corners->points[4].x = p_object->tracker_output->vcs.position.x + (cos_heading*target_object_corners.points[2].x) - (sin_heading*target_object_corners.points[2].y);
	p_target_corners->points[4].y = p_object->tracker_output->vcs.position.y + (sin_heading*target_object_corners.points[2].x) + (cos_heading*target_object_corners.points[2].y);
	p_target_corners->points[6].x = p_object->tracker_output->vcs.position.x + (cos_heading*target_object_corners.points[3].x) - (sin_heading*target_object_corners.points[3].y);
	p_target_corners->points[6].y = p_object->tracker_output->vcs.position.y + (sin_heading*target_object_corners.points[3].x) + (cos_heading*target_object_corners.points[3].y);

	Vector_Alg_Middle(&(p_target_corners->points[0]), &(p_target_corners->points[2]), &(p_target_corners->points[1]));
	Vector_Alg_Middle(&(p_target_corners->points[2]), &(p_target_corners->points[4]), &(p_target_corners->points[3]));
	Vector_Alg_Middle(&(p_target_corners->points[4]), &(p_target_corners->points[6]), &(p_target_corners->points[5]));
	Vector_Alg_Middle(&(p_target_corners->points[6]), &(p_target_corners->points[0]), &(p_target_corners->points[7]));
}

static float32_T calculate_dyn_ttc(unsigned16_T mode, float32_T relvel_y, const CALIBRATION_CTA_T *p_cals)/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
{
    float32_T dyn_ttc;
    //coverity[misra_c_2012_rule_10_4_violation]  USM_SENSITIVITY_CTA_LATE's type casting is normally doing    
    if(USM_SENSITIVITY_CTA_LATE == mode)
    {
        /* LATE */
        //coverity[misra_c_2012_rule_10_4_violation]  relvel_y's type casting is normally doing
        if (FAST_ABS(relvel_y) < p_cals->k_cta_min_dynttc_lower_threshold_spd)
        {
            dyn_ttc = p_cals->k_cta_min_dynttc; //2.9sec
        }
        else
        {
            /* dyn_TTC = Rev_vel*(-0.144) + 3.3 */
            //coverity[misra_c_2012_rule_10_4_violation]  relvel_y's type casting is normally doing
            dyn_ttc = max((FAST_ABS(relvel_y) * p_cals->k_cta_dynttc_scale_factor) + p_cals->k_cta_dynttc_offset, p_cals->k_cta_max_dynttc);
        }
    }
    else
    {
        /* NORMAL, EARLY */
        //coverity[misra_c_2012_rule_10_4_violation]  relvel_y's type casting is normally doing
        if (FAST_ABS(relvel_y) < p_cals->k_cta_min_dynttc_lower_threshold_spd)
        {
            dyn_ttc = p_cals->k_cta_min_dynttc_usm_lately; //3.5sec
        }
        else
        {
            /* dyn_TTC = Rev_vel*(-0.249) + 4.25 */
            //coverity[misra_c_2012_rule_10_4_violation]  relvel_y's type casting is normally doing            
            dyn_ttc = max((FAST_ABS(relvel_y) * p_cals->k_cta_dynttc_scale_factor_usm_lately) + p_cals->k_cta_dynttc_offset_usm_lately, p_cals->k_cta_max_dynttc_usm_lately);
        }    
    }

    return dyn_ttc;
}

/**
* FUNCTION: calculateTargetReferencePoint
* This functions computes 8 reference points of the target and then determine the reference point with
* minimum distance to the ego's rear bumper as well as its two neighborhoods
* - Computes 8 reference points of the target: \n
* -- for odd ref_point:
* 1) ref_point.corners_x[2 * i] = corners_point.x[i]; \n
* 2) ref_point.corners_y[2 * i] = corners_point.y[i]; \n
* -- for even ref_point: \n
* 1) ref_point.corners_x[2 * i + 1] = 0.5f*(corners_point.x[i] + corners_point.x[(i + 1) % 4]); \n
* 2) ref_point.corners_y[2 * i + 1] = 0.5f*(corners_point.y[i] + corners_point.y[(i + 1) % 4]); \n
* - Return ref-point with minimum distance to the ego's rear bumper as well as its two neighborhoods
*
* \param[in] p_object
* \param[in] tracker_cs_to_reference_cs
* \param[in] CTA_Advanced_Object_Corner *p_target_object_corners
* \param[in] p_cals
* \param[out] CTA_Target_Reference *p_target_reference
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/
static void calculateTargetReferencePoint(const CTA_Object_Data_T* p_object,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const float32_T tracker_cs_to_reference_cs,
	const CTA_Advanced_Object_Corner *p_target_object_corners,
	CTA_Target_Reference *p_target_reference,
	const CALIBRATION_CTA_T *p_cals)
{
	float32_T min_distance_to_ego_rear_bumper = INFTY;
	uint8_T index_min_distance = 0;
	uint8_T loop_index;
	/* Determination of the point with the minimum distance to the EGO's rear bumper
	according to the RCTA LAH ID 1193 */
	//coverity[misra_c_2012_rule_10_4_violation]  loop_index's type casting is normally doing
	for (loop_index = 0; loop_index < 8; loop_index++)
	{
		float32_T x_rear_bumper;
		float32_T distance_to_cs_origin;

		x_rear_bumper = p_target_object_corners->points[loop_index].x + tracker_cs_to_reference_cs;
		distance_to_cs_origin = SQRT((x_rear_bumper*x_rear_bumper) + (p_target_object_corners->points[loop_index].y * p_target_object_corners->points[loop_index].y));

		if (distance_to_cs_origin < min_distance_to_ego_rear_bumper)
		{
			min_distance_to_ego_rear_bumper = distance_to_cs_origin;
			index_min_distance = loop_index;
		}
	}

	/* Assign default reference point to 2 with RightSide, 0 with LeftSide */
    //coverity[misra_c_2012_rule_10_3_violation]  index_min_distance's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  index_min_distance's type casting is normally doing
	//if (FAST_ABS(p_target_object_corners->points[index_min_distance].y) >= p_cals->k_cta_min_lat_posn_for_default_ref_point)
    if (FAST_ABS(p_target_object_corners->points[index_min_distance].y) >= 100.0F)
	{
        //coverity[misra_c_2012_rule_10_4_violation]  index_min_distance's type casting is normally doing
		if ((RightSide == p_object->attributes->side) && (2 != index_min_distance))
		{
			index_min_distance = 2;
		}
        //coverity[misra_c_2012_rule_10_4_violation]  index_min_distance's type casting is normally doing
		else if ((LeftSide == p_object->attributes->side) && (0 != index_min_distance))
		{
			index_min_distance = 0;
		}
        else{/* intentional empty for coverity */}
	}
	
	p_target_reference->point = p_target_object_corners->points[index_min_distance];
    //coverity[misra_c_2012_rule_10_4_violation]  index_min_distance's type casting is normally doing
	p_target_reference->index = index_min_distance + 1; 

	if ((p_target_reference->index == 3) || (p_target_reference->index == 5))
	{
		p_target_reference->neighbor1 = p_target_reference->index + 1;
		p_target_reference->neighbor2 = p_target_reference->index + 2;
	}
	else if ((p_target_reference->index == 1) || (p_target_reference->index == 7))
	{
        //coverity[misra_c_2012_rule_10_4_violation]  index_min_distance's type casting is normally doing
		p_target_reference->neighbor1 = ((index_min_distance + 6) % 8) + 1;
		p_target_reference->neighbor2 = ((index_min_distance + 6) % 8) + 2;
	}
	else
	{
        //coverity[misra_c_2012_rule_10_4_violation]  index_min_distance's type casting is normally doing
		p_target_reference->neighbor1 = ((index_min_distance + 1) % 8) + 1;
		p_target_reference->neighbor2 = ((index_min_distance + 7) % 8) + 1;
	}
}


/**
 * FUNCTION: Check_Single_Level
 * This function evaluates criticality level of target from relevant object list based on criteria determined by
 * - TTC in x-axis \n
 * - radial distance \n
 * - intersection point in x-axis.
 *
 * \param[in,out] last_cycle_krit_level
 * \param[in,out] p_waiting_counter
 * \param[in,out] p_holding_counter
 * \param[in,out]  f_level_1_is_active
 *
 * \param[in]  krit_level
 * \param[in]  in_zone
 * \param[in]  p_target_attributes
 * \param[in]  max_ttc
 * \param[in]  max_radial_distance
 * \param[in]  min_intersec_point
 * \param[in]  max_intersec_point
 * \param[in]  suppress_cycle_count
 * \param[in]  hold_cycle_count
 * \param[in]  f_prevent_fall_back_level_1
 *
 * \return     boolean_T	Return true when current criticality level is reached
 * \remark
 * \ServID         xx
 * \Reentrancy     non-reentrant
 * \Synchronism    synchronous
 * \Precondition   none
 * \Caveats        none
 * \Requirements
 * \reqtrace{}{}
 */
static boolean_T Check_Single_Level(uint8_T last_cycle_krit_level,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:LEVEL [Justified:Low] "acceptable callng number"*/
	                                   uint8_T* p_waiting_counter,
									   uint8_T* p_holding_counter,
									   const uint8_T krit_level,
									   const boolean_T in_zone,
									   const TargetAttributes_T* p_target_attributes,
									   const float32_T max_ttc,
									   const float32_T max_radial_distance,
									   const float32_T min_intersec_point,
									   const float32_T max_intersec_point,
									   const float32_T intersec_accuracy_thres,
									   const uint8_T suppress_cycle_count,
									   const uint8_T hold_cycle_count,
									   const uint8_T f_prevent_fall_back_level_1,
									   uint8_T* f_level_1_is_active)
{
    boolean_T ret;
    //coverity[misra_c_2012_rule_10_3_violation]  CTA_NO_CRIT's type casting is normally doing
	uint8_T alert_level = CTA_NO_CRIT;
	if ((TRUE == in_zone)
        //coverity[misra_c_2012_rule_10_4_violation]  ttc's type casting is normally doing
		&& (p_target_attributes->ttc < max_ttc)
		&& (p_target_attributes->radial_distance < max_radial_distance)
		&& (p_target_attributes->intersect_points.x > min_intersec_point)
		&& (p_target_attributes->intersect_points.x < max_intersec_point)
		&& (p_target_attributes->intersect_point_accuracy < intersec_accuracy_thres))
	{
		if (*p_waiting_counter < suppress_cycle_count)
		{
                //coverity[misra_c_2012_rule_10_4_violation]  p_waiting_counter's type casting is normally doing
			*p_waiting_counter += 1;
		}
		else
		{
			alert_level = krit_level;
            //coverity[misra_c_2012_rule_10_4_violation]  krit_level's type casting is normally doing
			if (krit_level == CTA_CRIT_LEVEL_1)
			{
                //coverity[misra_c_2012_rule_10_3_violation]  f_level_1_is_active's type casting is normally doing
                //coverity[misra_c_2012_rule_10_4_violation]  f_level_1_is_active's type casting is normally doing
				*f_level_1_is_active = TRUE;
			}

			if (*p_holding_counter < hold_cycle_count)
			{
                //coverity[misra_c_2012_rule_10_4_violation]  p_holding_counter's type casting is normally doing
				*p_holding_counter += 1;
			}
		}
	}
    //coverity[misra_c_2012_rule_10_4_violation]  f_prevent_fall_back_level_1's type casting is normally doing
	else if ((last_cycle_krit_level >= krit_level) || (FALSE != f_prevent_fall_back_level_1))
	{
        //coverity[misra_c_2012_rule_10_4_violation]  f_prevent_fall_back_level_1's type casting is normally doing
		if (FALSE != f_prevent_fall_back_level_1)
		{
            //coverity[misra_c_2012_rule_10_4_violation]  last_cycle_krit_level's  type casting is normally doing
			if (last_cycle_krit_level >= CTA_CRIT_LEVEL_2)
			{
				alert_level = last_cycle_krit_level;
                //coverity[misra_c_2012_rule_10_4_violation]  last_cycle_krit_level's  type casting is normally doing
				if ((last_cycle_krit_level >= CTA_CRIT_LEVEL_3) || (*p_holding_counter >= hold_cycle_count))
				{
					*p_holding_counter = hold_cycle_count;
				}
				else
				{
					(*p_holding_counter)++;
				}
			}
		}
		else if (*p_holding_counter < hold_cycle_count)
		{
            //coverity[misra_c_2012_rule_10_3_violation]  krit_level's type casting is normally doing
			alert_level = krit_level;
            //coverity[misra_c_2012_rule_10_4_violation]  p_holding_counter's type casting is normally doing
			*p_holding_counter += 1;
		}
		else
		{
            //coverity[misra_c_2012_rule_10_3_violation]  krit_level's type casting is normally doing
            //coverity[misra_c_2012_rule_10_4_violation]  krit_level's type casting is normally doing
			alert_level = krit_level - 1;
			*p_holding_counter = 0;
			*p_waiting_counter = 0;
		}
	}
	else
	{
		*p_waiting_counter = 0;
	}

	if (alert_level >= krit_level)
	{
		ret = TRUE;
	}
	else
	{
		ret = FALSE;
	}

    return ret;
}


/**
* FUNCTION: Check_All_Levels
* This functions checks if current object is with the highest criticality level
* - Initialization of the criticality criteria: TTC, radial distance, intersection point with x axis
* - Adapt the thresholds when the corresponding criticality level is activated in the previous cycle
* - Evaluate the criticality level of the target_id
* - Determine the criticality level in the current cycle \n
* Return TRUE if: \n
* - current criticality level is higher than the previous cycle
* - current criticality level is equal to the previous cycle: \n
* 1) TTC value shall be no larger than the previous cycle \n
* 2) when TTC value equals, radial distance shall be no larger than the previous cycle
*
* \param[in,out] max_level
* \param[in,out] p_object_highest_crit
*
* \param[in] p_object
* \param[in] f_target_in_zone
* \param[in] p_cals
* \param[in] p_cta_cals
*
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/
static void Check_All_Levels(CTA_Crit_Level_T* max_level,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/ /*polyspace CODE-METRIC:LEVEL [Justified:Low] "acceptable callng number"*/
	CTA_Object_Data_T *p_object_highest_crit,
	const CTA_Object_Data_T *p_object,
	const boolean_T f_target_in_zone,
	const Level_Logic_Calibration_T* p_level_cals,
	const CALIBRATION_CTA_T* p_cals)
{
	uint8_t k;
    //coverity[misra_c_2012_rule_10_3_violation]  f_level_1_is_active's type casting is normally doing
	uint8_t f_level_1_is_active = FALSE;
	CTA_Crit_Level_T current_int_level = CTA_NO_CRIT;
	uint8_T* last_cycle_krit_level = &(p_object->persistent->prev_RCTA_enLastCycleCritLevelOn.val[p_object->attributes->side]);

	/* Evaluates criticality level 1 to 5 */
	for (k = 0; k < p_cals->k_cta_num_level; k++)
	{
		CTA_Crit_Level_T crit_level;
		float32_T ttcThres;
		float32_T radDistThres;
		float32_T interSecMin;
		float32_T interSecMax;
		float32_T interSecAcc;
		uint8_T f_prevent_fall_back_level_1;
		uint8_T* p_number_cycles_level_qualif = &(p_object->persistent->prev_RCTA_enAnzZyklKritQualif.level[k].val[p_object->attributes->side]);
		uint8_T* p_number_cycles_level_hold = &(p_object->persistent->prev_RCTA_enAnzZyklKritHalten.level[k].val[p_object->attributes->side]);
        //coverity[misra_c_2012_rule_10_4_violation]  k's type casting is normally doing
        //coverity[misra_c_2012_rule_10_8_violation]  k's type casting is normally doing
		crit_level = (CTA_Crit_Level_T)(k + 1);
		ttcThres = p_level_cals->tTTCMaxKritLevel[k];
		radDistThres = p_level_cals->lAbstMaxKritLevel[k];
		interSecMin = p_level_cals->sSPXMinKritLevel[k];
		interSecMax = p_level_cals->sSPXMaxKritLevel[k];
		interSecAcc = p_cals->k_cta_threshold_error_estimation_intersect_point;
        //coverity[misra_c_2012_rule_10_4_violation]  last_cycle_krit_level's type casting is normally doing
		if (*last_cycle_krit_level >= crit_level)
		{
			float32_T diff_hyst_intersect;
			/* In case a criticality level is active, the respective thresholds are adapted here by applying a hysteresis */
			diff_hyst_intersect = (p_level_cals->hysteresisGain - 1.0f)*(interSecMax - interSecMin)*0.5f;
			ttcThres *= p_level_cals->hysteresisGain;
			radDistThres *= p_level_cals->hysteresisGain;
			interSecMin -= diff_hyst_intersect;
			interSecMax += diff_hyst_intersect;
			interSecAcc *= p_level_cals->hysteresisGain;
		}
        //coverity[misra_c_2012_rule_10_4_violation]  k_cta_f_prevent_fall_back_to_critlevel_1's type casting is normally doing
    #if 0 // HKMC doesnt use
		if (TRUE == p_cals->k_cta_f_prevent_fall_back_to_critlevel_1)
		{
            //coverity[misra_c_2012_rule_10_4_violation]  f_level_1_is_active's type casting is normally doing
			if ((crit_level == CTA_CRIT_LEVEL_2) && (*last_cycle_krit_level >= CTA_CRIT_LEVEL_1) && (TRUE == f_level_1_is_active))
			{
                //coverity[misra_c_2012_rule_10_3_violation]  f_prevent_fall_back_level_1's type casting is normally doing
				f_prevent_fall_back_level_1 = TRUE;
			}
			else
			{
                //coverity[misra_c_2012_rule_10_3_violation]  f_prevent_fall_back_level_1's type casting is normally doing
				f_prevent_fall_back_level_1 = FALSE;
			}
		}
		else
   #endif
		{
            //coverity[misra_c_2012_rule_10_3_violation]  f_prevent_fall_back_level_1's type casting is normally doing
			f_prevent_fall_back_level_1 = FALSE;
		}
		if (TRUE == Check_Single_Level(*last_cycle_krit_level,
			p_number_cycles_level_qualif,
			p_number_cycles_level_hold,
    		//coverity[misra_c_2012_rule_10_3_violation]  crit_level's type casting is normally doing
			crit_level,
			f_target_in_zone,
			p_object->attributes,
			ttcThres,
			radDistThres,
			interSecMin,
			interSecMax,
			interSecAcc,
			p_level_cals->cycle_count_suppress_true_warning,
			p_level_cals->cycle_count_hold_true_warning,
			f_prevent_fall_back_level_1,
			&f_level_1_is_active))
		{
			/* check if current target has highest criticality level */
			current_int_level = crit_level;
			if (current_int_level > *max_level)
			{
				*max_level = current_int_level;
                //coverity[misra_c_2012_rule_10_1_violation]  p_object->attributes->side's type casting is normally doing
				Set_Object_highest_crit(&(p_object_highest_crit[p_object->attributes->side]), p_object);
			}
			else if ((current_int_level == *max_level)
                //coverity[misra_c_2012_rule_10_1_violation]  ttc's type casting is normally doing
                //coverity[misra_c_2012_rule_10_4_violation]  ttc's type casting is normally doing
				&& (p_object->attributes->ttc <= p_object_highest_crit[p_object->attributes->side].attributes->ttc))
			{
				/* if level is equal, than smallest TTC decides; if TTC is also equal, the smallest radial distance is crucial */
                //coverity[misra_c_2012_rule_10_1_violation]  ttc's type casting is normally doing
                //coverity[misra_c_2012_rule_10_4_violation]  ttc's type casting is normally doing
				if (!(((FAST_ABS(p_object->attributes->ttc) - FAST_ABS(p_object_highest_crit[p_object->attributes->side].attributes->ttc)) <= 1.0e-3f)
					&& (p_object->attributes->radial_distance >= p_object_highest_crit[p_object->attributes->side].attributes->radial_distance)))
				{
                    //coverity[misra_c_2012_rule_10_1_violation]  p_object->attributes->side's type casting is normally doing
					Set_Object_highest_crit(&(p_object_highest_crit[p_object->attributes->side]), p_object);
				}
			}
			else
			{
				/* doing nothing*/
			}
		}
	}
    //coverity[misra_c_2012_rule_10_3_violation]  type casting is normally doing
	*last_cycle_krit_level = current_int_level;
}


/**
* FUNCTION: Set_Object_highest_crit
* This functions set data arrays to pointer p_object_highest_crit
*
* \param[in,out] p_object_highest_crit
*
* \param[in] p_object
*
* \return void
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}{}
*/
static void Set_Object_highest_crit(CTA_Object_Data_T* p_object_highest_crit,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const CTA_Object_Data_T* p_object)
{
	p_object_highest_crit->attributes = p_object->attributes;
	p_object_highest_crit->persistent = p_object->persistent;
	*(p_object_highest_crit->tracker_output) = *(p_object->tracker_output);
}


static void adaptIntersectLinesToSteeringAngle(const CALIBRATION_CTA_T* p_cals,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	Level_Logic_Calibration_T* p_level_cals,
	const float32_T steering_angle)
{

	float32_T adaptation_factor;
	uint8_T i;
	float32_T adapted_line_segment_length;

	if (FRONT_CTA == CTA_FrontOrRear)
	{
        //coverity[misra_c_2012_rule_10_4_violation]  k_cta_fcta_steer_angle_table's type casting is normally doing
		adaptation_factor = getSteeringBasedIntersectLinesAdaptionFactor(FAST_ABS(steering_angle), p_cals->k_cta_fcta_steer_angle_table, p_cals->k_cta_fcta_steer_factor_table);
	}
	else
	{
        //coverity[misra_c_2012_rule_10_4_violation]  k_cta_fcta_steer_angle_table's type casting is normally doing
		adaptation_factor = getSteeringBasedIntersectLinesAdaptionFactor(FAST_ABS(steering_angle), p_cals->k_cta_rcta_steer_angle_table, p_cals->k_cta_rcta_steer_factor_table);
	}

	for (i = 0; i < p_cals->k_cta_num_level; i++)
	{
		adapted_line_segment_length = (p_level_cals->sSPXMaxKritLevel[i] - p_level_cals->sSPXMinKritLevel[i]) * adaptation_factor;

		/* keep the line segment point which is closest to the
		   ego, but draw the point further away closer to the ego
		   by adding the adapted line segment length to the closest
		   segment point, thereby respecting the direction (front/rear) */
		if (FRONT_CTA == CTA_FrontOrRear)
		{
			p_level_cals->sSPXMaxKritLevel[i] = p_level_cals->sSPXMinKritLevel[i] + adapted_line_segment_length;
		}
		else
		{
			p_level_cals->sSPXMinKritLevel[i] = p_level_cals->sSPXMaxKritLevel[i] - adapted_line_segment_length;
		}
	}
}

/**
 * Calculates the adaption factor to be applied to the conflict zone length, based on the
 * steering angle given.
 */
static float32_T getSteeringBasedIntersectLinesAdaptionFactor(const float32_T steering_angle,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const float32_T* steer_angle_table,
	const float32_T* steer_factor_table)
{
	return interp1(steer_angle_table, steer_factor_table, CTA_STEER_SUPPORT_POINTS_CNT, steering_angle);
}
/* end of float32_T getSteeringBasedCZLengthAdaptionFactor(const CTA_Position_T position, const float32_T steering_angle) */

static void adaptIntersectLinesToObjectHeading(Level_Logic_Calibration_T* p_level_cals,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const CALIBRATION_CTA_T* p_cals,
	const float32_T obj_heading,
	const float32_T sin_min_park_angle)
{

	float32_T adaptation_factor;
	uint8_T i;
	float32_T adapted_line_length;

	/* compute the adaptation factor "traditionally" */
    //coverity[misra_c_2012_rule_10_1_violation]  obj_heading's type casting is normally doing
    //coverity[misra_c_2012_rule_10_4_violation]  obj_heading's type casting is normally doing
	adaptation_factor = 1.0f / max(FAST_SIN(FAST_ABS(obj_heading)), sin_min_park_angle);

	for (i = 0; i < p_cals->k_cta_num_level; i++)
	{
		adapted_line_length = (p_level_cals->sSPXMaxKritLevel[i] - p_level_cals->sSPXMinKritLevel[i]) * adaptation_factor;

		/* keep the line segment point which is closest to the
		   ego, but draw the point further away closer to the ego
		   by adding the adapted line segment length to the closest
		   segment point, thereby respecting the direction (front/rear) */
		if (CTA_FrontOrRear == FRONT_CTA)
		{
			p_level_cals->sSPXMaxKritLevel[i] = p_level_cals->sSPXMinKritLevel[i] + adapted_line_length;
		}
		else
		{
			p_level_cals->sSPXMinKritLevel[i] = p_level_cals->sSPXMaxKritLevel[i] - adapted_line_length;
		}
	}
}

/**
* FUNCTION: adaptIntersectPointToObjectHeading
* This function adapts the intersection point considering
* the geometric constraints of target heading and ego width
*
* \param[in,out] p_object
*
* \param[in] p_vehicle_data
* \param[in] sin_min_park_angle
* \param[in] cos_min_park_angle
*
* \return void
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}
*/
static void adaptIntersectPointToObjectHeading(CTA_Object_Data_T* p_object,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const VEHICLE_DATA_FLT_T* p_vehicle_data,
	const float32_T sin_min_park_angle,
	const float32_T cos_min_park_angle)
{
	float32_T sin_object_heading;
	float32_T cos_object_heading;
	float32_T coordinate_shift = 0.0f;

	sin_object_heading = FAST_SIN(p_object->tracker_output->heading);

	/* due to limited target approach angle, the following if-condition shall never be true */
	/* Comment assertion to make ReSim continue in specific scenario */
	//	assert(FAST_ABS(sin_object_heading) >= sin_min_park_angle);

	/* when target heading is outside the defined interval, heading threshold will be used instead of estimated heading in the following computations */
    //coverity[misra_c_2012_rule_10_4_violation]  sin_object_heading's type casting is normally doing
	if (FAST_ABS(sin_object_heading) < sin_min_park_angle)
	{
		sin_object_heading = sin_min_park_angle;
		cos_object_heading = cos_min_park_angle;
	}
	else
	{
		cos_object_heading = FAST_COS(p_object->tracker_output->heading);
	}

	/* Half of ego width is not divided by tanges of heading but instead multiplied with cosine and divided by sine.
	   This is basically the same as using tangens but avoids dividing by an undefined value at 90 or 270 ?  */
	//coverity[misra_c_2012_rule_10_4_violation]  sin_object_heading's type casting is normally doing
	p_object->attributes->intersect_adaption = FAST_ABS(0.5f*p_vehicle_data->host_vehicle_width*cos_object_heading / sin_object_heading);

	/* In order to caclulate the distance of the intersection point to the relevant bumper, the length of the ego vehicle needs to be considered in case of rear CTA. */
	if (CTA_FrontOrRear == REAR_CTA)
	{
		coordinate_shift = p_vehicle_data->host_vehicle_length;
	}

	/* The sign of intersection point adaption is chosen here by comparing the distance to the rear or front bumper of the intersection point after applying the correction with negative and positive sign.
	   The sign is chosen that leads to the closest distance */
    //coverity[misra_c_2012_rule_10_4_violation]  intersect_points's type casting is normally doing
	if (FAST_ABS(p_object->attributes->intersect_points.x + p_object->attributes->intersect_adaption + coordinate_shift) > FAST_ABS(p_object->attributes->intersect_points.x - p_object->attributes->intersect_adaption + coordinate_shift))
	{
		p_object->attributes->intersect_adaption = -p_object->attributes->intersect_adaption;
	}

	p_object->attributes->intersect_points.x += p_object->attributes->intersect_adaption;
}

/**
* FUNCTION: getRelativeLongitudinalVelocityObject
* This function returns the target longitudinal relative velocity. Different calculation methods
* can be implemented here.
*
* \param[out] lon_rel_vel
*
* \param[in] p_target_object_data
* \param[in] p_vehicle_data
* \param[in] p_cals
*
* \return void
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{FERHR-SDD-C2RCTA-getRelativeLongitudinalVelocityObject}{FERHR-SWS-C2RCTA-452-1, FERHR-SWS-C2RCTA-463-1}
*/
static float32_T getRelativeLongitudinalVelocityObject(const Tracker_Output_Object_T* p_target_object_data,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const VEHICLE_DATA_FLT_T *p_vehicle_data,
	const CALIBRATION_CTA_T* p_cals)
{
	float32_T cos_heading;
	float32_T lon_rel_vel;

	if (p_cals->k_cta_f_use_heading_for_relative_velocity_calculation)
	{
		cos_heading = FAST_COS(p_target_object_data->heading);

		/*% compute projection vector */
		lon_rel_vel = (p_target_object_data->speed * cos_heading) - p_vehicle_data->speed;
	}
	else
	{
		lon_rel_vel = p_target_object_data->vcs.relative_velocity.x;
	}

	return lon_rel_vel;
}

/**
* FUNCTION: getRelativeLateralVelocityObject
* This function returns the target lateral relative velocity. Different calculation methods
* can be implemented here.
*
* \param[out] relative_velocity
*
* \param[in] p_target_object_data
* \param[in] p_vehicle_data
* \param[in] p_cals
*
* \return void
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}
*/
static float32_T getRelativeLateralVelocityObject(const Tracker_Output_Object_T* p_target_object_data,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const CALIBRATION_CTA_T* p_cals)
{
	float32_T sin_heading;
	float32_T lat_rel_vel;

	if (p_cals->k_cta_f_use_heading_for_relative_velocity_calculation)
	{
		sin_heading = FAST_SIN(p_target_object_data->heading);

		/*% compute projection vector */
		lat_rel_vel = (p_target_object_data->speed * sin_heading);
	}
	else
	{
		lat_rel_vel = p_target_object_data->vcs.relative_velocity.y;
	}

	return lat_rel_vel;
}

/**
* FUNCTION: getRelativeVelocityObject
* This function returns the target relative velocity vector. Different calculation methods
* can be implemented here.
*
* \param[out] relative_velocity
*
* \param[in] p_target_object_data
* \param[in] p_vehicle_data
* \param[in] p_cals
*
* \return void
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}
*/
static Vector_2d_T getRelativeVelocityObject(const Tracker_Output_Object_T* p_target_object_data,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const VEHICLE_DATA_FLT_T *p_vehicle_data,
	const CALIBRATION_CTA_T* p_cals)
{
	Vector_2d_T obj_relative_velocity;

	/*% compute projection vector */
	obj_relative_velocity.x = getRelativeLongitudinalVelocityObject(p_target_object_data,
		p_vehicle_data,
		p_cals);

	obj_relative_velocity.y = getRelativeLateralVelocityObject(p_target_object_data,
		p_cals);	

	return obj_relative_velocity;
}

/**
* FUNCTION: getIntersectionPointAccuracy
* This function returns an estimate of the accuracy of the intersection point prediction.
*
* \param[out] accuracy_estimate
*
* \param[in] p_target_object_data
* \param[in] relative_velocity
* \param[in] p_cals
*
* \return void
* \remark
* \ServID         xx
* \Reentrancy     non-reentrant
* \Synchronism    synchronous
* \Precondition   none
* \Caveats        none
* \Requirements
* \reqtrace{}
*/
static float32_T getIntersectionPointAccuracy(const Tracker_Output_Object_T* p_target_object_data,/*polyspace CODE-METRIC:VG [Justified:Low] "acceptable complexity"*/
	const Vector_2d_T obj_relative_velocity,
	const CALIBRATION_CTA_T* p_cals)
{
	float32_T denominator;
	float32_T accuracy_estimate = 0.0f;
    //coverity[misra_c_2012_rule_10_4_violation]  k_cta_f_use_heading_for_relative_velocity_calculation's type casting is normally doing
	if (FALSE == p_cals->k_cta_f_use_heading_for_relative_velocity_calculation)
	{
        //coverity[misra_c_2012_rule_10_4_violation]  denominator's type casting is normally doing
		denominator = ((obj_relative_velocity.y*obj_relative_velocity.y) - (FAST_ABS(obj_relative_velocity.y)*p_target_object_data->accuracy.velocity.y));
		if (FAST_ABS(denominator) < 1.0e-5f)
		{
			denominator = 1.0e-5f;
		}
        //coverity[misra_c_2012_rule_10_4_violation]  vcs.position's type casting is normally doing
		accuracy_estimate = (FAST_ABS(p_target_object_data->vcs.position.y)*((FAST_ABS(obj_relative_velocity.y)*p_target_object_data->accuracy.velocity.x)+ (fabsf(obj_relative_velocity.x)*p_target_object_data->accuracy.velocity.y))) / denominator;
	}

	return accuracy_estimate;
}


/*===========================================================================================================================================================
* File Revision History (top to bottom: first revision to last revision)
*============================================================================================================================================================
*
*  Date          Name        Change Description
* ------------  ----------  ---------------------------------------------------------------------------------------------------------------------------------
* 16-July-13    Susan Dong  SCR kok_css2#19292: first CTA M script model to C code.
* 13-SEP-2013   AJS		    BMW - kok_css2#19872 : Integrate the new CTA algorithm and create CTA and LCDA interface files
* 31-JAN-2014   AJS		    kok_css2#20947: BMW: Integrate FCTA into the RCTA algorithm
* 24-FEB-2014   AJS         kok_css2#21176: BMW:Implement message 0x01 and 0x02 of PCAN dbc v10
* 12-JUN-2014   Penchal     kok_css2#22320: BMW:CTA V2.4and RECW V2.3 Integration and LCDA changes required for ARMV1.9.46
* 18-SEP-2014   Penchal     kok_css2#23313: Calibration tools Improvements Integration
* 10-June-2015  raghu       replaced sqrt calls with assembly implementation
* 29-JUL-2015   Ananthesh   AUDI_CTA:Replace sqrtf with mrr_base_ops_sqrt
* 26-AUG-2015   Ananthesh   task47175:AUDI_CTA:Integrate CTA Version 1.3
* 30-Sep-2015   Maus		#28682 fix velocity thresholds for Audi Kritikalitaetslevel
* 30-Sep-2015   Maus		#28427 fill XCP outputs with logic
* 07.10.2015    Maus		Implement "Eingeschraenkte Funktion"
* 07.10.2015    Maus        #28624 Do coordinate transformation for intersection point x
* 07.10.2015    Maus        Refactoring: remove RCTA_Status and RCTA_Modus
* 21.10.2015    Haak		#29271: removed memory access violation in function 'intersectRoadShape'
* 21.10.2015    Haak		#29363: implement missing emergency brake holding conditions
* 21.10.2015    Haak        #29272: Implementation of additional activation conditions for Krit.-Level 4 and 5
* 22.10.2015    Haak        #29272: Adapted implementation: Activation logic of actuators had to be changed
* 05.11.2015    Haak        #29514: Removed unnecessary code and improved formatting
* 05.11.2015    Haak        #29726: Fixed memory access violation and enabled output of target attributes in case no warning or alert is active
* 09.11.2015    Haak        #28233: Prevent Krit.-Level fallback from higher levels to 1 and seperation of BMW- and AUDI-code
* 20.11.2015    Haak        #29517: Moved redundant code to subfunctions, reduced number of persistant and function stack variables, cleaned up
* 03.12.2015    Haak        #29517: Moved more code to subfunctions in order to group the code; made some improvements in hysteresis calculation
* 11.12.2015    Haak        #29517: Moved more code to subfunctions in order to group the code; removed the side-loop in order to reduce calculation overhead
* 11.12.2015    Haak        #30332: Seperated variable declaration and initialization with variables for compatibility reasons with embedded system compiler
* 22.12.2015    Haak        #28919: Implemented BAP RCTA BrakeIntervention flag to suppress the respective actors
* 22.12.2015    Haak        #29975: Implemented missing activation and suppression conditions based on missing FR signals
* 23.12.2015    Haak        #30059: expanded the use of the blocking timer for break jerk only to break jerk and emergency break
* 16.02.2016	Chen		AEH-5: modified calculation on radial distance. Current reference point will be used when activated.
* 22.02.2016    Haak        #31406: Added the use of hysteresis to minimum existence probability threshold. Created subfunctions as object list filter.
* 26.02.2016    Chen        AEH-10: modified function of evaluation on criticality level
* 26.02.2016    Haak        Added limitation function to saturate debugs signals
* 10.03.2016	Chen        AEH-11 AEH-13: Structure changes. Add Post Run for reset, xcpOutput. Remove AUDI Trigger Logic
* 15.03.2016	Chen		Added DocGen tool compliant comments for each local function definition
* 13.04.2016    Haak        AEH-20: Removed several local variables from checkCTA scope and changed some local function interfaces
* 25.04.2016    Wappler     AEH-01 & AEH-23: adapt criticality level intersection point criteria to steering angle & use Audi level logic for BMW
* 27.04.2016    Haak        AEH-27: Fixed a bug related to the intersection point hysteresis
* 28.04.2016    Ananthesh   AUDI_CTA:kok_css2#33005:Replace H2A_Bmw_Data_T with H2A_Cust_Data_T and A2H_Bmw_Data_T with A2H_Cust_Data_T
* 09.05.2016    Wappler     Debug functionality removed for merge into main branch
* 10.05.2016    Chen		AEH-28: Fix MISRA warning, constraint error, implicit conversions, operation equality
* 19.05.2016    Chen        AEH-28: Remove redundant codes, introduce sub-functions to reduce code complexity, fix MISRA Warning
* 25.05.2016    Chen        AEH-30: Remove unit conversion, transformation from AUDI_CS to VCS, replace k_cta_host_veh_length with vehicle data, refactoring function
* 16.06.2016    Chen        AEH-14: Modify function Check_All_Levels and Check_Single_Level to enhance code readability
* 23.06.2016	Krieg		AEH-33: Added new struct to encapsulate the object spezific data from the Trackerput. Introduced an iterator to loop over all objects in the tracker_output
* 27.06.2016	Krieg		AEH-39:	Created new function for th body of the loop of the checkCTA function
* 28.06.2016	Krieg		AEH-45: Removed headingEstimationLogic and functions that are only called by this function. Simplified the code of the CheckCTA_Single_Object function.
* 30.06.2016	Krieg		AEH-34: Addaption for the changes in CTA_Persistent_T.h: cta_reference_point_t, cta_corners_point_t, cta_zone_point_t structures changed.
* 30.06.2016	Krieg		Changed the usage of the tracker_iterator. Now only a pointer is being used.
* 05.07.2016	Krieg		Refactored getRadialDistanceToOEMOsCoordinateOrigin and Check_Object_Validity
* 19.07.2016    Chen        AEH-14: Fix MISRA warning in function Check_All_Levels and Check_Single_Level
* 02.08.2016	Krieg		Introduced CTA_Iterator
* 03.08.2016    Chen        AEH-65: Add cta_internal_debug in compliance to new bin writer
* 11.08.2016    Chen        AEH-68 & AEH-66 Bug fix in checkCTA and getIntersectionPointWithXandYAxis
* 18.08.2016    Chen        AEH-69: Implement stop TTC hysteresis to avoid toggling
* 07.09.2016    Litty       CR kok_css2#35582: [BMW SRR3] - Make feature functions modular by removing external dependencies
* 12.10.2016    Chen        AEH-83: Implement new approach to calculate TTC with regards to half ego width
* 24.10.2016    Chen        AEH-82: Modify Function Check_All_Levels, introduce new CAL k_cta_num_level 
* 24.11.2016    Chen        AEH-96: Integrate new algorithm on path tracking to enhance feature performance
* 27.02.2018    Srinidhi    AHH-1529: New Feature Function Integeration for Hyundai Project
* 16.10.2018    Justin.Jee  AQB-271 : Limit dynamic Y distance to avoid unwanted RCCW warning at sleeping shelter
* 26.03.2019    Justin.Jee  ATH-166 : Remove compile warning for coverity 
* 23.05.2019    Justin.Jee  ATH-195EBE-157 : Feature zone re-arrange
=============================================================================================================================================================*/
