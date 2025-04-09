/*
 * Scurve.c
 *
 *  Created on: May 29, 2023
 *      Author: pawee
 */
#include "Scurve.h"

//Scurve_GenStruct Y_Traj;

//void traject(Scurve_GenStruct *Traj)
//{
//	Traj->t1 = 0.001;
//	Traj->dir = 0;
//	Traj->t2 = Traj->t1;
//}
double p1,p2,p3,p4,p5,p6 = 0.0;

void Trajectory_Generator(volatile Scurve_GenStruct *genScurve,float initial_p,float target_p,float vmax,float amax,float jmax){
	//Set parameter
	uint32_t s = 0;
	uint8_t M = 0;
	uint8_t N = 0;
	uint8_t pattern = 0;
	double va = 0.0f;
	double sa = 0.0f;
	double sv = 0.0f;
	double tj,ta,tv;

	genScurve->dir = 0;
	genScurve->time_total = 0.0;
	genScurve->t1 = 0.0;
	genScurve->t2 = 0.0;
	genScurve->t3 = 0.0;
	genScurve->t4 = 0.0;
	genScurve->t5 = 0.0;
	genScurve->t6 = 0.0;
	genScurve->t7 = 0.0;

	// Check for the feasibility of the trajectory
	if(vmax*jmax < amax*amax){
		M = 1;
		N = 0;
	}
	else{
		M = 0;
		N = 1;
	}

    // Determine the direction of the motion
    if (target_p - initial_p < 0){
    	genScurve->dir = -1;
    }
    else{
    	genScurve->dir = 1;
    }

    // Calculate the required distance
    s = fabs(target_p - initial_p);

    // Calculate the values of va, sa and sv
    va = (amax*amax)/jmax;
    sa = 2*(amax*amax*amax)/(jmax*jmax);
    sv = vmax*((M * 2 * sqrt(vmax/jmax)) + N*((vmax/amax)+(amax/jmax)));

    // Determine the shape of the trajectory based on the values of va, sa and sv
    if ((vmax <= va) && (s >= sa)){
        pattern = 1;
    }
    else if ((vmax > va) && (s < sa)){
        pattern = 2;
    }
    else if ((vmax < va) && (s < sa) && (s > sv)){
        pattern = 3;
    }
    else if ((vmax < va) && (s < sa) && (s < sv)){
        pattern = 4;
    }
    else if ((vmax >= va) && (s >= sa) && (s >= sv)){
        pattern = 5;
    }
    else if ((vmax >= va) && (s >= sa) && (s < sv)){
        pattern = 6;
    }

    // Calculate the values of tj, ta and tv for each trajectory pattern
    switch (pattern){
        case 1:
            tj = (float)pow((vmax/jmax),(0.5));
            ta = tj;
            tv = s/vmax;
            break;
        case 2:
            tj = (float)pow(s/(2.0*jmax),1.0/3.0);
            ta = tj;
            tv = 2*tj;
            break;
        case 3:
            tj = (float)pow((s/(2*jmax)),(1.0/3.0));
            ta = tj;
            tv = 2*tj;
            break;
        case 4:
            tj = (float)pow((s/(2*jmax)),(1.0/3.0));
            ta = tj;
            tv = 2*tj;
            break;
        case 5:
            tj = amax/jmax;
            ta = vmax/amax;
            tv = s/vmax;
            break;
        case 6:
            tj = amax/jmax;
            ta = 0.5*(sqrt(((4*s*jmax*jmax)+(amax*amax*amax)) / (amax * jmax*jmax)) - (amax/jmax));
            tv = ta + tj;
            break;
    }
    // Calculate the values of t1 to t7 and the total time
    genScurve->t1 = tj;
    genScurve->t2 = ta;
    genScurve->t3 = ta + tj;
    genScurve->t4 = tv;
    genScurve->t5 = tv + tj;
    genScurve->t6 = tv + ta;
    genScurve->t7 = tv + tj + ta;
    genScurve->time_total = genScurve->t7;
    if (s == 0){
    	genScurve->time_total = 0;
    }
}

void Trajectory_Evaluated(volatile Scurve_GenStruct *genScurve,volatile Scurve_EvaStruct *evaScurve,float initial_p,float target_p,float vmax,float amax,float jmax){
	static double v1,v2,v3,v4,v5,v6 = 0.0;
	static double a1,a2,a3,a4,a5,a6 = 0.0;

	evaScurve->t += 1.0/1000.0;
	if (evaScurve->t <= genScurve->time_total){
		evaScurve->isFinised = false;
		if(evaScurve->t <= genScurve->t1){
			evaScurve->setposition = initial_p + 1/6.0 * jmax * pow(evaScurve->t,3.0) * genScurve->dir;
			evaScurve->setvelocity =  1/2.0 * jmax * evaScurve->t * evaScurve->t * genScurve->dir;
			evaScurve->setacceleration = jmax *evaScurve->t* genScurve->dir;
			p2 = evaScurve->setposition;
			p1 = p2;
			v2 = evaScurve->setvelocity;
			v1 = v2;
			a2 = evaScurve->setacceleration;
			a1 = a2;
		}
		else if (genScurve->t1 < evaScurve->t && evaScurve->t <= genScurve->t2){
			evaScurve->setposition =  p1 + v1 * (evaScurve->t - genScurve->t1) + 1/2.0 * a1 * pow((evaScurve->t - genScurve->t1),2.0);
			evaScurve->setvelocity = v1 + a1*(evaScurve->t- genScurve->t1);
			evaScurve->setacceleration = a1;
			p3 = evaScurve->setposition;
			p2 = p3;
			v3 = evaScurve->setvelocity;
			v2 = v3;
			a3 = evaScurve->setacceleration;
			a2 = a3;
		}

		else if (genScurve->t2 <= evaScurve->t && evaScurve->t <= genScurve->t3){
			evaScurve->setposition = p2 + v2 * (evaScurve->t - genScurve->t2) + 1/2.0 * a2 * pow((evaScurve->t - genScurve->t2),2.0) + 1.0/6.0 * - jmax * genScurve->dir * pow((evaScurve->t - genScurve->t2),3.0);
			evaScurve->setvelocity = v2 + a2 * (evaScurve->t - genScurve->t2) + 1/2.0 * -jmax * genScurve->dir * pow((evaScurve->t - genScurve->t2),2.0);
			evaScurve->setacceleration = a2 - (jmax * (evaScurve->t - genScurve->t2))*genScurve->dir;
			p6 = evaScurve->setposition;
			p4 = p6;
			p3 = p4;
			v6 = evaScurve->setvelocity;
			v4 = v6;
			v3 = v4;
			a6 = evaScurve->setacceleration;
			a4 = a6;
			a3 = a4;
		}
		else if (genScurve->t3 < evaScurve->t && evaScurve->t < genScurve->t4){
			evaScurve->setposition = p3 + v3 * (evaScurve->t - genScurve->t3);
			evaScurve->setvelocity = v3;
			evaScurve->setacceleration = 0;
			p5 = evaScurve->setposition;
			p4 = p5;
			v5 = evaScurve->setvelocity;
			v4 = v5;
			a5 = evaScurve->setacceleration;
			a4 = a5;
		}
		else if (genScurve->t4 <= evaScurve->t && evaScurve->t <= genScurve->t5){
			evaScurve->setposition = p4 + v4 * (evaScurve->t - genScurve->t4) + (1.0/6.0) * genScurve->dir *- jmax * pow((evaScurve->t - genScurve->t4),3.0);
			evaScurve->setvelocity = v4 + 1.0/2.0 * - jmax * genScurve->dir * pow((evaScurve->t - genScurve->t4),2.0);
			evaScurve->setacceleration = (-jmax * (evaScurve->t - genScurve->t4))*genScurve->dir;
			p6 = evaScurve->setposition;
			p5 = p6;
			v6 = evaScurve->setvelocity;
			v5 = v6;
			a6 = evaScurve->setacceleration;
			a5 = a6;
		}

		else if (genScurve->t5 <  evaScurve->t &&  evaScurve->t <= genScurve->t6){
			evaScurve->setposition = p5 + v5 * (evaScurve->t - genScurve->t5) + 1/2.0 * a5 * pow(( evaScurve->t - genScurve->t5),2.0);
			evaScurve->setvelocity = v5 + a5 * (evaScurve->t - genScurve->t5);
			evaScurve->setacceleration = a5;
			p6 = evaScurve->setposition;
			v6 = evaScurve->setvelocity;
			a6 = evaScurve->setacceleration;
		}
		else if (genScurve->t6 <  evaScurve->t &&  evaScurve->t <= genScurve->t7){
			evaScurve->setposition = p6 + v6 * (evaScurve->t - genScurve->t6) + 1.0/2.0 * a6 * pow((evaScurve->t - genScurve->t6),2) + 1/6.0 * jmax* genScurve->dir * pow((evaScurve->t - genScurve->t6),3);
			evaScurve->setvelocity = v6 + a6 * (evaScurve->t - genScurve->t6) + 1.0/2.0 * jmax*genScurve->dir * pow((evaScurve->t - genScurve->t6),2);
			evaScurve->setacceleration = a6 + genScurve->dir *jmax * (evaScurve->t - genScurve->t6);
		}
	}
	else{evaScurve->isFinised = true;}
}
