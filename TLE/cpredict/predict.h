/*****************************************************************************/
/* predict.h								     */
/*****************************************************************************/

#ifndef	__PREDICT_H_INCLUDED
#define	__PREDICT_H_INCLUDED	1

/*****************************************************************************/

struct observation 
{
 double		epoch;
 char		orbital_model[5];
 int		norad_id;
 char		name[25];
 double		latitude;
 double		longitude;
 double		altitude;
 double		orbital_velocity;
 double		footprint;
 double		eclipse_depth;
 double		orbital_phase;
 char		sunlit;
 int		orbit;
 char		geostationary;
 double		azimuth;
 double		elevation;
 double		slant_range;
 char		visibility;
 char		has_aos;
 char		decayed;
 double		doppler;
 double		eci_x;
 double		eci_y;
 double		eci_z;
 double		eci_vx;
 double		eci_vy;
 double		eci_vz;
 double		eci_sun_x;
 double		eci_sun_y;
 double		eci_sun_z;
 double		eci_obs_x;
 double		eci_obs_y;
 double 	eci_obs_z;
 double		beta_angle;
};

int	predict_is_valid(char *tle1,char *tle2);
int	predict_observation(char *tle0,char *tle1,char *tle2,double epoch,double stnlat,double stnlong,double stnalt,struct observation *obs);
int 	predict_fprint_observation(FILE *fw,struct observation *obs);

/*****************************************************************************/

#endif

/*****************************************************************************/


