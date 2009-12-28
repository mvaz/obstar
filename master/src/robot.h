/********************************************************
 *************** ROBOT'S PHYSICAL CONSTANTS *************
 ********************************************************/

/* DIMENSIONS */
#define  RADIUS_MM       26.15
#define  FICT_RADIUS_MM  50.00

static integer RADIUS;
static integer FICT_RADIUS;

/* SENSORS */
#define  NIR             6

static real theta[NIR]       = { -67, -43, -14, 14, 43, 67 };
static real delta_theta[NIR] = { 18, 30, 28, 28, 30, 18};
static real theta_square[NIR];



