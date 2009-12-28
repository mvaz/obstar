/*
 *
 *
 *
 *
 */

#define LINEMAX         255
#define MAX_ARGS        20

#define EQUALS(x, y)    ( !( strcasecmp(x, y) ) )
#define GETLINE(s)      ( fgets(s, LINEMAX - 1, stdin) )

#define PROMPT         ">> "
#define EXIT_CONFIRMATION "u sure? ( yes / [no] )\r\n"
#define FAREWELL       "\r\n  Ciao!\r\n"

#define SEPS           " \t\r\n"

/* TODO implemente the hierarchy of time constants? */
#define VALID_TOBS(x)  ( (x) > 0 )
#define INVALID_TOBS   "Invalid value for 'tobs': must be greater than 0!\r\n"

#define VALID_TTAR(x)  ( (x) > 0 )
#define INVALID_TTAR   "Invalid value for 'ttar': must be greater than 0!\r\n"

#define VALID_T2TAR(x) ( (x) > 0 )
#define INVALID_T2TAR  "Invalid value for 't2tar': must be greater than 0!\r\n"

#define VALID_T2C(x)   ( (x) > 0 )
#define INVALID_T2C    "Invalid value for 't2c': must be greater than 0!\r\n"

#define VALID_B2(x)    ( (x) > 0 )
#define INVALID_B2     "Invalid value for 'Beta2': must be greater than 0!\r\n"

#define VALID_Q(x)     ( (x) >= 0 )
#define INVALID_Q      "Invalid value for 'Q': must be greater than 0!\r\n"

#define VALID_PHI(x)   ( ( (x) >= 0 ) && ( (x) < 360 ) )
#define INVALID_PHI    "Invalid value for 'Phi': must be an integer greater/equal than 0 and smaller than 360!\r\n"

#define VALID_CLOSE(x) ( (x) >= 0 )
#define INVALID_CLOSE   "Invalid value for 'close': must be greater than 0!\r\n"

#define VALID_LAMBDAV(x) ( (x) >= 0 )
#define INVALID_LAMBDAV "Invalid value for 'lambdav': must be greater than 0!\r\n"

#define NO_HIST_MESSAGE "No history yet\r\n"
